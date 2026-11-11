/*
 * Buttons: an NES or SNES Classic Mini controller on the dev kit's I2C connector (SDA 7, SCL 8, the
 * board's shared bus). No paddle or battery measurement on this board; otherwise input is USB (and touch).
 *
 * The controller speaks Nintendo's I2C extension protocol at 0x52: unencrypted set-up (F0=55, FB=00),
 * FE=03 for 8-byte reports, buttons active low in bytes 6 and 7. Each poll reads the report requested
 * by the poll before (no wait between request and read) and requests the next one. A SNES pad has
 * every button; a NES pad reports A/B/Select/Start/D-pad in the same bits. Buttons map by name
 * (A = A, B = B, ...), as the emulators read them; the ZL/ZR bits some pads send = L2/R2.
 * Select+Start held is the launcher's HOME chord.
 *
 * The controller can be plugged in at any time: the pad is always registered as P1's board pad, and
 * while none answers a probe runs every RETRY_US. Implausible reports (opposite directions together)
 * and failed polls keep the last buttons; FAILS_MAX failures in a row count as unplugged.
 */
#include "board_internal.h"
#include "board_pins.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "board_pad";

#define PAD_ADDR    0x52
#define PAD_HZ      100000
#define PAD_TIMEOUT 20          /* ms per transfer */
#define REPORT_LEN  8
#define POLL_US     10000       /* the input task asks every 5 ms: read the pad at 100 Hz */
#define RETRY_US    500000
#define FAILS_MAX   5

static const struct {
    uint8_t byte, bit;
    uint32_t button;
} s_map[] = {
    {7, 0x01, BSP_BTN_UP},  {6, 0x40, BSP_BTN_DOWN},   {7, 0x02, BSP_BTN_LEFT}, {6, 0x80, BSP_BTN_RIGHT},
    {7, 0x10, BSP_BTN_A},   {7, 0x40, BSP_BTN_B},      {7, 0x08, BSP_BTN_X},    {7, 0x20, BSP_BTN_Y},
    {6, 0x20, BSP_BTN_L1},  {6, 0x02, BSP_BTN_R1},     {7, 0x80, BSP_BTN_L2},   {7, 0x04, BSP_BTN_R2},
    {6, 0x04, BSP_BTN_START}, {6, 0x10, BSP_BTN_SELECT},
};

static i2c_master_dev_handle_t s_pad;
static bool s_ready, s_requested;
static unsigned s_fails;
static uint32_t s_buttons;
static int64_t s_next_us;

static bool pad_write(uint8_t reg, uint8_t val)
{
    const uint8_t msg[2] = {reg, val};
    return i2c_master_transmit(s_pad, msg, sizeof(msg), PAD_TIMEOUT) == ESP_OK;
}

/* Probe first: a write to an absent device would log an error on every retry. */
static bool pad_setup(void)
{
    if (i2c_master_probe(board_i2c_bus(), PAD_ADDR, PAD_TIMEOUT) != ESP_OK) return false;
    if (!pad_write(0xF0, 0x55)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    if (!pad_write(0xFB, 0x00)) return false;
    vTaskDelay(pdMS_TO_TICKS(10));
    return pad_write(0xFE, 0x03);
}

static bool pad_collect(uint32_t *out)
{
    uint8_t d[REPORT_LEN];
    if (i2c_master_receive(s_pad, d, sizeof(d), PAD_TIMEOUT) != ESP_OK) return false;
    uint32_t b = 0;
    for (size_t i = 0; i < sizeof(s_map) / sizeof(s_map[0]); i++)
        if (!(d[s_map[i].byte] & s_map[i].bit)) b |= s_map[i].button;
    /* Up+down or left+right cannot be pressed together: a corrupted report. */
    if ((b & (BSP_BTN_UP | BSP_BTN_DOWN)) == (BSP_BTN_UP | BSP_BTN_DOWN) ||
        (b & (BSP_BTN_LEFT | BSP_BTN_RIGHT)) == (BSP_BTN_LEFT | BSP_BTN_RIGHT))
        return false;
    *out = b;
    return true;
}

esp_err_t bsp_buttons_init(void)
{
    if (!board_i2c_bus()) return ESP_ERR_NOT_FOUND;
    i2c_device_config_t dev = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PAD_ADDR,
        .scl_speed_hz = PAD_HZ,
    };
    esp_err_t err = i2c_master_bus_add_device(board_i2c_bus(), &dev, &s_pad);
    if (err != ESP_OK) return err;
    s_ready = pad_setup();
    ESP_LOGI(TAG, "Classic controller on I2C 0x%02X: %s", PAD_ADDR, s_ready ? "found" : "not plugged in");
    s_next_us = esp_timer_get_time() + (s_ready ? POLL_US : RETRY_US);
    board_set_cap(BSP_CAP_GPIO_PAD);
    return ESP_OK;
}

uint32_t bsp_buttons_read(void)
{
    if (!s_pad) return 0;
    int64_t now = esp_timer_get_time();
    if (now < s_next_us) return s_buttons;
    if (!s_ready) {
        s_ready = pad_setup();
        s_next_us = now + (s_ready ? POLL_US : RETRY_US);
        if (s_ready) {
            ESP_LOGI(TAG, "Classic controller plugged in");
            s_requested = false;
            s_fails = 0;
        }
        return 0;
    }
    s_next_us = now + POLL_US;
    bool ok = true;
    if (s_requested) ok = pad_collect(&s_buttons);
    uint8_t req = 0x00;
    s_requested = i2c_master_transmit(s_pad, &req, 1, PAD_TIMEOUT) == ESP_OK;
    if (ok && s_requested) {
        s_fails = 0;
    } else if (++s_fails >= FAILS_MAX) {
        ESP_LOGI(TAG, "Classic controller unplugged");
        s_ready = false;
        s_buttons = 0;
        s_next_us = now + RETRY_US;
    }
    return s_buttons;
}

int bsp_analog_read(int ch)
{
    return -1;
}

int bsp_battery_mv(void)
{
    return -1;
}
