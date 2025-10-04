/**
 * @file main.c
 * @brief Minimal example for driving an HD44780 16x2 LCD via a PCF8574 backpack (I2C) on ESP-IDF.
 *
 * This example uses the modern ESP-IDF driver API (`esp_driver_i2c`) and the public
 * LCD API provided by `i2c-lcd1602.h`.
 *
 * ## What it does
 * - Sets up the I2C master bus
 * - Adds the PCF8574 I2C device (typical addresses: 0x27 or 0x3F)
 * - Turns the LCD backlight on
 * - Writes two lines of text
 * - Toggles cursor/blink briefly
 * - Shows a degree symbol (depends on LCD ROM variant)
 * - Demonstrates display scroll left/right
 * - Blinks backlight quickly
 * - Updates a counter on the second line in a loop
 *
 * ## Hardware notes
 * - Change `I2C_SDA_GPIO` and `I2C_SCL_GPIO` to match your board.
 * - It is recommended to use external 4.7–10 kΩ pull-ups.
 * - If the LCD doesn’t respond, scan the bus or try address 0x3F.
 *
 * ## LCD character map note
 * - Special glyph codes (e.g., degree symbol) depend on the LCD ROM variant
 *   (A00 vs A02, etc.). The example uses codes defined in `i2c-lcd1602.h`.
 *
 * @author Dragos
 * @copyright MIT
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"   // esp_driver_i2c (ESP-IDF v5+)
#include "i2c-lcd1602.h"
#include "i2c_config.h"

/**
 * @brief Application entry point.
 *
 * Initializes the I2C master bus and the LCD1602 driver, writes some text,
 * demonstrates cursor/blink and a small scroll, then periodically updates a
 * counter on the second line.
 *
 * @details
 * Steps:
 *  1. Create I2C master bus
 *  2. Add PCF8574 device on the bus
 *  3. Initialize LCD1602 driver (2 rows, 40 columns internal, 16 visible)
 *  4. Write two lines
 *  5. Toggle cursor/blink briefly
 *  6. Show a degree symbol (depends on LCD ROM)
 *  7. Scroll display left/right
 *  8. Blink backlight quickly
 *  9. Loop updating a counter on line 2
 *
 * @note This function never returns; it runs under FreeRTOS scheduler.
 * @warning If your ESP-IDF version has slightly different field names in
 *          `i2c_master_bus_config_t` or `i2c_device_config_t`, adjust them accordingly.
 */
void app_main(void)
{
    esp_err_t err;
    
    // 1) Init I2C master bus and device
    i2c_master_bus_handle_t i2c_bus = NULL;
    i2c_master_dev_handle_t dev = NULL;
    err = i2c_init(&i2c_bus, &dev);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c init failed: %d", err); goto cleanup; }
    
    // 2) Initialize the LCD1602 driver
    i2c_lcd1602_info_t *lcd = i2c_lcd1602_malloc();
    if (!lcd) { ESP_LOGE(TAG, "lcd malloc failed"); return; }

    // LCD 16x2: internally 40 columns per line, 16 visible
    // num_rows=2, num_columns=40, num_visible_columns=16
    err = i2c_lcd1602_init(lcd, dev, /*backlight=*/true, 2, 40, 16);
    if (err != ESP_OK) { ESP_LOGE(TAG, "lcd init failed: %d", err); goto cleanup; }

    // 3) Basic writes
    i2c_lcd1602_clear(lcd);
    i2c_lcd1602_move_cursor(lcd, 0, 0);
    i2c_lcd1602_write_string(lcd, "ESP-IDF LCD1602");
    // Infinity symbol at end of first line (ROM type dependent)
    i2c_lcd1602_move_cursor(lcd, 15, 0);
    i2c_lcd1602_write_char(lcd, I2C_LCD1602_CHARACTER_INFINITY);
    i2c_lcd1602_move_cursor(lcd, 0, 1);
    i2c_lcd1602_write_string(lcd, "Hello, Buddy!");

    vTaskDelay(pdMS_TO_TICKS(2500));

    // 4) Cursor + blink ON briefly
    i2c_lcd1602_set_cursor(lcd, true);
    i2c_lcd1602_set_blink(lcd, true);
    vTaskDelay(pdMS_TO_TICKS(2500));
    i2c_lcd1602_set_blink(lcd, false);
    i2c_lcd1602_set_cursor(lcd, false);


    // 5) Small scroll left then right
    for (int i = 0; i < 8; ++i) {
        i2c_lcd1602_scroll_display_left(lcd);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    for (int i = 0; i < 8; ++i) {
        i2c_lcd1602_scroll_display_right(lcd);
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // 6) Blink backlight
    vTaskDelay(pdMS_TO_TICKS(300));
    i2c_lcd1602_set_backlight(lcd, false);
    vTaskDelay(pdMS_TO_TICKS(800));
    i2c_lcd1602_set_backlight(lcd, true);
    vTaskDelay(pdMS_TO_TICKS(800));
    i2c_lcd1602_set_backlight(lcd, false);
    vTaskDelay(pdMS_TO_TICKS(800));
    i2c_lcd1602_set_backlight(lcd, true);
    vTaskDelay(pdMS_TO_TICKS(800));

    // 7) Simple loop: update a counter on the second line
    int counter = 0;
    while (1) {
        char buf[20];
        snprintf(buf, sizeof(buf), "Count: %5d", counter++);
        i2c_lcd1602_move_cursor(lcd, 0, 1);
        i2c_lcd1602_write_string(lcd, "                "); // clear visible 16 chars
        i2c_lcd1602_move_cursor(lcd, 0, 1);
        i2c_lcd1602_write_string(lcd, buf);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

cleanup:
    // This label is used only for early errors (we never reach here in the loop).
    if (dev)        { (void)i2c_master_bus_rm_device(dev);  dev = NULL; }
    if (i2c_bus)    { (void)i2c_del_master_bus(i2c_bus);    i2c_bus = NULL; }
    if (lcd)        { i2c_lcd1602_free(&lcd); }
    // Note: i2c_bus/dev remain alive since app_main never exits in this example.
}
