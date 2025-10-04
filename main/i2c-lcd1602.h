/*
 * MIT License
 *
 * Copyright (c) 2025 <Dragos Hategan>
 *
 * Attribution: Original work by David Antliff (https://github.com/DavidAntliff).
 * This version is adapted to the current ESP-IDF (esp_driver_i2c).
 */

#ifndef I2C_LCD1602_H
#define I2C_LCD1602_H

/**
 * @file i2c-lcd1602.h
 * @brief Public API for HD44780-compatible LCDs driven via a PCF8574/PCF8574A I/O expander (I2C).
 *
 * The public API is documented here (header). Internal static helpers are documented in the .c file.
 */

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"   // esp_driver_i2c

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Opaque driver state for an I2C-connected LCD1602/HD44780. */
typedef struct
{
    bool init;                                  ///< True if struct has been initialised, otherwise false
    i2c_master_dev_handle_t dev_handle;         ///< I2C device handle (PCF8574/PCF8574A)
    uint8_t backlight_flag;                     ///< Non-zero if backlight is to be enabled, otherwise zero
    uint8_t num_rows;                           ///< Number of configured rows
    uint8_t num_columns;                        ///< Number of configured columns, including offscreen columns
    uint8_t num_visible_columns;                ///< Number of visible columns
    uint8_t display_control_flags;              ///< Currently active display control flags
    uint8_t entry_mode_flags;                   ///< Currently active entry mode flags
} i2c_lcd1602_info_t;

/* -------- Special characters (HD44780 ROM Code A00) --------
 * These codes depend on the LCD’s ROM variant. Values below assume A00.
 * If your module uses a different ROM (e.g., A02), glyph codes may differ.
 */
#define I2C_LCD1602_CHARACTER_CUSTOM_0     0b00001000
#define I2C_LCD1602_CHARACTER_CUSTOM_1     0b00001001
#define I2C_LCD1602_CHARACTER_CUSTOM_2     0b00001010
#define I2C_LCD1602_CHARACTER_CUSTOM_3     0b00001011
#define I2C_LCD1602_CHARACTER_CUSTOM_4     0b00001100
#define I2C_LCD1602_CHARACTER_CUSTOM_5     0b00001101
#define I2C_LCD1602_CHARACTER_CUSTOM_6     0b00001110
#define I2C_LCD1602_CHARACTER_CUSTOM_7     0b00001111

#define I2C_LCD1602_CHARACTER_ALPHA        0b11100000
#define I2C_LCD1602_CHARACTER_BETA         0b11100010
#define I2C_LCD1602_CHARACTER_THETA        0b11110010
#define I2C_LCD1602_CHARACTER_PI           0b11110111
#define I2C_LCD1602_CHARACTER_OMEGA        0b11110100
/* NOTE: The previous header had an invalid binary literal for SIGMA (…60).
 * I set it to 0b11110110 as a best guess for A00. If you need strict accuracy for your LCD’s ROM,
 * please verify against your module’s character map. */
#define I2C_LCD1602_CHARACTER_SIGMA        0b11110110
#define I2C_LCD1602_CHARACTER_INFINITY     0b11110011
#define I2C_LCD1602_CHARACTER_DEGREE       0b11011111
#define I2C_LCD1602_CHARACTER_ARROW_RIGHT  0b01111110
#define I2C_LCD1602_CHARACTER_ARROW_LEFT   0b01111111
#define I2C_LCD1602_CHARACTER_SQUARE       0b11011011
#define I2C_LCD1602_CHARACTER_DOT          0b10100101
#define I2C_LCD1602_CHARACTER_DIVIDE       0b11111101
#define I2C_LCD1602_CHARACTER_BLOCK        0b11111111

/** @brief CGRAM custom glyph index (0..7). */
typedef enum
{
    I2C_LCD1602_INDEX_CUSTOM_0 = 0,
    I2C_LCD1602_INDEX_CUSTOM_1,
    I2C_LCD1602_INDEX_CUSTOM_2,
    I2C_LCD1602_INDEX_CUSTOM_3,
    I2C_LCD1602_INDEX_CUSTOM_4,
    I2C_LCD1602_INDEX_CUSTOM_5,
    I2C_LCD1602_INDEX_CUSTOM_6,
    I2C_LCD1602_INDEX_CUSTOM_7,
} i2c_lcd1602_custom_index_t;

#ifndef TAG
#define TAG "i2c-lcd1602"
#endif

/** @brief Log a warning if an ESP-IDF call fails (does not abort). */
#define I2C_LCD1602_ERROR_CHECK(x) do {                                     \
        esp_err_t rc = (x);                                                 \
        if (rc != ESP_OK) {                                                 \
            ESP_LOGW(TAG, "I2C error %d at %s:%d", rc, __FILE__, __LINE__); \
        }                                                                   \
    } while(0)

/**
 * @brief Initialize the I2C subsystem (bus + device).
 *
 * This function is a convenience wrapper that initializes the I2C master bus
 * and then attaches the configured device to it. It should be called once
 * during application startup before performing any I2C transactions.
 *
 * @param[out] i2c_bus Pointer to the I2C master bus handle.
 * @param[out] dev     Pointer to the I2C device handle.
 *
 * @retval ESP_OK              Both bus and device successfully initialized.
 * @retval ESP_ERR_INVALID_ARG Invalid arguments passed to bus/device init.
 * @retval ESP_FAIL            Failure in creating bus or adding device.
 *
 * @note In case of failure, the error is logged and the caller is responsible
 *       for handling cleanup (e.g. deleting bus if created).
 */
esp_err_t i2c_init(i2c_master_bus_handle_t *i2c_bus, i2c_master_dev_handle_t *dev);

/**
 * @brief Allocate a zeroed driver state structure.
 *
 * @return Pointer to a newly allocated ::i2c_lcd1602_info_t, or NULL on allocation failure.
 */
i2c_lcd1602_info_t * i2c_lcd1602_malloc(void);

/**
 * @brief Free and null a previously allocated driver state.
 *
 * @param[in,out] i2c_lcd1602_info Address of a pointer returned by ::i2c_lcd1602_malloc().
 *                                 On success, the pointer is freed and set to NULL.
 */
void i2c_lcd1602_free(i2c_lcd1602_info_t ** i2c_lcd1602_info);

/**
 * @brief Initialise a driver instance with an existing I2C device handle.
 *
 * Also performs the power-on wait and calls ::i2c_lcd1602_reset().
 *
 * @param[in,out] i2c_lcd1602_info    Driver state (allocated by ::i2c_lcd1602_malloc()).
 * @param[in]     dev_handle          I2C device handle (from i2c_master_bus_add_device()).
 * @param[in]     backlight           Initial backlight state (true = on).
 * @param[in]     num_rows            Max LCD rows (e.g., 2 or 4).
 * @param[in]     num_columns         Max LCD columns (e.g., 40).
 * @param[in]     num_visible_columns Visible columns (e.g., 16 or 20).
 * @return ESP_OK on success, ESP_FAIL/other on error.
 */
esp_err_t i2c_lcd1602_init(i2c_lcd1602_info_t * i2c_lcd1602_info,
                           i2c_master_dev_handle_t dev_handle,
                           bool backlight,
                           uint8_t num_rows,
                           uint8_t num_columns,
                           uint8_t num_visible_columns);

/**
 * @brief Re-initialise the LCD to 4-bit mode and default flags.
 *
 * Sends the HD44780 initialisation sequence and applies current flags from @ref i2c_lcd1602_info_t.
 *
 * @param[in] i2c_lcd1602_info Driver state (must be initialised).
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_reset(const i2c_lcd1602_info_t * i2c_lcd1602_info);

/**
 * @brief Clear the display and reset the cursor to (0,0).
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_clear(const i2c_lcd1602_info_t * i2c_lcd1602_info);

/**
 * @brief Return the cursor to (0,0) without clearing RAM.
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_home(const i2c_lcd1602_info_t * i2c_lcd1602_info);

/**
 * @brief Move the cursor to a given column and row.
 *
 * Columns beyond configured width and rows beyond configured height are clamped.
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @param[in] col              Column (0..num_columns-1).
 * @param[in] row              Row    (0..num_rows-1).
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_move_cursor(const i2c_lcd1602_info_t * i2c_lcd1602_info,
                                  uint8_t col, uint8_t row);

/**
 * @brief Enable/disable the backlight (if wired through PCF8574 bit 3).
 *
 * @param[in,out] i2c_lcd1602_info Driver state.
 * @param[in]     enable           true to enable; false to disable.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_set_backlight(i2c_lcd1602_info_t * i2c_lcd1602_info, bool enable);

/**
 * @brief Turn the display ON/OFF (does not affect RAM).
 *
 * @param[in,out] i2c_lcd1602_info Driver state.
 * @param[in]     enable           true = display ON, false = OFF.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_set_display(i2c_lcd1602_info_t * i2c_lcd1602_info, bool enable);

/**
 * @brief Show/hide the underline cursor.
 *
 * @param[in,out] i2c_lcd1602_info Driver state.
 * @param[in]     enable           true = cursor ON, false = OFF.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_set_cursor(i2c_lcd1602_info_t * i2c_lcd1602_info, bool enable);

/**
 * @brief Enable/disable blinking of the cursor position.
 *
 * @param[in,out] i2c_lcd1602_info Driver state.
 * @param[in]     enable           true = blink ON, false = OFF.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_set_blink(i2c_lcd1602_info_t * i2c_lcd1602_info, bool enable);

/**
 * @brief Set left-to-right text entry mode.
 *
 * @param[in,out] i2c_lcd1602_info Driver state.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_set_left_to_right(i2c_lcd1602_info_t * i2c_lcd1602_info);

/**
 * @brief Set right-to-left text entry mode.
 *
 * @param[in,out] i2c_lcd1602_info Driver state.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_set_right_to_left(i2c_lcd1602_info_t * i2c_lcd1602_info);

/**
 * @brief Enable/disable automatic display shift on character writes.
 *
 * @param[in,out] i2c_lcd1602_info Driver state.
 * @param[in]     enable           true to enable autoscroll; false to disable.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_set_auto_scroll(i2c_lcd1602_info_t * i2c_lcd1602_info, bool enable);

/**
 * @brief Shift the entire display left by one position.
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_scroll_display_left(const i2c_lcd1602_info_t * i2c_lcd1602_info);

/**
 * @brief Shift the entire display right by one position.
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_scroll_display_right(const i2c_lcd1602_info_t * i2c_lcd1602_info);

/**
 * @brief Move the cursor left by one position (without shifting display).
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_move_cursor_left(const i2c_lcd1602_info_t * i2c_lcd1602_info);

/**
 * @brief Move the cursor right by one position (without shifting display).
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_move_cursor_right(const i2c_lcd1602_info_t * i2c_lcd1602_info);

/**
 * @brief Define a custom 5x8 character in CGRAM.
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @param[in] index            Custom slot (0..7).
 * @param[in] pixelmap         8 bytes, each using lower 5 bits for columns.
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_define_char(const i2c_lcd1602_info_t * i2c_lcd1602_info,
                                  i2c_lcd1602_custom_index_t index,
                                  const uint8_t pixelmap[8]);

/**
 * @brief Write a single byte (glyph code) to DDRAM at the current cursor.
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @param[in] chr              Character code (includes custom codes 0..7).
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_write_char(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t chr);

/**
 * @brief Write a null-terminated ASCII string to the display.
 *
 * The string is written sequentially from the current cursor; wrapping behavior is defined by the LCD.
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @param[in] string           Null-terminated string (UTF-8 not supported by HD44780).
 * @return ESP_OK on success or error code.
 */
esp_err_t i2c_lcd1602_write_string(const i2c_lcd1602_info_t * i2c_lcd1602_info,
                                   const char * string);

#ifdef __cplusplus
}
#endif

#endif // I2C_LCD1602_H
