/*
 * MIT License
 *
 * Copyright (c) 2025 <Dragos Hategan>
 *
 * This project uses the MIT License. The original codebase and ideas were
 * created by David Antliff — see https://github.com/DavidAntliff — and this
 * version is adapted to the current ESP-IDF (esp_driver_i2c).
 */

 /**
  * @file i2c-lcd1602.c
  * @brief HD44780-compatible 16x2 LCD driver via PCF8574 I/O expander (I2C).
  *
  * Public API is documented in the corresponding header file.
  * This source file documents only the internal (static) functions.
  */

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"

#include "i2c-lcd1602.h"
#include "i2c_config.h"

#define TAG "i2c-lcd1602"

// Delays (microseconds)
#define DELAY_POWER_ON            50000
#define DELAY_INIT_1               4500
#define DELAY_INIT_2               4500
#define DELAY_INIT_3                120

#define DELAY_CLEAR_DISPLAY        2000
#define DELAY_RETURN_HOME          2000

#define DELAY_ENABLE_PULSE_WIDTH      1
#define DELAY_ENABLE_PULSE_SETTLE    50

// Commands
#define COMMAND_CLEAR_DISPLAY       0x01
#define COMMAND_RETURN_HOME         0x02
#define COMMAND_ENTRY_MODE_SET      0x04
#define COMMAND_DISPLAY_CONTROL     0x08
#define COMMAND_SHIFT               0x10
#define COMMAND_FUNCTION_SET        0x20
#define COMMAND_SET_CGRAM_ADDR      0x40
#define COMMAND_SET_DDRAM_ADDR      0x80

// COMMAND_ENTRY_MODE_SET flags
#define FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT       0x02
#define FLAG_ENTRY_MODE_SET_ENTRY_DECREMENT       0x00
#define FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON        0x01
#define FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_OFF       0x00

// COMMAND_DISPLAY_CONTROL flags
#define FLAG_DISPLAY_CONTROL_DISPLAY_ON  0x04
#define FLAG_DISPLAY_CONTROL_DISPLAY_OFF 0x00
#define FLAG_DISPLAY_CONTROL_CURSOR_ON   0x02
#define FLAG_DISPLAY_CONTROL_CURSOR_OFF  0x00
#define FLAG_DISPLAY_CONTROL_BLINK_ON    0x01
#define FLAG_DISPLAY_CONTROL_BLINK_OFF   0x00

// COMMAND_SHIFT flags
#define FLAG_SHIFT_MOVE_DISPLAY          0x08
#define FLAG_SHIFT_MOVE_CURSOR           0x00
#define FLAG_SHIFT_MOVE_LEFT             0x04
#define FLAG_SHIFT_MOVE_RIGHT            0x00

// COMMAND_FUNCTION_SET flags
#define FLAG_FUNCTION_SET_MODE_8BIT      0x10
#define FLAG_FUNCTION_SET_MODE_4BIT      0x00
#define FLAG_FUNCTION_SET_LINES_2        0x08
#define FLAG_FUNCTION_SET_LINES_1        0x00
#define FLAG_FUNCTION_SET_DOTS_5X10      0x04
#define FLAG_FUNCTION_SET_DOTS_5X8       0x00

// PCF8574 bit flags
#define FLAG_BACKLIGHT_ON    0b00001000
#define FLAG_BACKLIGHT_OFF   0b00000000
#define FLAG_ENABLE          0b00000100
#define FLAG_READ            0b00000010
#define FLAG_WRITE           0b00000000
#define FLAG_RS_DATA         0b00000001
#define FLAG_RS_COMMAND      0b00000000

#ifndef I2C_LCD1602_I2C_TIMEOUT_MS
#define I2C_LCD1602_I2C_TIMEOUT_MS 1000
#endif

/**
 * @brief Initialize the I2C master bus.
 *
 * This function creates and configures a new I2C master bus using the default
 * GPIO pins and clock source defined in the project configuration.
 * Internal pull-ups are enabled by default but can be disabled if external
 * pull-up resistors are used.
 *
 * @param[out] i2c_bus Pointer to the I2C master bus handle. On success,
 *                     this will contain the initialized bus handle.
 *
 * @retval ESP_OK              Bus successfully initialized.
 * @retval ESP_ERR_INVALID_ARG Invalid arguments.
 * @retval ESP_FAIL            Internal driver error.
 */
static esp_err_t i2c_bus_init(i2c_master_bus_handle_t *i2c_bus)
{
    // Init I2C master bus
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,                
        .flags.enable_internal_pullup = true,   // could be set false if you use external pull-ups
    };
    return i2c_new_master_bus(&bus_cfg, i2c_bus);
}

/**
 * @brief Attach an I2C device to the specified bus.
 *
 * This function registers an I2C device (e.g. PCF8574 I/O expander used for
 * an LCD backpack) to an existing I2C bus.
 *
 * @param[in]  i2c_bus I2C master bus handle previously created with
 *                     ::i2c_bus_init().
 * @param[out] dev     Pointer to the I2C device handle. On success,
 *                     this will contain the initialized device handle.
 *
 * @retval ESP_OK              Device successfully added to the bus.
 * @retval ESP_ERR_INVALID_ARG Invalid arguments.
 * @retval ESP_FAIL            Internal driver error.
 */
static esp_err_t i2c_device_init(i2c_master_bus_handle_t i2c_bus, i2c_master_dev_handle_t *dev)
{
    // Add the PCF8574 device (the LCD backpack)
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PCF8574_I2C_ADDRESS,
        .scl_speed_hz = I2C_CLK_HZ,
    };
    return i2c_master_bus_add_device(i2c_bus, &dev_cfg, dev);
}

esp_err_t i2c_init(i2c_master_bus_handle_t *i2c_bus, i2c_master_dev_handle_t *dev)
{
    esp_err_t err = i2c_bus_init(i2c_bus);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c_new_master_bus failed: %d", err); return err;}

    err = i2c_device_init(*i2c_bus, dev);
    if (err != ESP_OK) { ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %d", err);}

    return err;
}

/**
 * @brief Check if the driver info struct is initialized.
 *
 * @param[in] i2c_lcd1602_info Pointer to driver state.
 * @return true if initialized; false otherwise (and logs an error).
 */
static bool _is_init(const i2c_lcd1602_info_t * i2c_lcd1602_info)
{
    bool ok = false;
    if (i2c_lcd1602_info != NULL)
    {
        if (i2c_lcd1602_info->init)
        {
            ok = true;
        }
        else
        {
            ESP_LOGE(TAG, "i2c_lcd1602_info is not initialised");
        }
    }
    else
    {
        ESP_LOGE(TAG, "i2c_lcd1602_info is NULL");
    }
    return ok;
}

/**
 * @brief Conditionally set or clear a bit flag.
 *
 * @param[in]  flags     Current flags byte.
 * @param[in]  condition If true, \p flag is set; otherwise it is cleared.
 * @param[in]  flag      Bit mask to set/clear.
 * @return Updated flags.
 */
static uint8_t _set_or_clear(uint8_t flags, bool condition, uint8_t flag)
{
    if (condition) flags |= flag;
    else flags &= ~flag;
    return flags;
}

/**
 * @brief Write one byte to the PCF8574 with current backlight state.
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @param[in] data             Raw byte to send to the expander.
 * @return ESP_OK on success or an I2C error code.
 */
static esp_err_t _write_to_expander(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t data)
{
    uint8_t b = data | i2c_lcd1602_info->backlight_flag;
    ESP_LOGD(TAG, "_write_to_expander 0x%02x", b);
    return i2c_master_transmit(i2c_lcd1602_info->dev_handle, &b, 1, I2C_LCD1602_I2C_TIMEOUT_MS);
}

/**
 * @brief Generate an enable strobe on E pin (PCF8574 bit) for the given data nibble.
 *
 * HD44780 latches data on the falling edge of E. We write with E high, then E low,
 * with small timing gaps to satisfy the controller specs.
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @param[in] data             Byte already containing RS/RW and data nibble.
 * @return ESP_OK on success or last error.
 */
static esp_err_t _strobe_enable(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t data)
{
    esp_err_t err1 = _write_to_expander(i2c_lcd1602_info, data | FLAG_ENABLE);
    esp_err_t err2 = _write_to_expander(i2c_lcd1602_info, data & ~FLAG_ENABLE);
    esp_rom_delay_us(DELAY_ENABLE_PULSE_WIDTH);
    esp_rom_delay_us(DELAY_ENABLE_PULSE_SETTLE);
    return err1 ? err1 : err2;
}

/**
 * @brief Write the top nibble of a byte (4-bit mode).
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @param[in] data             Upper nibble in bits [7:4]; control bits included.
 * @return ESP_OK on success or last error.
 */
static esp_err_t _write_top_nibble(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t data)
{
    ESP_LOGD(TAG, "_write_top_nibble 0x%02x", data);
    esp_err_t err1 = _write_to_expander(i2c_lcd1602_info, data);
    esp_err_t err2 = _strobe_enable(i2c_lcd1602_info, data);
    return err1 ? err1 : err2;
}

/**
 * @brief Write a full byte by sending two nibbles in 4-bit mode.
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @param[in] value            8-bit value to send.
 * @param[in] register_select_flag RS bit: data or command.
 * @return ESP_OK on success or last error.
 */
static esp_err_t _write(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t value, uint8_t register_select_flag)
{
    ESP_LOGD(TAG, "_write 0x%02x | 0x%02x", value, register_select_flag);
    esp_err_t err1 = _write_top_nibble(i2c_lcd1602_info, (value & 0xf0) | register_select_flag);
    esp_err_t err2 = _write_top_nibble(i2c_lcd1602_info, ((value & 0x0f) << 4) | register_select_flag);
    return err1 ? err1 : err2;
}

/**
 * @brief Send a command byte to the controller.
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @param[in] command          Command byte.
 * @return ESP_OK on success or error code.
 */
static esp_err_t _write_command(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t command)
{
    ESP_LOGD(TAG, "_write_command 0x%02x", command);
    return _write(i2c_lcd1602_info, command, FLAG_RS_COMMAND);
}

/**
 * @brief Send a data byte to the controller (e.g., a character).
 *
 * @param[in] i2c_lcd1602_info Driver state.
 * @param[in] data             Data byte.
 * @return ESP_OK on success or error code.
 */
static esp_err_t _write_data(const i2c_lcd1602_info_t * i2c_lcd1602_info, uint8_t data)
{
    ESP_LOGD(TAG, "_write_data 0x%02x", data);
    return _write(i2c_lcd1602_info, data, FLAG_RS_DATA);
}

/* ===== Public API (documented in header) ===== */

i2c_lcd1602_info_t * i2c_lcd1602_malloc(void)
{
    i2c_lcd1602_info_t * i = (i2c_lcd1602_info_t *)malloc(sizeof(*i));
    if (i) {
        memset(i, 0, sizeof(*i));
        ESP_LOGD(TAG, "malloc i2c_lcd1602_info_t %p", i);
    } else {
        ESP_LOGE(TAG, "malloc i2c_lcd1602_info_t failed");
    }
    return i;
}

void i2c_lcd1602_free(i2c_lcd1602_info_t ** info)
{
    if (info && *info) {
        ESP_LOGD(TAG, "free i2c_lcd1602_info_t %p", *info);
        free(*info);
        *info = NULL;
    } else {
        ESP_LOGE(TAG, "free i2c_lcd1602_info_t failed");
    }
}

esp_err_t i2c_lcd1602_init(i2c_lcd1602_info_t * info, i2c_master_dev_handle_t dev_handle,
                           bool backlight, uint8_t num_rows, uint8_t num_columns, uint8_t num_visible_columns)
{
    if (!info) {
        ESP_LOGE(TAG, "i2c_lcd1602_info is NULL");
        return ESP_FAIL;
    }

    info->dev_handle = dev_handle;
    info->backlight_flag = backlight ? FLAG_BACKLIGHT_ON : FLAG_BACKLIGHT_OFF;
    info->num_rows = num_rows;
    info->num_columns = num_columns;
    info->num_visible_columns = num_visible_columns;

    info->display_control_flags = (FLAG_DISPLAY_CONTROL_DISPLAY_ON |
                                   FLAG_DISPLAY_CONTROL_CURSOR_OFF |
                                   FLAG_DISPLAY_CONTROL_BLINK_OFF);

    info->entry_mode_flags = (FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT |
                              FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_OFF);

    info->init = true;

    // Wait at least 40ms after power rises
    esp_rom_delay_us(DELAY_POWER_ON);

    return i2c_lcd1602_reset(info);
}

esp_err_t i2c_lcd1602_reset(const i2c_lcd1602_info_t * info)
{
    esp_err_t first_err = ESP_OK, last_err = ESP_OK;

    if ((last_err = _write_to_expander(info, 0)) != ESP_OK) {
        if (first_err == ESP_OK) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_to_expander 1 failed: %d", last_err);
    }

    esp_rom_delay_us(1000);

    if ((last_err = _write_top_nibble(info, 0x03 << 4)) != ESP_OK) {
        if (first_err == ESP_OK) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 1 failed: %d", last_err);
    }
    esp_rom_delay_us(DELAY_INIT_1);

    if ((last_err = _write_top_nibble(info, 0x03 << 4)) != ESP_OK) {
        if (first_err == ESP_OK) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 2 failed: %d", last_err);
    }
    esp_rom_delay_us(DELAY_INIT_2);

    if ((last_err = _write_top_nibble(info, 0x03 << 4)) != ESP_OK) {
        if (first_err == ESP_OK) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 3 failed: %d", last_err);
    }
    esp_rom_delay_us(DELAY_INIT_3);

    if ((last_err = _write_top_nibble(info, 0x02 << 4)) != ESP_OK) {
        if (first_err == ESP_OK) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_top_nibble 4 failed: %d", last_err);
    }

    if ((last_err = _write_command(info, COMMAND_FUNCTION_SET |
                  FLAG_FUNCTION_SET_MODE_4BIT |
                  FLAG_FUNCTION_SET_LINES_2 |
                  FLAG_FUNCTION_SET_DOTS_5X8)) != ESP_OK) {
        if (first_err == ESP_OK) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 1 failed: %d", last_err);
    }

    if ((last_err = _write_command(info, COMMAND_DISPLAY_CONTROL |
                  info->display_control_flags)) != ESP_OK) {
        if (first_err == ESP_OK) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 2 failed: %d", last_err);
    }

    if ((last_err = i2c_lcd1602_clear(info)) != ESP_OK) {
        if (first_err == ESP_OK) first_err = last_err;
        ESP_LOGE(TAG, "reset: i2c_lcd1602_clear failed: %d", last_err);
    }

    if ((last_err = _write_command(info, COMMAND_ENTRY_MODE_SET |
                  info->entry_mode_flags)) != ESP_OK) {
        if (first_err == ESP_OK) first_err = last_err;
        ESP_LOGE(TAG, "reset: _write_command 3 failed: %d", last_err);
    }

    if ((last_err = i2c_lcd1602_home(info)) != ESP_OK) {
        if (first_err == ESP_OK) first_err = last_err;
        ESP_LOGE(TAG, "reset: i2c_lcd1602_home failed: %d", last_err);
    }

    return first_err;
}

esp_err_t i2c_lcd1602_clear(const i2c_lcd1602_info_t * info)
{
    if (!_is_init(info)) return ESP_FAIL;
    esp_err_t err = _write_command(info, COMMAND_CLEAR_DISPLAY);
    if (err == ESP_OK) esp_rom_delay_us(DELAY_CLEAR_DISPLAY);
    return err;
}

esp_err_t i2c_lcd1602_home(const i2c_lcd1602_info_t * info)
{
    if (!_is_init(info)) return ESP_FAIL;
    esp_err_t err = _write_command(info, COMMAND_RETURN_HOME);
    if (err == ESP_OK) esp_rom_delay_us(DELAY_RETURN_HOME);
    return err;
}

esp_err_t i2c_lcd1602_move_cursor(const i2c_lcd1602_info_t * info, uint8_t col, uint8_t row)
{
    if (!_is_init(info)) return ESP_FAIL;
    const int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
    if (row >= info->num_rows) row = info->num_rows - 1;
    if (col >= info->num_columns) col = info->num_columns - 1;
    return _write_command(info, COMMAND_SET_DDRAM_ADDR | (col + row_offsets[row]));
}

esp_err_t i2c_lcd1602_set_backlight(i2c_lcd1602_info_t * info, bool enable)
{
    if (!_is_init(info)) return ESP_FAIL;
    info->backlight_flag = _set_or_clear(info->backlight_flag, enable, FLAG_BACKLIGHT_ON);
    return _write_to_expander(info, 0);
}

esp_err_t i2c_lcd1602_set_display(i2c_lcd1602_info_t * info, bool enable)
{
    if (!_is_init(info)) return ESP_FAIL;
    info->display_control_flags = _set_or_clear(info->display_control_flags, enable,
                                                FLAG_DISPLAY_CONTROL_DISPLAY_ON);
    return _write_command(info, COMMAND_DISPLAY_CONTROL | info->display_control_flags);
}

esp_err_t i2c_lcd1602_set_cursor(i2c_lcd1602_info_t * info, bool enable)
{
    if (!_is_init(info)) return ESP_FAIL;
    info->display_control_flags = _set_or_clear(info->display_control_flags, enable,
                                                FLAG_DISPLAY_CONTROL_CURSOR_ON);
    return _write_command(info, COMMAND_DISPLAY_CONTROL | info->display_control_flags);
}

esp_err_t i2c_lcd1602_set_blink(i2c_lcd1602_info_t * info, bool enable)
{
    if (!_is_init(info)) return ESP_FAIL;
    info->display_control_flags = _set_or_clear(info->display_control_flags, enable,
                                                FLAG_DISPLAY_CONTROL_BLINK_ON);
    return _write_command(info, COMMAND_DISPLAY_CONTROL | info->display_control_flags);
}

esp_err_t i2c_lcd1602_set_left_to_right(i2c_lcd1602_info_t * info)
{
    if (!_is_init(info)) return ESP_FAIL;
    info->entry_mode_flags |= FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT;
    return _write_command(info, COMMAND_ENTRY_MODE_SET | info->entry_mode_flags);
}

esp_err_t i2c_lcd1602_set_right_to_left(i2c_lcd1602_info_t * info)
{
    if (!_is_init(info)) return ESP_FAIL;
    info->entry_mode_flags &= ~FLAG_ENTRY_MODE_SET_ENTRY_INCREMENT;
    return _write_command(info, COMMAND_ENTRY_MODE_SET | info->entry_mode_flags);
}

esp_err_t i2c_lcd1602_set_auto_scroll(i2c_lcd1602_info_t * info, bool enable)
{
    if (!_is_init(info)) return ESP_FAIL;
    info->entry_mode_flags = _set_or_clear(info->entry_mode_flags, enable,
                                           FLAG_ENTRY_MODE_SET_ENTRY_SHIFT_ON);
    return _write_command(info, COMMAND_ENTRY_MODE_SET | info->entry_mode_flags);
}

esp_err_t i2c_lcd1602_scroll_display_left(const i2c_lcd1602_info_t * info)
{
    if (!_is_init(info)) return ESP_FAIL;
    return _write_command(info, COMMAND_SHIFT | FLAG_SHIFT_MOVE_DISPLAY | FLAG_SHIFT_MOVE_LEFT);
}

esp_err_t i2c_lcd1602_scroll_display_right(const i2c_lcd1602_info_t * info)
{
    if (!_is_init(info)) return ESP_FAIL;
    return _write_command(info, COMMAND_SHIFT | FLAG_SHIFT_MOVE_DISPLAY | FLAG_SHIFT_MOVE_RIGHT);
}

esp_err_t i2c_lcd1602_move_cursor_left(const i2c_lcd1602_info_t * info)
{
    if (!_is_init(info)) return ESP_FAIL;
    return _write_command(info, COMMAND_SHIFT | FLAG_SHIFT_MOVE_CURSOR | FLAG_SHIFT_MOVE_RIGHT);
}

esp_err_t i2c_lcd1602_move_cursor_right(const i2c_lcd1602_info_t * info)
{
    if (!_is_init(info)) return ESP_FAIL;
    return _write_command(info, COMMAND_SHIFT | FLAG_SHIFT_MOVE_CURSOR | FLAG_SHIFT_MOVE_LEFT);
}

esp_err_t i2c_lcd1602_define_char(const i2c_lcd1602_info_t * info,
                                  i2c_lcd1602_custom_index_t index,
                                  const uint8_t pixelmap[8])
{
    if (!_is_init(info)) return ESP_FAIL;
    index &= 0x07;
    esp_err_t err = _write_command(info, COMMAND_SET_CGRAM_ADDR | (index << 3));
    for (int i = 0; err == ESP_OK && i < 8; ++i) {
        err = _write_data(info, pixelmap[i]);
    }
    return err;
}

esp_err_t i2c_lcd1602_write_char(const i2c_lcd1602_info_t * info, uint8_t chr)
{
    if (!_is_init(info)) return ESP_FAIL;
    return _write_data(info, chr);
}

esp_err_t i2c_lcd1602_write_string(const i2c_lcd1602_info_t * info, const char * string)
{
    if (!_is_init(info)) return ESP_FAIL;
    esp_err_t err = ESP_OK;
    for (int i = 0; err == ESP_OK && string && string[i]; ++i) {
        err = _write_data(info, (uint8_t)string[i]);
    }
    return err;
}
