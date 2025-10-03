# i2c-lcd1602 (ESP-IDF, I²C, HD44780 via PCF8574)
## Why this repo?
A minimal, production-oriented driver for HD44780-compatible LCDs (16×2/20×4) over I²C using a PCF8574/PCF8574A expander and the modern ESP-IDF esp_driver_i2c API. Fork-level rewrite from David Antliff’s driver to match current ESP-IDF (handles, bus/device objects, timing).

## Features

- 4-bit mode over PCF8574(A) (backlight control supported)
- Clear/home, cursor on/off, blink, left↔right entry, autoscroll
- Cursor/display shift, absolute cursor positioning
- Write char / C-string
- Define up to 8 custom 5×8 glyphs (CGRAM)
- Works with LCDs up to 2 or 4 rows, 16–40 columns
- Clean init/reset sequence with spec-compliant delays
- Small surface: i2c_lcd1602_* API only, no globals

## Hardware

- MCU: ESP32 family (ESP-IDF v5+)
- I²C bus: any pins supported by your SoC (open-drain, pull-ups required)
- Expander: PCF8574 or PCF8574A breakout (typical I²C addr 0x27 or 0x3F)
- LCD: HD44780-compatible (e.g., 16×2, 20×4)
- Note: Backpack wiring is commonly P7..P0 = D7..D4, BL, E, RW, RS but variants exist. This driver assumes backlight on P3; adjust hardware if your board differs.

## Configuration
- I²C speed: 100 kHz is safest. 400 kHz usually works with short wires and solid pull-ups.
- Pull-ups: 4.7–10 kΩ to 3.3 V on SDA/SCL (prefer external; internal pull-ups are weak).
- Addresses: PCF8574 (0x20–0x27), PCF8574A (0x38–0x3F). Most backpacks use 0x27 or 0x3F.
- Rows/columns: Pass actual LCD geometry: e.g., num_rows=2, num_columns=40, num_visible_columns=16.
- Row DDRAM offsets: {0x00, 0x40, 0x14, 0x54} are used for 4-line addressing.

## Notes & tips
- Delays: Clear/Home require ~1.52–1.64 ms; driver uses 2 ms guard. Avoid tight loops of clear/home.
- UTF-8: Not supported by HD44780. Send 8-bit glyph codes. Roman/Greek symbols depend on LCD ROM (A00 vs A02).
- Backlight polarity: This driver assumes P3 high = backlight on (common on backpacks). If yours is inverted, consider hardware fix.
- Interference: Long wires and noisy loads can corrupt I²C. Keep LCD cable short; verify pull-ups and ground.
- Error handling: Use I2C_LCD1602_ERROR_CHECK(x) to log ESP-IDF errors without aborting.

## Acknowledgments
- David Antliff — original HD44780/PCF8574 driver design and public discussion: https://github.com/DavidAntliff/esp32-i2c-lcd1602
- Espressif — ESP-IDF and esp_driver_i2c: https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html
