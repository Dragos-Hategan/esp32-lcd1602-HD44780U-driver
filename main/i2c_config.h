#ifndef I2C_CONFIG_H
#define I2C_CONFIG_H

// --- Hardware configuration (adjust for your board/wiring) ---
#define I2C_PORT               I2C_NUM_0
#define I2C_SDA_GPIO           23
#define I2C_SCL_GPIO           22
#define I2C_CLK_HZ             100 * 1000   // 100kHz is safe as default
#define PCF8574_I2C_ADDRESS    0x27         // Common backpacks: 0x27 or 0x3F

#endif // I2C_CONFIG_H