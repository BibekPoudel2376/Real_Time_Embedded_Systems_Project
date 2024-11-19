#include "sensor.h"
#include "mbed.h"

// Register addresses
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define OUT_X_L 0x28

// Register configurations
#define CTRL_REG1_CONFIG 0b01'11'1'1'1'1  // ODR 760Hz, Cut-off 100, Normal mode, X, Y, Z enabled
#define CTRL_REG4_CONFIG 0b0'0'10'0'00'0  // ±500 dps scale

#define SPI_FLAG 1

// Sensitivity for ±500 dps scale (from datasheet)
#define SENSITIVITY 0.017500f  // 17.50 mdps/digit

EventFlags flags;

void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

void init_spi(SPI &spi, uint8_t *write_buf, uint8_t *read_buf) {
    spi.format(8, 3);
    spi.frequency(1'000'000);

    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);
}

void read_sensor_data(SPI &spi, uint8_t *write_buf, uint8_t *read_buf, float &gx, float &gy, float &gz) {
    int16_t raw_gx, raw_gy, raw_gz;

    write_buf[0] = OUT_X_L | 0x80 | 0x40;  // Read operation, auto-increment
    spi.transfer(write_buf, 7, read_buf, 7, spi_cb);

    raw_gx = (int16_t)((read_buf[2] << 8) | read_buf[1]);
    raw_gy = (int16_t)((read_buf[4] << 8) | read_buf[3]);
    raw_gz = (int16_t)((read_buf[6] << 8) | read_buf[5]);

    gx = (float)raw_gx * SENSITIVITY;
    gy = (float)raw_gy * SENSITIVITY;
    gz = (float)raw_gz * SENSITIVITY;
}