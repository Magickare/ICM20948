# ICM20948
The ICM-20948 is a 9-axis motion tracking sensor featuring a 3-axis gyroscope, accelerometer, and magnetometer. To interface it with a Raspberry Pi 3B+ via either SPI or I2C protocols.

#ICM20948  To interface it with a Raspberry Pi 3B+ via SPI
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>

#define SPI_CHANNEL 0
#define SPI_SPEED   1000000  // 1 MHz

// ICM20948 Registers
#define ICM20948_WHO_AM_I       0x00
#define ICM20948_WHO_AM_I_RESP  0xEA
#define ICM20948_USER_CTRL      0x03
#define ICM20948_PWR_MGMT_1     0x06
#define ICM20948_ACCEL_XOUT_H   0x2D
#define ICM20948_GYRO_XOUT_H    0x33
#define ICM20948_MAG_XOUT_L 0x11 //ally different, assuming AK09916

// SPI Write Function
void spi_write_byte(unsigned char reg, unsigned char data) {
    unsigned char buf[2] = { reg & 0x7F, data };
    wiringPiSPIDataRW(SPI_CHANNEL, buf, 2);
}

// SPI Read Function
unsigned char spi_read_byte(unsigned char reg) {
    unsigned char buf[2] = { reg | 0x80, 0x00 };
    wiringPiSPIDataRW(SPI_CHANNEL, buf, 2);
    return buf[1];
}

// SPI Read Multiple Bytes Function
void spi_read_bytes(unsigned char reg, unsigned char *buf, int len) {
    buf[0] = reg | 0x80;
    wiringPiSPIDataRW(SPI_CHANNEL, buf, len + 1);
}

// Initialization Function
void icm20948_init() {
    spi_write_byte(ICM20948_PWR_MGMT_1, 0x01);  // Wake up the sensor
    delay(100);
}

// Read Accelerometer Data
void read_accel_data(int *ax, int *ay, int *az) {
    unsigned char buf[7];
    spi_read_bytes(ICM20948_ACCEL_XOUT_H, buf, 6);
    *ax = (buf[1] << 8) | buf[2];
    *ay = (buf[3] << 8) | buf[4];
    *az = (buf[5] << 8) | buf[6];
}

// Read Gyroscope Data
void read_gyro_data(int *gx, int *gy, int *gz) {
    unsigned char buf[7];
    spi_read_bytes(ICM20948_GYRO_XOUT_H, buf, 6);
    *gx = (buf[1] << 8) | buf[2];
    *gy = (buf[3] << 8) | buf[4];
    *gz = (buf[5] << 8) | buf[6];
}

// Read Magnetometer Data (Assuming AK09916 connected via I2C on ICM20948)
void read_mag_data(int *mx, int *my, int *mz) {
    unsigned char buf[8];
    spi_read_bytes(ICM20948_MAG_XOUT_L, buf, 6);
    *mx = (buf[1] << 8) | buf[0];
    *my = (buf[3] << 8) | buf[2];
    *mz = (buf[5] << 8) | buf[4];
}

int main() {
    // Initialize WiringPi
    if (wiringPiSetup() == -1) {
        printf("WiringPi setup failed.\n");
        return -1;
    }

    // Initialize SPI
    if (wiringPiSPISetup(SPI_CHANNEL, SPI_SPEED) == -1) {
        printf("SPI setup failed.\n");
        return -1;
    }

    // Initialize ICM20948
    icm20948_init();

    // Check WHO_AM_I register
    unsigned char who_am_i = spi_read_byte(ICM20948_WHO_AM_I);
    if (who_am_i == ICM20948_WHO_AM_I_RESP) {
        printf("ICM20948 detected successfully.\n");
    } else {
        printf("ICM20948 detection failed.\n");
        return -1;
    }

    // Continuous reading loop
    int ax, ay, az, gx, gy, gz, mx, my, mz;
    while (1) {
        read_accel_data(&ax, &ay, &az);
        read_gyro_data(&gx, &gy, &gz);
        read_mag_data(&mx, &my, &mz);

        printf("Accel: X=%d Y=%d Z=%d\n", ax, ay, az);
        printf("Gyro:  X=%d Y=%d Z=%d\n", gx, gy, gz);


        delay(500);  // Delay for 500 milliseconds
    }

    return 0;
}
