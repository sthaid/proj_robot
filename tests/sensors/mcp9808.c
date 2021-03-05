// General I2C References:
// - https://learn.adafruit.com/adafruits-raspberry-pi-lesson-4-gpio-setup/configuring-i2c
// - https://www.robot-electronics.co.uk/i2c-tutorial
// - https://www.kernel.org/doc/Documentation/i2c/dev-interface
// - https://docs.huihoo.com/linux/kernel/2.6.26/kernel-api/re1210.html
//
// Configuring Raspberry Pi to use i2c:
// - sudo apt install i2c-tools
// - enable i2c using raspi-config
// - verify /dev/i2c-* exists
// - sudo usermod -aG i2c $(whoami)
// - logout and back in, so group change takes affect
// - groups           # verify you are in the i2c group
// - i2cdetect -y 1   # scan the i2c bus for your devices
//
// MCP9808
// - https://ww1.microchip.com/downloads/en/DeviceDoc/25095A.pdf
//      (Section 5.1 Registers, Page 16)
// - https://www.seeedstudio.com/Grove-I2C-High-Accuracy-Temperature-Sensor-MCP9808.html
// - https://wiki.seeedstudio.com/Grove-I2C_High_Accuracy_Temperature_Sensor-MCP9808/
// - https://github.com/Seeed-Studio/Grove_Temperature_sensor_MCP9808
//
// I2C
//   5v or 3.3v
//   0x18 to 0x1f   (default ix 0x18)

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define DEVNAME "/dev/i2c-1"
#define ADDRESS 0x18

#define REG_TEMPERATURE 5

int read_reg(int fd, int reg_ptr, unsigned int *val);

int main(int argc, char **argv)
{
    int fd, rc;
    unsigned int val;

    // open i2c device
    fd = open(DEVNAME, O_RDWR);
    if (fd < 0) {
        printf("ERROR open %s, %s\n", DEVNAME, strerror(errno));
        exit(1);
    }

    // read temperature
    rc = read_reg(fd, REG_TEMPERATURE, &val);
    if (rc < 0) {
        printf("ERROR read_reg failed\n");
        exit(1);
    }

    // print result
    bool critical_limit, upper_limit, lower_limit;
    double temp_deg_c, temp_deg_f;

    critical_limit = (val & 0x8000);
    upper_limit = (val & 0x4000);
    lower_limit = (val & 0x2000);
    temp_deg_c = (double)(val & 0xfff) / 16;
    if (val & 0x1000) {
        temp_deg_c = -temp_deg_c;
    }
    temp_deg_f = temp_deg_c * (9./5.) + 32;

    printf("RAW_VAL = 0x%04x\n", val);
    printf("LIMITS  = %s%s%s\n",
           critical_limit ? "CRITICAL " : "",
           upper_limit ? "UPPER " : "",
           lower_limit ? "UPPER " : "");
    printf("TEMP    = %0.1f C  %0.1f F\n", temp_deg_c, temp_deg_f);
    
    return 0;
}

int read_reg(int fd, int reg_ptr, unsigned int *val)
{
    unsigned char reg_ptr_msg[1] = { reg_ptr };
    unsigned char reg_val_msg[2];
    struct i2c_msg messages[] = {
        { ADDRESS, 0,        sizeof(reg_ptr_msg), reg_ptr_msg },
        { ADDRESS, I2C_M_RD, sizeof(reg_val_msg), reg_val_msg } };

    struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };

    int rc;

    rc = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (rc < 0) {
        printf("ERROR read register %d, %s\n", reg_ptr, strerror(errno));
        return -1;
    }

    *val = (reg_val_msg[0] << 8) | reg_val_msg[1];
    return 0;
}
