#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <time.h>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

#include "util_sensors.h"
#include "util_misc.h"

#include "bme680/bme680.h"  // XXX put in subdir
#include "u8g2/u8g2.h"

#define DEVNAME "/dev/i2c-1"

static int bme680_thpg_init(void);
static int ssd1306_oled_u8g2_init(void);

static int i2c_init(void);
static int8_t i2c_read(uint8_t addr, uint8_t reg_addr, uint8_t * reg_data, uint16_t len);
static int8_t i2c_write(uint8_t addr, uint8_t reg_addr, uint8_t * reg_data, uint16_t len);

// -----------------  INIT  -----------------------------

int sensors_init(void)
{
    int rc;

    rc = i2c_init();
    if (rc) {
        ERROR("i2c_init failed\n");
        return -1;
    }

    rc = bme680_thpg_init();
    if (rc) {
        ERROR("bme680_thpg_init failed\n");
        return -1;
    }

    rc = ssd1306_oled_u8g2_init();
    if (rc) {
        ERROR("ssd1306_oled_u8g2_init failed\n");
        return -1;
    }

    return 0;
}

void sensors_exit(void)
{
    // xxx all should have exit
}

// -----------------  SEEED HAT ADC  --------------------

// range 0 - 3.3 V

#define SEED_HAT_ADC_ADDR        0x04
#define SEED_HAT_ADC_REG_VOLTAGE 0x20

int stm32_seeed_hat_adc_read(int chan, double *voltage)
{
    uint8_t data[2];
    int rc;

    if (chan < 0 || chan > 8) {
        ERROR("chan %d invalid\n", chan);
        return -1;
    }

    rc = i2c_read(SEED_HAT_ADC_ADDR, SEED_HAT_ADC_REG_VOLTAGE+chan, data, 2);
    if (rc < 0) {
        ERROR("i2c_read, %s\n", strerror(errno));
        *voltage = 0;
        return -1;
    }

    *voltage = ((data[1] << 8) | data[0]) / 1000.;
    return 0;
}

// -----------------  MCP9808 TEMPERATURE SENSOR  -------

#define MCP9808_ADDR            0x18
#define MCP9808_REG_TEMPERATURE 5

#define MCP9808_CRIT_LIMIT      0x8000
#define MCP9808_UPPER_LIMIT     0x4000
#define MCP9808_LOWER_LIMIT     0x2000
#define MCP9808_SIGN            0x1000

int mcp9808_temperature_read(double *degc)
{
    int rc;
    uint8_t data[2];
    uint32_t val;

    rc = i2c_read(MCP9808_ADDR, MCP9808_REG_TEMPERATURE, data, 2);
    if (rc < 0) {
        ERROR("i2c_read, %s\n", strerror(errno));
        *degc = 0;
        return -1;
    }

    val = (data[0] << 8) | data[1];
    *degc = (val & 0xfff) / 16.;
    if (val & MCP9808_SIGN) {
        *degc = -(*degc);
    }

    return 0;
}

// -----------------  BME680 TEMP/PRESSUE/HUMIDITY  -----

#define BME680_ADDR  0x76

static struct bme680_dev dev;

static void delay_ms(uint32_t ms)
{
    // XXX have a common sleep routine
    // XXX signals
    usleep(1000*ms);
}

static int bme680_thpg_init(void)
{
    int rc;

    // init the bme680.cpp code, and provide the i2c access and ms delay routines
    dev.dev_id   = BME680_ADDR;
    dev.intf     = BME680_I2C_INTF;
    dev.read     = i2c_read;
    dev.write    = i2c_write;
    dev.delay_ms = delay_ms;
    rc = bme680_init(&dev);
    if (rc != 0) {
        ERROR("bme680_init rc=%d\n", rc);
        return -1;
    }

    // reset bme680
    rc = bme680_soft_reset(&dev);
    if (rc != 0) {
        ERROR("bme680_soft_reset rc=%d\n", rc);
        return -1;
    }

    // apply bme680 settings
    // - the oversampling and filter settings are copied from
    //   grove_temperature_humidity_bme680.py
    // - gas measurement is disabled; the grove_temperature_humidity_bme680.py
    //   has gas measurement working, but I was not able to get a stable reading
    dev.tph_sett.os_hum  = BME680_OS_2X;
    dev.tph_sett.os_pres = BME680_OS_4X;
    dev.tph_sett.os_temp = BME680_OS_8X;
    dev.tph_sett.filter  = BME680_FILTER_SIZE_3;
#if 1
    dev.gas_sett.run_gas = BME680_DISABLE_GAS_MEAS;
#else
    dev.gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    dev.gas_sett.heatr_dur  = 100;
    dev.gas_sett.heatr_temp = 300;
#endif
    rc = bme680_set_sensor_settings(
            (BME680_OST_SEL | BME680_OSH_SEL | BME680_OSP_SEL |
             BME680_FILTER_SEL | BME680_GAS_SENSOR_SEL),
            &dev);
    if (rc != 0) {
        ERROR("bme680_set_sensor_settings rc=%d\n", rc);
        return -1;
    }

#if 0
    // print the profile duration, it should be short (43 ms) when gas measure is disabled
    uint16_t ms;
    bme680_get_profile_dur(&ms, &dev);
    INFO("profile duration %d ms\n", ms);
#endif

    return 0;
}

int bme680_thpg_read(double *temperature, double *humidity, double *pressure, double *gas_resistance)
{
    int rc;
    struct bme680_field_data sensor_data;

    // request bme680 start to obtain the temperature, humidity and pressure;
    // note - the gas_resistance is not currently working, just pass in NULL for gas_resistance
    dev.power_mode = BME680_FORCED_MODE;
    rc = bme680_set_sensor_mode(&dev);
    if (rc != 0) {
        ERROR("ERROR bme680_set_sensor_mode rc=%d\n", rc);
        return -1;
    }

    // retrieve the sensor_data from the be680
    memset(&sensor_data,0,sizeof(sensor_data));
    rc = bme680_get_sensor_data(&sensor_data, &dev);
    if (rc != 0) {
        ERROR("bme680_get_sensor_data rc=%d\n", rc);
        return -1;
    }
    
    // return the data to caller
    if (temperature) {
        *temperature = sensor_data.temperature / 100.;
    }
    if (humidity) {
        *humidity = sensor_data.humidity / 1000.;
    }
    if (pressure) {
        *pressure = sensor_data.pressure;
    }
    if (gas_resistance) {
        *gas_resistance = ((sensor_data.status & BME680_HEAT_STAB_MSK) 
                           ? sensor_data.gas_resistance : 0);
    }

    // success
    return 0;
}

// -----------------  SSD1306 OLED DISPLAY  -------------

#define SSD1306_ADDR  0x3c

static uint8_t u8x8_byte_linux_i2c(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);
static uint8_t u8x8_linux_i2c_delay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr);

static u8g2_t u8g2;

static int ssd1306_oled_u8g2_init(void)
{
    u8g2_Setup_ssd1306_i2c_128x32_univision_f(
        &u8g2, 
        U8G2_R0, 
        u8x8_byte_linux_i2c, 
        u8x8_linux_i2c_delay);

    u8g2_SetI2CAddress(&u8g2, SSD1306_ADDR);

    u8g2_InitDisplay(&u8g2);

    u8g2_SetPowerSave(&u8g2, 0);

    u8g2_ClearBuffer(&u8g2);

    u8g2_SetFont(&u8g2, u8g2_font_logisoso32_tf);

    u8g2_SetFontRefHeightText(&u8g2);  // XXX what is this

    u8g2_SetFontPosTop(&u8g2);

    u8g2_DrawStr(&u8g2, 0, 0, "  ---  ");

    u8g2_SendBuffer(&u8g2);

    return 0;
}

int ssd1306_oled_u8g2_drawstr(char *s)
{
    u8g2_ClearBuffer(&u8g2);
    u8g2_DrawStr(&u8g2, 0, 0, s);
    u8g2_SendBuffer(&u8g2);

    return 0;
}

// return 1 for success, 0 for failure
static uint8_t u8x8_byte_linux_i2c(u8x8_t * u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    static int fd;
    static uint8_t data[32];
    static int idx;

    switch (msg) {
    case U8X8_MSG_BYTE_SEND:
        for (int i = 0; i < arg_int && idx < sizeof(data); i++, idx++) {
            data[idx] = *(uint8_t *) (arg_ptr + i);
        }
        break;
    case U8X8_MSG_BYTE_INIT:
        // one-time setup
        fd = open(DEVNAME, O_RDWR);
        if (fd < 0) {
            ERROR("can't open %s, %s\n", DEVNAME, strerror(errno));
            return 0;
        }
        if (ioctl(fd, I2C_SLAVE, SSD1306_ADDR) < 0) { // u8x8_GetI2CAddress(u8x8)
            ERROR("can't set addr %0x, %s\n", SSD1306_ADDR, strerror(errno));
            return 0;
        }
        break;
    case U8X8_MSG_BYTE_SET_DC:
        // ignored for i2c 
        break;
    case U8X8_MSG_BYTE_START_TRANSFER:
        memset(data, 0, sizeof(data));
        idx = 0;
        break;
    case U8X8_MSG_BYTE_END_TRANSFER:
        if (i2c_smbus_write_i2c_block_data(fd, data[0], idx - 1, &data[1]) < 0) {
            ERROR("can't write cmd %0x: %s\n", data[0], strerror(errno));
            return 0;
        }
        break;
    default:
        ERROR("unknown msg type %d\n", msg);
        return 0;
    }

    return 1;
}

// return 1 for success, 0 for failure
static uint8_t u8x8_linux_i2c_delay(u8x8_t * u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr)
{
    struct timespec req;
    struct timespec rem;
    int ret;

    req.tv_sec = 0;

    switch (msg) {
    case U8X8_MSG_DELAY_NANO:    // delay arg_int * 1 nano second
        req.tv_nsec = arg_int;
        break;
    case U8X8_MSG_DELAY_100NANO: // delay arg_int * 100 nano seconds
        req.tv_nsec = arg_int * 100;
        break;
    case U8X8_MSG_DELAY_10MICRO: // delay arg_int * 10 micro seconds
        req.tv_nsec = arg_int * 10000;
        break;
    case U8X8_MSG_DELAY_MILLI:   // delay arg_int * 1 milli second
        req.tv_nsec = arg_int * 1000000;
        break;
    default:
        return 0;
    }

    while ((ret = nanosleep(&req, &rem)) && errno == EINTR) {
        req = rem;
    }
    if (ret) {
        ERROR("nanosleep, %s\n", strerror(errno));
        return 0;
    }

    return 1;
}

// -----------------  I2C ACCESS  -----------------------

static int fd;

static int i2c_init(void)
{
    // don't allow multiple calls to this routine
    if (fd != 0) {
        ERROR("already initialized\n");
        return -1;
    }

    // open i2c device
    fd = open(DEVNAME, O_RDWR);
    if (fd < 0) {
        ERROR("open %s, %s\n", DEVNAME, strerror(errno));
        return -1;
    }

    return 0;
}

static int8_t i2c_read(uint8_t addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    struct i2c_msg messages[] = {
        { addr, 0,        1,   &reg_addr },
        { addr, I2C_M_RD, len, reg_data  } };
    struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };
    int rc;

    if (fd == 0) {
        ERROR("not initialized\n");
        return -1;
    }

    rc = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (rc < 0) {
        ERROR("ioctl I2C_RDWR, %s\n", strerror(errno));
        return -1;
    }

    return 0;
}

// note: only tested with BME680 so far
static int8_t i2c_write(uint8_t addr, uint8_t reg_addr, uint8_t * reg_data, uint16_t len)
{
    uint8_t tmp[100];
    struct i2c_msg messages[] = {
        { addr, 0, len+1, tmp  } };
    struct i2c_rdwr_ioctl_data ioctl_data = { messages, 1 };
    int rc;

    if (fd == 0) {
        ERROR("not initialized\n");
        return -1;
    }

    if (len+1 > sizeof(tmp)) {
        ERROR("len %d too large\n", len);
        return -1;
    }

    tmp[0] = reg_addr;
    memcpy(tmp+1, reg_data, len);

    rc = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (rc < 0) {
        ERROR("ioctl I2C_RDWR, %s\n", strerror(errno));
        return -1;
    }

    return 0;
}

