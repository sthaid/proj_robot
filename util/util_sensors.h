#ifndef __UTIL_SENSORS_H__
#define __UTIL_SENSORS_H__

int sensors_init(void);
void sensors_exit(void);

int stm32_seeed_hat_adc_read(int chan, double *voltage);
int mcp9808_temperature_read(double *degc);
int bme680_thpg_read(double *temperature, double *humidity, double *pressure, double *gas_resistance);
int ssd1306_oled_u8g2_drawstr(char *s);

#endif
