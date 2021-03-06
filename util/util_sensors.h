#ifndef __UTIL_SENSORS_H__
#define __UTIL_SENSORS_H__

int sensors_init(void);
void sensors_exit(void);

int seeed_hat_adc_read(int chan, double *voltage);
int mcp9808_read(double *degc);
int bme680_sensor_get_data(double *temperature, double *humidity, double *pressure, double *gas_resistance);

#endif
