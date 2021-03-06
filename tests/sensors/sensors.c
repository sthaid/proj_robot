#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>

#include "../../util/util_sensors.h"

int main(int argc, char **argv)
{
    int rc, chan;
    double degc, voltage;
    double temp_degc, humidity_percent, pressure_pascal, pressure_in_hg;

    sensors_init();

    printf("SEED_HAT_ADC: ");
    for (chan = 0; chan < 8; chan++) {
        rc = seeed_hat_adc_read(chan, &voltage);
        if (rc < 0) exit(1);
        printf("%6.3f ", voltage);
    }
    printf("V\n\n");

    rc = mcp9808_read(&degc);
    if (rc < 0) exit(1);
    printf("MCP9808_TEMP = %0.1f C\n\n", degc);

    rc = bme680_sensor_get_data(&temp_degc, &humidity_percent, &pressure_pascal, NULL);
    if (rc < 0) exit(1);
    pressure_in_hg = pressure_pascal / 3386.;
    printf("BME680 TEMP=%0.1f C   HUMIDITY=%0.1f %%    PRESSURE=%0.0f Pa  %0.2f inch_Hg\n\n",
           temp_degc, humidity_percent, pressure_pascal, pressure_in_hg);

    sensors_exit();

    return 0;
}
