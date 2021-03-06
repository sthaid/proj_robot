#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

#include "../../util/util_gpio.h"
#include "../../util/util_relay.h"

int main(int argc, char **argv)
{
    int rc;

    rc = gpio_init();
    if (rc < 0) return 1;
    rc = relay_init();
    if (rc < 0) return 1;

    while (true) {
        printf("enabling relay\n");
        relay_ctrl(RELAY_TEST, true);
        sleep(1);

        printf("disabling relay\n");
        relay_ctrl(RELAY_TEST, false);
        sleep(1);
    }

    return 0;
}
