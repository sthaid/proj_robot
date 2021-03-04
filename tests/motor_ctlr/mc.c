#include <stdio.h>
#include <unistd.h>
#include "../../util/util_mc.h"

int main(int argc, char **argv)
{
    mc_t *mc;
    int rc;
    int error_status;
    int product_id;
    int fw_ver_maj_bcd, fw_ver_min_bcd;
    char str[200];

    mc_init_module();

    mc = mc_new(0);
    if (mc == NULL) {
        printf("ERROR: mc_new failed\n");
        return 1;
    }

    error_status = mc_status(mc);
    printf("error_status 0x%x\n", error_status);

    rc = mc_get_fw_ver(mc, &product_id, &fw_ver_maj_bcd, &fw_ver_min_bcd);
    if (rc < 0) {
        printf("ERROR: mc_get_fw_ver failed\n");
        return 1;
    }
    printf("product_id = 0x%4.4x fw_ver = %x.%x\n", product_id, fw_ver_maj_bcd, fw_ver_min_bcd);

    rc = mc_enable(mc);
    if (rc < 0) {
        printf("ERROR: mc_enable failed\n");
        return 1;
    }

    while (printf("? "), fgets(str,sizeof(str),stdin) != NULL) {
        int cnt, value;
        char cmd;

        #define VALUE_REQUIRED \
            if (cnt != 2) { \
                printf("ERROR: value required\n"); \
                continue; \
            }

        cnt = sscanf(str, "%c %d", &cmd, &value);
        if (cnt == 0) {
            continue;
        }

        switch (cmd) {
        case 'f':
            VALUE_REQUIRED;
            mc_speed(mc, value);
            break;
        case 'r':
            VALUE_REQUIRED;
            mc_speed(mc, -value);
            break;
        case 'x': {
            int current_speed, target_speed;

            target_speed = -3200;
            mc_speed(mc, target_speed);
            do {
                rc = mc_get_variable(mc, VAR_CURRENT_SPEED, &current_speed);
                if (rc < 0) {
                    printf("ERROR: get current speed failed\n");
                    return 1;
                }
                printf("current_speed = %d\n", current_speed);
                usleep(10000);
            } while (current_speed != target_speed);
            sleep(5);

            target_speed = +3200;
            mc_speed(mc, target_speed);
            do {
                rc = mc_get_variable(mc, VAR_CURRENT_SPEED, &current_speed);
                if (rc < 0) {
                    printf("ERROR:  current speed failed\n");
                    return 1;
                }
                printf("current_speed = %d\n", current_speed);
                usleep(10000);
            } while (current_speed != target_speed);
            sleep(5);

            target_speed = 0;
            mc_speed(mc, target_speed);
            do {
                rc = mc_get_variable(mc, VAR_CURRENT_SPEED, &current_speed);
                if (rc < 0) {
                    printf("ERROR: get current speed failed\n");
                    return 1;
                }
                printf("current_speed = %d\n", current_speed);
                usleep(10000);
            } while (current_speed != target_speed);

            break; }
        default:
            printf("ERROR: cmd '%c' invalid\n", cmd);
            break;
        }
    }

    return 0;
}
