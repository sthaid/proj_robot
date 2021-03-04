#ifndef __UTIL_MC_H__
#define __UTIL_MC_H__

#include <pthread.h>

//
// defines: variable identifiers
//

// status flag variables
#define VAR_ERROR_STATUS                  0
#define VAR_ERRORS_OCCURRED               1
#define VAR_SERIAL_ERRORS_OCCURRED        2
#define VAR_LIMIT_STATUS                  3
#define VAR_RESET_FLAGS                   127
// diagnostic variables
#define VAR_TARGET_SPEED                  20   // SIGNED
#define VAR_CURRENT_SPEED                 21   // SIGNED
#define VAR_BRAKE_AMOUNT                  22
#define VAR_INPUT_VOLTAGE                 23   // mV
#define VAR_TEMPERATURE_A                 24   // 0.1 C
#define VAR_TEMPERATURE_B                 25   // 0.1 C
#define VAR_BAUD_RATE_REGISTER            27   // BAUD = 72000000 / BRR
#define VAR_UP_TIME_LOW                   28   // ms
#define VAR_UP_TIME_HIGH                  29   // 65536 ms
// motor limit variables
#define VAR_MAX_SPEED_FORWARD             30
#define VAR_MAX_ACCEL_FORWARD             31
#define VAR_MAX_DECEL_FORWARD             32
#define VAR_BRAKE_DUR_FORWARD             33
#define VAR_STARTING_SPEED_FORWARD        34
#define VAR_MAX_SPEED_REVERSE             36
#define VAR_MAX_ACCEL_REVERSE             37
#define VAR_MAX_DECEL_REVERSE             38
#define VAR_BRAKE_DUR_REVERSE             39
#define VAR_STARTING_SPEED_REVERSE        40
// current limittng and measurement variables
#define VAR_CURRENT_LIMIT                 42   // internal units
#define VAR_RAW_CURRENT                   43   // internal units
#define VAR_CURRENT                       44   // mA
#define VAR_CURRENT_LIMITING_CONSEC_CNT   45   // number of consecutive 10ms intvls of current limiting
#define VAR_CURRENT_LIMITTING_OCCUR_CNT   46   // number of 10 ms intvls that current limit was activated

//
// defines: motor limit identifiers
// - the update period is set in the GUI  xxx confirm this
//

#define MTRLIM_MAX_SPEED_FORWARD          4    // 0-3200
#define MTRLIM_MAX_ACCEL_FORWARD          5    // delta speed per update period
#define MTRLIM_MAX_DECEL_FORWARD          6    // delta speed per update period
#define MTRLIM_BRAKE_DUR_FORWARD          7    // 4 ms
#define MTRLIM_MAX_SPEED_REVERSE          8    // 0-3200
#define MTRLIM_MAX_ACCEL_REVERSE          9    // delta speed per update period
#define MTRLIM_MAX_DECEL_REVERSE         10    // delta speed per update period
#define MTRLIM_BRAKE_DUR_REVERSE         11    // 4 ms

//
// typedefs
//

typedef struct {
    int fd;
    char device[100];
    pthread_t monitor_thread_id;
} mc_t;

//
// prototypes
//

void mc_init_module(void);

mc_t *mc_new(int id);

int mc_enable(mc_t *mc);
int mc_status(mc_t *mc);

int mc_speed(mc_t *mc, int speed);
int mc_brake(mc_t *mc);
int mc_coast(mc_t *mc);
int mc_stop(mc_t *mc);

int mc_get_variable(mc_t *mc, int id, int *value);
int mc_set_motor_limit(mc_t *mc, int id, int value);
int mc_set_current_limit(mc_t *mc, int milli_amps);

int mc_get_fw_ver(mc_t *mc, int *product_id, int *fw_ver_maj_bcd, int *fw_ver_min_bcd);

#endif
