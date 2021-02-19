// public api
#define VAR_ERROR_STATUS  0  //xxx check
typedef struct {
    int fd;
    pthread_t monitor_thread_id;
} mc_t;

mc_t *mc_new(int id);

int mc_enable(mc_t *mc);
int mc_status(mc_t *mc, int *error_status);

int mc_speed(mc_t *mc);
int mc_brake(mc_t *mc);
int mc_stop(mc_t *mc);

int mc_get_variable(mc_t *mc, int id, int *value);
int mc_set_motor_limit(mc_t *mc, int id, int value);
int mc_set_current_limit(mc_t *mc, int milli_amps);

int mc_get_fw_ver(mc_t *mc, int *product_id, int *fw_version);


