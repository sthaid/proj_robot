#define _GNU_SOURCE

#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include <signal.h>

#include "../../util/util_gpio.h"

// notes:
// - 32 bit position var overflow after 69 hours full speed:
//      980 cnt/rev * 500 rpm / 60 ~=  8000 count/sec
//      2e9 / 8000 / 3600 = 69 hours
// - encoder_tbl is from
//   https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf

// defines

#define GPIO_ENC1  16
#define GPIO_ENC2  17

#define IS_BIT_SET(n,pos) (((n) >> (pos)) & 1)

// variables

int encoder_tbl[4][4] = 
    { {  0, -1, +1,  2 },
      { +1,  0,  2, -1 },
      { -1,  2,  0, +1 },
      {  2, +1, -1,  0 } };
int poll_count, error_count, position;

// prototypes

void *encoder_thread(void *cx);
void sig_handler(int num);

// -----------------  MAIN  ------------------------------------

int main(int argc, char **argv)
{
    int rc;
    pthread_t tid;
    struct sigaction act;

    // init gpio access
    rc = gpio_init();
    if (rc < 0) {
        exit(1);
    }

    // register for SIGQUIT, which will be used to reset the position var
    memset(&act, 0, sizeof(act));
    act.sa_handler = sig_handler;
    sigaction(SIGQUIT, &act, NULL);

    // create the thread to process the encoder gpio values, and to
    // keep track of accumulated shaft position
    pthread_create(&tid, NULL,*encoder_thread, NULL);

    // print status once per second
    while (true) {
        sleep(1);
        printf("avg_poll_intvl = %d us   error_count = %d   position = %d\n", 
               1000000/poll_count, error_count, position);
        poll_count = 0;
    }

    // done
    return 0;
}

void sig_handler(int num)
{
    // reseet position count to 0 on SIGQUIT (ctrl backslash)
    position = 0;
}

// -----------------  ENCODER THREAD  --------------------------

void *encoder_thread(void *cx)
{
    int val, last_val, x, rc;
    unsigned int gpio_all;
    struct timespec ts = {0,10000};  // 10 us
    struct sched_param param;
    cpu_set_t cpu_set;

    // set affinity to cpu 2
    printf("setting affinity\n");
    CPU_ZERO(&cpu_set);
    CPU_SET(2, &cpu_set);
    rc = sched_setaffinity(0,sizeof(cpu_set_t),&cpu_set);
    if (rc < 0) {
        printf("ERROR: sched_setaffinity, %s\n", strerror(errno));
        exit(1);
    }

    // set realtime priority
    printf("setting priority\n");
    memset(&param, 0, sizeof(param));
    param.sched_priority = 99;
    rc = sched_setscheduler(0, SCHED_FIFO, &param);
    if (rc < 0) {
        printf("ERROR: sched_setscheduler, %s\n", strerror(errno));
        exit(1);
    }

    // loop, reading the encode gpio, and updating:
    // - position:    accumulated shaft position, this can be reset to 0 via ctrl+backslash
    // - error_count: number of out of sequence encoder gpio values
    // - poll_count:  number of times the gpio values have been read,
    //                this gets reset to 0 every second in main()
    gpio_all = gpio_read_all();
    last_val = (IS_BIT_SET(gpio_all,GPIO_ENC1) << 1) | IS_BIT_SET(gpio_all,GPIO_ENC2);
    while (true) {
        gpio_all = gpio_read_all();
        val = (IS_BIT_SET(gpio_all,GPIO_ENC1) << 1) | IS_BIT_SET(gpio_all,GPIO_ENC2);
        x = encoder_tbl[last_val][val];
        last_val = val;

        if (x == 2) {
            error_count++;
        } else {
            position += x;
        }

        poll_count++;

        nanosleep(&ts, NULL);
    }
}
