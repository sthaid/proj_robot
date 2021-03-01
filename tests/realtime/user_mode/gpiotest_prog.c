// NOTES:
//   ps -eLo comm,rtprio
//   ps -eLo pid,comm,rtprio,sched

#define _GNU_SOURCE

#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>

#include <pthread.h>
#include <sched.h>
#include <time.h>

#include "../../../util/util_gpio.h"

void *test_thread(void *cx);

int main(int argc, char **argv)
{
    pthread_t tid;

    // init
    setlinebuf(stdout);
    gpio_init();
    set_gpio_func(26, FUNC_OUT);

    // create test thread
    pthread_create(&tid, NULL, test_thread, NULL);

    // pause
    pause();

    // done
    return 0;
}

// This test is trying to create a square wave with a 2 us period.
// However, the result is a square wave with 18 us period, and some jitter 
//  observed on oscilloscope.
void *test_thread(void *cx)
{
    struct timespec ts;
    struct sched_param param;
    cpu_set_t cpu_set;
    int rc;

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

    // loop, attempting to create a square wave with 2 us period
    printf("create square wave loop\n");
    ts.tv_sec = 0;
    ts.tv_nsec = 1000;
    while (true) {
        // set gpio 26, and 1 usec sleep
        gpio_write(26, 1);
        nanosleep(&ts, NULL);

        // clear gpio 26, and 1 usec sleep
        gpio_write(26, 0);
        nanosleep(&ts, NULL);
    }

    return NULL;
}
