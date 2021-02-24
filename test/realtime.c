// XXX tbd
// - encoder gpio signals toggle 100 us (check this)

// ps -eLo comm,rtprio
// ps -eLo pid,comm,rtprio,sched


#define _GNU_SOURCE 

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <pthread.h>
#include <sys/mman.h>

#include "dra_gpio.h"

// defines

#define MAX_HISTOGRAM 500000

// variables

uint64_t histogram[MAX_HISTOGRAM];
uint64_t overflow_cnt;
uint64_t max_duration_us;
bool     done;
int rdcnt;

// prototypes

void *thread(void *cx);
void timer_init(void);
uint64_t timer_read(void);


// --------------------------------------------------------------------------

int main(int argc, char **argv)
{
    pthread_t tid;
    int i, rc;
    int cnt=0;

    setlinebuf(stdout);

    // init timer
    timer_init();

    gpio_init();
    printf("DID gpio init\n");

#if 0
    // XXX test
    while (true) {
        printf("%lld\n", timer_read());
        sleep(1);
    }
#endif

#if 0
    struct sched_param param;
    memset(&param, 0, sizeof(param));
    param.sched_priority = 95;
    rc = sched_setscheduler(26, SCHED_FIFO, &param);
    if (rc < 0) {
        printf("ERROR XXX %s\n", strerror(errno));
    }
#endif

    // create thread
    pthread_create(&tid, NULL, thread, NULL);

    // monitor results published by the thread
    while (true) {
        sleep(5);
        //printf("max_duration_us = %lld\n", max_duration_us);
        printf("overflow_cnt = %lld  max_duration_us=%lld rdcnt=%d\n",
                overflow_cnt, max_duration_us, rdcnt);
        for (i = 0; i < MAX_HISTOGRAM; i++) {
            if (histogram[i]) {
                printf("%d - %lld\n", i, histogram[i]);
            }
        }
        printf("**************************************\n");

#if 0
        cnt++;
        if (cnt > 10) {
            done = 1;
            break;
        }
#endif
    }

    return 0;
}

void *thread(void *cx)
{
    struct timespec ts;
    cpu_set_t cpu_set;
    int rc;
    uint64_t time_now_us, time_last_us, duration_us;
    struct sched_param param;
    uint64_t cnt=0;

    // set affinity to cpu 3
    CPU_ZERO(&cpu_set);
    CPU_SET(3, &cpu_set);
    rc = sched_setaffinity(0,sizeof(cpu_set_t),&cpu_set);
    if (rc < 0) {
        printf("ERROR: sched_setaffinity, %s\n", strerror(errno));
        exit(1);
    }

#if 1
    // set realtime priority
    // XXX tbd, check for raspberry pi realtime threads
    memset(&param, 0, sizeof(param));
    param.sched_priority = 99;
    sched_setscheduler(0, SCHED_FIFO, &param);
#endif

    // loop
    time_last_us = timer_read();
    while (!done) {
#if 1
        // short sleep
        ts.tv_sec = 0;
        ts.tv_nsec = 10000;   // 10 usec
        nanosleep(&ts, NULL);
#endif

        // do some work
        // XXX tbd
        rdcnt += gpio_read(1);
        rdcnt += gpio_read(2);
        rdcnt += gpio_read(3);
        rdcnt += gpio_read(4);
        rdcnt += gpio_read(1);
        rdcnt += gpio_read(2);
        rdcnt += gpio_read(3);
        rdcnt += gpio_read(4);

        // read timer and determine the duration of the above sleep + work 
        time_now_us = timer_read();

        // store duration in histogram
        duration_us = time_now_us - time_last_us;
        if (duration_us < 0) {
            printf("ERROR: duration_us=%lld is < 0\n", duration_us);
            exit(1);
        }

        cnt++;
        if (cnt > 10000) {
            if (duration_us > max_duration_us) {
                max_duration_us = duration_us;
            }
            if (duration_us < MAX_HISTOGRAM) {
                histogram[duration_us]++;
            } else {
                overflow_cnt++;
            }
        }
        time_last_us = time_now_us;

        //__sync_synchronize();
    }

    return NULL;
}

// -----------------  TIMER  ------------------------------------

unsigned int *timer_regs;
unsigned int initial_value;
unsigned int last_value;

void timer_init(void)
{
    int fd;

    #define TIMER_BASE_ADDR (0xfe000000 + 0x3000)

    if ((fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        printf("ERROR: can't open /dev/mem, %s\n", strerror(errno));
        exit(1);
    }

    timer_regs = mmap(NULL,
                      4096,
                      PROT_READ,
                      MAP_SHARED,
                      fd,
                      TIMER_BASE_ADDR);

    close(fd);

    if (timer_regs == MAP_FAILED) {
        printf("ERROR: mmap failed\n");
        exit(1);
    }

    initial_value = last_value = timer_regs[1];
}

uint64_t timer_read(void)
{
    unsigned int value;
    static uint64_t high_part;

    value = timer_regs[1];

    if (value < last_value) {
        high_part += ((uint64_t)1 << 32);
    }
    last_value = value;

    return high_part + value - initial_value;
}
