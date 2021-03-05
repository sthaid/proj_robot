#ifndef __UTIL_TIME_H__
#define __UTIL_TIME_H__

// This code is for bcm2711.
//
// Reference https://datasheets.raspberrypi.org/bcm2711/bcm2711-peripherals.pdf

#ifndef __KERNEL__
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#else
#include <linux/module.h>
#endif

#define PI4B_PERIPHERAL_BASE_ADDR 0xfe000000
#define TIMER_BASE_ADDR           (PI4B_PERIPHERAL_BASE_ADDR + 0x3000)
#define TIMER_BLOCK_SIZE          0x1000

volatile unsigned int *timer_regs;
unsigned int           timer_initial_value;
unsigned int           timer_last_value;

static inline int time_init(void)
{
#ifndef __KERNEL__
    int fd, rc;
    int okay;

    // verify bcm version
    rc = system("grep BCM2711 /proc/cpuinfo > /dev/null");
    okay = WIFEXITED(rc) && WEXITSTATUS(rc) == 0;
    if (!okay) {
        printf("ERROR: this program requires BCM2711\n");
        return -1;
    }

    // map timer regs
    if ((fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        printf("ERROR: can't open /dev/mem \n");
        return -1;
    }
    timer_regs = mmap(NULL,
                     TIMER_BLOCK_SIZE,
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED,
                     fd,
                     TIMER_BASE_ADDR);
    if (timer_regs == MAP_FAILED) {
        printf("ERROR: mmap failed\n");
        return -1;
    }
    close(fd);

    timer_initial_value = timer_last_value = timer_regs[1];
    return 0;
#else
    timer_regs = ioremap(TIMER_BASE_ADDR, TIMER_BLOCK_SIZE);
    timer_initial_value = timer_last_value = timer_regs[1];
    return 0;
#endif
}

static inline void time_exit(void)
{
#ifndef __KERNEL__
    // program termination will cleanup
    // XXX should unmap
#else
    iounmap(timer_regs);
#endif
}

// returns time in us since the module was loaded
static inline uint64_t time_get(void)
{
    unsigned int value;
    static uint64_t high_part;

    value = timer_regs[1];

    if (value < timer_last_value) {
        high_part += ((uint64_t)1 << 32);
    }
    timer_last_value = value;

    return high_part + value - timer_initial_value;
}

// delays for duration us, and returns the time at the end of the delay
static inline uint64_t time_delay(int duration)
{
    uint64_t start, now;

    start = time_get();
    while (1) {
        if ((now=time_get()) > start + duration) {
            break;
        }
    }

    return now;
}

#endif
