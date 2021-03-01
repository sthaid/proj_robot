#ifndef __UTIL_GPIO_H__
#define __UTIL_GPIO_H__

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
#define GPIO_BASE_ADDR            (PI4B_PERIPHERAL_BASE_ADDR + 0x200000)
#define GPIO_BLOCK_SIZE           0x1000

#define PULL_NONE  0
#define PULL_UP    1
#define PULL_DOWN  2

#define FUNC_IN    0
#define FUNC_OUT   1

static volatile unsigned int *gpio_regs;

// -----------------  GPIO: INIT & EXIT  ------------------

static inline int gpio_init(void)
{
#ifndef __KERNEL__
    int fd, rc;
    bool okay;

    // verify bcm version
    rc = system("grep BCM2711 /proc/cpuinfo > /dev/null");
    okay = WIFEXITED(rc) && WEXITSTATUS(rc) == 0;
    if (!okay) {
        printf("ERROR: this program requires BCM2711\n");
        exit(1);
    }

    // map gpio regs
    if ((fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        printf("ERROR: can't open /dev/mem \n");
        return -1;
    }
    gpio_regs = mmap(NULL,
                     GPIO_BLOCK_SIZE,
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED,
                     fd,
                     GPIO_BASE_ADDR);
    if (gpio_regs == MAP_FAILED) {
        printf("ERROR: mmap failed\n");
        return -1;
    }
    close(fd);

    return 0;
#else
    gpio_regs = ioremap(GPIO_BASE_ADDR, 0x1000);
    return 0;
#endif
}

static inline void gpio_exit(void)
{
#ifndef __KERNEL__
    // program termination will cleanup
#else
    iounmap(gpio_regs);
#endif
}

// -----------------  GPIO: READ & WRITE  -----------------

static inline int gpio_read(int pin)
{
    return (gpio_regs[13] & (1 << pin)) != 0;
}

static inline void gpio_write(int pin, int value)
{
    if (value) {
        gpio_regs[7] = (1 << pin);
    } else {
        gpio_regs[10] = (1 << pin);
    }
}

// -----------------  GPIO: CONFIGURATION  ----------------

static inline void set_gpio_func(int pin, int func)
{
    int regidx, bit;
    unsigned int tmp;

    regidx = 0 + (pin / 10);
    bit = (pin % 10) * 3;

    tmp = gpio_regs[regidx];
    tmp &= ~(7 << bit);  // 3 bit field
    tmp |= (func << bit);

    gpio_regs[regidx] = tmp;
}

static inline void set_gpio_pull(int pin, int pull)
{
    int regidx, bit;
    unsigned int tmp;

    regidx = 57 + (pin / 16);
    bit = (pin % 16) * 2;

    tmp = gpio_regs[regidx];
    tmp &= ~(3 << bit);   // 2 bit field
    tmp |= (pull << bit);

    gpio_regs[regidx] = tmp;
}

#endif
