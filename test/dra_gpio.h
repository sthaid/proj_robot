#ifndef __DRA_GPIO_H__
#define __DRA_GPIO_H__

// enable this for unit testing on non raspberry pi systems
//#define UNITTEST_USE_GPIO_STUBS

#ifndef UNITTEST_USE_GPIO_STUBS

// reference: https://elinux.org/RPi_GPIO_Code_Samples#Direct_register_access

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define PIN_MODE_INPUT   1
#define PIN_MODE_OUTPUT  2

#define PULL_UP          1
#define PULL_DOWN        2

volatile unsigned int *gpio_regs;

static inline int gpio_init(void)
{
    #define PI4B_PERIPHERAL_BASE_ADDR 0xfe000000
    #define GPIO_BASE_ADDR            (PI4B_PERIPHERAL_BASE_ADDR + 0x200000)
    #define GPIO_BLOCK_SIZE           0x1000

    int fd;

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

    close(fd);

    if (gpio_regs == MAP_FAILED) {
        printf("ERROR: mmap failed\n");
        return -1;
    }

    return 0;
}

static inline void gpio_write(int pin, int value)
{
    if (value) {
        gpio_regs[7] = (1 << pin);
    } else {
        gpio_regs[10] = (1 << pin);
    }
}

static inline int gpio_read(int pin)
{
    return (gpio_regs[13] & (1 << pin)) != 0;
}

static inline void set_gpio_pin_mode(int pin, int mode)
{
    switch (mode) {
    case PIN_MODE_INPUT:
        gpio_regs[pin/10] &= ~(7 << ((pin % 10) * 3));
        break;
    case PIN_MODE_OUTPUT:
        gpio_regs[pin/10] &= ~(7 << ((pin % 10) * 3));
        gpio_regs[pin/10] |=  (1 << ((pin % 10) * 3));
        break;
    default:
        printf("ERROR: %s: invalid mode %d\n", __func__, mode);
        exit(1);
        break;
    }
}

// this routine doesn't work yet
static inline void set_gpio_pull_mode(int pin, int pull_mode)
{
    #define GPIO_PULL      gpio_regs[37]
    #define GPIO_PULLCLK0  gpio_regs[38]

    GPIO_PULL = (pull_mode == PULL_UP ? 2 : 0);
    usleep(10000);

    GPIO_PULLCLK0 = (1 << pin);
    usleep(10000);
    GPIO_PULL = 0;
    GPIO_PULLCLK0 = 0;
}

#else // UNITTEST_USE_GPIO_STUBS

#define PIN_MODE_INPUT   1
#define PIN_MODE_OUTPUT  2
#define PULL_UP          1
#define PULL_DOWN        2

static inline int gpio_init(void) { return 0; }
static inline void gpio_write(int pin, int value) {}
static inline int gpio_read(int pin) { return 0; }
static inline void set_gpio_pin_mode(int pin, int mode) {}
static inline void set_gpio_pull_mode(int pin, int pull_mode) {}

#endif

#endif
