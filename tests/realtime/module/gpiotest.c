#include <linux/module.h>
#include <linux/io.h>
#include <linux/delay.h>

#include "../../../util/util_gpio.h"

//
// defines
//

//
// typedefs
//

//
// variables
//

static volatile bool gpiotest_exit_request;
static bool          gpiotest_exitted;

//
// prototypes
//

// the new kernel API
extern int run_offline_cpu(int cpu, void (*proc)(void));

int gpiotest_init(void);
void gpiotest_exit(void);

static void gpiotest_proc(void);

static void time_init(void);
static void time_exit(void);
static uint64_t time_get(void);
static uint64_t time_delay(int duration);

//
// module info
//
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Steven Haid");
MODULE_VERSION("1.0.0");

module_init(gpiotest_init);
module_exit(gpiotest_exit);

// -----------------  MODULE INIT & EXIT  -------------------------------------

int gpiotest_init(void)
{
    int rc;

    // init
    time_init();
    gpio_init();
    set_gpio_func(26, FUNC_OUT);

    // run gpiotest_proc on cpu 3
    rc = run_offline_cpu(3, gpiotest_proc);
    if (rc != 0) {
        printk("ERROR run_offline_cpu rc=%d\n", rc);
        time_exit();
        gpio_exit();
        return rc;
    }

    // return success
    printk("gpiotest_init complete\n");
    return 0;
}

void gpiotest_exit(void)
{
    // cause gpiotest_proc to return
    gpiotest_exit_request = true;
    while (gpiotest_exitted == false) {
        msleep(100);
    }
    msleep(100);

    // exit  
    time_exit();
    gpio_exit();

    // done
    printk("gpiotest_exit complete\n");
}

// -----------------  GPIOTEST PROC & SUPPORT ROUTINES -----------------------

// test:   square wave with 2 us period
// result: good square wave, with some jitter
static void gpiotest_proc(void)
{
    while (true) {
        // set gpio 26 and delay until the next hardware timer microsec
        gpio_write(26, 1);
        time_delay(0);

        // clear gpio 26 and delay until the next hardware timer microsec
        gpio_write(26, 0);
        time_delay(0);

        // check if time to exit
        if (gpiotest_exit_request) {
            break;
        }
    }

    // ack that this routine is exitting
    gpiotest_exitted = true;
}

// - - - - - - - - -  TIME & DELAY SUPPORT - - - - - - - - - 

#define TIMER_BASE_ADDR (0xfe000000 + 0x3000)

static volatile unsigned int *timer_regs;
static unsigned int           timer_initial_value;
static unsigned int           timer_last_value;

static void time_init(void)
{
    timer_regs = ioremap(TIMER_BASE_ADDR, 0x1000);
    timer_initial_value = timer_last_value = timer_regs[1];
}

static void time_exit(void)
{
    iounmap(timer_regs);
}

// returns time in us since the module was loaded
static uint64_t time_get(void)
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
static uint64_t time_delay(int duration)
{
    uint64_t start, now;

    start = time_get();
    while (true) {
        if ((now=time_get()) > start + duration) {
            break;
        }
    }

    return now;
}
