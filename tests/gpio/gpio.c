// References:
// - https://datasheets.raspberrypi.org/bcm2711/bcm2711-peripherals.pdf

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <time.h>

//
// defines
//

#define PI4B_PERIPHERAL_BASE_ADDR 0xfe000000
#define GPIO_BASE_ADDR            (PI4B_PERIPHERAL_BASE_ADDR + 0x200000)
#define GPIO_BLOCK_SIZE           0x1000

#define PULL_NONE  0
#define PULL_UP    1
#define PULL_DOWN  2

#define FUNC_IN    0
#define FUNC_OUT   1

#define MAX_GPIO_LIST (sizeof(gpio_list)/sizeof(gpio_list[0]))
#define GPIO_LAST  57

//
// typedefs
//

//
// variables
//

volatile unsigned int *gpio_regs;
int gpio_list[] = {
        4, 5, 6, 12, 13, 16, 17, 22, 23, 24, 25, 26, 27,    // general purpose
        2, 3,                                               // i2c
        7, 8, 9, 10, 11,                                    // spi
        18, 19, 20, 21,                                     // pcm
        12, 14,                                             // uart
            };

//
// prototypes
//

void init(void);
void status(void);
void pull(int n, int pullsel);
void set(int n, int v);
void func(int n, int fsel);
void square_wave_test(int n);

// -----------------  MAIN  ----------------------------

int main(int argc, char **argv)
{
    char s[100], cmd[100];
    int cnt, len, gpionum;

    // init
    init();

    // loop until done
    while (status(), printf("\n> "), fgets(s, sizeof(s), stdin)) {
        // remove trailing newline
        len = strlen(s);
        if (len > 0 && s[len-1] == '\n') {
            s[len-1] = '\0';
        }

        // parse cmdline, and verify 
        gpionum = -1;
        cmd[0] = '\0';
        cnt = sscanf(s, "%s %d", cmd, &gpionum);
        if (cnt <= 0) {
            continue;
        }
        if ((strcmp(cmd, "help") != 0) &&
            (strcmp(cmd, "q") != 0) &&
            (gpionum < 0 || gpionum > GPIO_LAST)) 
        {
            printf("ERROR: invalid gpionum\n");
            continue;
        }

        // process cmd
        if (strcmp(cmd, "help") == 0) {
            printf("func_in <gpionum>\n");
            printf("func_out <gpionum>\n");
            printf("pull_up <gpionum>\n");
            printf("pull_down <gpionum>\n");
            printf("pull_none <gpionum>\n");
            printf("set <gpionum>\n");
            printf("clear <gpionum>\n");
            printf("square_wave_test <gpionum>\n");
            printf("q\n");
            printf("\n");
        } else if (strcmp(cmd, "pull_up") == 0 || 
                   strcmp(cmd, "pull_down") == 0 ||
                   strcmp(cmd, "pull_none") == 0)
        {
            pull(gpionum, (strcmp(cmd, "pull_up") == 0   ? PULL_UP :
                           strcmp(cmd, "pull_down") == 0 ? PULL_DOWN
                                                         : PULL_NONE));
        } else if (strcmp(cmd, "set") == 0 || 
                   strcmp(cmd, "clear") == 0)
        {
            set(gpionum, (strcmp(cmd, "set") == 0 ? 1 : 0));
        } else if (strcmp(cmd, "func_in") == 0 || 
                   strcmp(cmd, "func_out") == 0)
        {
            func(gpionum, (strcmp(cmd, "func_in") == 0 ? FUNC_IN : FUNC_OUT));
        } else if (strcmp(cmd, "square_wave_test") == 0) {
            square_wave_test(gpionum);
        } else if (strcmp(cmd, "q") == 0) {
            break;
        } else {
            printf("ERROR: invalid cmd '%s'\n", cmd);
        }
    }

    // done
    return 0;
}

void init(void)
{
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
        printf("ERROR: can't open /dev/mem, %s \n", strerror(errno));
        exit(1);
    }
    gpio_regs = mmap(NULL,
                     GPIO_BLOCK_SIZE,
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED,
                     fd,
                     GPIO_BASE_ADDR);
    if (gpio_regs == MAP_FAILED) {
        printf("ERROR: mmap failed\n");
        exit(1);
    }
    close(fd);
}
 
// -----------------  CMDS  ----------------------------

void status(void)
{
    int i;

    // print header line containing list of gpio numbers
    printf("%-6s", "GPIO");
    for (i = 0; i < MAX_GPIO_LIST; i++) {
        int n = gpio_list[i];
        printf("%4d", n);
    }
    printf("\n");

    // print the GPFSEL registers, regidx 0..5
    printf("%-6s", "FSEL");
    for (i = 0; i < MAX_GPIO_LIST; i++) {
        int n = gpio_list[i];
        int regidx, bit, fsel;

        // gpsel regs
        regidx = 0 + (n / 10);
        bit = (n % 10) * 3;
        fsel = (gpio_regs[regidx] >> bit) & 7;

        if (fsel == 0) {
            printf("  IN");
        } else if (fsel == 1) {
            printf(" OUT");
        } else {
            printf("   %c",
                   (fsel == 4 ? '0' :
                    fsel == 5 ? '1' :
                    fsel == 6 ? '2' :
                    fsel == 7 ? '3' :
                    fsel == 3 ? '4' :
                    fsel == 2 ? '5' :
                                '?'));
        }
    }
    printf("\n");

    // print the GPLEV registers, regidx 13..14
    printf("%-6s", "LEV");
    for (i = 0; i < MAX_GPIO_LIST; i++) {
        int n = gpio_list[i];
        int regidx, bit, lev;

        regidx = 13 + (n / 32);
        bit = (n % 32);
        lev = (gpio_regs[regidx] >> bit) & 1;

        printf("%4d", lev);
    }
    printf("\n");

    // print the GPIO_PUP_PDN_CNTRL registers, regidx 57..60
    printf("%-6s", "PULL");
    for (i = 0; i < MAX_GPIO_LIST; i++) {
        int n = gpio_list[i];
        int regidx, bit, pullsel;
    
        regidx = 57 + (n / 16);
        bit = (n % 16) * 2;
        pullsel = (gpio_regs[regidx] >> bit) & 3;

        printf("%4s", 
               (pullsel == 0 ? "  --" :
                pullsel == 1 ? "  UP" :
                pullsel == 2 ? "  DN" :
                               " ERR"));
    }
    printf("\n");
}

void pull(int n, int pullsel)
{
    int regidx, bit;
    unsigned int tmp;

    regidx = 57 + (n / 16);
    bit = (n % 16) * 2;

    tmp = gpio_regs[regidx];
    tmp &= ~(3 << bit);   // 2 bit field
    tmp |= (pullsel << bit);

    gpio_regs[regidx] = tmp;
}

void set(int n, int v)
{
    int regidx, bit;

    if (v) {
        // set uses base regidx 7, 1 bit field
        regidx = 7 + (n / 32);
        bit = (n % 32);
        gpio_regs[regidx] = (1 << bit);
    } else {
        // clr uses base regidx 10, 1 bit field
        regidx = 10 + (n / 32);
        bit = (n % 32);
        gpio_regs[regidx] = (1 << bit);
    }
}

void func(int n, int fsel)
{
    int regidx, bit;
    unsigned int tmp;

    regidx = 0 + (n / 10);
    bit = (n % 10) * 3;

    tmp = gpio_regs[regidx];
    tmp &= ~(7 << bit);  // 3 bit field
    tmp |= (fsel << bit);

    gpio_regs[regidx] = tmp;
}

void square_wave_test(int n)
{
    struct timespec ts={0,1000};
    while (true) {
        set(n, 1);
        nanosleep(&ts, NULL);
        set(n, 0);
        nanosleep(&ts, NULL);
    }
}
