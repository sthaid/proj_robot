#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <malloc.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>

#include "util_mc.h"
#include "util_misc.h"

//
// defines
//

#define MAX_MC_LIST 4

//
// typedefs
//

//
// variables
//

static mc_t * mc_list[MAX_MC_LIST];
static int    max_mc_list;

//
// prototypes
//

static void *monitor_thread(void *cx);

static int issue_cmd(mc_t *mc, unsigned char *cmd, int cmdlen, unsigned char *resp, int resplen);
static int open_serial_port(const char * device, uint32_t baud_rate);
static int write_port(mc_t *mc, const uint8_t * buffer, size_t size);
static ssize_t read_port(mc_t *mc, uint8_t * buffer, size_t size);

static void crc_init(void);
static unsigned char crc(unsigned char message[], unsigned char length);

// -----------------  UNIT TEST---------------------------------------------

// gcc -o util_mc -Wall -g -O2 -pthread -DUNIT_TEST util_mc.c util_misc.c

#ifdef UNIT_TEST
int main(int argc, char **argv)
{
    mc_t *mc;
    int rc;
    int error_status;
    int product_id;
    int fw_ver_maj_bcd, fw_ver_min_bcd;
    char str[200];

    mc_init_module();

    mc = mc_new(0);
    if (mc == NULL) {
        ERROR("mc_new failed\n");
        return 1;
    }

    error_status = mc_status(mc);
    INFO("error_status 0x%x\n", error_status);

    rc = mc_get_fw_ver(mc, &product_id, &fw_ver_maj_bcd, &fw_ver_min_bcd);
    if (rc < 0) {
        ERROR("mc_get_fw_ver failed\n");
        return 1;
    }
    INFO("product_id = 0x%4.4x fw_ver = %x.%x\n", product_id, fw_ver_maj_bcd, fw_ver_min_bcd);

    while (printf("? "), fgets(str,sizeof(str),stdin) != NULL) {
        int cnt, value;
        char cmd;

        #define VALUE_REQUIRED \
            if (cnt != 2) { \
                printf("ERROR: value required\n"); \
                continue; \
            }

        cnt = sscanf(str, "%c %d", &cmd, &value);
        if (cnt == 0) {
            continue;
        }

        switch (cmd) {
        case 'f':
            VALUE_REQUIRED;
            mc_speed(mc, value);
            break;
        case 'r':
            VALUE_REQUIRED;
            mc_speed(mc, -value);
            break;
        default:
            printf("ERROR: cmd '%c' invalid\n", cmd);
            break;
        }
    }

    return 0;
}
#endif

// -----------------  INIT API  --------------------------------------------

void mc_init_module(void)
{
    crc_init();
}

// -----------------  RUN TIME API  ----------------------------------------

mc_t *mc_new(int id)
{
    int fd, rc, error_status;
    char device[100];
    mc_t *mc;

    // open usb serial device
    sprintf(device, "/dev/ttyACM%d", id);
    fd = open_serial_port(device, 115200);
    if (fd < 0) {
        return NULL;
    }

    // allocate and init the mc handle
    mc = malloc(sizeof(mc_t));
    mc->fd = fd;
    strcpy(mc->device, device);

    // get error status, to confirm we can communicate to the ctlr
    rc = mc_get_variable(mc, VAR_ERROR_STATUS, &error_status);
    if (rc < 0) {
        free(mc);
        close(fd);
        return NULL;
    }

    // add mc to list
    if (max_mc_list >= MAX_MC_LIST) {
        free(mc);
        close(fd);
        return NULL;
    }
    mc_list[max_mc_list] = mc;
    __sync_synchronize();
    max_mc_list++;

    // create monitor_thread to periodically check status of this mc
    pthread_create(&mc->monitor_thread_id, NULL, monitor_thread, mc);

    // return handle
    return mc;
}

// return 0 success, -1 failed to enable
int mc_enable(mc_t *mc)
{
    unsigned char cmd[1] = { 0x83 };

    // exit safe start
    if (issue_cmd(mc, cmd, sizeof(cmd), NULL, 0) < 0) {
        return -1;
    }

    // return success if error_status is 0, else failure
    return mc_status(mc) == 0 ? 0 : -1;
}

// return -1 could not get error_status, else error_status is returned
int mc_status(mc_t *mc)
{
    int error_status;

    if (mc_get_variable(mc, VAR_ERROR_STATUS, &error_status) < 0) {
        return -1;
    }

    return error_status;
}

// return 0 if cmd was successfully issued, else return -1
int mc_speed(mc_t *mc, int speed)
{
    if (speed >= 0) {
        return issue_cmd(mc, (unsigned char []){0x85, speed & 0x1f, speed >> 5 & 0x7f}, 3, NULL, 0);
    } else {
        speed = -speed;
        return issue_cmd(mc, (unsigned char []){0x86, speed & 0x1f, speed >> 5 & 0x7f}, 3, NULL, 0);
    }
}

// return 0 if cmd was successfully issued, else return -1
int mc_brake(mc_t *mc)
{
    return issue_cmd(mc, (unsigned char []){0x92, 0x20}, 2, NULL, 0);
}

// return 0 if cmd was successfully issued, else return -1
int mc_coast(mc_t *mc)
{
    return issue_cmd(mc, (unsigned char []){0x92, 0x00}, 2, NULL, 0);
}

// return 0 if cmd was successfully issued, else return -1
int mc_stop(mc_t *mc)
{
    return issue_cmd(mc, (unsigned char []){0xe0}, 1, NULL, 0);
}

// return 0 if cmd was successfully issued, else return -1;
// when 0 is returned the variable's value has been set
int mc_get_variable(mc_t *mc, int id, int *value)
{
    unsigned char cmd[1] = { 0xa1 };
    unsigned char resp[2];
    bool value_is_signed;

    if (issue_cmd(mc, cmd, sizeof(cmd), resp, sizeof(resp)) < 0) {
        return -1;
    }

    *value = resp[0] + 256 * resp[1];

    value_is_signed = false;  // XXX tbd
    if (value_is_signed && *value > 32767 ) {
        *value -= 65536;
    }

    return 0;
}

// return 0 if motor limit was set, else return -1
int mc_set_motor_limit(mc_t *mc, int id, int value)
{
    unsigned char resp[1];

    if (issue_cmd(mc, (unsigned char []){0xa2, value & 0x7f, value >> 7}, 3, resp, sizeof(resp)) < 0) {
        return -1;
    }

    return resp[0] == 0 ? 0 : -1;
}

// return 0 if cmd was successfully issued, else return -1
int mc_set_current_limit(mc_t *mc, int milli_amps)
{
    int limit;
    int current_scale_cal  = 8057;  // default
    int current_offset_cal = 993;   // default
    unsigned char resp[1];

    // XXX probably won't be using the default cal values
    // XXX need to verify this

    limit = (milli_amps * 3200 * 2 / current_scale_cal + current_offset_cal) * 3200 / 65536;

    return issue_cmd(mc, 
                     (unsigned char []){0x91, limit & 0x7f, limit >> 7}, 3, 
                     resp, sizeof(resp));
}

// return 0 if cmd was successfully issued, else return -1;
// when 0 is returned the product_id and fw_ver values have been set
int mc_get_fw_ver(mc_t *mc, int *product_id, int *fw_ver_maj_bcd, int *fw_ver_min_bcd)
{
    unsigned char resp[4];

    if (issue_cmd(mc, (unsigned char []){0xc2}, 1, resp, 4) < 0) {
        return -1;
    }

    *product_id = resp[0] + 256 * resp[1];
    *fw_ver_min_bcd = resp[2];
    *fw_ver_maj_bcd = resp[3];

    return 0;
}

// -----------------  MONITOR THREAD  --------------------------------------

static void *monitor_thread(void *cx)
{
    mc_t *mc = (mc_t *)cx;
    int i, error_status, error_status_last=1;

    while (true) {
        // get error status 
        error_status = mc_status(mc);

        // if not okay now, and was okay last time then
        // stop all motors
        if (error_status != 0 && error_status_last == 0) {
            ERROR("stopping all motors becuase %s failed, error_status=0x%x\n", 
                  mc->device, error_status);
            for (i = 0; i < max_mc_list; i++) {
                mc_stop(mc_list[i]);
            }
        }

        // remember last error_status
        error_status_last = error_status;

        // sleep for 1 sec
        sleep(1);
    }

    return NULL;
}

// -----------------  SEND COMMAND  ----------------------------------------

static int issue_cmd(mc_t *mc, unsigned char *cmd, int cmdlen, unsigned char *resp, int resplen)
{
    unsigned char lcl_cmd[64], lcl_resp[64];
    int rc;

    // copy cmd to lcl_cmd, and append crc byte, and send
    memcpy(lcl_cmd, cmd, cmdlen);
    lcl_cmd[cmdlen] = crc(lcl_cmd, cmdlen);
    rc = write_port(mc, lcl_cmd, cmdlen+1);
    if (rc < 0) {
        return rc;
    }

    // if resplen then read the response, verify the crc, and 
    // copy the lcl_resp to caller's buffer
    if (resplen) {
        rc = read_port(mc, lcl_resp, resplen+1);
        if (rc != resplen+1) {
            return -1;
        }
        if (lcl_resp[resplen] != crc(lcl_resp, resplen)) {
            return -1;
        }
        memcpy(resp, lcl_resp, resplen);
    }

    // return success
    return 0;
}

// -----------------  SERIAL PORT  -----------------------------------------

// https://www.pololu.com/docs/0J77/8.6
// 8.6. Example serial code for Linux and macOS in C

// Opens the specified serial port, sets it up for binary communication,
// configures its read timeouts, and sets its baud rate.
// Returns a non-negative file descriptor on success, or -1 on failure.
static int open_serial_port(const char * device, uint32_t baud_rate)
{
    int fd;

    fd = open(device, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        ERROR("open %s, %s\n", device, strerror(errno));
        return -1;
    }
 
    // Flush away any bytes previously read or written.
    int result = tcflush(fd, TCIOFLUSH);
    if (result) {
        ERROR("tcflush %s, %s\n", device, strerror(errno));
    }
 
    // Get the current configuration of the serial port.
    struct termios options;
    result = tcgetattr(fd, &options);
    if (result) {
        ERROR("tcgetattr %s, %s\n", device, strerror(errno));
        close(fd);
        return -1;
    }
 
    // Turn off any options that might interfere with our ability to send and
    // receive raw binary bytes.
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
 
    // Set up timeouts: Calls to read() will return as soon as there is
    // at least one byte available or when 100 ms has passed.
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;
 
    // This code only supports certain standard baud rates. Supporting
    // non-standard baud rates should be possible but takes more work.
    switch (baud_rate) {
    case 4800:   cfsetospeed(&options, B4800);   break;
    case 9600:   cfsetospeed(&options, B9600);   break;
    case 19200:  cfsetospeed(&options, B19200);  break;
    case 38400:  cfsetospeed(&options, B38400);  break;
    case 115200: cfsetospeed(&options, B115200); break;
    default:
        fprintf(stderr, "warning: baud rate %u is not supported, using 9600.\n",
                baud_rate);
        cfsetospeed(&options, B9600);
        break;
    }
    cfsetispeed(&options, cfgetospeed(&options));
 
    result = tcsetattr(fd, TCSANOW, &options);
    if (result) {
        ERROR("tcsetattr %s, %s\n", device, strerror(errno));
        close(fd);
        return -1;
    }
 
    return fd;
}
 
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
static int write_port(mc_t *mc, const uint8_t * buffer, size_t size)
{
    ssize_t result;

    result = write(mc->fd, buffer, size);
    if (result != (ssize_t)size) {
        ERROR("write %s, %s\n", mc->device, strerror(errno));
        return -1;
    }

    return 0;
}
 
// Reads bytes from the serial port.
// Returns after all the desired bytes have been read, or if there is a
// timeout or other error.
// Returns the number of bytes successfully read into the buffer, or -1 if
// there was an error reading.
static ssize_t read_port(mc_t *mc, uint8_t * buffer, size_t size)
{
    size_t received = 0;

    while (received < size) {
        ssize_t r = read(mc->fd, buffer + received, size - received);
        if (r < 0) {
            ERROR("read %s, %s\n", mc->device, strerror(errno));
            return -1;
        }
        if (r == 0) {
            ERROR("read timedout %s\n", mc->device);
            break;
        }
        received += r;
    }
    return received;
}

// -----------------  CRC  -------------------------------------------------

// https://www.pololu.com/docs/0J77/8.13
// 8.13. Example CRC computation in C
 
static const unsigned char CRC7_POLY = 0x91;
static unsigned char crc_table[256];
 
static unsigned char get_crc_for_byte(unsigned char val)
{
    unsigned char j;
 
    for (j = 0; j < 8; j++) {
        if (val & 1)
            val ^= CRC7_POLY;
        val >>= 1;
    }
 
    return val;
}
 
static void crc_init()
{
    int i;
 
    // fill an array with CRC values of all 256 possible bytes
    for (i = 0; i < 256; i++) {
        crc_table[i] = get_crc_for_byte(i);
    }
}
 
static unsigned char crc(unsigned char message[], unsigned char length)
{
    unsigned char i, crc = 0;
 
    for (i = 0; i < length; i++)
        crc = crc_table[crc ^ message[i]];

    return crc;
}
