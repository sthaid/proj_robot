#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <malloc.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <pthread.h>

#include "util_mc.h"
#include "util_misc.h"

//
// defines
//

//
// typedefs
//

//
// variables
//

//
// prototypes
//

static void *monitor_thread(void *cx);

static int issue_cmd(int fd, unsigned char *cmd, int cmdlen, unsigned char *resp, int resplen);
static int open_serial_port(const char * device, uint32_t baud_rate);
static int write_port(int fd, const uint8_t * buffer, size_t size);
static ssize_t read_port(int fd, uint8_t * buffer, size_t size);

static void crc_init(void);
static unsigned char crc(unsigned char message[], unsigned char length);

// -----------------  UNIT TEST---------------------------------------------

// gcc -o util_mc -Wall -g -O2 -pthread -DUNIT_TEST util_mc.c util_misc.c

#ifdef UNIT_TEST
int main(int argc, char **argv)
{
    mc_t *mc;
    int rc, error_status;

    mc = mc_new(0);
    if (mc == NULL) {
        ERROR("mc_new failed\n");
        return 1;
    }

    rc = mc_status(mc, &error_status);
    if (rc < 0) {
        ERROR("mc_status failed\n");
        return 1;
    }

    return 0;
}
#endif

// -----------------  API  -------------------------------------------------

mc_t *mc_new(int id)
{
    int fd, rc, error_status;
    char device[100];
    mc_t *mc;

    static bool first_call = true;

    // init crc table
    if (first_call) {
        crc_init();
        first_call = false;
    }

    // open usb serial device
    sprintf(device, "/dev/ttyACM%d", id);
    fd = open_serial_port(device, 115200);
    if (fd < 0) {
        return NULL;
    }

    // allocate and init the mc handle
    mc = malloc(sizeof(mc_t));
    mc->fd = fd;

    // get error status, to confirm we can communicate to the ctlr
    rc = mc_get_variable(mc, VAR_ERROR_STATUS, &error_status);
    if (rc < 0) {
        free(mc);
        return NULL;
    }

    // create monitor_thread to periodically check status of this mc
    pthread_create(&mc->monitor_thread_id, NULL, monitor_thread, mc);

    // return handle
    return mc;
}

int mc_enable(mc_t *mc)
{
    return 0;
}

int mc_status(mc_t *mc, int *error_status)
{
    return mc_get_variable(mc, VAR_ERROR_STATUS, error_status);
}

int mc_speed(mc_t *mc)
{
    return 0;
}

int mc_brake(mc_t *mc)
{
    return 0;
}

int mc_stop(mc_t *mc)
{
    return 0;
}

int mc_get_variable(mc_t *mc, int id, int *value)
{
    unsigned char cmd[1] = { 0xa1 };
    unsigned char resp[2];
    int rc;
    bool value_is_signed;

    rc = issue_cmd(mc->fd, cmd, sizeof(cmd), resp, sizeof(resp));
    if (rc < 0) {
        return rc;
    }

    *value = resp[0] + 256 * resp[1];

    value_is_signed = false;  // xxx tbd
    if (value_is_signed && *value > 32767 ) {
        *value -= 65536;
    }

    return 0;
}

int mc_set_motor_limit(mc_t *mc, int id, int value)
{
    return 0;
}

int mc_set_current_limit(mc_t *mc, int milli_amps)
{
    return 0;
}

int mc_get_fw_ver(mc_t *mc, int *product_id, int *fw_version)
{
    return 0;
}

// -----------------  MONITOR THREAD  --------------------------------------

static void *monitor_thread(void *cx)
{
    mc_t *mc = (mc_t *)cx;
    int rc, error_status;

    while (true) {
        // get error status 
        rc = mc_get_variable(mc, VAR_ERROR_STATUS, &error_status);
        if (rc < 0) {
            printf("ERROR: XXX\n");
        }

        // if all mc were okay last iteration, and are not all okay now
        // then stop them all

        // sleep for 1 sec
        sleep(1);
    }

    return NULL;
}

// -----------------  SEND COMMAND  ----------------------------------------

static int issue_cmd(int fd, unsigned char *cmd, int cmdlen, unsigned char *resp, int resplen)
{
    unsigned char lcl_cmd[64], lcl_resp[64];
    int rc;

    // copy cmd to lcl_cmd, and append crc byte, and send
    memcpy(lcl_cmd, cmd, cmdlen);
    lcl_cmd[cmdlen] = crc(lcl_cmd, cmdlen);
    rc = write_port(fd, lcl_cmd, cmdlen+1);
    if (rc < 0) {
        return rc;
    }

    // if resplen then read the response, verify the crc, and 
    // copy the lcl_resp to caller's buffer
    if (resplen) {
        rc = read_port(fd, lcl_resp, resplen+1);
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
        perror(device);
        return -1;
    }
 
    // Flush away any bytes previously read or written.
    int result = tcflush(fd, TCIOFLUSH);
    if (result) {
        perror("tcflush failed");  // just a warning, not a fatal error
    }
 
    // Get the current configuration of the serial port.
    struct termios options;
    result = tcgetattr(fd, &options);
    if (result) {
        perror("tcgetattr failed");
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
        perror("tcsetattr failed");
        close(fd);
        return -1;
    }
 
    return fd;
}
 
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
static int write_port(int fd, const uint8_t * buffer, size_t size)
{
    ssize_t result;

    result = write(fd, buffer, size);
    if (result != (ssize_t)size) {
        perror("failed to write to port");
        return -1;
    }

    return 0;
}
 
// Reads bytes from the serial port.
// Returns after all the desired bytes have been read, or if there is a
// timeout or other error.
// Returns the number of bytes successfully read into the buffer, or -1 if
// there was an error reading.
static ssize_t read_port(int fd, uint8_t * buffer, size_t size)
{
    size_t received = 0;

    while (received < size) {
        ssize_t r = read(fd, buffer + received, size - received);
        if (r < 0) {
            perror("failed to read from port");
            return -1;
        }
        if (r == 0) {
            // Timeout
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
 




#if 0
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

int main()
{
  unsigned char message[3] = {0x83, 0x01, 0x00};
  int i, j;
 
  buildCRCTable();
  message[2] = getCRC(message, 2);
 
  for (i = 0; i < sizeof(message); i++)
  {
    for (j = 0; j < 8; j++)
      printf("%d", (message[i] >> j) % 2);
    printf(" ");
  }
  printf("\n");
}

int main()
{
  // Choose the serial port name.
  // Linux USB example:  "/dev/ttyACM0"  (see also: /dev/serial/by-id)
  // macOS USB example:  "/dev/cu.usbmodem001234562"
  // Cygwin example:     "/dev/ttyS7"
  const char * device = "/dev/ttyACM0";
 
  // Choose the baud rate (bits per second).  This does not matter if you are
  // connecting to the SMC over USB.  If you are connecting via the TX and RX
  // lines, this should match the baud rate in the SMC G2's serial settings.
  uint32_t baud_rate = 9600;
 
  int fd = open_serial_port(device, baud_rate);
  if (fd < 0) { return 1; }
 
  int result = smc_exit_safe_start(fd);
  if (result) { return 1; }
 
  uint16_t error_status;
  result = smc_get_error_status(fd, &error_status);
  if (result) { return 1; }
  printf("Error status: 0x%04x\n", error_status);
 
  int16_t target_speed;
  result = smc_get_target_speed(fd, &target_speed);
  if (result) { return 1; }
  printf("Target speed is %d.\n", target_speed);
 
  int16_t new_speed = (target_speed <= 0) ? 3200 : -3200;
  printf("Setting target speed to %d.\n", new_speed);
  result = smc_set_target_speed(fd, new_speed);
  if (result) { return 1; }
 
  close(fd);
  return 0;
}

// ------------------------------------------------------------------------

// Reads a variable from the SMC.
// Returns 0 on success or -1 on failure.
int smc_get_variable(int fd, uint8_t variable_id, uint16_t * value)
{
  uint8_t command[] = { 0xA1, variable_id };
  int result = write_port(fd, command, sizeof(command));
  if (result) { return -1; }
  uint8_t response[2];
  ssize_t received = read_port(fd, response, sizeof(response));
  if (received < 0) { return -1; }
  if (received != 2)
  {
    fprintf(stderr, "read timeout: expected 2 bytes, got %zu\n", received);
    return -1;
  }
  *value = response[0] + 256 * response[1];
  return 0;
}
 
// Gets the target speed (-3200 to 3200).
// Returns 0 on success, -1 on failure.
int smc_get_target_speed(int fd, int16_t * value)
{
  return smc_get_variable(fd, 20, (uint16_t *)value);
}
 
// Gets a number where each bit represents a different error, and the
// bit is 1 if the error is currently active.
// See the user's guide for definitions of the different error bits.
// Returns 0 on success, -1 on failure.
int smc_get_error_status(int fd, uint16_t * value)
{
  return smc_get_variable(fd, 0, value);
}
 
// Sends the Exit Safe Start command, which is required to drive the motor.
// Returns 0 on success, -1 on failure.
int smc_exit_safe_start(int fd)
{
  const uint8_t command = 0x83;
  return write_port(fd, &command, 1);
}
 
// Sets the SMC's target speed (-3200 to 3200).
// Returns 0 on success, -1 on failure.
int smc_set_target_speed(int fd, int speed)
{
  uint8_t command[3];
 
  if (speed < 0)
  {
    command[0] = 0x86; // Motor Reverse
    speed = -speed;
  }
  else
  {
    command[0] = 0x85; // Motor Forward
  }
  command[1] = speed & 0x1F;
  command[2] = speed >> 5 & 0x7F;
 
  return write_port(fd, command, sizeof(command));
}

// ------------------------------------------------------------------------
// ------------------------------------------------------------------------
// ------------------------------------------------------------------------

// ------------------------------------------------------------------------
#endif
