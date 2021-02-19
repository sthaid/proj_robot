// Uses the Linux I2C API to send and receive data from an SMC G2.
// NOTE: The SMC's input mode must be "Serial / I2C / USB".
// NOTE: For reliable operation on a Raspberry Pi, enable the i2c-gpio
//   overlay and use the I2C device it provides (usually /dev/i2c-3).
// NOTE: You might need to change the 'const char * device' line below
//   to specify the correct I2C device.
// NOTE: You might need to change the `const uint8_t address' line below
// to match the device number of your Simple Motor Controller.
 
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>
 
// Opens the specified I2C device.  Returns a non-negative file descriptor
// on success, or -1 on failure.
int open_i2c_device(const char * device)
{
  int fd = open(device, O_RDWR);
  if (fd == -1)
  {
    perror(device);
    return -1;
  }
  return fd;
}
 
// Reads a variable from the SMC.
// Returns 0 on success or -1 on failure.
int smc_get_variable(int fd, uint8_t address, uint8_t variable_id,
  uint16_t * value)
{
  uint8_t command[] = { 0xA1, variable_id };
  uint8_t response[2];
  struct i2c_msg messages[] = {
    { address, 0, sizeof(command), command },
    { address, I2C_M_RD, sizeof(response), response },
  };
  struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };
  int result = ioctl(fd, I2C_RDWR, &ioctl_data);
  if (result != 2)
  {
    perror("failed to get variable");
    return -1;
  }
  *value = response[0] + 256 * response[1];
  return 0;
}
 
// Gets the target speed (-3200 to 3200).
// Returns 0 on success, -1 on failure.
int smc_get_target_speed(int fd, uint8_t address, int16_t * value)
{
  return smc_get_variable(fd, address, 20, (uint16_t *)value);
}
 
// Gets a number where each bit represents a different error, and the
// bit is 1 if the error is currently active.
// See the user's guide for definitions of the different error bits.
// Returns 0 on success, -1 on failure.
int smc_get_error_status(int fd, uint8_t address, uint16_t * value)
{
  return smc_get_variable(fd, address, 0, value);
}
 
// Sends the Exit Safe Start command, which is required to drive the motor.
// Returns 0 on success, -1 on failure.
int smc_exit_safe_start(int fd, uint8_t address)
{
  uint8_t command = 0x83;
  struct i2c_msg message = { address, 0, 1, &command };
  struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
  int result = ioctl(fd, I2C_RDWR, &ioctl_data);
  if (result != 1)
  {
    perror("failed to exit safe start");
    return -1;
  }
  return 0;
}
 
// Sets the SMC's target speed (-3200 to 3200).
// Returns 0 on success, -1 on failure.
int smc_set_target_speed(int fd, uint8_t address, int16_t speed)
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
 
  struct i2c_msg message = { address, 0, sizeof(command), command };
  struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
  int result = ioctl(fd, I2C_RDWR, &ioctl_data);
  if (result != 1)
  {
    perror("failed to set speed");
    return -1;
  }
  return 0;
}
 
int main()
{
  // Choose the I2C device.
  const char * device = "/dev/i2c-1";
 
  // Set the I2C address of the SMC (the device number).
  const uint8_t address = 13;
 
  int fd = open_i2c_device(device);
  if (fd < 0) { return 1; }
 
  int result = smc_exit_safe_start(fd, address);
  if (result) { return 1; }
 
  uint16_t error_status;
  result = smc_get_error_status(fd, address, &error_status);
  if (result) { return 1; }
  printf("Error status: 0x%04x\n", error_status);
 
  int16_t target_speed;
  result = smc_get_target_speed(fd, address, &target_speed);
  if (result) { return 1; }
  printf("Target speed is %d.\n", target_speed);
 
  int16_t new_speed = (target_speed <= 0) ? 3200 : -3200;
  printf("Setting target speed to %d.\n", new_speed);
  result = smc_set_target_speed(fd, address, new_speed);
  if (result) { return 1; }
 
  close(fd);
  return 0;
}

