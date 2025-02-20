// Uses the Linux I2C API to send and receive data from a Tic.
// NOTE: The Tic's control mode must be "Serial / I2C / USB".
// NOTE: For reliable operation on a Raspberry Pi, enable the i2c-gpio
// overlay and use the I2C device it provides (usually /dev/i2c-3).
// NOTE: You might need to change the 'const char * device' line below
// to specify the correct I2C device.
// NOTE: You might need to change the `const uint8_t address' line below
// to match the device number of your Tic.

#include "tic.h"
// Opens the specified I2C device.  Returns a non-negative file descriptor
// on success, or -1 on failure.
int open_i2c_device(const char * device)
{
    int fd = open(device, O_RDWR);
    if (fd == -1) {
        perror(device);
        return -1;
    }
    return fd;
}

// Sends the "Exit safe start" command.
// Returns 0 on success and -1 on failure.
int tic_exit_safe_start(int fd, uint8_t address)
{
    uint8_t command[] = { 0x83 };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to exit safe start");
        return -1;
    }
    return 0;
}

// Sets the target position, returning 0 on success and -1 on failure.
//
// For more information about what this command does, see the
// "Set Target Position" command in the "Command reference" section of the
// Tic user's guide.
int tic_set_target_position(int fd, uint8_t address, int32_t target)
{
    uint8_t command[] = {
        0xE0,
        (uint8_t)(target >> 0 & 0xFF),
        (uint8_t)(target >> 8 & 0xFF),
        (uint8_t)(target >> 16 & 0xFF),
        (uint8_t)(target >> 24 & 0xFF),
    };
    struct i2c_msg message = { address, 0, sizeof(command), command };
    struct i2c_rdwr_ioctl_data ioctl_data = { &message, 1 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 1) {
        perror("failed to set target position");
        return -1;
    }
    return 0;
}

// Gets one or more variables from the Tic (without clearing them).
// Returns 0 for success, -1 for failure.
int tic_get_variable(int fd, uint8_t address, uint8_t offset,
uint8_t * buffer, uint8_t length)
{
    uint8_t command[] = { 0xA1, offset };
    struct i2c_msg messages[] = {
        { address, 0, sizeof(command), command },
        { address, I2C_M_RD, length, buffer },
    };
    struct i2c_rdwr_ioctl_data ioctl_data = { messages, 2 };
    int result = ioctl(fd, I2C_RDWR, &ioctl_data);
    if (result != 2) {
    perror("failed to get variables");
    return -1;
    }

    return 0;
}

// Gets the "Current position" variable from the Tic.
// Returns 0 for success, -1 for failure.
int tic_get_current_position(int fd, uint8_t address, int32_t * output)
{
    *output = 0;
    uint8_t buffer[4];
    int result = tic_get_variable(fd, address, 0x22, buffer, sizeof(buffer));
    if (result) { return -1; }
    *output = buffer[0] + ((uint32_t)buffer[1] << 8) +
    ((uint32_t)buffer[2] << 16) + ((uint32_t)buffer[3] << 24);
    return 0;
}

// int main()
// {
// // Choose the I2C device.
// const char * device = "/dev/i2c-1";

// // Set the I2C address of the Tic (the device number).
// const uint8_t address = 14;

// int fd = open_i2c_device(device);
// if (fd < 0) { return 1; }

// int result;

// int32_t position;
// result = tic_get_current_position(fd, address, &position);
// if (result) { return 1; }
// printf("Current position is %d.\n", position);

// int32_t new_target = position > 0 ? -200 : 200;
// printf("Setting target position to %d.\n", new_target);
// result = tic_exit_safe_start(fd, address);
// if (result) { return 1; }
// result = tic_set_target_position(fd, address, new_target);
// if (result) 
//     { return 1; }

// close(fd);
// return 0;
// }
