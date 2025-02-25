#ifndef TIC_H
#define TIC_H 

#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>


#define TIC_I2C_ADDRESS_DEVICE "/dev/i2c-1"
#define TIC_DEVICE_NUMBER (14)

extern "C" 
{
    int open_i2c_device(const char * device);
    int tic_exit_safe_start(int fd, uint8_t address);
    int tic_set_target_position(int fd, uint8_t address, int32_t target);
    int tic_get_variable(int fd, uint8_t address, uint8_t offset, uint8_t * buffer, uint8_t length);
    int tic_get_current_position(int fd, uint8_t address, int32_t * output);
    int tic_get_current_velocity(int fd, uint8_t address, int32_t * output);
}


#endif // MY_FUNCTIONS_H
