/*
 * lsm6d.h
 *
 *  Created on: May 29, 2018
 *      Author: Jason Berger
 */

#ifndef LSM6D_H_
#define LSM6D_H_

#include <stdint.h>
#include <stdbool.h>
#include "Platforms/Common/mrt_platform.h"

#define LSM6D_8BIT_ADDR 0xD4 // 0x30
#define LSM6D_ID			 0x6A

#define LSM6d_XL_PWRDOWN	 0

#define LSM6D_XL_ODR_12HZ5       0x10
#define LSM6D_XL_ODR_26HZ        0x20
#define LSM6D_XL_ODR_52HZ        0x30
#define LSM6D_XL_ODR_104HZ       0x40
#define LSM6D_XL_ODR_208HZ       0x50
#define LSM6D_XL_ODR_416HZ       0x60
#define LSM6D_XL_ODR_833HZ       0x70

#define LSM6D_FIFO_ODR_DIABLE	 0x00
#define LSM6D_FIFO_ODR_12HZ5     0x08
#define LSM6D_FIFO_ODR_26HZ      0x10
#define LSM6D_FIFO_ODR_52HZ      0x18
#define LSM6D_FIFO_ODR_104HZ     0x20
#define LSM6D_FIFO_ODR_208HZ     0x28
#define LSM6D_FIFO_ODR_416HZ     0x30
#define LSM6D_FIFO_ODR_833HZ     0x38


#define LSM6D_GYRO_PWRDOWN		 0

#define LSM6D_REG_FIFO_CTRL1	0x06
#define LSM6D_REG_FIFO_CTRL2	0x07
#define LSM6D_REG_FIFO_CTRL3	0x08
#define LSM6D_REG_FIFO_CTRL4	0x09
#define LSM6D_REG_FIFO_CTRL5 	0x0A
#define LSM6D_REG_INT1_CTRL		0x0D
#define LSM6D_REG_CTRL1_XL   	0x10
#define LSM6D_REG_CTRL2_G   	0x11
#define LSM6D_REG_CTRL3_C 		0x12
#define LSM6D_REG_CTRL5_C 		0x14
#define LSM6D_REG_CTRL3   		0x22
#define LSM6D_REG_CTRL4   		0x23
#define LSM6D_REG_CTRL5   		0x24
#define LSM6D_REG_CTRL6   		0x25
#define LSM6D_REG_CTRL7   		0x25
#define LSM6D_REG_CTRL8_XL  	0x17
#define LSM6D_REG_CTRL9_XL  	0x18


#define LSM6D_REG_WHOAMI  0x0F
#define LSM6D_REG_STATUS  0x27
#define LSM6D_REG_OUT_X   0x28
#define LSM6D_REG_OUT_Y   0x2A
#define LSM6D_REG_OUT_Z   0x2C

#define LSM6D_REG_FIFO_DATA_L 0x3E

#define LSM6D_CONVERSION_2G 0.061

typedef struct{
	float x;
	float y;
	float z;
}accel_vector_t;




mrt_status_t lsm6d_init(mrt_i2c_handle_t handle);
mrt_i2c_status_t lsm6d_set_xl_odr(uint8_t rate);
mrt_i2c_status_t lsm6d_set_gyro_odr(uint8_t rate);
mrt_i2c_status_t lsm6d_get_fifo(accel_vector_t* result);
mrt_i2c_status_t lsm6d_poll(accel_vector_t* result);
void lsm6d_reset_fifo(uint8_t rate);
mrt_i2c_status_t lsm6d_write_reg(uint8_t reg, uint8_t data);
uint8_t lsm6d_read_reg(uint8_t reg);
mrt_i2c_status_t lsm6d_write_regs(uint8_t reg, uint8_t* data, uint8_t len);
mrt_i2c_status_t lsm6d_read_regs(uint8_t reg, uint8_t* data, uint8_t len);


#endif/* LSM6D_H_ */
