/*	lsm6d.c
 *	Created by: Jason Berger
 */

#include "lsm6d.h"
#include "stm32l4xx_hal.h"

uint8_t auto_flag = 0;

float scaling_factor = LSM6D_CONVERSION_2G;
mrt_i2c_handle_t I2cHandle;

void lsm6d_init(mrt_i2c_handle_t  handle)
{
  uint32_t err_code;
  I2cHandle = handle;
  uint8_t id;

  lsm6d_write_reg(LSM6D_REG_CTRL3_C, 0x84);// reboot sensor
  MRT_DELAY_MS(100);	//wait for reboot
  id = lsm6d_read_reg(LSM6D_REG_WHOAMI-1); // make sure multi read is working

  if(id != LSM6D_ID )
  {
	 //TODO report com error
  }
}

mrt_i2c_status_t lsm6d_set_xl_odr(uint8_t rate)
{
	return lsm6d_write_reg(LSM6D_REG_CTRL1_XL, rate);
}

mrt_i2c_status_t lsm6d_set_gyro_odr(uint8_t rate)
{
	return lsm6d_write_reg(LSM6D_REG_CTRL2_G, rate);
}

mrt_i2c_status_t lsm6d_get_fifo(accel_vector_t* result)
{
	int16_t raw_x, raw_y, raw_z;
  mrt_i2c_status_t status;
	status = lsm6d_read_regs(LSM6D_REG_FIFO_DATA_L, (uint8_t*) &raw_x, 2);
	status = lsm6d_read_regs(LSM6D_REG_FIFO_DATA_L, (uint8_t*) &raw_y, 2);
	status = lsm6d_read_regs(LSM6D_REG_FIFO_DATA_L, (uint8_t*) &raw_z, 2);

	result->x = (float)raw_x * scaling_factor;
	result->y = (float)raw_y * scaling_factor;
	result->z = (float)raw_z * scaling_factor;

  return status;

}
mrt_i2c_status_t lsm6d_poll(accel_vector_t* result)
{
  int16_t data[3];
  mrt_i2c_status_t status;

  status = lsm6d_read_regs(LSM6D_REG_OUT_X, (uint8_t*) data, 6);

  result->x = (float)data[0] * scaling_factor;
  result->y = (float)data[1] * scaling_factor;
  result->z = (float)data[2] * scaling_factor;

  return status;
}

mrt_status_t lsm6d_reset_fifo(uint8_t rate)
{
	static uint8_t fifo5;
	lsm6d_write_reg(LSM6D_REG_FIFO_CTRL5, 0);
	lsm6d_write_reg(LSM6D_REG_FIFO_CTRL5, rate | 0x01);
	fifo5 = lsm6d_read_reg(LSM6D_REG_FIFO_CTRL5);
	fifo5 = fifo5+6;
}

void lsm6d_write_reg(uint8_t reg, uint8_t data)
{
	if (lsm6d_write_regs(reg,&data,1) != HAL_OK)
	{
		MRT_ERROR_HANDLER();
	}
}

uint8_t lsm6d_read_reg(uint8_t reg)
{
	uint8_t val;
	if (lsm6d_read_regs(reg,&val,1) != HAL_OK)
	{
		MRT_ERROR_HANDLER();
	}
	return val;
}

mrt_i2c_status_t lsm6d_write_regs(uint8_t reg, uint8_t* data, uint8_t len)
{
	mrt_i2c_status_t result = MRT_I2C_MEM_WRITE(I2cHandle, LSM6D_8BIT_ADDR, (uint16_t) reg , I2C_MEMADD_SIZE_8BIT, data , (uint16_t) len, 2000);
	MRT_DELAY_MS(5);
	return result;
}

mrt_i2c_status_t lsm6d_read_regs(uint8_t reg, uint8_t* data, uint8_t len)
{
	mrt_i2c_status_t result = MRT_I2C_MEM_READ(I2cHandle, LSM6D_8BIT_ADDR, (uint16_t) reg  , I2C_MEMADD_SIZE_8BIT, data, (uint16_t) len, 2000);
	MRT_DELAY_MS(5);
	return result;
}
