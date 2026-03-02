#include "as5600.h"

#define I2C_TIMEOUT 100

HAL_StatusTypeDef AS5600_Init(AS5600_Handle_t *dev, I2C_HandleTypeDef *hi2c)
{
  dev->hi2c = hi2c;
  dev->turn_count = 0;
  dev->prev_raw = 0;
  dev->motor_deg = 0.0f;
  dev->output_deg = 0.0f;
  dev->initialized = false;

  /* Retry loop: wait for AS5600 to power up and respond */
  HAL_StatusTypeDef status = HAL_ERROR;
  uint8_t buf[2];
  while (status != HAL_OK) {
    HAL_I2C_Init(dev->hi2c);

    /* Wait for I2C device to power up */
    HAL_Delay(100);

    status = HAL_I2C_Mem_Read(dev->hi2c, AS5600_I2C_ADDR, AS5600_RAW_ANGLE_REG,
                              I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT);
  }

  dev->raw_angle = ((uint16_t)buf[0] << 8) | buf[1];
  dev->prev_raw = dev->raw_angle;
  dev->initialized = true;

  return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadAngle(AS5600_Handle_t *dev)
{
  if (!dev->initialized) return HAL_ERROR;

  uint8_t buf[2];

  HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(dev->hi2c, AS5600_I2C_ADDR, AS5600_RAW_ANGLE_REG,
                                           I2C_MEMADD_SIZE_8BIT, buf, 2, I2C_TIMEOUT);
  if (ret != HAL_OK) return ret;

  dev->raw_angle = ((uint16_t)buf[0] << 8) | buf[1];

  /* Multi-turn tracking: detect zero crossings */
  int32_t delta = (int32_t)dev->raw_angle - (int32_t)dev->prev_raw;
  if (delta > AS5600_HALF_COUNTS) {
    dev->turn_count--;
  } else if (delta < -AS5600_HALF_COUNTS) {
    dev->turn_count++;
  }
  dev->prev_raw = dev->raw_angle;

  /* Calculate absolute position */
  dev->motor_deg = (float)dev->turn_count * 360.0f
                 + (float)dev->raw_angle * (360.0f / (float)AS5600_COUNTS);
  dev->output_deg = dev->motor_deg / GEAR_RATIO;

  return HAL_OK;
}

float AS5600_GetMotorDeg(AS5600_Handle_t *dev)
{
  return dev->motor_deg;
}

float AS5600_GetOutputDeg(AS5600_Handle_t *dev)
{
  return dev->output_deg;
}

void AS5600_SetHome(AS5600_Handle_t *dev)
{
  dev->turn_count = 0;
  dev->prev_raw = dev->raw_angle;
  dev->motor_deg = 0.0f;
  dev->output_deg = 0.0f;
}
