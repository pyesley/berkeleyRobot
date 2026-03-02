#include "as5600.h"

#define I2C_TIMEOUT 5

HAL_StatusTypeDef AS5600_Init(AS5600_Handle_t *dev, I2C_HandleTypeDef *hi2c)
{
  dev->hi2c = hi2c;
  dev->turn_count = 0;
  dev->prev_raw = 0;
  dev->motor_deg = 0.0f;
  dev->output_deg = 0.0f;
  dev->initialized = false;

  /* Verify device is present by reading status register */
  uint8_t status;
  uint8_t reg = AS5600_STATUS_REG;
  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(hi2c, AS5600_I2C_ADDR, &reg, 1, I2C_TIMEOUT);
  if (ret != HAL_OK) return ret;

  ret = HAL_I2C_Master_Receive(hi2c, AS5600_I2C_ADDR, &status, 1, I2C_TIMEOUT);
  if (ret != HAL_OK) return ret;

  /* Read initial angle */
  reg = AS5600_RAW_ANGLE_REG;
  uint8_t buf[2];
  ret = HAL_I2C_Master_Transmit(hi2c, AS5600_I2C_ADDR, &reg, 1, I2C_TIMEOUT);
  if (ret != HAL_OK) return ret;

  ret = HAL_I2C_Master_Receive(hi2c, AS5600_I2C_ADDR, buf, 2, I2C_TIMEOUT);
  if (ret != HAL_OK) return ret;

  dev->raw_angle = ((uint16_t)buf[0] << 8) | buf[1];
  dev->prev_raw = dev->raw_angle;
  dev->initialized = true;

  return HAL_OK;
}

HAL_StatusTypeDef AS5600_ReadAngle(AS5600_Handle_t *dev)
{
  if (!dev->initialized) return HAL_ERROR;

  uint8_t reg = AS5600_RAW_ANGLE_REG;
  uint8_t buf[2];

  HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(dev->hi2c, AS5600_I2C_ADDR, &reg, 1, I2C_TIMEOUT);
  if (ret != HAL_OK) return ret;

  ret = HAL_I2C_Master_Receive(dev->hi2c, AS5600_I2C_ADDR, buf, 2, I2C_TIMEOUT);
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
