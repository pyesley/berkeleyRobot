#ifndef AS5600_H
#define AS5600_H

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define AS5600_I2C_ADDR       (0x36 << 1)
#define AS5600_RAW_ANGLE_REG  0x0C
#define AS5600_STATUS_REG     0x0B
#define AS5600_COUNTS         4096
#define AS5600_HALF_COUNTS    2048

#define GEAR_RATIO            14.0f

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint16_t raw_angle;
  int32_t  turn_count;
  uint16_t prev_raw;
  float    motor_deg;
  float    output_deg;
  bool     initialized;
} AS5600_Handle_t;

HAL_StatusTypeDef AS5600_Init(AS5600_Handle_t *dev, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef AS5600_ReadAngle(AS5600_Handle_t *dev);
float AS5600_GetMotorDeg(AS5600_Handle_t *dev);
float AS5600_GetOutputDeg(AS5600_Handle_t *dev);
void AS5600_SetHome(AS5600_Handle_t *dev);

#endif /* AS5600_H */
