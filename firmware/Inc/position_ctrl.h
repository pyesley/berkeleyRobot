#ifndef POSITION_CTRL_H
#define POSITION_CTRL_H

#include "as5600.h"
#include "mc_api.h"
#include <stdbool.h>

typedef enum {
  POS_STATE_DISABLED,
  POS_STATE_IDLE,
  POS_STATE_STARTING,
  POS_STATE_RUNNING,
  POS_STATE_STOPPING,
  POS_STATE_FAULT
} PosCtrl_State_t;

typedef struct {
  /* PID gains */
  float kp;
  float ki;
  float kd;

  /* PID state */
  float integral;
  float prev_error;

  /* Limits */
  float max_speed_rpm;
  float integral_limit;
  float deadband_deg;

  /* Target and feedback */
  float target_deg;
  float actual_deg;
  float speed_cmd_rpm;

  /* Motor management */
  PosCtrl_State_t state;
  float min_speed_rpm;
  uint16_t start_delay_count;
  uint16_t start_delay_target;

  /* Encoder */
  AS5600_Handle_t *encoder;
} PosCtrl_Handle_t;

void PosCtrl_Init(PosCtrl_Handle_t *ctrl, AS5600_Handle_t *encoder);
void PosCtrl_Run(PosCtrl_Handle_t *ctrl);
void PosCtrl_Enable(PosCtrl_Handle_t *ctrl);
void PosCtrl_Disable(PosCtrl_Handle_t *ctrl);
void PosCtrl_SetTarget(PosCtrl_Handle_t *ctrl, float target_deg);
void PosCtrl_SetGains(PosCtrl_Handle_t *ctrl, float kp, float ki);
void PosCtrl_SetHome(PosCtrl_Handle_t *ctrl);

#endif /* POSITION_CTRL_H */
