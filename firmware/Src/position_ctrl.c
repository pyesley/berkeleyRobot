#include "position_ctrl.h"
#include <math.h>

void PosCtrl_Init(PosCtrl_Handle_t *ctrl, AS5600_Handle_t *encoder)
{
  ctrl->encoder = encoder;

  /* Conservative initial gains */
  ctrl->kp = 10.0f;
  ctrl->ki = 1.0f;
  ctrl->kd = 0.5f;

  /* PID state */
  ctrl->integral = 0.0f;
  ctrl->prev_error = 0.0f;

  /* Limits */
  ctrl->max_speed_rpm = 2000.0f;
  ctrl->integral_limit = 500.0f;
  ctrl->deadband_deg = 0.1f;

  /* Motor management */
  ctrl->min_speed_rpm = 50.0f;
  ctrl->start_delay_count = 0;
  ctrl->start_delay_target = 3500; /* ~3.5s at 1kHz for sensorless startup */

  /* Target / feedback */
  ctrl->target_deg = 0.0f;
  ctrl->actual_deg = 0.0f;
  ctrl->speed_cmd_rpm = 0.0f;

  ctrl->state = POS_STATE_DISABLED;
}

void PosCtrl_Run(PosCtrl_Handle_t *ctrl)
{
  /* Always read encoder so we can monitor position even when disabled */
  if (AS5600_ReadAngle(ctrl->encoder) == HAL_OK) {
    ctrl->actual_deg = AS5600_GetOutputDeg(ctrl->encoder);
  }

  if (ctrl->state == POS_STATE_DISABLED || ctrl->state == POS_STATE_FAULT)
    return;

  /* Compute position error */
  float error = ctrl->target_deg - ctrl->actual_deg;

  /* Apply deadband */
  if (fabsf(error) < ctrl->deadband_deg) {
    error = 0.0f;
  }

  /* PID computation */
  ctrl->integral += error;

  /* Anti-windup clamp */
  if (ctrl->integral > ctrl->integral_limit)
    ctrl->integral = ctrl->integral_limit;
  else if (ctrl->integral < -ctrl->integral_limit)
    ctrl->integral = -ctrl->integral_limit;

  float derivative = error - ctrl->prev_error;
  ctrl->prev_error = error;

  ctrl->speed_cmd_rpm = ctrl->kp * error
                      + ctrl->ki * ctrl->integral
                      + ctrl->kd * derivative;

  /* Clamp speed output */
  if (ctrl->speed_cmd_rpm > ctrl->max_speed_rpm)
    ctrl->speed_cmd_rpm = ctrl->max_speed_rpm;
  else if (ctrl->speed_cmd_rpm < -ctrl->max_speed_rpm)
    ctrl->speed_cmd_rpm = -ctrl->max_speed_rpm;

  /* Motor state machine */
  MCI_State_t mc_state = MC_GetSTMStateMotor1();

  switch (ctrl->state)
  {
  case POS_STATE_IDLE:
    if (fabsf(ctrl->speed_cmd_rpm) > ctrl->min_speed_rpm) {
      /* Need to start motor */
      MC_ProgramSpeedRampMotor1_F(ctrl->speed_cmd_rpm, 0);
      if (MC_StartMotor1()) {
        ctrl->state = POS_STATE_STARTING;
        ctrl->start_delay_count = 0;
      }
    }
    break;

  case POS_STATE_STARTING:
    ctrl->start_delay_count++;
    if (mc_state == RUN) {
      ctrl->state = POS_STATE_RUNNING;
    } else if (mc_state == FAULT_NOW || mc_state == FAULT_OVER) {
      MC_AcknowledgeFaultMotor1();
      ctrl->state = POS_STATE_IDLE;
      ctrl->start_delay_count = 0;
    } else if (ctrl->start_delay_count > ctrl->start_delay_target) {
      /* Startup timeout */
      ctrl->state = POS_STATE_IDLE;
      ctrl->start_delay_count = 0;
    }
    break;

  case POS_STATE_RUNNING:
    if (mc_state == FAULT_NOW || mc_state == FAULT_OVER) {
      MC_AcknowledgeFaultMotor1();
      ctrl->state = POS_STATE_IDLE;
      ctrl->integral = 0.0f;
      break;
    }

    if (fabsf(ctrl->speed_cmd_rpm) < ctrl->min_speed_rpm) {
      /* Close enough to target, stop motor. Gearbox holds position. */
      MC_StopMotor1();
      ctrl->state = POS_STATE_STOPPING;
      ctrl->integral = 0.0f;
    } else {
      /* Update speed command immediately (0ms ramp) */
      MC_ProgramSpeedRampMotor1_F(ctrl->speed_cmd_rpm, 0);
    }
    break;

  case POS_STATE_STOPPING:
    if (mc_state == IDLE) {
      ctrl->state = POS_STATE_IDLE;
    } else if (mc_state == FAULT_NOW || mc_state == FAULT_OVER) {
      MC_AcknowledgeFaultMotor1();
      ctrl->state = POS_STATE_IDLE;
    }
    break;

  default:
    break;
  }
}

void PosCtrl_Enable(PosCtrl_Handle_t *ctrl)
{
  if (ctrl->state == POS_STATE_DISABLED) {
    ctrl->state = POS_STATE_IDLE;
    ctrl->integral = 0.0f;
    ctrl->prev_error = 0.0f;
    /* Set target to current position */
    if (AS5600_ReadAngle(ctrl->encoder) == HAL_OK) {
      ctrl->target_deg = AS5600_GetOutputDeg(ctrl->encoder);
      ctrl->actual_deg = ctrl->target_deg;
    }
  }
}

void PosCtrl_Disable(PosCtrl_Handle_t *ctrl)
{
  if (ctrl->state == POS_STATE_RUNNING) {
    MC_StopMotor1();
  }
  ctrl->state = POS_STATE_DISABLED;
  ctrl->integral = 0.0f;
  ctrl->prev_error = 0.0f;
}

void PosCtrl_SetTarget(PosCtrl_Handle_t *ctrl, float target_deg)
{
  ctrl->target_deg = target_deg;
}

void PosCtrl_SetGains(PosCtrl_Handle_t *ctrl, float kp, float ki)
{
  ctrl->kp = kp;
  ctrl->ki = ki;
  ctrl->integral = 0.0f;
}

void PosCtrl_SetHome(PosCtrl_Handle_t *ctrl)
{
  AS5600_SetHome(ctrl->encoder);
  ctrl->target_deg = 0.0f;
  ctrl->actual_deg = 0.0f;
  ctrl->integral = 0.0f;
  ctrl->prev_error = 0.0f;
}
