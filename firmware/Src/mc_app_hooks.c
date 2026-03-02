
/**
  ******************************************************************************
  * @file    mc_app_hooks.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file implements default motor control app hooks.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  * @ingroup MCAppHooks
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "mc_app_hooks.h"
#include "as5600.h"
#include "position_ctrl.h"
#include "can_interface.h"

extern I2C_HandleTypeDef hi2c1;
extern FDCAN_HandleTypeDef hfdcan1;

static AS5600_Handle_t g_encoder;
static PosCtrl_Handle_t g_pos_ctrl;

/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup MCTasks
  * @{
  */

/**
 * @defgroup MCAppHooks Motor Control Applicative hooks
 * @brief User defined functions that are called in the Motor Control tasks.
 *
 *
 * @{
 */

/**
 * @brief Hook function called right before the end of the MCboot function.
 *
 *
 *
 */
__weak void MC_APP_BootHook(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed at the end of MCboot().
   */

/* USER CODE BEGIN BootHook */
  AS5600_Init(&g_encoder, &hi2c1);
  PosCtrl_Init(&g_pos_ctrl, &g_encoder);
  CAN_Init(&g_can, &hfdcan1, &g_pos_ctrl);
/* USER CODE END BootHook */
}

/**
 * @brief Hook function called right after the Medium Frequency Task for Motor 1.
 *
 *
 *
 */
__weak void MC_APP_PostMediumFrequencyHook_M1(void)
{
  /*
   * This function can be overloaded or the application can inject
   * code into it that will be executed right after the Medium
   * Frequency Task of Motor 1
   */

/* USER SECTION BEGIN PostMediumFrequencyHookM1 */
  PosCtrl_Run(&g_pos_ctrl);
  CAN_SendTelemetry(&g_can);
/* USER SECTION END PostMediumFrequencyHookM1 */
}

/** @} */

/** @} */

/** @} */

/************************ (C) COPYRIGHT 2025 STMicroelectronics *****END OF FILE****/
