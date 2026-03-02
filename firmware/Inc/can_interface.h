#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#include "stm32g4xx_hal.h"
#include "position_ctrl.h"

/* CAN message IDs */
#define CAN_ID_SET_POSITION   0x101
#define CAN_ID_SET_PID_GAINS  0x102
#define CAN_ID_COMMAND        0x103
#define CAN_ID_TELEMETRY      0x110
#define CAN_ID_STATUS         0x111

/* Command bytes */
#define CAN_CMD_ENABLE        0x01
#define CAN_CMD_DISABLE       0x02
#define CAN_CMD_HOME          0x03
#define CAN_CMD_ESTOP         0x04

/* Telemetry rates (in 1kHz ticks) */
#define TELEM_DIVIDER         10   /* 100 Hz */
#define STATUS_DIVIDER        100  /* 10 Hz  */

typedef struct {
  FDCAN_HandleTypeDef *hfdcan;
  PosCtrl_Handle_t    *pos_ctrl;
  uint16_t             telem_count;
  uint16_t             status_count;
} CAN_Interface_t;

HAL_StatusTypeDef CAN_Init(CAN_Interface_t *can, FDCAN_HandleTypeDef *hfdcan,
                           PosCtrl_Handle_t *pos_ctrl);
void CAN_ProcessRx(CAN_Interface_t *can);
void CAN_SendTelemetry(CAN_Interface_t *can);
void CAN_RxCallback(CAN_Interface_t *can);

/* Global instance for ISR access */
extern CAN_Interface_t g_can;

#endif /* CAN_INTERFACE_H */
