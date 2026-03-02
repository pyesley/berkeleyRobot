#include "can_interface.h"
#include "mc_api.h"
#include <string.h>

CAN_Interface_t g_can;

static FDCAN_RxHeaderTypeDef rx_header;
static uint8_t rx_data[8];

HAL_StatusTypeDef CAN_Init(CAN_Interface_t *can, FDCAN_HandleTypeDef *hfdcan,
                           PosCtrl_Handle_t *pos_ctrl)
{
  can->hfdcan = hfdcan;
  can->pos_ctrl = pos_ctrl;
  can->telem_count = 0;
  can->status_count = 0;

  /* Configure RX filter: accept IDs 0x101-0x103 */
  FDCAN_FilterTypeDef filter;
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_RANGE;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = CAN_ID_SET_POSITION;
  filter.FilterID2 = CAN_ID_COMMAND;

  HAL_StatusTypeDef ret = HAL_FDCAN_ConfigFilter(hfdcan, &filter);
  if (ret != HAL_OK) return ret;

  /* Reject non-matching frames */
  ret = HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  if (ret != HAL_OK) return ret;

  /* Enable RX FIFO 0 new message interrupt */
  ret = HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  if (ret != HAL_OK) return ret;

  /* Start FDCAN */
  ret = HAL_FDCAN_Start(hfdcan);
  return ret;
}

void CAN_RxCallback(CAN_Interface_t *can)
{
  if (HAL_FDCAN_GetRxMessage(can->hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    return;

  uint32_t id = rx_header.Identifier;
  float fval;

  switch (id)
  {
  case CAN_ID_SET_POSITION:
    memcpy(&fval, rx_data, 4);
    PosCtrl_SetTarget(can->pos_ctrl, fval);
    break;

  case CAN_ID_SET_PID_GAINS:
  {
    float kp, ki;
    memcpy(&kp, &rx_data[0], 4);
    memcpy(&ki, &rx_data[4], 4);
    PosCtrl_SetGains(can->pos_ctrl, kp, ki);
    break;
  }

  case CAN_ID_COMMAND:
    switch (rx_data[0])
    {
    case CAN_CMD_ENABLE:
      PosCtrl_Enable(can->pos_ctrl);
      break;
    case CAN_CMD_DISABLE:
      PosCtrl_Disable(can->pos_ctrl);
      break;
    case CAN_CMD_HOME:
      PosCtrl_SetHome(can->pos_ctrl);
      break;
    case CAN_CMD_ESTOP:
      PosCtrl_Disable(can->pos_ctrl);
      MC_StopMotor1();
      break;
    }
    break;

  default:
    break;
  }
}

void CAN_SendTelemetry(CAN_Interface_t *can)
{
  can->telem_count++;
  can->status_count++;

  /* Telemetry at 100 Hz */
  if (can->telem_count >= TELEM_DIVIDER) {
    can->telem_count = 0;

    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.Identifier = CAN_ID_TELEMETRY;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    float actual = can->pos_ctrl->actual_deg;
    float target = can->pos_ctrl->target_deg;
    memcpy(&tx_data[0], &actual, 4);
    memcpy(&tx_data[4], &target, 4);

    HAL_FDCAN_AddMessageToTxFifoQ(can->hfdcan, &tx_header, tx_data);
  }

  /* Status at 10 Hz */
  if (can->status_count >= STATUS_DIVIDER) {
    can->status_count = 0;

    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    tx_header.Identifier = CAN_ID_STATUS;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    float speed = can->pos_ctrl->speed_cmd_rpm;
    memcpy(&tx_data[0], &speed, 4);

    uint8_t state = (uint8_t)can->pos_ctrl->state;
    uint16_t faults = MC_GetCurrentFaultsMotor1();
    uint8_t mc_state = (uint8_t)MC_GetSTMStateMotor1();
    tx_data[4] = state;
    tx_data[5] = mc_state;
    tx_data[6] = (uint8_t)(faults & 0xFF);
    tx_data[7] = (uint8_t)(faults >> 8);

    HAL_FDCAN_AddMessageToTxFifoQ(can->hfdcan, &tx_header, tx_data);
  }
}

/* FDCAN RX callback - called from ISR context via HAL */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
    CAN_RxCallback(&g_can);
  }
}
