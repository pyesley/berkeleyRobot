#include <Arduino.h>
#include <SimpleFOC.h>

// ── CAN Protocol IDs (match host-side motor_protocol.hpp) ────────────────────
#define CAN_ID_NMT         0x001   // Mode commands: [mode, device_id]
#define CAN_ID_PDO2_TX     0x301   // Host->motor: [velocity_f32, unused_f32]
#define CAN_ID_PDO2_RX     0x281   // Motor->host: [position_f32, velocity_f32]
#define CAN_ID_HEARTBEAT   0x701   // Keepalive (0 bytes)
#define DEVICE_ID          1

// Motor modes (match host-side)
#define MODE_DISABLED      0x00
#define MODE_IDLE          0x01
#define MODE_VELOCITY      0x12
#define MODE_POSITION      0x13

// ── Motor + Sensor (declared early so CAN functions can reference them) ──────

MagneticSensorI2C as5600 = MagneticSensorI2C(AS5600_I2C);

BLDCMotor motor = BLDCMotor(14);
BLDCDriver6PWM driver = BLDCDriver6PWM(
  A_PHASE_UH, A_PHASE_UL,
  A_PHASE_VH, A_PHASE_VL,
  A_PHASE_WH, A_PHASE_WL
);

LowsideCurrentSense current_sense = LowsideCurrentSense(
  0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT
);

Commander commander = Commander(Serial);
float target_velocity = 0.0f;

// ── FDCAN ────────────────────────────────────────────────────────────────────
FDCAN_HandleTypeDef hfdcan1;
FDCAN_TxHeaderTypeDef txHeader;
volatile bool can_initialized = false;
volatile uint32_t can_rx_count = 0;
volatile uint32_t can_tx_count = 0;
volatile uint32_t can_err_count = 0;

#define WATCHDOG_TIMEOUT_MS 500
volatile uint32_t last_heartbeat_ms = 0;
volatile bool watchdog_active = false;

bool canInit() {
  __HAL_RCC_FDCAN_CLK_ENABLE();

  // Set FDCAN clock source to PCLK1 (170 MHz) — default is HSE (8 MHz)
  MODIFY_REG(RCC->CCIPR, RCC_CCIPR_FDCANSEL, RCC_CCIPR_FDCANSEL_1);

  // Enable CAN transceiver: PC11 = CAN_SHDN, active low
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef shdn = {};
  shdn.Pin = GPIO_PIN_11;
  shdn.Mode = GPIO_MODE_OUTPUT_PP;
  shdn.Pull = GPIO_NOPULL;
  shdn.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &shdn);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);  // LOW = transceiver enabled

  // Enable CAN termination: PC14 = CAN_TERM, HIGH = 120 ohm ON
  GPIO_InitTypeDef term = {};
  term.Pin = GPIO_PIN_14;
  term.Mode = GPIO_MODE_OUTPUT_PP;
  term.Pull = GPIO_NOPULL;
  term.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &term);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);  // HIGH = termination enabled

  // Configure GPIO: PA11 = CAN_RX, PB9 = CAN_TX
  GPIO_InitTypeDef gpio = {};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_11;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_9;
  gpio.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOB, &gpio);

  // FDCAN config: 1 Mbps, classic CAN
  // 170 MHz / prescaler=10 = 17 MHz time quanta
  // 1 + Seg1(13) + Seg2(3) = 17 tq per bit → 1 Mbps
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = ENABLE;
  hfdcan1.Init.NominalPrescaler = 10;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 3;
  hfdcan1.Init.NominalSyncJumpWidth = 3;
  hfdcan1.Init.DataPrescaler = 10;
  hfdcan1.Init.DataTimeSeg1 = 13;
  hfdcan1.Init.DataTimeSeg2 = 3;
  hfdcan1.Init.DataSyncJumpWidth = 3;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
    Serial.println("CAN: Init FAILED");
    return false;
  }

  // Accept all standard IDs into FIFO0
  FDCAN_FilterTypeDef filter = {};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0x000;
  filter.FilterID2 = 0x000;  // mask=0 → accept all
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    Serial.println("CAN: Filter config FAILED");
    return false;
  }

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT,
                                FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
    Serial.println("CAN: Start FAILED");
    return false;
  }

  // TX header template
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  can_initialized = true;
  Serial.println("CAN: Initialized at 1 Mbps");
  return true;
}

bool canSend(uint32_t id, const uint8_t* data, uint8_t len) {
  if (!can_initialized) return false;

  txHeader.Identifier = id;
  txHeader.DataLength = (uint32_t)len;  // FDCAN_DLC_BYTES_0 through _8 are just 0-8

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &txHeader, (uint8_t*)data) != HAL_OK) {
    can_err_count++;
    return false;
  }
  can_tx_count++;
  return true;
}

void canSendFeedback() {
  float pos = motor.shaft_angle;      // Continuous multi-turn position (rad)
  float vel = motor.shaft_velocity;   // Filtered velocity from SimpleFOC (rad/s)
  uint8_t data[8];
  memcpy(data, &pos, 4);
  memcpy(data + 4, &vel, 4);
  canSend(CAN_ID_PDO2_RX, data, 8);
}

void canProcessRx() {
  if (!can_initialized) return;

  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0) {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
      can_err_count++;
      break;
    }
    can_rx_count++;

    uint32_t id = rxHeader.Identifier;

    if (id == CAN_ID_HEARTBEAT) {
      last_heartbeat_ms = millis();
      watchdog_active = true;
      canSend(CAN_ID_HEARTBEAT, nullptr, 0);

    } else if (id == CAN_ID_NMT) {
      uint8_t dlc = rxHeader.DataLength;
      if (dlc >= 2 && rxData[1] == DEVICE_ID) {
        uint8_t mode = rxData[0];
        Serial.print("CAN: Mode 0x");
        Serial.println(mode, HEX);

        if (mode == MODE_DISABLED || mode == MODE_IDLE) {
          target_velocity = 0;
          motor.disable();
        } else if (mode == MODE_VELOCITY) {
          motor.enable();
          motor.controller = MotionControlType::velocity;
        } else if (mode == MODE_POSITION) {
          motor.enable();
          motor.controller = MotionControlType::velocity;
        }
      }

    } else if (id == CAN_ID_PDO2_TX) {
      uint8_t dlc = rxHeader.DataLength;
      if (dlc >= 4) {
        float cmd_vel;
        memcpy(&cmd_vel, rxData, 4);
        target_velocity = cmd_vel;
        last_heartbeat_ms = millis();
      }
    }
  }
}

// ── Serial Commander Callbacks ───────────────────────────────────────────────

void onTarget(char* cmd) { commander.scalar(&target_velocity, cmd); }
void onVelP(char* cmd) { commander.scalar(&motor.PID_velocity.P, cmd); }
void onVelI(char* cmd) { commander.scalar(&motor.PID_velocity.I, cmd); }
void onVelD(char* cmd) { commander.scalar(&motor.PID_velocity.D, cmd); }
void onVlim(char* cmd) {
  commander.scalar(&motor.voltage_limit, cmd);
  motor.PID_velocity.limit = motor.voltage_limit;
}
void onLpf(char* cmd)  { commander.scalar(&motor.LPF_velocity.Tf, cmd); }

void onDebug(char* cmd) {
  Serial.println("\n========== DEBUG ==========");
  Serial.print("CAN: init="); Serial.print(can_initialized);
  Serial.print("  rx="); Serial.print(can_rx_count);
  Serial.print("  tx="); Serial.print(can_tx_count);
  Serial.print("  err="); Serial.println(can_err_count);
  Serial.print("  FDCAN PSR=0x"); Serial.println(FDCAN1->PSR, HEX);
  Serial.print("  FDCAN ECR=0x"); Serial.println(FDCAN1->ECR, HEX);
  Serial.print("  FDCAN RXF0S=0x"); Serial.println(FDCAN1->RXF0S, HEX);
  Serial.print("  FDCAN IR=0x"); Serial.println(FDCAN1->IR, HEX);
  Serial.print("  FDCAN RXGFC=0x"); Serial.println(FDCAN1->RXGFC, HEX);
  Serial.print("  FDCAN RXFIFO0 fill="); Serial.println(HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0));
  // GPIO debug: PA11=CAN_RX (MODER, AFR), PB9=CAN_TX
  Serial.print("  RCC CCIPR FDCANSEL="); Serial.println((RCC->CCIPR >> 24) & 0x3);
  Serial.print("  GPIOA MODER=0x"); Serial.println(GPIOA->MODER, HEX);
  Serial.print("  GPIOA AFRH=0x"); Serial.println(GPIOA->AFR[1], HEX);
  Serial.print("  GPIOB MODER=0x"); Serial.println(GPIOB->MODER, HEX);
  Serial.print("  GPIOB AFRH=0x"); Serial.println(GPIOB->AFR[1], HEX);
  Serial.print("Watchdog: active="); Serial.print(watchdog_active);
  Serial.print("  last_hb="); Serial.print(millis() - last_heartbeat_ms);
  Serial.println("ms ago");
  Serial.print("Motor: enabled="); Serial.print(motor.enabled);
  Serial.print("  Vlim="); Serial.print(motor.voltage_limit, 2);
  Serial.print("  P="); Serial.print(motor.PID_velocity.P, 3);
  Serial.print("  I="); Serial.print(motor.PID_velocity.I, 3);
  Serial.print("  LPF="); Serial.println(motor.LPF_velocity.Tf, 3);
  Serial.print("TIM1 MOE="); Serial.println((TIM1->BDTR & TIM_BDTR_MOE) ? "ON" : "OFF");
  Serial.println("===========================\n");
}

// ── Setup & Loop ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1000);
  SimpleFOCDebug::enable(&Serial);

  Wire.begin();
  as5600.init();

  driver.voltage_power_supply = 24.0;
  driver.init();
  motor.linkDriver(&driver);

  current_sense.linkDriver(&driver);
  current_sense.init();
  motor.linkCurrentSense(&current_sense);
  motor.linkSensor(&as5600);

  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit = 3.0;
  motor.voltage_sensor_align = 0.5;  // lower alignment voltage

  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 20;
  motor.PID_velocity.limit = 3.0;

  motor.LPF_velocity.Tf = 0.05;
  motor.motion_downsample = 5;

  motor.init();
  motor.initFOC();

  // Start with motor disabled — enable via serial 'E' or CAN NMT command
  motor.disable();
  Serial.println("Motor initialized but DISABLED (send E to enable)");

  canInit();

  // PA11 is shared with USB_DM — USB init may have reclaimed it.
  // Force PA11 back to AF9 (FDCAN1_RX) after all other init is done.
  {
    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOA, &gpio);
    Serial.println("CAN: PA11 re-configured for FDCAN1_RX");
  }

  commander.add('V', onTarget, "target velocity (rad/s)");
  commander.add('E', [](char* cmd) { motor.enable(); Serial.println("Motor ENABLED"); }, "enable motor");
  commander.add('X', [](char* cmd) { motor.disable(); target_velocity = 0; Serial.println("Motor DISABLED"); }, "disable motor");
  commander.add('P', onVelP,  "velocity P gain");
  commander.add('I', onVelI,  "velocity I gain");
  commander.add('K', onVelD,  "velocity D gain");
  commander.add('L', onVlim,  "voltage limit (V)");
  commander.add('F', onLpf,   "velocity LPF Tf (s)");
  commander.add('D', onDebug, "dump debug + CAN status");

  Serial.println("=== Velocity Control + CAN ===");
  Serial.println("Serial: E=enable X=disable V<rad/s> P I K L F D");
  Serial.println("CAN: PDO2_TX(0x301)=[vel_f32,...] NMT(0x001)=[mode,id]");
}

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);
  commander.run();

  canProcessRx();

  // Watchdog: stop motor if no heartbeat/command received
  if (watchdog_active && (millis() - last_heartbeat_ms > WATCHDOG_TIMEOUT_MS)) {
    target_velocity = 0;
    watchdog_active = false;
    Serial.println("CAN: Watchdog timeout - motor stopped");
  }

  // Send CAN feedback at ~50Hz (only after host has sent a heartbeat)
  static uint32_t last_can_fb = 0;
  if (can_initialized && watchdog_active && millis() - last_can_fb >= 20) {
    last_can_fb = millis();
    canSendFeedback();
  }

  // Serial status at 2Hz
  static uint32_t last_print = 0;
  if (millis() - last_print > 500) {
    last_print = millis();

    Serial.print("Vel: ");
    Serial.print(motor.shaft_velocity, 2);
    Serial.print("  Tgt: ");
    Serial.print(target_velocity, 2);
    Serial.print("  Ang: ");
    Serial.print(as5600.getAngle(), 2);
    Serial.print("  Iq: ");
    Serial.print(current_sense.getFOCCurrents(motor.electrical_angle).q, 2);
    Serial.print("  CAN rx:");
    Serial.print(can_rx_count);
    Serial.print(" tx:");
    Serial.print(can_tx_count);
    Serial.print("  MOE=");
    Serial.println((TIM1->BDTR & TIM_BDTR_MOE) ? "1" : "0");
  }
}
