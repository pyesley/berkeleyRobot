#include <Arduino.h>
#include <SimpleFOC.h>

// AS5600 on motor shaft — used for FOC commutation
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
void onTarget(char* cmd) { commander.scalar(&target_velocity, cmd); }
void onVelP(char* cmd) { commander.scalar(&motor.PID_velocity.P, cmd); }
void onVelI(char* cmd) { commander.scalar(&motor.PID_velocity.I, cmd); }
void onVelD(char* cmd) { commander.scalar(&motor.PID_velocity.D, cmd); }
void onVlim(char* cmd) {
  commander.scalar(&motor.voltage_limit, cmd);
  motor.PID_velocity.limit = motor.voltage_limit;
}
void onLpf(char* cmd)  { commander.scalar(&motor.LPF_velocity.Tf, cmd); }

// ── Hardware debug helpers ───────────────────────────────────────────────────

void printHex32(const char* label, uint32_t val) {
  Serial.print(label);
  Serial.print("0x");
  for (int i = 28; i >= 0; i -= 4) {
    Serial.print((val >> i) & 0xF, HEX);
  }
}

void dumpTimerRegisters() {
  uint32_t sr   = TIM1->SR;
  uint32_t bdtr = TIM1->BDTR;
  uint32_t cr1  = TIM1->CR1;
  uint32_t af1  = TIM1->AF1;
  uint32_t af2  = TIM1->AF2;

  Serial.println("── TIM1 ──");
  printHex32("  SR:   ", sr);
  Serial.print("  [");
  if (sr & TIM_SR_BIF)  Serial.print(" BIF");
  if (sr & TIM_SR_B2IF) Serial.print(" B2IF");
  if (sr & TIM_SR_SBIF) Serial.print(" SBIF");
  Serial.println(" ]");

  printHex32("  BDTR: ", bdtr);
  Serial.print("  MOE=");
  Serial.print((bdtr & TIM_BDTR_MOE) ? "ON" : "**OFF**");
  Serial.print("  BKE=");
  Serial.print((bdtr & TIM_BDTR_BKE) ? 1 : 0);
  Serial.print("  BK2E=");
  Serial.println((bdtr & TIM_BDTR_BK2E) ? 1 : 0);

  printHex32("  CR1:  ", cr1);
  Serial.print("  CEN=");
  Serial.println((cr1 & TIM_CR1_CEN) ? 1 : 0);

  Serial.print("  CCR1=");
  Serial.print(TIM1->CCR1);
  Serial.print("  CCR2=");
  Serial.print(TIM1->CCR2);
  Serial.print("  CCR3=");
  Serial.print(TIM1->CCR3);
  Serial.print("  ARR=");
  Serial.println(TIM1->ARR);
}

void dumpAll() {
  Serial.println("\n========== DEBUG DUMP ==========");
  dumpTimerRegisters();

  Serial.println("── COMP ──");
  for (int i = 0; i < 3; i++) {
    COMP_TypeDef* comp = (i == 0) ? COMP1 : (i == 1) ? COMP2 : COMP3;
    Serial.print("  COMP"); Serial.print(i+1);
    Serial.print(" EN="); Serial.print((comp->CSR & COMP_CSR_EN) ? 1 : 0);
    Serial.print(" OUT="); Serial.println((comp->CSR & COMP_CSR_VALUE) ? 1 : 0);
  }

  Serial.println("── Motor ──");
  Serial.print("  enabled="); Serial.print(motor.enabled);
  Serial.print("  status="); Serial.println(motor.motor_status);
  Serial.print("  Vlim="); Serial.print(motor.voltage_limit, 2);
  Serial.print("  PID P="); Serial.print(motor.PID_velocity.P, 4);
  Serial.print("  I="); Serial.print(motor.PID_velocity.I, 4);
  Serial.print("  D="); Serial.print(motor.PID_velocity.D, 4);
  Serial.print("  LPF Tf="); Serial.println(motor.LPF_velocity.Tf, 4);
  Serial.println("================================\n");
}

void onDebug(char* cmd) { dumpAll(); }

void onReset(char* cmd) {
  Serial.println("Clearing break flags and re-enabling MOE...");
  TIM1->SR &= ~(TIM_SR_BIF | TIM_SR_B2IF | TIM_SR_SBIF);
  TIM1->BDTR |= TIM_BDTR_MOE;
  Serial.print("MOE is now: ");
  Serial.println((TIM1->BDTR & TIM_BDTR_MOE) ? "ON" : "OFF");
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

  // Velocity control with voltage torque mode
  motor.controller = MotionControlType::velocity;
  motor.torque_controller = TorqueControlType::voltage;

  // Voltage limits — caps how much voltage the PID can command
  motor.voltage_limit = 3.0;            // max voltage to motor (limits current)
  motor.voltage_sensor_align = 1.0;     // alignment voltage during initFOC

  // Velocity PID — start conservative
  motor.PID_velocity.P = 0.1;
  motor.PID_velocity.I = 1.0;
  motor.PID_velocity.D = 0.0;
  motor.PID_velocity.output_ramp = 20;  // V/s ramp limit — gentle acceleration
  motor.PID_velocity.limit = 3.0;       // match voltage_limit

  // Low-pass filter on velocity measurement — heavier filtering for I2C sensor
  motor.LPF_velocity.Tf = 0.05;         // 50ms filter (AS5600 I2C is slow ~150Hz)

  // Downsample velocity loop — AS5600 I2C is slow, no point running velocity PID at FOC rate
  motor.motion_downsampling = 5;

  motor.init();
  motor.initFOC();

  commander.add('V', onTarget, "target velocity (rad/s)");
  commander.add('P', onVelP,  "velocity P gain");
  commander.add('I', onVelI,  "velocity I gain");
  commander.add('K', onVelD,  "velocity D gain");
  commander.add('L', onVlim,  "voltage limit (V)");
  commander.add('F', onLpf,   "velocity LPF Tf (s)");
  commander.add('D', onDebug, "dump debug registers");
  commander.add('R', onReset, "reset MOE / clear break");

  Serial.println("=== Velocity Control with Current Limiting ===");
  Serial.println("Commands:");
  Serial.println("  V<rad/s>  - set target velocity");
  Serial.println("  P/I/K     - tune PID gains");
  Serial.println("  L<volts>  - set voltage limit");
  Serial.println("  F<sec>    - set LPF time constant");
  Serial.println("  D         - dump registers");
  Serial.println("  R         - reset break/MOE");

  dumpAll();
}

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);
  commander.run();

  // Track break events in real-time
  static bool break_detected = false;
  uint32_t sr = TIM1->SR;
  uint32_t bdtr = TIM1->BDTR;
  bool moe_off = !(bdtr & TIM_BDTR_MOE);
  bool brk = (sr & (TIM_SR_BIF | TIM_SR_B2IF));

  if ((moe_off || brk) && !break_detected) {
    break_detected = true;
    Serial.println("\n*** BREAK EVENT DETECTED ***");
    Serial.print("  BIF="); Serial.print((sr & TIM_SR_BIF) ? 1 : 0);
    Serial.print("  B2IF="); Serial.print((sr & TIM_SR_B2IF) ? 1 : 0);
    Serial.print("  MOE="); Serial.println(moe_off ? "**OFF**" : "ON");
    dumpAll();
  }
  if (!moe_off && !brk) break_detected = false;

  static uint32_t last_print = 0;
  if (millis() - last_print > 500) {
    last_print = millis();

    float vel = motor.shaft_velocity;
    float angle = as5600.getAngle();
    PhaseCurrent_s c = current_sense.getPhaseCurrents();
    DQCurrent_s dq = current_sense.getFOCCurrents(motor.electrical_angle);

    Serial.print("Vel: ");
    Serial.print(vel, 2);
    Serial.print("  Tgt: ");
    Serial.print(target_velocity, 2);
    Serial.print("  Ang: ");
    Serial.print(angle, 2);
    Serial.print("  Vq: ");
    Serial.print(motor.voltage.q, 2);
    Serial.print("  Iq: ");
    Serial.print(dq.q, 2);
    Serial.print("  Id: ");
    Serial.print(dq.d, 2);
    Serial.print("  |I|: ");
    Serial.print(sqrtf(c.a*c.a + c.b*c.b + c.c*c.c), 2);
    Serial.print("  MOE=");
    Serial.println((TIM1->BDTR & TIM_BDTR_MOE) ? "1" : "0");
  }
}
