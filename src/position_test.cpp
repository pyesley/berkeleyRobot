/**
 * Linux-side position controller using TORQUE mode with fast CAN feedback.
 *
 * Motor is connected through a 14:1 gearbox.
 * Encoder is on the motor shaft (not gearbox output).
 * Target angle is specified in OUTPUT shaft degrees.
 * Motor must turn 14x more than the output.
 *
 * Uses FAST_FRAME (TPDO4 at 0x481) for motor position feedback at 500 Hz,
 * sends torque commands via PDO3 (RPDO3 at 0x401).
 * Trapezoidal velocity trajectory planner with PD+I tracking controller.
 *
 * Usage: ./position_test [output_degrees] [duration_seconds]
 */

#include "can_interface.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <optional>
#include <algorithm>

volatile sig_atomic_t g_running = 1;
void signalHandler(int) { g_running = 0; }

// CAN protocol
constexpr uint8_t DEVICE_ID = 1;
constexpr uint32_t makeCanId(uint32_t func, uint32_t dev) { return (func << 7) | dev; }

constexpr uint32_t CAN_ID_NMT        = makeCanId(0x0, 0);          // 0x000
constexpr uint32_t CAN_ID_PDO3_TX    = makeCanId(0x8, DEVICE_ID);  // 0x401 - torque cmd
constexpr uint32_t CAN_ID_FAST_FRAME = makeCanId(0x9, DEVICE_ID);  // 0x481 - fast feedback
constexpr uint32_t CAN_ID_SDO_TX     = makeCanId(0xB, DEVICE_ID);  // 0x581 - SDO response
constexpr uint32_t CAN_ID_SDO_RX     = makeCanId(0xC, DEVICE_ID);  // 0x601 - SDO request
constexpr uint32_t CAN_ID_HEARTBEAT  = makeCanId(0xE, DEVICE_ID);  // 0x701

// Gearbox
constexpr float GEAR_RATIO = 14.0f;   // motor turns per output turn

// Control parameters
constexpr float TORQUE_LIMIT   = 0.3f;    // Nm (at motor shaft)
constexpr float I_LIMIT        = 5.0f;    // A
constexpr uint32_t FAST_HZ     = 500;

// PD+I gains (all in motor-shaft units)
constexpr float KP = 0.3f;    // Nm/rad — gentle: 0.3 Nm per rad of motor error
constexpr float KD = 0.02f;   // Nm/(rad/s) — damping at motor shaft
constexpr float KI = 0.05f;   // Nm/(rad*s) — slow integral

// Trajectory limits (motor shaft)
constexpr float MAX_VEL   = 20.0f;   // rad/s motor (~100 deg/s output)
constexpr float MAX_ACCEL = 30.0f;   // rad/s^2 motor

// SDO parameter IDs
constexpr uint16_t PARAM_MODE       = 0x010;
constexpr uint16_t PARAM_ERROR      = 0x014;
constexpr uint16_t PARAM_FAST_HZ    = 0x00C;
constexpr uint16_t PARAM_TORQUE_LIM = 0x030;
constexpr uint16_t PARAM_I_LIM      = 0x074;
constexpr uint16_t PARAM_POSITION   = 0x134;

static double now_s() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

class MotorCAN {
    CANInterface& can_;
public:
    explicit MotorCAN(CANInterface& c) : can_(c) {}

    void sendNMT(uint8_t mode) {
        uint8_t d[2] = {mode, DEVICE_ID};
        can_.send(CAN_ID_NMT, d, 2);
    }
    void sendHeartbeat() { can_.send(CAN_ID_HEARTBEAT, nullptr, 0); }

    void sendTorque(float torque) {
        uint8_t d[8] = {};
        float zero = 0.0f;
        std::memcpy(d, &zero, 4);
        std::memcpy(d + 4, &torque, 4);
        can_.send(CAN_ID_PDO3_TX, d, 8);
    }

    bool sdoWriteFloat(uint16_t param, float val) {
        uint8_t d[8] = {};
        d[0] = 0x20;
        std::memcpy(d + 1, &param, 2);
        std::memcpy(d + 4, &val, 4);
        return can_.send(CAN_ID_SDO_RX, d, 8);
    }

    bool sdoWriteU32(uint16_t param, uint32_t val) {
        uint8_t d[8] = {};
        d[0] = 0x20;
        std::memcpy(d + 1, &param, 2);
        std::memcpy(d + 4, &val, 4);
        return can_.send(CAN_ID_SDO_RX, d, 8);
    }

    std::optional<uint32_t> sdoReadU32(uint16_t param) {
        uint8_t d[3];
        d[0] = 0x40;
        std::memcpy(d + 1, &param, 2);
        can_.send(CAN_ID_SDO_RX, d, 3);
        auto deadline = now_s() + 0.5;
        while (now_s() < deadline) {
            auto f = can_.receive(50);
            if (f && f->id == CAN_ID_SDO_TX && f->data.size() >= 4) {
                uint32_t v;
                std::memcpy(&v, f->data.data(), 4);
                return v;
            }
        }
        return std::nullopt;
    }

    std::optional<float> sdoReadFloat(uint16_t param) {
        auto v = sdoReadU32(param);
        if (!v) return std::nullopt;
        float f;
        uint32_t raw = *v;
        std::memcpy(&f, &raw, 4);
        return f;
    }

    void drain() { while (can_.receive(0)) {} }
    std::optional<CANFrame> receive(int ms) { return can_.receive(ms); }
};

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    float output_deg = (argc > 1) ? std::stof(argv[1]) : 90.0f;
    float duration   = (argc > 2) ? std::stof(argv[2]) : 5.0f;

    float motor_deg = output_deg * GEAR_RATIO;

    std::cout << "Position Test (C++) — Torque mode + trajectory planner\n"
              << "Gear ratio: " << GEAR_RATIO << ":1\n"
              << "Output target: " << output_deg << " deg\n"
              << "Motor target:  " << motor_deg << " deg\n"
              << "Duration: " << duration << " s\n";

    CANInterface can;
    if (!can.open("can0")) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }
    MotorCAN motor(can);

    // ---- Reset ----
    std::cout << "Resetting motor...\n";
    motor.sdoWriteU32(PARAM_ERROR, 0);
    motor.sendHeartbeat();
    motor.sendNMT(0x00);  // DISABLED
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for (int i = 0; i < 5; i++) {
        motor.sendHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    motor.sendNMT(0x01);  // IDLE
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for (int i = 0; i < 5; i++) {
        motor.sendHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    {
        auto m = motor.sdoReadU32(PARAM_MODE);
        auto e = motor.sdoReadU32(PARAM_ERROR);
        std::cout << "  mode=0x" << std::hex;
        if (m) std::cout << *m; else std::cout << "??";
        std::cout << ", error=0x";
        if (e) std::cout << *e; else std::cout << "??";
        std::cout << std::dec << "\n";
    }

    // ---- Configure ----
    motor.sdoWriteFloat(PARAM_I_LIM, I_LIMIT);
    motor.sdoWriteFloat(PARAM_TORQUE_LIM, TORQUE_LIMIT);
    motor.sdoWriteU32(PARAM_FAST_HZ, FAST_HZ);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // ---- Read start position (motor shaft) ----
    motor.drain();
    auto pos0 = motor.sdoReadFloat(PARAM_POSITION);
    if (!pos0) {
        std::cerr << "ERROR: Cannot read position\n";
        return 1;
    }
    float pos_start  = *pos0;
    float target_rad = pos_start + motor_deg * static_cast<float>(M_PI) / 180.0f;

    std::cout << std::fixed << std::setprecision(1)
              << "Motor start:  " << (pos_start * 180.0f / M_PI) << " deg\n"
              << "Motor target: " << (target_rad * 180.0f / M_PI) << " deg  (+"
              << motor_deg << " deg motor = " << output_deg << " deg output)\n"
              << "Gains: Kp=" << KP << " Kd=" << KD << " Ki=" << KI << "\n"
              << "Traj:  max_vel=" << MAX_VEL << " rad/s  max_accel=" << MAX_ACCEL << " rad/s^2\n\n";

    // ---- Enter TORQUE mode ----
    motor.sendTorque(0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    motor.sendNMT(0x11);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    motor.sendHeartbeat();
    motor.drain();

    {
        auto m = motor.sdoReadU32(PARAM_MODE);
        auto e = motor.sdoReadU32(PARAM_ERROR);
        std::cout << "  Torque mode: mode=0x" << std::hex;
        if (m) std::cout << *m; else std::cout << "??";
        std::cout << ", error=0x";
        if (e) std::cout << *e; else std::cout << "??";
        std::cout << std::dec << "\n";
        if (!m || *m != 0x11) {
            std::cerr << "ERROR: Failed to enter TORQUE mode!\n";
            return 1;
        }
    }

    // ---- Control loop state ----
    float prev_pos    = pos_start;
    double prev_time  = now_s();
    float vel_filt    = 0.0f;
    float integral    = 0.0f;
    float traj_pos    = pos_start;
    float traj_vel    = 0.0f;

    constexpr float VEL_ALPHA   = 0.15f;
    constexpr float INT_LIMIT   = TORQUE_LIMIT * 0.5f;

    double t0         = now_s();
    double last_print = 0.0;
    double last_hb    = 0.0;
    uint32_t frames   = 0;

    // Header: show output shaft degrees
    std::cout << std::right
              << std::setw(5)  << "time"   << "  "
              << std::setw(9)  << "out_err" << "  "
              << std::setw(9)  << "out_pos" << "  "
              << std::setw(9)  << "out_spt" << "  "
              << std::setw(9)  << "mot_vel" << "  "
              << std::setw(8)  << "torque"  << "\n"
              << std::string(62, '-') << "\n";

    // ---- Main loop ----
    while (g_running) {
        double now = now_s();
        double elapsed = now - t0;
        if (elapsed >= duration) break;

        // Heartbeat every 50ms
        if (now - last_hb >= 0.05) {
            motor.sendHeartbeat();
            last_hb = now;
        }

        auto frame = can.receive(2);  // 2ms timeout
        if (!frame) continue;
        if (frame->id != CAN_ID_FAST_FRAME || frame->data.size() < 8) continue;

        frames++;
        float pos;
        std::memcpy(&pos, frame->data.data(), 4);

        double dt = now - prev_time;
        if (dt < 0.0005) continue;
        float fdt = static_cast<float>(dt);

        // Velocity estimation (IIR filter)
        float vel_raw = (pos - prev_pos) / fdt;
        vel_filt = VEL_ALPHA * vel_raw + (1.0f - VEL_ALPHA) * vel_filt;
        prev_pos  = pos;
        prev_time = now;

        // ---- Trajectory generator (trapezoidal velocity profile) ----
        float to_go = target_rad - traj_pos;
        float dir   = (to_go > 0) ? 1.0f : -1.0f;
        float decel = traj_vel * traj_vel / (2.0f * MAX_ACCEL);

        if (std::abs(to_go) <= 0.001f && std::abs(traj_vel) < 0.1f) {
            traj_pos = target_rad;
            traj_vel = 0.0f;
        } else if (std::abs(to_go) <= decel) {
            traj_vel -= dir * MAX_ACCEL * fdt;
            if (dir * traj_vel < 0) traj_vel = 0.0f;
        } else {
            traj_vel += dir * MAX_ACCEL * fdt;
            traj_vel = std::clamp(traj_vel, -MAX_VEL, MAX_VEL);
        }
        traj_pos += traj_vel * fdt;

        // ---- PD + I controller tracking trajectory ----
        float pos_err = traj_pos - pos;
        float vel_err = traj_vel - vel_filt;

        integral += KI * pos_err * fdt;
        integral = std::clamp(integral, -INT_LIMIT, INT_LIMIT);

        float torque = KP * pos_err + KD * vel_err + integral;
        torque = std::clamp(torque, -TORQUE_LIMIT, TORQUE_LIMIT);

        motor.sendTorque(torque);

        // Print at ~4 Hz — display in output shaft degrees
        if (elapsed - last_print >= 0.25) {
            float out_pos = (pos - pos_start) / GEAR_RATIO * 180.0f / M_PI;
            float out_spt = (traj_pos - pos_start) / GEAR_RATIO * 180.0f / M_PI;
            float out_err = (pos_err) / GEAR_RATIO * 180.0f / M_PI;

            std::cout << std::fixed
                      << std::setw(4) << std::setprecision(1) << elapsed << "s  "
                      << std::showpos
                      << std::setw(7) << std::setprecision(2) << out_err << "deg  "
                      << std::noshowpos
                      << std::setw(7) << std::setprecision(1) << out_pos << "deg  "
                      << std::setw(7) << std::setprecision(1) << out_spt << "deg  "
                      << std::showpos
                      << std::setw(7) << std::setprecision(1) << vel_filt << "r/s  "
                      << std::setprecision(4) << torque << "Nm"
                      << std::noshowpos << "\n";
            last_print = elapsed;
        }
    }

    // ---- Stop ----
    std::cout << "\nStopping...\n";
    motor.sendTorque(0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    motor.sendNMT(0x02);  // DAMPING

    double total = now_s() - t0;
    std::cout << "Frames: " << frames << " ("
              << static_cast<int>(frames / std::max(total, 0.1)) << " Hz)\n";

    auto fp = motor.sdoReadFloat(PARAM_POSITION);
    if (fp) {
        float out_final = (*fp - pos_start) / GEAR_RATIO * 180.0f / M_PI;
        float out_error = output_deg - out_final;
        std::cout << std::fixed << std::setprecision(1)
                  << "Output: " << out_final << " deg  (error: "
                  << std::showpos << out_error << " deg)\n"
                  << std::noshowpos;
    }

    std::cout << "Done.\n";
    return 0;
}
