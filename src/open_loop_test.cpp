/**
 * Open-loop motor controller via TORQUE mode with CAN feedback.
 *
 * Motor has 11:1 gearbox. Encoder is on the motor shaft.
 * Angles specified as OUTPUT shaft degrees.
 *
 * Subcommands:
 *   ramp  <torque> <seconds>         — constant torque, measure response
 *   goto  <output_deg> [torque=0.05] — smart open-loop move + fine correction
 *
 * The goto command:
 *   Phase 1 (ACCEL): apply constant torque, measure acceleration rate
 *   Phase 2 (DECEL): brake when remaining distance <= stopping distance
 *     stopping dist estimated from measured accel (friction assists braking)
 *   Phase 3 (SETTLE): zero torque, wait for motor to stop
 *   Phase 4 (CORRECT): gentle nudge-and-wait to close final error
 *
 * Usage: ./open_loop_test goto 90
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
#include <string>

volatile sig_atomic_t g_running = 1;
void signalHandler(int) { g_running = 0; }

bool json_mode = false;

constexpr uint8_t DEVICE_ID = 1;
constexpr uint32_t makeCanId(uint32_t func, uint32_t dev) { return (func << 7) | dev; }

constexpr uint32_t CAN_ID_NMT        = makeCanId(0x0, 0);
constexpr uint32_t CAN_ID_PDO3_TX    = makeCanId(0x8, DEVICE_ID);
constexpr uint32_t CAN_ID_FAST_FRAME = makeCanId(0x9, DEVICE_ID);
constexpr uint32_t CAN_ID_SDO_TX     = makeCanId(0xB, DEVICE_ID);
constexpr uint32_t CAN_ID_SDO_RX     = makeCanId(0xC, DEVICE_ID);
constexpr uint32_t CAN_ID_HEARTBEAT  = makeCanId(0xE, DEVICE_ID);

constexpr float GEAR_RATIO = 11.0f;
constexpr float DEG2RAD    = M_PI / 180.0f;
constexpr float RAD2DEG    = 180.0f / M_PI;

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
        uint8_t d[3] = {0x40};
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
        float f; uint32_t raw = *v;
        std::memcpy(&f, &raw, 4);
        return f;
    }
    void drain() { while (can_.receive(0)) {} }
    std::optional<CANFrame> receive(int ms) { return can_.receive(ms); }
};

bool initMotor(MotorCAN& motor) {
    if (!json_mode) std::cout << "Resetting motor...\n";
    motor.sdoWriteU32(PARAM_ERROR, 0);
    motor.sendHeartbeat();
    motor.sendNMT(0x00);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for (int i = 0; i < 5; i++) {
        motor.sendHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    motor.sendNMT(0x01);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for (int i = 0; i < 5; i++) {
        motor.sendHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    auto m = motor.sdoReadU32(PARAM_MODE);
    auto e = motor.sdoReadU32(PARAM_ERROR);
    if (!json_mode) {
        std::cout << "  mode=0x" << std::hex;
        if (m) std::cout << *m; else std::cout << "??";
        std::cout << ", error=0x";
        if (e) std::cout << *e; else std::cout << "??";
        std::cout << std::dec << "\n";
    }

    motor.sdoWriteFloat(PARAM_I_LIM, 5.0f);
    motor.sdoWriteFloat(PARAM_TORQUE_LIM, 1.0f);
    motor.sdoWriteU32(PARAM_FAST_HZ, 500);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    motor.sendTorque(0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    motor.sendNMT(0x11);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    motor.sendHeartbeat();
    motor.drain();

    m = motor.sdoReadU32(PARAM_MODE);
    e = motor.sdoReadU32(PARAM_ERROR);
    if (!json_mode) {
        std::cout << "  Torque mode: mode=0x" << std::hex;
        if (m) std::cout << *m; else std::cout << "??";
        std::cout << ", error=0x";
        if (e) std::cout << *e; else std::cout << "??";
        std::cout << std::dec << "\n";
    }

    return m && *m == 0x11;
}

void stopMotor(MotorCAN& motor) {
    motor.sendTorque(0.0f);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    motor.sendNMT(0x02);
}

// Wait for a fast frame and return the position, keeping heartbeat alive.
// Returns nullopt on timeout.
std::optional<float> waitPosition(CANInterface& can, MotorCAN& motor, double timeout_s = 0.5) {
    double deadline = now_s() + timeout_s;
    double last_hb = 0;
    while (now_s() < deadline && g_running) {
        double now = now_s();
        if (now - last_hb >= 0.05) { motor.sendHeartbeat(); last_hb = now; }
        auto f = can.receive(5);
        if (f && f->id == CAN_ID_FAST_FRAME && f->data.size() >= 8) {
            float pos;
            std::memcpy(&pos, f->data.data(), 4);
            return pos;
        }
    }
    return std::nullopt;
}

// ---- ramp command ----
int cmd_ramp(CANInterface& can, MotorCAN& motor, float torque, float seconds) {
    std::cout << "\n=== RAMP: " << torque << " Nm for " << seconds << " s ===\n";
    if (!initMotor(motor)) return 1;

    motor.drain();
    auto pos0 = motor.sdoReadFloat(PARAM_POSITION);
    if (!pos0) { std::cerr << "Cannot read position\n"; return 1; }
    float pos_start = *pos0;
    std::cout << "Start: " << std::fixed << std::setprecision(1)
              << (pos_start * RAD2DEG) << " deg motor\n\n";

    float prev_pos = pos_start;
    double prev_time = now_s();
    float vel_filt = 0.0f;
    double t0 = now_s();
    double last_hb = 0, last_print = 0;
    float vel_at_100ms = 0;

    while (g_running && (now_s() - t0 < seconds)) {
        double now = now_s();
        double elapsed = now - t0;
        if (now - last_hb >= 0.05) { motor.sendHeartbeat(); last_hb = now; }

        auto frame = can.receive(2);
        if (!frame || frame->id != CAN_ID_FAST_FRAME || frame->data.size() < 8) continue;

        motor.sendTorque(torque);

        float pos;
        std::memcpy(&pos, frame->data.data(), 4);
        double dt = now - prev_time;
        if (dt < 0.0005) continue;
        float vel_raw = (pos - prev_pos) / static_cast<float>(dt);
        vel_filt = 0.15f * vel_raw + 0.85f * vel_filt;
        prev_pos = pos; prev_time = now;

        if (elapsed >= 0.1 && vel_at_100ms == 0) vel_at_100ms = vel_filt;

        if (elapsed - last_print >= 0.25) {
            float mot_deg = (pos - pos_start) * RAD2DEG;
            std::cout << std::fixed << std::setprecision(2) << elapsed << "s  "
                      << "mot:" << std::showpos << std::setprecision(1) << mot_deg << "deg  "
                      << "vel:" << vel_filt << "r/s  "
                      << std::noshowpos << "\n";
            last_print = elapsed;
        }
    }

    stopMotor(motor);
    float accel = vel_at_100ms / 0.1f;
    float inertia = (std::abs(accel) > 1) ? std::abs(torque / accel) : 0;
    std::cout << "\nAccel: " << accel << " rad/s^2"
              << "  Terminal vel: " << vel_filt << " rad/s"
              << "  Inertia: " << inertia * 1000 << " g*m^2\n";
    return 0;
}

// ---- goto command ----
int cmd_goto(CANInterface& can, MotorCAN& motor, float output_deg, float torque) {
    float motor_rad = output_deg * GEAR_RATIO * DEG2RAD;
    float dir = (motor_rad > 0) ? 1.0f : -1.0f;

    if (!json_mode) {
        std::cout << "\n=== GOTO: " << output_deg << " deg output ("
                  << (output_deg * GEAR_RATIO) << " deg motor) at "
                  << torque << " Nm ===\n";
    }

    if (!initMotor(motor)) return 1;

    motor.drain();
    auto pos0 = motor.sdoReadFloat(PARAM_POSITION);
    if (!pos0) { std::cerr << "Cannot read position\n"; return 1; }
    float pos_start = *pos0;
    float target_rad = pos_start + motor_rad;

    if (!json_mode) {
        std::cout << std::fixed << std::setprecision(1)
                  << "Start:  " << (pos_start * RAD2DEG / GEAR_RATIO) << " deg output\n"
                  << "Target: " << (target_rad * RAD2DEG / GEAR_RATIO) << " deg output  ("
                  << std::showpos << output_deg << " deg)" << std::noshowpos << "\n\n";
    }

    // ---- State ----
    enum Phase { ACCEL, DECEL, SETTLE, CORRECT, DONE };
    Phase phase = ACCEL;

    float prev_pos = pos_start;
    double prev_time = now_s();
    float vel_filt = 0.0f;
    float measured_accel = 0.0f;  // measured during accel phase
    double t0 = now_s();
    double last_hb = 0, last_print = 0;
    double phase_start = t0;
    uint32_t frames = 0;
    int nudge_count = 0;
    constexpr int MAX_NUDGES = 30;
    constexpr float OUTPUT_TOLERANCE = 0.5f;  // degrees at output shaft
    constexpr float MOTOR_TOLERANCE = OUTPUT_TOLERANCE * GEAR_RATIO * DEG2RAD;
    constexpr float NUDGE_TORQUE = 0.06f;  // Nm — well above static friction
    constexpr float NUDGE_DURATION = 0.040f;  // 40ms torque pulse
    constexpr float NUDGE_SETTLE = 0.15f;  // 150ms settle after each nudge
    double nudge_end = 0;  // when current nudge pulse ends
    bool nudging = false;

    while (g_running) {
        double now = now_s();
        double elapsed = now - t0;
        if (elapsed >= 15.0) {
            if (!json_mode) std::cout << "  TIMEOUT\n";
            break;
        }

        if (now - last_hb >= 0.05) { motor.sendHeartbeat(); last_hb = now; }

        auto frame = can.receive(2);
        if (!frame || frame->id != CAN_ID_FAST_FRAME || frame->data.size() < 8) continue;

        frames++;
        float pos;
        std::memcpy(&pos, frame->data.data(), 4);

        double dt = now - prev_time;
        if (dt < 0.0005) continue;
        float fdt = static_cast<float>(dt);
        float vel_raw = (pos - prev_pos) / fdt;
        vel_filt = 0.15f * vel_raw + 0.85f * vel_filt;
        prev_pos = pos;
        prev_time = now;

        float traveled = (pos - pos_start) * dir;       // positive = toward target
        float remaining = std::abs(motor_rad) - traveled;
        float speed = std::abs(vel_filt);
        float error_rad = target_rad - pos;
        float error_out_deg = error_rad / GEAR_RATIO * RAD2DEG;

        // ---- Phase logic ----
        float cmd_torque = 0.0f;

        switch (phase) {
        case ACCEL: {
            cmd_torque = torque * dir;

            // Measure accel after 100ms of acceleration
            double accel_elapsed = now - phase_start;
            if (accel_elapsed >= 0.1 && measured_accel == 0.0f && speed > 1.0f) {
                measured_accel = speed / static_cast<float>(accel_elapsed);
            }

            // Check if we should start braking
            // Decel rate ≈ 3.5x accel rate (friction strongly assists braking)
            float decel_rate = (measured_accel > 10.0f) ? measured_accel * 3.5f : 500.0f;
            float stop_dist = speed * speed / (2.0f * decel_rate);

            if (remaining <= stop_dist * 1.05f && speed > 2.0f) {
                phase = DECEL;
                phase_start = now;
                if (!json_mode) {
                    std::cout << "  >> DECEL  traveled=" << std::fixed << std::setprecision(1)
                              << (traveled * RAD2DEG / GEAR_RATIO) << " out_deg"
                              << "  vel=" << vel_filt << " r/s"
                              << "  stop_dist=" << (stop_dist * RAD2DEG / GEAR_RATIO) << " deg\n";
                }
            }
            break;
        }
        case DECEL: {
            cmd_torque = -torque * dir;

            // Transition to settle when velocity is near zero
            if (speed < 1.0f) {
                phase = SETTLE;
                phase_start = now;
                if (!json_mode) {
                    std::cout << "  >> SETTLE vel=" << vel_filt << " r/s"
                              << "  pos=" << (error_out_deg > 0 ? "short" : "past")
                              << " by " << std::abs(error_out_deg) << " deg\n";
                }
            }
            break;
        }
        case SETTLE: {
            cmd_torque = 0.0f;

            // Wait 300ms for everything to come to rest
            if (now - phase_start >= 0.3) {
                if (std::abs(error_out_deg) <= OUTPUT_TOLERANCE) {
                    phase = DONE;
                    if (!json_mode) {
                        std::cout << "  >> DONE  error=" << std::showpos
                                  << error_out_deg << " deg (within tolerance)\n"
                                  << std::noshowpos;
                    }
                } else {
                    phase = CORRECT;
                    phase_start = now;
                    nudging = false;
                    if (!json_mode) {
                        std::cout << "  >> CORRECT  error=" << std::showpos
                                  << error_out_deg << " deg\n" << std::noshowpos;
                    }
                }
            }
            break;
        }
        case CORRECT: {
            if (std::abs(error_out_deg) <= OUTPUT_TOLERANCE || nudge_count >= MAX_NUDGES) {
                phase = DONE;
                if (!json_mode) {
                    std::cout << "  >> DONE  error=" << std::showpos
                              << error_out_deg << " deg"
                              << " (" << nudge_count << " nudges)\n" << std::noshowpos;
                }
                cmd_torque = 0.0f;
                break;
            }

            if (!nudging) {
                // Start a new nudge pulse — full torque to overcome friction
                float nudge_dir = (error_rad > 0) ? 1.0f : -1.0f;
                cmd_torque = NUDGE_TORQUE * nudge_dir;
                nudge_end = now + NUDGE_DURATION;
                nudging = true;
                nudge_count++;
            } else if (now < nudge_end) {
                // Still in nudge pulse
                float nudge_dir = (error_rad > 0) ? 1.0f : -1.0f;
                cmd_torque = NUDGE_TORQUE * nudge_dir;
            } else if (now < nudge_end + NUDGE_SETTLE) {
                // Settle period after nudge — zero torque
                cmd_torque = 0.0f;
            } else {
                // Settle period over — ready for next nudge
                nudging = false;
                cmd_torque = 0.0f;
            }
            break;
        }
        case DONE:
            cmd_torque = 0.0f;
            break;
        }

        motor.sendTorque(cmd_torque);

        if (phase == DONE) {
            // Read final position from fast-frame before stopping
            // (SDO read after mode switch can show drift)
            float out_final = (pos - pos_start) / GEAR_RATIO * RAD2DEG;
            float out_error = output_deg - out_final;

            stopMotor(motor);

            double total = now_s() - t0;
            if (json_mode) {
                std::cout << std::fixed
                          << "{\"type\":\"result\""
                          << ",\"out\":" << std::setprecision(2) << out_final
                          << ",\"target\":" << std::setprecision(2) << output_deg
                          << ",\"error\":" << std::setprecision(2) << out_error
                          << ",\"nudges\":" << nudge_count
                          << ",\"time\":" << std::setprecision(1) << total
                          << "}" << std::endl;
            } else {
                std::cout << "\nResult:\n"
                          << std::fixed << std::setprecision(2)
                          << "  Output:  " << out_final << " deg\n"
                          << "  Target:  " << output_deg << " deg\n"
                          << "  Error:   " << std::showpos << out_error << " deg\n" << std::noshowpos
                          << "  Nudges:  " << nudge_count << "\n"
                          << "  Time:    " << std::setprecision(1) << total << " s\n"
                          << "  Frames:  " << frames << " (" << (int)(frames / std::max(total, 0.1)) << " Hz)\n";
            }
            return 0;
        }

        // JSON: emit every fast-frame sample
        if (json_mode) {
            float out_pos = (pos - pos_start) / GEAR_RATIO * RAD2DEG;
            const char* pnames[] = {"ACCEL", "DECEL", "SETTLE", "CORRECT", "DONE"};
            std::cout << std::fixed
                      << "{\"t\":" << std::setprecision(3) << elapsed
                      << ",\"phase\":\"" << pnames[phase] << "\""
                      << ",\"out\":" << std::setprecision(2) << out_pos
                      << ",\"target\":" << std::setprecision(2) << output_deg
                      << ",\"vel\":" << std::setprecision(1) << vel_filt
                      << ",\"torque\":" << std::setprecision(4) << cmd_torque
                      << "}" << std::endl;
        } else if (elapsed - last_print >= 0.25) {
            // Print at ~4 Hz
            float out_pos = (pos - pos_start) / GEAR_RATIO * RAD2DEG;
            const char* pname[] = {"ACCEL", "DECEL", "SETTL", "CORR ", "DONE "};
            std::cout << std::fixed
                      << std::setw(5) << std::setprecision(2) << elapsed << "s  "
                      << pname[phase] << "  "
                      << "out:" << std::showpos
                      << std::setw(7) << std::setprecision(2) << out_pos << "deg  "
                      << "err:" << std::setw(6) << std::setprecision(2) << error_out_deg << "deg  "
                      << "vel:" << std::setw(7) << std::setprecision(1) << vel_filt << "r/s  "
                      << "tau:" << std::setprecision(4) << cmd_torque << "Nm"
                      << std::noshowpos << "\n";
            last_print = elapsed;
        }
    }

    // Timeout or interrupted
    stopMotor(motor);
    if (!json_mode) std::cout << "  Stopped (timeout or interrupt)\n";
    return 1;
}

void usage(const char* prog) {
    std::cout << "Usage:\n"
              << "  " << prog << " ramp <torque_Nm> <seconds>\n"
              << "      Apply constant torque, measure acceleration\n\n"
              << "  " << prog << " goto <output_degrees> [torque=0.05] [--json]\n"
              << "      Smart open-loop move with fine correction\n"
              << "      Handles any angle, auto-detects braking point\n"
              << "      --json: output JSON lines for GUI integration\n";
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    if (argc < 2) { usage(argv[0]); return 1; }

    // Check for --json flag anywhere in args and remove it
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) == "--json") {
            json_mode = true;
            // Shift remaining args down
            for (int j = i; j < argc - 1; j++) argv[j] = argv[j + 1];
            argc--;
            i--;  // re-check this position
        }
    }

    std::string cmd = argv[1];

    CANInterface can;
    if (!can.open("can0")) {
        std::cerr << "Failed to open CAN\n";
        return 1;
    }
    MotorCAN motor(can);

    if (cmd == "ramp" && argc >= 4) {
        return cmd_ramp(can, motor, std::stof(argv[2]), std::stof(argv[3]));
    } else if (cmd == "goto" && argc >= 3) {
        float torque = (argc > 3) ? std::stof(argv[3]) : 0.05f;
        return cmd_goto(can, motor, std::stof(argv[2]), torque);
    } else {
        usage(argv[0]);
        return 1;
    }
}
