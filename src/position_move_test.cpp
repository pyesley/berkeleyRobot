/**
 * Two-stage position control test over CAN.
 *
 * Architecture:
 *   STM32 (inner loop):  Velocity PID via SimpleFOC (~10kHz)
 *   This program (outer loop): Position P-controller over CAN (~50Hz)
 *
 * The AS5600 sensor updates at ~150Hz over I2C. The motor-side SimpleFOC
 * velocity loop runs much faster using interpolated sensor readings.
 * This host-side loop reads position feedback at 50Hz and outputs
 * velocity setpoints, with acceleration limiting for smooth motion.
 *
 * Usage: ./position_move_test [options]
 *   --vel-limit <rad/s>   Max velocity (default 5.0)
 *   --move-deg <degrees>  Move distance (default 360)
 *   --kp <gain>           Position P gain (default 2.0)
 *   --accel <rad/s^2>     Acceleration limit (default 10.0)
 *   --cycles <n>          Back-and-forth cycles (default 1)
 *   --settle <seconds>    Settle time at each end (default 1.5)
 *   -i <interface>        CAN interface (default can0)
 */

#include "can_interface.hpp"
#include "motor_protocol.hpp"

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

volatile sig_atomic_t running = 1;
void signalHandler(int) { running = 0; }

static uint64_t nowMs() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
}

static double nowSec() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::microseconds>(
        now.time_since_epoch()).count() / 1e6;
}

// Clamp helper
static float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

struct Config {
    std::string can_interface = "can0";
    float vel_limit = 5.0f;     // rad/s
    float move_deg = 360.0f;    // degrees
    float kp = 2.0f;            // P gain: vel_cmd = kp * pos_error
    float accel_limit = 10.0f;  // rad/s^2
    float settle_time = 1.5f;   // seconds
    int cycles = 1;
};

static Config parseArgs(int argc, char* argv[]) {
    Config cfg;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--vel-limit" && i + 1 < argc) cfg.vel_limit = std::stof(argv[++i]);
        else if (arg == "--move-deg" && i + 1 < argc) cfg.move_deg = std::stof(argv[++i]);
        else if (arg == "--kp" && i + 1 < argc) cfg.kp = std::stof(argv[++i]);
        else if (arg == "--accel" && i + 1 < argc) cfg.accel_limit = std::stof(argv[++i]);
        else if (arg == "--cycles" && i + 1 < argc) cfg.cycles = std::stoi(argv[++i]);
        else if (arg == "--settle" && i + 1 < argc) cfg.settle_time = std::stof(argv[++i]);
        else if (arg == "-i" && i + 1 < argc) cfg.can_interface = argv[++i];
        else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "  --vel-limit <rad/s>   Max velocity (default 5.0)\n"
                      << "  --move-deg <degrees>  Move distance (default 360)\n"
                      << "  --kp <gain>           Position P gain (default 2.0)\n"
                      << "  --accel <rad/s^2>     Acceleration limit (default 10.0)\n"
                      << "  --cycles <n>          Back-and-forth cycles (default 1)\n"
                      << "  --settle <seconds>    Settle time at each end (default 1.5)\n"
                      << "  -i <interface>        CAN interface (default can0)\n";
            exit(0);
        }
    }
    return cfg;
}

// Drain all pending CAN frames, return latest feedback (position, velocity)
// Returns true if feedback was received
static bool drainAndGetLatest(CANInterface& can, MotorProtocol& motor,
                               float& pos_out, float& vel_out) {
    bool got_feedback = false;
    for (int i = 0; i < 50; i++) {
        auto frame = can.receive(i == 0 ? 1 : 0);
        if (!frame) break;
        if (motor.parseFrame(*frame)) {
            pos_out = motor.getState().position_rad;
            vel_out = motor.getState().velocity_rad_s;
            got_feedback = true;
        }
    }
    return got_feedback;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    Config cfg = parseArgs(argc, argv);
    float move_rad = cfg.move_deg * M_PI / 180.0f;

    std::cout << "=== Position Move Test (C++) ===" << std::endl;
    std::cout << "  Move: " << cfg.move_deg << " deg (" << move_rad << " rad)" << std::endl;
    std::cout << "  Vel limit: " << cfg.vel_limit << " rad/s" << std::endl;
    std::cout << "  Kp: " << cfg.kp << std::endl;
    std::cout << "  Accel limit: " << cfg.accel_limit << " rad/s^2" << std::endl;
    std::cout << "  Cycles: " << cfg.cycles << std::endl;
    std::cout << std::endl;

    // Open CAN
    CANInterface can;
    if (!can.open(cfg.can_interface)) {
        std::cerr << "Failed to open CAN interface" << std::endl;
        return 1;
    }

    MotorProtocol motor(can);

    // Wake up motor with heartbeats
    std::cout << "Connecting to motor..." << std::endl;
    for (int i = 0; i < 10; i++) {
        motor.sendHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Read initial position
    std::cout << "Reading initial position..." << std::endl;
    float pos = 0, vel = 0;
    bool got_pos = false;
    for (int attempt = 0; attempt < 20 && !got_pos; attempt++) {
        motor.sendHeartbeat();
        got_pos = drainAndGetLatest(can, motor, pos, vel);
        if (!got_pos)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!got_pos) {
        std::cerr << "ERROR: No feedback from motor. Check CAN connection." << std::endl;
        return 1;
    }

    float pos_start = pos;
    std::cout << "  Start position: " << (pos_start * 180.0 / M_PI) << " deg" << std::endl;

    // Enable velocity mode
    std::cout << "Enabling velocity mode..." << std::endl;
    {
        // Send zero velocity, then enable
        uint8_t vel_data[8] = {};
        float zero = 0.0f;
        std::memcpy(vel_data, &zero, 4);
        can.send(CAN_ID_PDO2_TX, vel_data, 8);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // NMT: MODE_VELOCITY = 0x12
    motor.setMode(0x12);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    motor.sendHeartbeat();

    std::cout << "  Motor enabled" << std::endl;
    std::cout << std::endl;

    // Control state
    float current_vel_cmd = 0.0f;
    float pos_target = pos_start;
    float integral = 0.0f;
    const float ki = 0.5f;              // Integral gain — builds velocity when stuck
    const float kd = 0.05f;             // Derivative gain — light damping
    const float integral_limit = 2.0f;  // Clamp integral windup (rad/s)
    const float friction_ff = 0.8f;     // Min velocity to overcome static friction (rad/s)

    // Build move list: forward, back (repeated)
    struct Move {
        std::string name;
        float target;
    };
    std::vector<Move> moves;
    for (int i = 0; i < cfg.cycles; i++) {
        moves.push_back({"forward", pos_start + move_rad});
        moves.push_back({"reverse", pos_start});
    }

    // Print header
    printf("%5s  %9s  %9s  %8s  %8s  %8s\n",
           "time", "target", "actual", "error", "vel_cmd", "vel_act");
    printf("--------------------------------------------------------------\n");

    uint64_t last_hb_ms = 0;
    double last_print_time = 0;
    double last_ctrl_time = nowSec();

    try {
        for (auto& move : moves) {
            if (!running) break;

            pos_target = move.target;
            integral = 0.0f;  // Reset integral for each new move
            printf("\n--- %s: target = %.1f deg ---\n",
                   move.name.c_str(), pos_target * 180.0 / M_PI);

            double move_start = nowSec();
            double settle_start = -1;
            bool settled = false;

            while (running && !settled) {
                uint64_t now_ms = nowMs();
                double now = nowSec();

                // Heartbeat every 80ms
                if (now_ms - last_hb_ms >= 80) {
                    motor.sendHeartbeat();
                    last_hb_ms = now_ms;
                }

                // Get latest feedback
                bool got = drainAndGetLatest(can, motor, pos, vel);
                if (!got) {
                    // Blocking wait for one frame
                    auto frame = can.receive(5);
                    if (frame) {
                        motor.parseFrame(*frame);
                        pos = motor.getState().position_rad;
                        vel = motor.getState().velocity_rad_s;
                        got = true;
                    }
                }
                if (!got) continue;

                // Compute dt from last control update
                double dt = now - last_ctrl_time;
                last_ctrl_time = now;
                if (dt <= 0 || dt > 0.1) dt = 0.02;  // Sanity clamp

                // PID position controller → outputs velocity setpoint
                // (Same structure as SimpleFOC angle mode cascade)
                float pos_error = pos_target - pos;
                float abs_error = std::fabs(pos_error);

                // Integral: accumulates when position error persists (overcomes friction)
                integral += ki * pos_error * (float)dt;
                integral = clamp(integral, -integral_limit, integral_limit);
                // Anti-windup: reset integral when error sign flips (overshoot)
                if (pos_error * integral < 0) integral *= 0.5f;

                // PID output
                float vel_desired = cfg.kp * pos_error + integral - kd * vel;

                // Friction feedforward: ensure minimum velocity when there's error
                // This overcomes gearbox static friction dead zone
                float dead_zone = 3.0f * M_PI / 180.0f;  // 3 degrees
                if (abs_error > dead_zone) {
                    float sign = pos_error > 0 ? 1.0f : -1.0f;
                    if (std::fabs(vel_desired) < friction_ff) {
                        vel_desired = sign * friction_ff;
                    }
                }

                // Clamp velocity command
                vel_desired = clamp(vel_desired, -cfg.vel_limit, cfg.vel_limit);

                // Rate-limit acceleration for smooth motion
                float max_dv = cfg.accel_limit * (float)dt;
                float dv = vel_desired - current_vel_cmd;
                if (dv > max_dv) dv = max_dv;
                if (dv < -max_dv) dv = -max_dv;
                current_vel_cmd += dv;

                // Send velocity command
                {
                    uint8_t data[8] = {};
                    std::memcpy(data, &current_vel_cmd, 4);
                    can.send(CAN_ID_PDO2_TX, data, 8);
                }

                // Print at ~4Hz
                if (now - last_print_time >= 0.25) {
                    printf("%5.1fs  %+8.1fd  %+8.1fd  %+7.1fd  %+7.2fr/s  %+7.2fr/s\n",
                           now - move_start,
                           pos_target * 180.0 / M_PI,
                           pos * 180.0 / M_PI,
                           pos_error * 180.0 / M_PI,
                           current_vel_cmd, vel);
                    fflush(stdout);
                    last_print_time = now;
                }

                // Check settled: within 5 deg and low velocity
                // (AS5600 through gearbox has ~5 deg quantization)
                if (std::fabs(pos_error) < (5.0 * M_PI / 180.0) &&
                    std::fabs(vel) < 0.5f) {
                    if (settle_start < 0) settle_start = now;
                    else if (now - settle_start >= cfg.settle_time) settled = true;
                } else {
                    settle_start = -1;
                }

                // Timeout
                if (now - move_start > 15) {
                    printf("  TIMEOUT at %.1f deg (error: %+.1f deg)\n",
                           pos * 180.0 / M_PI,
                           (pos_target - pos) * 180.0 / M_PI);
                    break;
                }
            }

            if (settled) {
                printf("  Settled at %.1f deg (error: %+.1f deg)\n",
                       pos * 180.0 / M_PI,
                       (pos_target - pos) * 180.0 / M_PI);
            }
        }
    } catch (...) {
        // Fall through to cleanup
    }

    // Stop motor
    std::cout << "\nStopping motor..." << std::endl;
    {
        uint8_t data[8] = {};
        float zero = 0.0f;
        std::memcpy(data, &zero, 4);
        can.send(CAN_ID_PDO2_TX, data, 8);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        can.send(CAN_ID_PDO2_TX, data, 8);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Disable motor
    motor.setMode(0x00);  // MODE_DISABLED
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Final position
    motor.sendHeartbeat();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (drainAndGetLatest(can, motor, pos, vel)) {
        printf("Final position: %.1f deg\n", pos * 180.0 / M_PI);
    }

    std::cout << "Done." << std::endl;
    return 0;
}
