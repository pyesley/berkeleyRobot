/**
 * Position control test over CAN using SimpleFOC angle mode.
 *
 * Architecture:
 *   STM32 (full cascade): Position P → Velocity PID → Voltage (SimpleFOC)
 *   This program: sends position targets over CAN, monitors feedback.
 *
 * SimpleFOC handles all the control internally at full sensor rate (~150Hz AS5600).
 * We just tell it where to go.
 *
 * Usage: ./position_move_test [options]
 *   --vel-limit <rad/s>   Velocity limit sent to motor (default 10.0)
 *   --move-deg <degrees>  Move distance (default 360)
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
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

static double nowSec() {
    return std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count() / 1e6;
}

struct Config {
    std::string can_interface = "can0";
    float vel_limit = 10.0f;    // rad/s — sent to motor as velocity_limit
    float move_deg = 360.0f;
    float settle_time = 1.5f;
    int cycles = 1;
};

static Config parseArgs(int argc, char* argv[]) {
    Config cfg;
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--vel-limit" && i + 1 < argc) cfg.vel_limit = std::stof(argv[++i]);
        else if (arg == "--move-deg" && i + 1 < argc) cfg.move_deg = std::stof(argv[++i]);
        else if (arg == "--cycles" && i + 1 < argc) cfg.cycles = std::stoi(argv[++i]);
        else if (arg == "--settle" && i + 1 < argc) cfg.settle_time = std::stof(argv[++i]);
        else if (arg == "-i" && i + 1 < argc) cfg.can_interface = argv[++i];
        else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "  --vel-limit <rad/s>   Velocity limit (default 10.0)\n"
                      << "  --move-deg <degrees>  Move distance (default 360)\n"
                      << "  --cycles <n>          Back-and-forth cycles (default 1)\n"
                      << "  --settle <seconds>    Settle time (default 1.5)\n"
                      << "  -i <interface>        CAN interface (default can0)\n";
            exit(0);
        }
    }
    return cfg;
}

// Send position target + velocity limit over CAN
static void sendPositionTarget(CANInterface& can, float pos_rad, float vel_limit) {
    uint8_t data[8];
    std::memcpy(data, &pos_rad, 4);
    std::memcpy(data + 4, &vel_limit, 4);
    can.send(CAN_ID_PDO2_TX, data, 8);
}

// Drain CAN frames, return latest feedback
static bool drainAndGetLatest(CANInterface& can, MotorProtocol& motor,
                               float& pos_out, float& vel_out) {
    bool got = false;
    for (int i = 0; i < 50; i++) {
        auto frame = can.receive(i == 0 ? 1 : 0);
        if (!frame) break;
        if (motor.parseFrame(*frame)) {
            pos_out = motor.getState().position_rad;
            vel_out = motor.getState().velocity_rad_s;
            got = true;
        }
    }
    return got;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    Config cfg = parseArgs(argc, argv);
    float move_rad = cfg.move_deg * M_PI / 180.0f;

    std::cout << "=== Position Move Test (SimpleFOC Angle Mode) ===" << std::endl;
    std::cout << "  Move: " << cfg.move_deg << " deg (" << move_rad << " rad)" << std::endl;
    std::cout << "  Vel limit: " << cfg.vel_limit << " rad/s" << std::endl;
    std::cout << "  Cycles: " << cfg.cycles << std::endl;
    std::cout << std::endl;

    CANInterface can;
    if (!can.open(cfg.can_interface)) {
        std::cerr << "Failed to open CAN interface" << std::endl;
        return 1;
    }
    MotorProtocol motor(can);

    // Wake up with heartbeats
    std::cout << "Connecting to motor..." << std::endl;
    for (int i = 0; i < 10; i++) {
        motor.sendHeartbeat();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Read initial position
    float pos = 0, vel = 0;
    bool got_pos = false;
    for (int i = 0; i < 20 && !got_pos; i++) {
        motor.sendHeartbeat();
        got_pos = drainAndGetLatest(can, motor, pos, vel);
        if (!got_pos) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (!got_pos) {
        std::cerr << "ERROR: No feedback. Check CAN." << std::endl;
        return 1;
    }

    float pos_start = pos;
    std::cout << "  Start position: " << (pos_start * 180.0 / M_PI) << " deg" << std::endl;

    // Enable position mode (NMT MODE_POSITION = 0x13)
    std::cout << "Enabling position mode..." << std::endl;
    sendPositionTarget(can, pos_start, cfg.vel_limit);  // Hold current position
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    motor.setMode(0x13);  // MODE_POSITION
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    motor.sendHeartbeat();
    std::cout << "  Motor enabled in angle mode" << std::endl << std::endl;

    // Build moves
    struct Move { std::string name; float target; };
    std::vector<Move> moves;
    for (int i = 0; i < cfg.cycles; i++) {
        moves.push_back({"forward", pos_start + move_rad});
        moves.push_back({"reverse", pos_start});
    }

    printf("%5s  %9s  %9s  %8s  %8s\n", "time", "target", "actual", "error", "vel");
    printf("---------------------------------------------------\n");

    uint64_t last_hb_ms = 0;
    double last_print_time = 0;

    try {
        for (auto& move : moves) {
            if (!running) break;

            float pos_target = move.target;
            printf("\n--- %s: target = %.1f deg ---\n",
                   move.name.c_str(), pos_target * 180.0 / M_PI);

            // Send position target to motor
            sendPositionTarget(can, pos_target, cfg.vel_limit);

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

                // Resend position target every 200ms (keep watchdog alive)
                static double last_cmd_time = 0;
                if (now - last_cmd_time >= 0.2) {
                    sendPositionTarget(can, pos_target, cfg.vel_limit);
                    last_cmd_time = now;
                }

                // Read feedback
                bool got = drainAndGetLatest(can, motor, pos, vel);
                if (!got) {
                    auto frame = can.receive(5);
                    if (frame) {
                        motor.parseFrame(*frame);
                        pos = motor.getState().position_rad;
                        vel = motor.getState().velocity_rad_s;
                        got = true;
                    }
                }
                if (!got) continue;

                float pos_error = pos_target - pos;

                // Print at ~4Hz
                if (now - last_print_time >= 0.25) {
                    printf("%5.1fs  %+8.1fd  %+8.1fd  %+7.1fd  %+7.2fr/s\n",
                           now - move_start,
                           pos_target * 180.0 / M_PI,
                           pos * 180.0 / M_PI,
                           pos_error * 180.0 / M_PI,
                           vel);
                    fflush(stdout);
                    last_print_time = now;
                }

                // Settled: within 3 deg and velocity under 0.5 rad/s
                if (std::fabs(pos_error) < (3.0 * M_PI / 180.0) &&
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
                           pos_error * 180.0 / M_PI);
                    break;
                }
            }

            if (settled) {
                float err = pos_target - pos;
                printf("  Settled at %.1f deg (error: %+.1f deg)\n",
                       pos * 180.0 / M_PI, err * 180.0 / M_PI);
            }
        }
    } catch (...) {}

    // Stop
    std::cout << "\nStopping motor..." << std::endl;
    motor.setMode(0x00);  // DISABLED
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    motor.sendHeartbeat();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (drainAndGetLatest(can, motor, pos, vel)) {
        printf("Final position: %.1f deg\n", pos * 180.0 / M_PI);
    }

    std::cout << "Done." << std::endl;
    return 0;
}
