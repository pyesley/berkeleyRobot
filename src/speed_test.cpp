/**
 * Speed test for SimpleFOC firmware over CAN.
 *
 * Matches the on-board protocol:
 *   - NMT  (0x001): [mode, device_id] — 0x13=position, 0x00=disabled
 *   - PDO2 TX (0x301): host→motor [position_f32, velocity_limit_f32]
 *   - PDO2 RX (0x281): motor→host [position_f32, velocity_f32]
 *   - Heartbeat (0x701): keepalive (0 bytes), 500ms timeout on motor side
 *
 * Usage:
 *   ./speed_test [move_deg] [vel_limit] [cycles]
 *
 *   move_deg   — degrees to move each direction (default 2520 = 7 turns)
 *   vel_limit  — velocity limit in rad/s (default 10)
 *   cycles     — number of back-and-forth cycles (default 1)
 */

#include "can_interface.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <cmath>
#include <cstring>
#include <iomanip>

volatile sig_atomic_t g_running = 1;
void signalHandler(int) { g_running = 0; }

// Protocol IDs (must match SimpleFOC firmware)
static constexpr uint32_t CAN_ID_NMT       = 0x001;
static constexpr uint32_t CAN_ID_PDO2_TX   = 0x301;  // host → motor
static constexpr uint32_t CAN_ID_PDO2_RX   = 0x281;  // motor → host
static constexpr uint32_t CAN_ID_HEARTBEAT = 0x701;
static constexpr uint8_t  DEVICE_ID        = 1;

// Modes
static constexpr uint8_t MODE_DISABLED = 0x00;
static constexpr uint8_t MODE_POSITION = 0x13;

static double now_s() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

class SimpleFOCMotor {
    CANInterface& can_;
    float position_ = 0.0f;
    float velocity_ = 0.0f;
    bool  has_feedback_ = false;

public:
    explicit SimpleFOCMotor(CANInterface& c) : can_(c) {}

    void sendHeartbeat() {
        can_.send(CAN_ID_HEARTBEAT, nullptr, 0);
    }

    void sendNMT(uint8_t mode) {
        uint8_t d[2] = {mode, DEVICE_ID};
        can_.send(CAN_ID_NMT, d, 2);
    }

    void sendPositionCommand(float position_rad, float vel_limit) {
        uint8_t d[8];
        std::memcpy(d, &position_rad, 4);
        std::memcpy(d + 4, &vel_limit, 4);
        can_.send(CAN_ID_PDO2_TX, d, 8);
    }

    // Drain and process all pending CAN frames, return true if we got feedback
    bool pollFeedback(int timeout_ms = 1) {
        bool got_any = false;
        for (int i = 0; i < 20; i++) {
            auto frame = can_.receive(i == 0 ? timeout_ms : 0);
            if (!frame) break;
            if (frame->id == CAN_ID_PDO2_RX && frame->data.size() >= 8) {
                std::memcpy(&position_, frame->data.data(), 4);
                std::memcpy(&velocity_, frame->data.data() + 4, 4);
                has_feedback_ = true;
                got_any = true;
            }
        }
        return got_any;
    }

    float position() const { return position_; }
    float velocity() const { return velocity_; }
    bool  hasFeedback() const { return has_feedback_; }
};

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    float move_deg   = (argc > 1) ? std::stof(argv[1]) : 2520.0f;
    float vel_limit  = (argc > 2) ? std::stof(argv[2]) : 10.0f;
    int   cycles     = (argc > 3) ? std::stoi(argv[3]) : 1;
    float settle_s   = 1.5f;

    float move_rad = move_deg * static_cast<float>(M_PI) / 180.0f;

    std::cout << "SimpleFOC Speed Test\n"
              << "  Move: " << move_deg << " deg (" << move_rad << " rad)\n"
              << "  Vel limit: " << vel_limit << " rad/s\n"
              << "  Cycles: " << cycles << "\n"
              << "  Settle time: " << settle_s << " s\n\n";

    // Open CAN
    CANInterface can;
    if (!can.open("can0")) {
        std::cerr << "Failed to open CAN. Run: sudo ./scripts/setup_can.sh\n";
        return 1;
    }

    SimpleFOCMotor motor(can);

    // Send heartbeats to wake up the motor's watchdog
    std::cout << "Connecting..." << std::flush;
    for (int i = 0; i < 10; i++) {
        motor.sendHeartbeat();
        motor.pollFeedback(50);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (!motor.hasFeedback()) {
        std::cerr << "\nERROR: No feedback from motor. Is the STM32 powered on?\n";
        return 1;
    }
    std::cout << " connected.\n";

    // Enable position mode
    motor.sendNMT(MODE_POSITION);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    motor.pollFeedback(50);

    float start_pos = motor.position();
    std::cout << "Start position: " << (start_pos * 180.0f / M_PI) << " deg\n\n";

    // Print header
    std::cout << std::right
              << std::setw(5)  << "time" << "  "
              << std::setw(8)  << "pos"   << "  "
              << std::setw(8)  << "vel"   << "  "
              << std::setw(8)  << "target" << "  "
              << std::setw(8)  << "error" << "\n"
              << std::string(48, '-') << "\n";

    // Run cycles
    for (int c = 0; c < cycles && g_running; c++) {
        // Two legs per cycle: forward then back
        for (int leg = 0; leg < 2 && g_running; leg++) {
            float direction = (leg == 0) ? 1.0f : -1.0f;
            float target = start_pos + direction * move_rad;

            // Estimated move time + settle
            float est_time = std::abs(move_rad) / vel_limit + settle_s;
            float timeout = est_time * 2.0f;  // generous timeout

            std::cout << "\n--- Cycle " << (c + 1) << "/" << cycles
                      << (leg == 0 ? " FORWARD" : " REVERSE")
                      << " (target: " << (target * 180.0f / M_PI) << " deg) ---\n";

            double t0 = now_s();
            double last_print = 0;
            double last_hb = 0;
            float peak_vel = 0.0f;
            bool settled = false;
            double settle_start = 0;

            while (g_running) {
                double now = now_s();
                double elapsed = now - t0;

                if (elapsed > timeout) {
                    std::cout << "  TIMEOUT after " << elapsed << "s\n";
                    break;
                }

                // Send heartbeat every 100ms
                if (now - last_hb >= 0.1) {
                    motor.sendHeartbeat();
                    last_hb = now;
                }

                // Send position command every ~20ms
                motor.sendPositionCommand(target, vel_limit);
                motor.pollFeedback(20);

                float pos = motor.position();
                float vel = motor.velocity();
                float error_deg = (target - pos) * 180.0f / M_PI;

                if (std::abs(vel) > std::abs(peak_vel)) peak_vel = vel;

                // Print at ~4Hz
                if (elapsed - last_print >= 0.25) {
                    std::cout << std::fixed
                              << std::setw(4) << std::setprecision(1) << elapsed << "s  "
                              << std::setw(7) << std::setprecision(1) << (pos * 180.0f / M_PI) << "d  "
                              << std::showpos
                              << std::setw(7) << std::setprecision(1) << vel << "r/s  "
                              << std::noshowpos
                              << std::setw(7) << std::setprecision(1) << (target * 180.0f / M_PI) << "d  "
                              << std::showpos
                              << std::setw(7) << std::setprecision(1) << error_deg << "d"
                              << std::noshowpos << "\n";
                    last_print = elapsed;
                }

                // Check if settled (error < 2 deg and low velocity)
                if (std::abs(error_deg) < 2.0f && std::abs(vel) < 0.5f) {
                    if (!settled) {
                        settled = true;
                        settle_start = elapsed;
                    }
                    if (elapsed - settle_start >= settle_s) {
                        std::cout << "  Settled in " << std::setprecision(2)
                                  << settle_start << "s"
                                  << "  peak_vel=" << std::setprecision(1)
                                  << std::abs(peak_vel) << " rad/s"
                                  << "  final_error=" << std::showpos
                                  << std::setprecision(1) << error_deg
                                  << std::noshowpos << " deg\n";
                        break;
                    }
                } else {
                    settled = false;
                }
            }
        }
    }

    // Return to start and disable
    std::cout << "\nReturning to start...\n";
    motor.sendPositionCommand(start_pos, vel_limit);
    for (int i = 0; i < 50 && g_running; i++) {
        motor.sendHeartbeat();
        motor.pollFeedback(50);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    motor.sendNMT(MODE_DISABLED);
    std::cout << "Done.\n";
    return 0;
}
