#include "can_interface.hpp"
#include "motor_protocol.hpp"
#include "tcp_server.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>
#include <cmath>

// Global flag for graceful shutdown
volatile sig_atomic_t running = 1;

void signalHandler(int) {
    running = 0;
}

uint64_t getCurrentTimeMs() {
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count();
}

int main(int argc, char* argv[]) {
    // Setup signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::string can_interface = "can0";
    int tcp_port = 9000;

    // Parse arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "-i" && i + 1 < argc) {
            can_interface = argv[++i];
        } else if (arg == "-p" && i + 1 < argc) {
            tcp_port = std::stoi(argv[++i]);
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [-i can_interface] [-p tcp_port]" << std::endl;
            std::cout << "  -i: CAN interface (default: can0)" << std::endl;
            std::cout << "  -p: TCP port for GUI (default: 9000)" << std::endl;
            return 0;
        }
    }

    std::cout << "Position Controller Starting..." << std::endl;
    std::cout << "CAN Interface: " << can_interface << std::endl;
    std::cout << "TCP Port: " << tcp_port << std::endl;

    // Initialize CAN interface
    CANInterface can;
    if (!can.open(can_interface)) {
        std::cerr << "Failed to open CAN interface. Is can0 up?" << std::endl;
        std::cerr << "Run: sudo ./scripts/setup_can.sh" << std::endl;
        return 1;
    }

    // Initialize motor protocol
    MotorProtocol motor(can);

    // Initialize TCP server
    TCPServer tcp;
    if (!tcp.start(tcp_port)) {
        std::cerr << "Failed to start TCP server" << std::endl;
        return 1;
    }

    // Set motor to position mode
    motor.setPositionMode();

    // Control loop timing
    const uint64_t HEARTBEAT_INTERVAL_MS = 100;
    const uint64_t STATUS_INTERVAL_MS = 100;  // 10Hz status updates to GUI

    uint64_t lastHeartbeatTime = 0;
    uint64_t lastStatusTime = 0;

    // Target position/velocity
    float targetPosition = 0.0f;
    float targetVelocity = 10.0f;  // Default velocity limit
    bool hasNewTarget = false;

    std::cout << "Entering main control loop..." << std::endl;

    while (running) {
        uint64_t now = getCurrentTimeMs();

        // 1. Send heartbeat periodically
        if (now - lastHeartbeatTime >= HEARTBEAT_INTERVAL_MS) {
            motor.sendHeartbeat();
            lastHeartbeatTime = now;
        }

        // 2. Receive all pending CAN frames
        {
            static uint64_t lastCanLogTime = 0;
            static uint32_t frameCount = 0;

            // Drain all available frames (up to 20 per iteration)
            for (int i = 0; i < 20; i++) {
                auto frame = can.receive(i == 0 ? 1 : 0);  // 1ms wait on first, non-blocking after
                if (!frame) break;
                frameCount++;
                motor.parseFrame(*frame);
            }

            if (now - lastCanLogTime >= 2000) {
                std::cout << "CAN RX: " << frameCount << " frames/2s"
                          << "  motor_connected=" << motor.isConnected() << std::endl;
                lastCanLogTime = now;
                frameCount = 0;
            }
        }

        // 3. Handle TCP commands from GUI
        if (tcp.poll(0)) {  // Non-blocking
            TCPCommand cmd = tcp.getCommand();
            if (cmd.valid) {
                if (cmd.cmd == "goto") {
                    targetPosition = cmd.position;
                    if (cmd.velocity > 0) {
                        targetVelocity = cmd.velocity;
                    }
                    hasNewTarget = true;
                    std::cout << "New target: " << (targetPosition * 180.0f / M_PI)
                              << " deg @ " << targetVelocity << " rad/s" << std::endl;
                } else if (cmd.cmd == "mode_position") {
                    motor.setPositionMode();
                } else if (cmd.cmd == "status") {
                    // Just triggers a status send
                }
            }
        }

        // 4. Send position command if we have a new target
        if (hasNewTarget) {
            motor.sendPositionCommand(targetPosition, targetVelocity);
            hasNewTarget = false;
        }

        // 5. Send status to GUI periodically
        if (tcp.hasClient() && now - lastStatusTime >= STATUS_INTERVAL_MS) {
            const MotorState& state = motor.getState();
            std::string mode = "position";
            tcp.sendStatus(state.position_rad, state.velocity_rad_s,
                          mode, motor.isConnected());
            lastStatusTime = now;
        }

        // Small sleep to prevent busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "\nShutting down..." << std::endl;

    return 0;
}
