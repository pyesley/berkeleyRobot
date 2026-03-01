#pragma once

#include "can_interface.hpp"
#include <cstdint>
#include <string>

// CAN IDs
constexpr uint32_t CAN_ID_NMT = 0x001;
constexpr uint32_t CAN_ID_PDO2_TX = 0x301;      // Position+velocity command
constexpr uint32_t CAN_ID_PDO2_RX = 0x281;      // Position+velocity feedback
constexpr uint32_t CAN_ID_FAST_FRAME = 0x481;  // Fast frame feedback
constexpr uint32_t CAN_ID_HEARTBEAT = 0x701;

// Motor modes
constexpr uint8_t MODE_POSITION = 0x13;

// Device ID
constexpr uint8_t DEVICE_ID = 1;

struct MotorState {
    float position_rad = 0.0f;
    float velocity_rad_s = 0.0f;
    uint8_t mode = 0;
    bool connected = false;
    uint64_t last_feedback_time_ms = 0;
};

class MotorProtocol {
public:
    MotorProtocol(CANInterface& can);

    // Set motor mode
    bool setMode(uint8_t mode);

    // Set position mode (convenience)
    bool setPositionMode();

    // Send position command
    bool sendPositionCommand(float position_rad, float velocity_rad_s);

    // Send heartbeat to keep watchdog alive
    bool sendHeartbeat();

    // Parse incoming CAN frame and update state
    bool parseFrame(const CANFrame& frame);

    // Get current motor state
    const MotorState& getState() const { return state_; }

    // Check if motor is connected (received feedback recently)
    bool isConnected(uint64_t timeout_ms = 500) const;

private:
    CANInterface& can_;
    MotorState state_;

    // Helper to get current time in milliseconds
    static uint64_t getCurrentTimeMs();

    // Helper to encode float to bytes (little-endian)
    static void encodeFloat(float value, uint8_t* buffer);

    // Helper to decode float from bytes (little-endian)
    static float decodeFloat(const uint8_t* buffer);
};
