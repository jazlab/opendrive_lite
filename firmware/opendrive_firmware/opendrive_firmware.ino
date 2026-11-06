/**
 * OpenDrive Console Firmware
 *
 * Teensy 4.0 firmware for controlling up to 6 stepper motor microdrives.
 *
 * Author: OpenDrive Project
 * License: MIT
 */

#include <Arduino.h>

// =============================================================================
// Configuration
// =============================================================================

namespace Config {
    // Match the GUI compatibility constants; bump when this contract changes.
    constexpr const char* FIRMWARE_PRODUCT = "OpenDrive";
    constexpr const char* FIRMWARE_VARIANT = "lite";
    constexpr const char* FIRMWARE_VERSION = "1.0.0";
    constexpr unsigned long SERIAL_BAUD = 115200;
    constexpr size_t SERIAL_BUFFER_SIZE = 128;

    constexpr unsigned long POSITION_UPDATE_INTERVAL_MS = 200;
    constexpr unsigned long HEARTBEAT_TIMEOUT_MS = 1000;

    constexpr int MAX_MOTORS = 6;
    constexpr float STEP_ANGLE_DEG = 18.0f;
    constexpr float GEARHEAD_RATIO = 64.0f;
    constexpr int MICROSTEPS = 4;
    constexpr float LEAD_SCREW_LEAD_MM = 0.5f;
    constexpr float BACKLASH_MM = 0.058f;

    constexpr float STEPS_PER_REV = (360.0f / STEP_ANGLE_DEG) * MICROSTEPS;
    constexpr float STEPS_PER_MM = (STEPS_PER_REV * GEARHEAD_RATIO) / LEAD_SCREW_LEAD_MM;
    constexpr long BACKLASH_STEPS = static_cast<long>(BACKLASH_MM * STEPS_PER_MM);

    constexpr float MAX_ACCELERATION_MM_S2 = 0.5f;
    constexpr float MIN_SPEED_MM_S = 0.001f;
    constexpr float MAX_SPEED_MM_S = 0.2f;

    // Position limits in steps
    constexpr float MIN_POSITION_MM = 0.0f;
    constexpr float MAX_POSITION_MM = 50.0f;
    constexpr long MIN_POSITION_STEPS = static_cast<long>(MIN_POSITION_MM * STEPS_PER_MM);
    constexpr long MAX_POSITION_STEPS = static_cast<long>(MAX_POSITION_MM * STEPS_PER_MM);

    // Tolerance for "at target" comparison (1 step)
    constexpr long POSITION_TOLERANCE_STEPS = 1;
}

namespace Pins {
    constexpr int ENABLE = 0;
    constexpr int STEP[Config::MAX_MOTORS] = {1, 2, 3, 4, 5, 6};
    constexpr int DIR[Config::MAX_MOTORS] = {7, 8, 9, 10, 11, 12};
}

// =============================================================================
// Unit Conversion Helpers
// =============================================================================

/**
 * Convert millimeters to steps (with rounding).
 */
inline long mmToSteps(float mm) {
    return static_cast<long>(mm * Config::STEPS_PER_MM + 0.5f);
}

/**
 * Convert steps to millimeters.
 */
inline float stepsToMm(long steps) {
    return static_cast<float>(steps) / Config::STEPS_PER_MM;
}

// =============================================================================
// Motor State
// =============================================================================

struct MotorState {
    bool isActive;
    long targetStepPosition;      // Target position in steps
    long currentStepPosition;     // Current position in steps
    float speed;                  // Speed in mm/s
    bool isRunning;
    int8_t direction;
    int8_t lastDirection;
    long stepsRemaining;
    long backlashPositionSteps;
    unsigned long stepIntervalUs;
    unsigned long lastStepTime;
    float currentSpeed;
    unsigned long lastAccelTime;

    void reset() {
        isActive = false;
        targetStepPosition = 0;
        currentStepPosition = 0;
        speed = 0.0f;
        isRunning = false;
        direction = 0;
        lastDirection = 0;
        stepsRemaining = 0;
        backlashPositionSteps = 0;
        stepIntervalUs = 0;
        lastStepTime = 0;
        currentSpeed = 0.0f;
        lastAccelTime = 0;
    }

    // Convenience accessors for mm values (for debugging and GUI communication)
    float currentPositionMm() const { return stepsToMm(currentStepPosition); }
    float targetPositionMm() const { return stepsToMm(targetStepPosition); }
};

// =============================================================================
// Global State
// =============================================================================

MotorState motors[Config::MAX_MOTORS];
char serialBuffer[Config::SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;
unsigned long lastHeartbeatReceived = 0;
bool heartbeatReceived = false;

bool heartbeatIsFresh() {
    // millis() wraps after ~49 days on Teensy. Unsigned elapsed time keeps
    // the same timeout behavior across that wrap.
    uint32_t elapsed = static_cast<uint32_t>(millis() - lastHeartbeatReceived);
    return heartbeatReceived && elapsed <= Config::HEARTBEAT_TIMEOUT_MS;
}

// =============================================================================
// Checksum Utilities
// =============================================================================

uint8_t calculateChecksum(const char* data, int len) {
    uint8_t checksum = 0;
    for (int i = 0; i < len; i++) {
        checksum ^= static_cast<uint8_t>(data[i]);
    }
    return checksum;
}

uint8_t parseHexChecksum(const char* hexStr) {
    uint8_t result = 0;
    for (int i = 0; i < 2; i++) {
        result <<= 4;
        char c = hexStr[i];
        if (c >= '0' && c <= '9') result |= (c - '0');
        else if (c >= 'A' && c <= 'F') result |= (c - 'A' + 10);
        else if (c >= 'a' && c <= 'f') result |= (c - 'a' + 10);
    }
    return result;
}

// =============================================================================
// Serial Response Functions
// =============================================================================

void sendResponse(const char* response) {
    int len = strlen(response);
    uint8_t checksum = calculateChecksum(response, len);
    Serial.print(response);
    Serial.print('*');
    if (checksum < 16) Serial.print('0');
    Serial.println(checksum, HEX);
}

void sendPositionUpdate(int motorId) {
    char buffer[32];
    float positionMm = motors[motorId - 1].currentPositionMm();
    snprintf(buffer, sizeof(buffer), "POS,%d,%.3f", motorId, positionMm);
    sendResponse(buffer);
}

void sendOK(int motorId) {
    char buffer[16];
    snprintf(buffer, sizeof(buffer), "OK,%d", motorId);
    sendResponse(buffer);
}

void sendError(int motorId, const char* message) {
    char buffer[48];
    snprintf(buffer, sizeof(buffer), "ERROR,%d,%s", motorId, message);
    sendResponse(buffer);
}

// =============================================================================
// Motor Enable/Disable
// =============================================================================

void enableMotors() { digitalWrite(Pins::ENABLE, LOW); }
void disableMotors() { digitalWrite(Pins::ENABLE, HIGH); }

bool anyMotorRunning() {
    for (int i = 0; i < Config::MAX_MOTORS; i++) {
        if (motors[i].isRunning) return true;
    }
    return false;
}

long backlashEngagedPosition(int8_t direction) {
    return (direction > 0) ? Config::BACKLASH_STEPS : 0;
}

// =============================================================================
// Motor Control
// =============================================================================

void startMotor(int idx, float speedMmS, float targetPosMm, float currentPosMm) {
    MotorState& m = motors[idx];

    // Convert mm inputs to steps
    long targetSteps = mmToSteps(targetPosMm);
    long currentSteps = mmToSteps(currentPosMm);

    m.targetStepPosition = targetSteps;
    m.currentStepPosition = currentSteps;
    m.speed = speedMmS;
    m.isActive = true;

    long distanceSteps = targetSteps - currentSteps;
    if (labs(distanceSteps) < Config::POSITION_TOLERANCE_STEPS) {
        m.isRunning = false;
        return;
    }

    int8_t newDir = (distanceSteps > 0) ? 1 : -1;
    digitalWrite(Pins::DIR[idx], (newDir == 1) ? HIGH : LOW);

    long stepsNeeded = labs(distanceSteps);
    if (m.lastDirection == 0 && !m.isRunning) {
        m.backlashPositionSteps = backlashEngagedPosition(newDir);
    }

    m.direction = newDir;
    m.stepsRemaining = stepsNeeded;
    m.currentSpeed = Config::MIN_SPEED_MM_S;
    m.stepIntervalUs = static_cast<unsigned long>(1000000.0f / (m.currentSpeed * Config::STEPS_PER_MM));
    m.lastStepTime = micros();
    m.lastAccelTime = micros();
    m.isRunning = true;

    enableMotors();
}

void updateTarget(int idx, float newTargetMm) {
    MotorState& m = motors[idx];
    long newTargetSteps = mmToSteps(newTargetMm);

    if (!m.isRunning) {
        m.targetStepPosition = newTargetSteps;
        return;
    }

    long distanceSteps = newTargetSteps - m.currentStepPosition;
    if (labs(distanceSteps) < Config::POSITION_TOLERANCE_STEPS) {
        m.targetStepPosition = newTargetSteps;
        m.stepsRemaining = 0;
        return;
    }

    int8_t newDir = (distanceSteps > 0) ? 1 : -1;
    if (newDir != m.direction) {
        digitalWrite(Pins::DIR[idx], (newDir == 1) ? HIGH : LOW);
        m.lastDirection = m.direction;
        m.direction = newDir;
    }

    m.targetStepPosition = newTargetSteps;
    m.stepsRemaining = labs(distanceSteps);
}

void stopMotor(int idx) {
    MotorState& m = motors[idx];
    if (m.isRunning) m.lastDirection = m.direction;

    m.isRunning = false;
    m.stepsRemaining = 0;
    m.currentSpeed = 0.0f;

    sendPositionUpdate(idx + 1);
    if (!anyMotorRunning()) disableMotors();
}

void stopAllMotors() {
    bool wasRunning[Config::MAX_MOTORS];
    for (int i = 0; i < Config::MAX_MOTORS; i++) {
        wasRunning[i] = motors[i].isRunning;
        if (motors[i].isRunning) motors[i].lastDirection = motors[i].direction;
        motors[i].isRunning = false;
        motors[i].stepsRemaining = 0;
        motors[i].currentSpeed = 0.0f;
    }
    disableMotors();
    // Disable every drive before serial output, which can be slow. Publish
    // final positions once for the affected drives, not on every stale loop.
    for (int i = 0; i < Config::MAX_MOTORS; i++) {
        if (wasRunning[i]) sendPositionUpdate(i + 1);
    }
}

void checkHeartbeat() {
    if (!heartbeatIsFresh() && anyMotorRunning()) {
        stopAllMotors();
    }
}

void updateMotors() {
    // Enforce the lease before any physical step.
    checkHeartbeat();
    unsigned long now = micros();

    for (int i = 0; i < Config::MAX_MOTORS; i++) {
        MotorState& m = motors[i];
        if (!m.isRunning) continue;
        if (now - m.lastStepTime < m.stepIntervalUs) continue;

        // Acceleration/Deceleration
        if (m.currentSpeed < m.speed) {
            if (now - m.lastAccelTime >= 10000) {
                m.currentSpeed += Config::MAX_ACCELERATION_MM_S2 * 0.01f;
                if (m.currentSpeed > m.speed) m.currentSpeed = m.speed;
                m.stepIntervalUs = static_cast<unsigned long>(1000000.0f / (m.currentSpeed * Config::STEPS_PER_MM));
                m.lastAccelTime = now;
            }
        } else if (m.currentSpeed > m.speed) {
            if (now - m.lastAccelTime >= 10000) {
                m.currentSpeed -= Config::MAX_ACCELERATION_MM_S2 * 0.01f;
                if (m.currentSpeed < m.speed) m.currentSpeed = m.speed;
                m.stepIntervalUs = static_cast<unsigned long>(1000000.0f / (m.currentSpeed * Config::STEPS_PER_MM));
                m.lastAccelTime = now;
            }
        }

        // Step pulse
        digitalWrite(Pins::STEP[i], HIGH);
        delayMicroseconds(2);
        digitalWrite(Pins::STEP[i], LOW);
        m.lastStepTime = now;

        // Backlash compensation changes only the internal slack state, not carriage position.
        long engagedPosition = backlashEngagedPosition(m.direction);
        if (m.backlashPositionSteps != engagedPosition) {
            m.backlashPositionSteps += m.direction;
            if (m.direction > 0 && m.backlashPositionSteps > engagedPosition) {
                m.backlashPositionSteps = engagedPosition;
            }
            if (m.direction < 0 && m.backlashPositionSteps < engagedPosition) {
                m.backlashPositionSteps = engagedPosition;
            }
            continue;
        }

        // Actual movement - integer step increment (no drift!)
        if (m.stepsRemaining > 0) {
            m.stepsRemaining--;
            m.currentStepPosition += m.direction;  // +1 or -1

            // Clamp position to safety limits
            if (m.currentStepPosition < Config::MIN_POSITION_STEPS) {
                m.currentStepPosition = Config::MIN_POSITION_STEPS;
            }
            if (m.currentStepPosition > Config::MAX_POSITION_STEPS) {
                m.currentStepPosition = Config::MAX_POSITION_STEPS;
            }

            // Check if target reached
            if (m.stepsRemaining == 0) {
                m.currentStepPosition = m.targetStepPosition;  // Snap to exact target
                stopMotor(i);
            }
        } else {
            stopMotor(i);
        }
    }
}

// =============================================================================
// Command Processing
// =============================================================================

void processCommand(const char* cmd) {
    const char* star = strchr(cmd, '*');
    if (!star) return;

    int dataLen = star - cmd;
    if (dataLen <= 0 || dataLen >= static_cast<int>(Config::SERIAL_BUFFER_SIZE) - 1) return;

    uint8_t expected = calculateChecksum(cmd, dataLen);
    uint8_t received = parseHexChecksum(star + 1);
    if (expected != received) return;

    char data[Config::SERIAL_BUFFER_SIZE];
    strncpy(data, cmd, dataLen);
    data[dataLen] = '\0';

    char* token = strtok(data, ",");
    if (!token) return;

    // Stop expired movement before even a late heartbeat renews the lease.
    checkHeartbeat();

    if (strcmp(token, "GET_FIRMWARE") == 0) {
        char response[80];
        snprintf(response, sizeof(response), "FIRMWARE,%s,%s,%s",
                 Config::FIRMWARE_PRODUCT, Config::FIRMWARE_VARIANT, Config::FIRMWARE_VERSION);
        sendResponse(response);
        return;
    }

    if (strcmp(token, "HEARTBEAT") == 0) {
        lastHeartbeatReceived = millis();
        heartbeatReceived = true;
        sendResponse("HEARTBEAT_ACK");
        return;
    }

    if (strcmp(token, "PAUSE_ALL") == 0) {
        stopAllMotors();
        sendResponse("OK,ALL");
        return;
    }

    if (strcmp(token, "PAUSE") == 0) {
        token = strtok(NULL, ",");
        if (!token) return;
        int id = atoi(token);
        if (id < 1 || id > Config::MAX_MOTORS) { sendError(id, "BadID"); return; }
        stopMotor(id - 1);
        sendOK(id);
        return;
    }

    if (strcmp(token, "TARGET") == 0) {
        token = strtok(NULL, ",");
        if (!token) return;
        int id = atoi(token);
        if (id < 1 || id > Config::MAX_MOTORS) { sendError(id, "BadID"); return; }
        if (!heartbeatIsFresh()) { sendError(id, "HeartbeatTimeout"); return; }
        token = strtok(NULL, ",");
        if (!token) return;
        updateTarget(id - 1, atof(token));
        sendOK(id);
        return;
    }

    if (strcmp(token, "RUN") == 0) {
        token = strtok(NULL, ",");
        if (!token) return;
        int id = atoi(token);
        if (id < 1 || id > Config::MAX_MOTORS) { sendError(id, "BadID"); return; }
        if (!heartbeatIsFresh()) { sendError(id, "HeartbeatTimeout"); return; }

        token = strtok(NULL, ",");
        if (!token) return;
        float speed = atof(token);

        token = strtok(NULL, ",");
        if (!token) return;
        float target = atof(token);

        token = strtok(NULL, ",");
        if (!token) return;
        float absolute = atof(token);

        MotorState& m = motors[id - 1];
        if (m.isRunning) {
            // Check if we can update seamlessly (same direction)
            long targetSteps = mmToSteps(target);
            long distSteps = targetSteps - m.currentStepPosition;
            int8_t newDir = (distSteps > 0) ? 1 : -1;

            if (newDir == m.direction || labs(distSteps) < Config::POSITION_TOLERANCE_STEPS) {
                m.speed = speed;
                updateTarget(id - 1, target);
                sendOK(id);
                return;
            }
        }

        // Use internal position if running (direction change), GUI position if stopped
        float startPos = m.isRunning ? m.currentPositionMm() : absolute;
        startMotor(id - 1, speed, target, startPos);
        sendOK(id);
    }
}

void readSerial() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serialBufferIndex > 0) {
                serialBuffer[serialBufferIndex] = '\0';
                processCommand(serialBuffer);
                serialBufferIndex = 0;
            }
        } else if (serialBufferIndex < static_cast<int>(Config::SERIAL_BUFFER_SIZE) - 1) {
            serialBuffer[serialBufferIndex++] = c;
        }
    }
}

void sendPositionUpdates() {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate >= Config::POSITION_UPDATE_INTERVAL_MS) {
        for (int i = 0; i < Config::MAX_MOTORS; i++) {
            if (motors[i].isRunning) sendPositionUpdate(i + 1);
        }
        lastUpdate = millis();
    }
}

// =============================================================================
// Arduino Setup and Loop
// =============================================================================

void setup() {
    Serial.begin(Config::SERIAL_BAUD);

    pinMode(Pins::ENABLE, OUTPUT);
    disableMotors();

    for (int i = 0; i < Config::MAX_MOTORS; i++) {
        pinMode(Pins::STEP[i], OUTPUT);
        pinMode(Pins::DIR[i], OUTPUT);
        digitalWrite(Pins::STEP[i], LOW);
        digitalWrite(Pins::DIR[i], LOW);
        motors[i].reset();
    }

    lastHeartbeatReceived = millis();
    heartbeatReceived = false;
    sendResponse("READY");
}

void loop() {
    readSerial();
    updateMotors();
    sendPositionUpdates();
}
