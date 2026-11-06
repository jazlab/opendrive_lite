// Compile the actual firmware, replacing only Arduino I/O and its clock.
#include <Arduino.h>
#include <iostream>
#include <stdexcept>

uint32_t testMillis = 0;
uint32_t testMicros = 0;
int testEnable = HIGH;
unsigned int testStepPulses[7] = {};
TestSerial Serial;

#include "../opendrive_firmware/opendrive_firmware.ino"

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

std::string packet(const char* command) {
    char suffix[8];
    snprintf(suffix, sizeof(suffix), "*%02X", calculateChecksum(command, static_cast<int>(strlen(command))));
    return std::string(command) + suffix;
}

void command(const char* text) { processCommand(packet(text).c_str()); }

void resetTest() {
    testMillis = testMicros = 0;
    Serial.input.clear();
    Serial.output.clear();
    serialBufferIndex = 0;
    setup();
    memset(testStepPulses, 0, sizeof(testStepPulses));
    Serial.output.clear();
}

void startMoving() {
    command("HEARTBEAT");
    command("RUN,1,0.02,10,5");
    require(motors[0].isRunning, "Fresh heartbeat must allow RUN");
}

void startupRequiresHeartbeat() {
    resetTest();
    command("RUN,1,0.02,10,5");
    require(!anyMotorRunning(), "RUN before first heartbeat must fail");
    require(Serial.output.find("HeartbeatTimeout") != std::string::npos, "Rejected RUN needs an error");
    command("TARGET,1,12");
    require(motors[0].targetStepPosition == 0, "Stale TARGET must not change target");
    require(testEnable == HIGH, "Startup must keep drivers disabled");
    command("PAUSE,1");
    command("PAUSE_ALL");
    require(Serial.output.find("OK,ALL") != std::string::npos, "Stop must work without heartbeat");
}

void expiredLeaseStopsBeforeNextStep() {
    resetTest();
    startMoving();
    testMillis = 200;
    testMicros = 200000;
    updateMotors();
    require(testStepPulses[1] > 0, "Fresh movement must produce physical steps");
    unsigned int pulses = testStepPulses[1];
    long position = motors[0].currentStepPosition;
    testMillis = Config::HEARTBEAT_TIMEOUT_MS + 1;
    testMicros += 2000000;
    updateMotors();
    require(!anyMotorRunning(), "Expired movement must stop");
    require(testEnable == HIGH, "Timeout must disable drivers");
    require(testStepPulses[1] == pulses, "No additional pulse may occur after expiry");
    require(motors[0].currentStepPosition == position, "Timeout must preserve position");
    require(motors[0].stepsRemaining == 0, "Timeout must cancel remaining movement");
    require(Serial.output.find("POS,1,") != std::string::npos, "Timeout must publish final position");
    std::string output = Serial.output;
    for (int i = 0; i < 10; ++i) checkHeartbeat();
    require(Serial.output == output, "Idle expired checks must not flood serial output");
}

void repeatedTimeoutStillStopsMotors() {
    resetTest();
    startMoving();
    testMillis = 1001;
    checkHeartbeat();
    // Recreate the previous failure: the timeout has fired but a drive becomes
    // running again. Every stale check must still enforce the stop.
    motors[1].isRunning = true;
    motors[1].stepsRemaining = 50;
    enableMotors();
    checkHeartbeat();
    require(!anyMotorRunning() && testEnable == HIGH, "Watchdog cannot be a one-shot stop");
    long position = motors[0].currentStepPosition;
    long target = motors[0].targetStepPosition;
    command("RUN,1,0.02,20,0");
    command("TARGET,1,25");
    require(!anyMotorRunning(), "Stale commands cannot restart motion");
    require(motors[0].currentStepPosition == position, "Rejected RUN cannot reset position");
    require(motors[0].targetStepPosition == target, "Rejected commands cannot change target");
}

void lateHeartbeatNeverResumesMovement() {
    resetTest();
    startMoving();
    testMillis = 1001;
    command("HEARTBEAT");
    require(heartbeatIsFresh(), "A valid new heartbeat restores the lease");
    require(!anyMotorRunning(), "Late heartbeat must stop old motion before renewing");
    updateMotors();
    require(testStepPulses[1] == 0, "Heartbeat alone must never resume movement");
    command("RUN,1,0.02,10,5");
    require(anyMotorRunning(), "An explicit fresh RUN can start again");
    testMillis += 1001;
    updateMotors();
    require(!anyMotorRunning(), "A second heartbeat lapse must stop again");
}

void serialBatchCannotBypassExpiry() {
    resetTest();
    startMoving();
    testMillis = 1001;
    Serial.input = packet("RUN,1,0.02,20,0") + "\n" + packet("HEARTBEAT") + "\n";
    loop();
    require(!anyMotorRunning(), "RUN preceding recovery heartbeat must remain rejected");
    require(testStepPulses[1] == 0, "Serial batch must not generate a stale step");
    Serial.input = packet("RUN,1,0.02,10,5") + "\n";
    loop();
    require(anyMotorRunning(), "RUN after recovery must work");
}

void invalidHeartbeatDoesNotRenew() {
    resetTest();
    startMoving();
    testMillis = 1001;
    processCommand("HEARTBEAT*00");
    updateMotors();
    require(!heartbeatIsFresh() && !anyMotorRunning(), "Bad checksum must not extend heartbeat");
}

void timeoutBoundaryAndClockWrap() {
    resetTest();
    testMillis = UINT32_MAX - 500;
    startMoving();
    testMillis = 499;
    require(heartbeatIsFresh(), "Heartbeat remains valid at exactly 1000 ms across wrap");
    testMillis = 500;
    require(!heartbeatIsFresh(), "Heartbeat expires after 1000 ms across wrap");
    checkHeartbeat();
    require(!anyMotorRunning(), "Wraparound must not bypass the watchdog");
}

void firmwareIdentityDoesNotAuthorizeMovement() {
    resetTest();
    command("GET_FIRMWARE");
    require(Serial.output.find(packet("FIRMWARE,OpenDrive,lite,1.0.0")) != std::string::npos,
            "Firmware must return its exact identity with a valid checksum");
    require(!heartbeatIsFresh() && !anyMotorRunning(), "Identity query must not enable motion");
    command("RUN,1,0.02,10,5");
    require(!anyMotorRunning(), "Identity query cannot substitute for a heartbeat");
    startMoving();
    testMillis = 1001;
    command("GET_FIRMWARE");
    require(!anyMotorRunning() && !heartbeatIsFresh(), "Identity query must enforce expiry");
}

int main() {
    try {
        firmwareIdentityDoesNotAuthorizeMovement();
        startupRequiresHeartbeat();
        expiredLeaseStopsBeforeNextStep();
        repeatedTimeoutStillStopsMotors();
        lateHeartbeatNeverResumesMovement();
        serialBatchCannotBypassExpiry();
        invalidHeartbeatDoesNotRenew();
        timeoutBoundaryAndClockWrap();
        std::cout << "8 firmware heartbeat scenarios passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
