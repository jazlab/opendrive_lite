"""Lite configuration and checksum-based controller protocol."""

from typing import Optional


class Config:
    """Global configuration parameters for the OpenDrive system."""

    # Firmware compatibility contract; keep in sync with the matching sketch.
    FIRMWARE_PRODUCT: str = "OpenDrive"
    FIRMWARE_VARIANT: str = "lite"
    FIRMWARE_VERSION: str = "1.0.0"
    FIRMWARE_CHECK_TIMEOUT: float = 2.0
    FIRMWARE_QUERY_INTERVAL: float = 0.25
    FIRMWARE_SKETCH: str = "firmware/opendrive_firmware/opendrive_firmware.ino"

    # Serial communication
    BAUD_RATE: int = 115200
    SERIAL_TIMEOUT: float = 0.1
    SERIAL_WRITE_TIMEOUT: float = 0.2
    SERIAL_COMMAND_QUEUE_LIMIT: int = 64
    SERIAL_RECEIVE_LIMIT: int = 4096
    GUI_WATCHDOG_TIMEOUT: float = 1.0
    SERIAL_SHUTDOWN_TIMEOUT: float = 1.0

    # Motor limits
    MAX_MOTORS: int = 6
    MIN_ABSOLUTE_POS: float = 0.0      # mm
    MAX_ABSOLUTE_POS: float = 50.0     # mm
    MIN_SPEED: float = 0.001           # mm/s (inclusive)
    MAX_SPEED: float = 0.2             # mm/s (inclusive)
    MIN_TARGET_POS: float = 0.0        # mm
    MAX_TARGET_POS: float = 50.0       # mm

    # Timing intervals (seconds)
    HEARTBEAT_INTERVAL: float = 0.5
    HEARTBEAT_TIMEOUT: float = 1.0
    POSITION_UPDATE_INTERVAL: float = 0.2
    AUTO_SAVE_INTERVAL: float = 5.0
    SERIAL_READ_INTERVAL: float = 0.02

    # Data persistence
    DATA_FILE: str = "opendrive_state.json"

    # GUI Layout
    WINDOW_TITLE: str = "OpenDrive Lite"
    DECIMAL_PLACES: int = 3
    ENTRY_WIDTH: int = 12
    COLOR_BG: str = "#f0f0f0"
    COLOR_FG: str = "#000000"
    COLOR_ENTRY_BG: str = "#ffffff"
    COLOR_READONLY_BG: str = "#d9d9d9"
    COLOR_ACTIVE_BG: str = "#e5e5e5"
    COLOR_SELECT_BG: str = "#4a90e2"

    # Column Weights and Min Sizes for Grid Layout
    # Col 0: Buttons/ID | Col 1: Speed | Col 2: Target | Col 3: Abs | Col 4: Rel | Col 5: Notes
    COL_MIN_WIDTHS: tuple = (120, 100, 100, 100, 100, 150)
    ROW_PAD_Y: int = 2
    HEADER_PAD_Y: tuple = (5, 10)


# =============================================================================
# Serial Communication Protocol
# =============================================================================

class SerialProtocol:
    """
    Serial communication protocol between GUI and Teensy console.

    All commands use XOR checksum for data integrity verification.
    Format: <command>,<arg1>,<arg2>,...*<checksum>
    """

    @staticmethod
    def calculate_checksum(data: str) -> str:
        """Calculate XOR checksum of data string and return as 2-digit hex."""
        checksum = 0
        for char in data:
            checksum ^= ord(char)
        return f"{checksum:02X}"

    @staticmethod
    def create_command(cmd: str, *args) -> str:
        """Create a command string with checksum."""
        if args:
            data = f"{cmd},{','.join(str(a) for a in args)}"
        else:
            data = cmd
        checksum = SerialProtocol.calculate_checksum(data)
        return f"{data}*{checksum}\n"

    @staticmethod
    def parse_response(response: str) -> Optional[tuple]:
        """
        Parse a response string and verify checksum.

        Returns:
            Tuple of (command, [args]) if valid, None if invalid.
        """
        response = response.strip()
        if not response or '*' not in response:
            return None

        try:
            data, received_checksum = response.rsplit('*', 1)
            expected_checksum = SerialProtocol.calculate_checksum(data)

            if received_checksum != expected_checksum:
                return None

            parts = data.split(',')
            return (parts[0], parts[1:])
        except (ValueError, IndexError):
            return None

    @staticmethod
    def cmd_run(motor_id: int, speed: float, target: float, absolute: float) -> str:
        """Create RUN command for motor movement."""
        return SerialProtocol.create_command(
            "RUN", motor_id, f"{speed:.6f}", f"{target:.3f}", f"{absolute:.3f}"
        )

    @staticmethod
    def cmd_pause(motor_id: int) -> str:
        """Create PAUSE command for a single motor."""
        return SerialProtocol.create_command("PAUSE", motor_id)

    @staticmethod
    def cmd_pause_all() -> str:
        """Create PAUSE_ALL command to stop all motors."""
        return SerialProtocol.create_command("PAUSE_ALL")

    @staticmethod
    def cmd_firmware_info() -> str:
        """Request the firmware product, variant, and compatibility version."""
        return SerialProtocol.create_command("GET_FIRMWARE")

    @staticmethod
    def cmd_heartbeat() -> str:
        """Create HEARTBEAT command for connection monitoring."""
        return SerialProtocol.create_command("HEARTBEAT")

    @staticmethod
    def cmd_update_target(motor_id: int, target: float) -> str:
        """Create TARGET command to update motor target position."""
        return SerialProtocol.create_command("TARGET", motor_id, f"{target:.3f}")
