import logging
import math
import time
from typing import Dict, Optional

from modbus_tk import modbus_rtu
import modbus_tk.defines as cst

from waywiser_hwbringup_py.dfrobot_ch432t import DFRobot_CH432T

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')

# Sensor register addresses
BROADCAST_ADDRESS = 0x00
MODULE_ADDRESS = 0x02
DISTANCE_ADDRESS = 0x05
TEMPERATURE_ADDRESS = 0x06
CONTROL_ADDRESS = 0x08
NOISE_ADDRESS = 0x09

# Control register bits
TEMPERATURE_COMPENSATION_SELECTION_BIT = 1 << 0
TEMPERATURE_COMPENSATION_ENABLE_BIT = 1 << 1
MEASUREMENT_MODE_BIT = 1 << 2
MEASUREMENT_TRIGGER_BIT = 1 << 3

# Sensor specifications
DEFAULT_BAUDRATE = 19200
DEFAULT_BYTESIZE = 8
DEFAULT_PARITY = 'N'
DEFAULT_STOPBITS = 1
DEFAULT_MEASUREMENT_TIME = 0.5  # seconds


class URM14SensorArrayManager:
    """Manager class for handling multiple URM14 sensors with different RS485 ports."""

    def __init__(self):
        self.masters: Dict[str, modbus_rtu.RtuMaster] = {}
        self.serial_objects: Dict[str, DFRobot_CH432T] = {}

    def initialize_sensor_port(self, port: str, timeout: float = DEFAULT_MEASUREMENT_TIME) -> bool:
        """
        Initialize a specific RS485 port for sensor communication.

        Args:
            port: RS485 port identifier (e.g., "CH432T_PORT_1", "CH432T_PORT_2")
            timeout: Communication timeout in seconds

        Returns
        -------
            bool: True if initialization successful, False otherwise

        """
        if port in self.masters:
            logging.warning(f'Port {port} already initialized')
            return True

        try:
            serial_object = DFRobot_CH432T(
                port=port,
                baudrate=DEFAULT_BAUDRATE,
                bytesize=DEFAULT_BYTESIZE,
                parity=DEFAULT_PARITY,
                stopbits=DEFAULT_STOPBITS,
            )

            master = modbus_rtu.RtuMaster(serial_object)
            master.set_timeout(timeout)

            self.serial_objects[port] = serial_object
            self.masters[port] = master

            logging.info(f'Successfully initialized port {port}')
            return True

        except Exception as e:
            logging.error(f'Failed to initialize port {port}: {e}')
            return False

    def get_master_for_port(self, port: str) -> Optional[modbus_rtu.RtuMaster]:
        """Get the Modbus master for a specific port."""
        return self.masters.get(port)

    def close_all_ports(self):
        """Close all initialized ports and clean up resources."""
        for port, serial_obj in self.serial_objects.items():
            try:
                if hasattr(serial_obj, 'close'):
                    serial_obj.close()
                logging.info(f'Closed port {port}')
            except Exception as e:
                logging.error(f'Error closing port {port}: {e}')

        self.masters.clear()
        self.serial_objects.clear()

    def close_port(self, port: str):
        """Close a specific port."""
        if port in self.serial_objects:
            try:
                if hasattr(self.serial_objects[port], 'close'):
                    self.serial_objects[port].close()
                del self.serial_objects[port]
                del self.masters[port]
                logging.info(f'Closed port {port}')
            except Exception as e:
                logging.error(f'Error closing port {port}: {e}')


# Global sensor manager instance
urm14_sensor_array_manager = URM14SensorArrayManager()


def initialize_master_bus(
    port: str, timeout: float = DEFAULT_MEASUREMENT_TIME
) -> Optional[modbus_rtu.RtuMaster]:
    """
    Initialize or get existing master bus for a specific port.

    Args:
        port: RS485 port identifier
        timeout: Communication timeout

    Returns
    -------
        RtuMaster instance or None if initialization failed

    """
    print('Initializing sensor port', port)
    if urm14_sensor_array_manager.initialize_sensor_port(port, timeout):
        return urm14_sensor_array_manager.get_master_for_port(port)
    return None


def safe_modbus_read(master: modbus_rtu.RtuMaster, slave: int, register: int) -> float:
    """Safely read a Modbus register with error handling."""
    try:
        return master.execute(slave, cst.READ_HOLDING_REGISTERS, register, 1)[0]
    except Exception as e:
        logging.error(f'Error reading Modbus register {register} from slave {hex(slave)}: {e}')
        return float('nan')


def safe_modbus_write(
    master: modbus_rtu.RtuMaster, slave: int, register: int, write_value: int
) -> bool:
    """Safely write to a Modbus register with error handling."""
    try:
        master.execute(slave, cst.WRITE_SINGLE_REGISTER, register, output_value=write_value)
        return True
    except Exception as e:
        logging.error(f'Error writing to Modbus register {register} on slave {hex(slave)}: {e}')
        return False


def write_distance_to_register(
    master: modbus_rtu.RtuMaster, slave: int, measurement_time: float = DEFAULT_MEASUREMENT_TIME
):
    """Configure sensor for distance measurement."""
    control_value = safe_modbus_read(master, slave, CONTROL_ADDRESS)
    if not math.isnan(control_value):
        # Enable internal temperature compensation
        control_value &= ~TEMPERATURE_COMPENSATION_SELECTION_BIT
        control_value &= ~TEMPERATURE_COMPENSATION_ENABLE_BIT
        # Set measurement mode to passive
        control_value |= MEASUREMENT_MODE_BIT
        # Trigger a new measurement
        control_value |= MEASUREMENT_TRIGGER_BIT
        safe_modbus_write(master, slave, CONTROL_ADDRESS, control_value)

    # Wait for measurement to complete
    time.sleep(measurement_time)


def measure_distance(
    master: modbus_rtu.RtuMaster, slave: int, measurement_time: float = DEFAULT_MEASUREMENT_TIME
) -> float:
    """
    Measure distance from URM14 sensor.

    Args:
        master: Modbus RTU master instance
        slave: Slave address of the sensor
        measurement_time: Time to wait for measurement completion

    Returns
    -------
        Distance in millimeters, or NaN if measurement failed

    """
    write_distance_to_register(master, slave, measurement_time)
    # Read distance register
    distance = safe_modbus_read(master, slave, DISTANCE_ADDRESS)
    if not math.isnan(distance):
        distance = distance if distance != 0xFFFF else float('nan')
    return distance / 10000.0  # Convert to meters


def measure_internal_temperature(master: modbus_rtu.RtuMaster, slave: int) -> float:
    """Measure internal temperature of URM14 sensor."""
    temperature = safe_modbus_read(master, slave, TEMPERATURE_ADDRESS)
    return temperature / 10  # Convert to °C


def measure_electrical_noise_level(master: modbus_rtu.RtuMaster, slave: int) -> float:
    """Measure electrical noise level of URM14 sensor."""
    noise = safe_modbus_read(master, slave, NOISE_ADDRESS)
    return noise * 10  # Convert to percentage (0-100%)


def change_slave_id(master: modbus_rtu.RtuMaster, new_slave_id: int):
    """Change the slave ID of a URM14 sensor."""
    logging.info(f'Attempting to change slave ID to {hex(new_slave_id)}')
    logging.info(f'Writing to register {MODULE_ADDRESS} ({hex(MODULE_ADDRESS)})')
    try:
        safe_modbus_write(master, BROADCAST_ADDRESS, MODULE_ADDRESS, new_slave_id)
        logging.info(f'Successfully sent command to change slave ID to {new_slave_id}. ')
        logging.info('\n--- IMPORTANT ---')
        logging.info(
            f"The sensor's address *should* now be set to {new_slave_id} ({hex(new_slave_id)})."
        )
        logging.info('You must POWER CYCLE the URM14 sensor for the new address to take effect.')
        logging.info('After power cycling, test communication using the new ID.')

    except Exception as e:
        logging.error(f'Failed to change slave ID: {e}')
        logging.error('Ensure ONLY the target sensor is connected to the serial port.')


def cleanup_all_sensors():
    """Clean up all sensor connections."""
    urm14_sensor_array_manager.close_all_ports()


def cleanup_sensor_port(port: str):
    """Clean up a specific sensor port."""
    urm14_sensor_array_manager.close_port(port)
