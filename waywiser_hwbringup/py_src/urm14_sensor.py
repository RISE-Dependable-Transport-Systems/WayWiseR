import time
import modbus_tk.defines as cst
from modbus_tk import modbus_rtu
from dfrobot_ch432t import DFRobot_CH432T
import logging

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

# Sensor register addresses
SLAVE_ADDRESS = 0x0C
DISTANCE_ADDRESS = 0x05
TEMPERATURE_ADDRESS = 0x06
CONTROL_ADDRESS = 0x08
NOISE_ADDRESS = 0x09

# Control register bits
TEMPERATURE_COMPENSATION_SELECTION_BIT = 1 << 0
TEMPERATURE_COMPENSATION_ENABLE_BIT = 1 << 1
MEASUREMENT_TRIGGER_BIT = 1 << 3

# Sensor specifications
RS485_PORT = "CH432T_PORT_1"
BAUDRATE = 19200
BYTESIZE = 8
PARITY = "N"
STOPBITS = 1

# Set as needed, tested up to 100 Hz
# SAMPLING_RATE_HZ = 5
# MEASUREMENT_TIME = 1 / (2 * SAMPLING_RATE_HZ)
MEASUREMENT_TIME = 0.5  # seconds


def safe_modbus_read(master, register):
    try:
        return master.execute(SLAVE_ADDRESS, cst.READ_HOLDING_REGISTERS, register, 1)[0]
    except Exception as e:
        logging.error(f"Error reading Modbus register {register}: {e}")
        return float("nan")


def safe_modbus_write(master, register, control_value):
    try:
        return master.execute(
            SLAVE_ADDRESS, cst.WRITE_SINGLE_REGISTER, register, output_value=control_value
        )
    except Exception as e:
        logging.error(f"Error writing to Modbus register {register}: {e}")


def initialize_master_bus(port, timeout=MEASUREMENT_TIME):
    serial_object = DFRobot_CH432T(
        port=port,
        baudrate=BAUDRATE,
        bytesize=BYTESIZE,
        parity=PARITY,
        stopbits=STOPBITS,
    )
    master = modbus_rtu.RtuMaster(serial_object)
    master.set_timeout(timeout)
    return master


def write_distance_to_register(master, measurement_time=MEASUREMENT_TIME):
    control_value = safe_modbus_read(master, CONTROL_ADDRESS)
    if not isinstance(control_value, float):
        # Enable internal temperature compensation
        control_value &= ~TEMPERATURE_COMPENSATION_SELECTION_BIT
        control_value &= ~TEMPERATURE_COMPENSATION_ENABLE_BIT

        # Trigger a new measurement
        control_value |= MEASUREMENT_TRIGGER_BIT
        safe_modbus_write(master, CONTROL_ADDRESS, control_value)

    # Wait some ms for measurement to complete
    time.sleep(measurement_time)


def measure_distance(master, measurement_time=MEASUREMENT_TIME):
    write_distance_to_register(master, measurement_time)
    # Read distance register
    distance = safe_modbus_read(master, DISTANCE_ADDRESS)
    if not isinstance(distance, float):
        distance = distance if hex(distance) != hex(0xFFFF) else float("nan")
    # time.sleep(measurement_time)
    return distance / 10  # Unit is now mm


def measure_internal_temperature(master):
    temperature = safe_modbus_read(master, TEMPERATURE_ADDRESS)
    return temperature / 10  # Unit is now °C


def measure_electrical_noise_level(master):
    noise = safe_modbus_read(master, NOISE_ADDRESS)
    return noise * 10  # Unit is now percents between 0% and 100%
