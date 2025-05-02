import logging
import math
import time

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
BAUDRATE = 19200
BYTESIZE = 8
PARITY = 'N'
STOPBITS = 1

# Set as needed, tested up to 100 Hz
MEASUREMENT_TIME = 0.5  # seconds


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


def safe_modbus_read(master, slave, register):
    try:
        return master.execute(slave, cst.READ_HOLDING_REGISTERS, register, 1)[0]
    except Exception as e:
        logging.error(f'Error reading Modbus register {register}: {e}')
        return float('nan')


def safe_modbus_write(master, slave, register, write_value):
    try:
        return master.execute(slave, cst.WRITE_SINGLE_REGISTER, register, output_value=write_value)
    except Exception as e:
        logging.error(f'Error writing to Modbus register {register}: {e}')


def write_distance_to_register(master, slave, measurement_time=MEASUREMENT_TIME):
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

    # Wait some ms for measurement to complete
    time.sleep(measurement_time)


def measure_distance(master, slave, measurement_time=MEASUREMENT_TIME):
    write_distance_to_register(master, slave, measurement_time)
    # Read distance register
    distance = safe_modbus_read(master, slave, DISTANCE_ADDRESS)
    if not math.isnan(distance):
        distance = distance if distance != 0xFFFF else float('nan')
    return distance / 10  # Unit is now mm


def measure_internal_temperature(master, slave):
    temperature = safe_modbus_read(master, slave, TEMPERATURE_ADDRESS)
    return temperature / 10  # Unit is now °C


def measure_electrical_noise_level(master, slave):
    noise = safe_modbus_read(master, slave, NOISE_ADDRESS)
    return noise * 10  # Unit is now percents between 0% and 100%


def change_slave_id(master, new_slave_id):
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
