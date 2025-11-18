# -*- coding: utf-8 -*
'''!
  @file  DFRobot_CH432T.py
  @brief  Define infrastructure of DFRobot_CH432T class
  @details  ch432t Raspberry Pi SPI to modbus expansion board driver
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license  The MIT License (MIT)
  @author  [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0
  @date  2021-12-14
  @url  https://github.com/DFRobot/DFRobot_CH432T
'''
import sys
import time

import spidev

import logging
from ctypes import *

from threading import Lock

from serial.serialutil import *
# from serial.serialutil import SerialBase, SerialException, to_bytes, \
#     portNotOpenError, writeTimeoutError, Timeout

logger = logging.getLogger()
# logger.setLevel(logging.INFO)   # Display all print information
logger.setLevel(logging.WARNING)   # If you don’t want to display too many prints, only print errors, please use this option
ph = logging.StreamHandler()
formatter = logging.Formatter("%(asctime)s - [%(filename)s %(funcName)s]:%(lineno)d - %(levelname)s: %(message)s")
ph.setFormatter(formatter)
logger.addHandler(ph)

ch432t_spi_lock = Lock()   # lock for read and write

# CH432T register definitions
## RX FIFO
CH432T_RBR_REG = 0x00
## TX FIFO
CH432T_THR_REG = 0x00
## Interrupt enable
CH432T_IER_REG = 0x01
## Interrupt Identification
CH432T_IIR_REG = 0x02
## FIFO control
CH432T_FCR_REG = 0x02
## Line Control
CH432T_LCR_REG = 0x03
## Modem Control
CH432T_MCR_REG = 0x04
## Line Status
CH432T_LSR_REG = 0x05
## Modem Status
CH432T_MSR_REG = 0x06
## Scratch Pad
CH432T_SCR_REG = 0x07
# Special Register set: Only if (LCR[7] == 1)
## Divisor Latch Low
CH432T_DLL_REG = 0x00
## Divisor Latch High
CH432T_DLH_REG = 0x01


# IER register bits
## Enable RX data interrupt
CH432T_IER_RDI_BIT = (1 << 0)
## Enable TX holding register interrupt
CH432T_IER_THRI_BIT = (1 << 1)
## Enable RX line status interrupt
CH432T_IER_RLSI_BIT = (1 << 2)
## Enable Modem status interrupt
CH432T_IER_MSI_BIT = (1 << 3)
# IER enhanced register bits
## Enable Soft reset
CH432T_IER_RESET_BIT = (1 << 7)
## Enable low power mode
CH432T_IER_LOWPOWER_BIT = (1 << 6)
## Enable sleep mode
CH432T_IER_SLEEP_BIT = (1 << 5)
## Enable clk * 2
CH432T_IER_CK2X_BIT = (1 << 5)
## Low power mode
CH432T_LOW_POWER_MODE = CH432T_IER_LOWPOWER_BIT
## Sleep mode
CH432T_SLEEP_MODE = CH432T_IER_SLEEP_BIT
## Standard mode
CH432T_STANDARD_MODE = 0


# IIR register bits
## Mask for the interrupt ID
CH432T_IIR_ID_MASK = 0x0e
## No interrupts pending
CH432T_IIR_NO_INT_BIT = (1 << 0)
## RX line status error
CH432T_IIR_RLSE_SRC = 0x06
## RX data interrupt
CH432T_IIR_RDI_SRC = 0x04
## RX time-out interrupt
CH432T_IIR_RTOI_SRC = 0x0c
## TX holding register empty
CH432T_IIR_THRI_SRC = 0x02
## Modem status interrupt
CH432T_IIR_MSI_SRC = 0x00


# FCR register bits
## Enable FIFO
CH432T_FCR_FIFO_BIT = (1 << 0)
## Reset RX FIFO
CH432T_FCR_RXRESET_BIT =(1 << 1)
## Reset TX FIFO
CH432T_FCR_TXRESET_BIT =(1 << 2)
## RX Trigger level
CH432T_FCR_RXLVL_BIT = (0x03 << 6)
# Set the trigger point for receiving FIFO interrupt and hardware flow control
CH432T_FCR_RECVTG_LEN_1 = (0x00 << 6)
CH432T_FCR_RECVTG_LEN_4 = (0x01 << 6)
CH432T_FCR_RECVTG_LEN_8 = (0x02 << 6)
CH432T_FCR_RECVTG_LEN_14 = (0x03 << 6)


# LCR register bits
## Word length bit
CH432T_LCR_LENGTH_BIT = (0x03 << 0)
## STOP length bit
CH432T_LCR_STOPLEN_BIT = (1 << 2)
## Parity enable bit
CH432T_LCR_PARITY_EN_BIT = (1 << 3)
## Parity mode bit
CH432T_LCR_PARITY_MODE_BIT = (0x03 << 4)
## TX break enable
CH432T_LCR_TXBREAK_BIT = (1 << 6)
## Divisor Latch enable
CH432T_LCR_DLAB_BIT	 = (1 << 7)
## Special reg set
CH432T_LCR_CONF_MODE_A = CH432T_LCR_DLAB_BIT
# Set word length
CH432T_LCR_WORD_LEN_5 = 0x00
CH432T_LCR_WORD_LEN_6 = 0x01
CH432T_LCR_WORD_LEN_7 = 0x02
CH432T_LCR_WORD_LEN_8 = 0x03
## 1 stop bit
CH432T_STOPBIT_1 = 0
## 1-1.5 stop bits if word length is 5, 2 stop bits otherwise
CH432T_STOPBIT_2 = CH432T_LCR_STOPLEN_BIT
## check bit ODD
CH432T_CHECKBIT_ODD = (0x00 << 4)
## check bit EVEN
CH432T_CHECKBIT_EVEN = (0x01 << 4)
## check bit MARK
CH432T_CHECKBIT_MARK = (0x02 << 4)
## check bit SPACE
CH432T_CHECKBIT_SPACE = (0x03 << 4)

# MCR register bits
## DTR complement
CH432T_MCR_DTR_BIT = (1 << 0)
## RTS complement
CH432T_MCR_RTS_BIT = (1 << 1)
## OUT1
CH432T_MCR_OUT1 = (1 << 2)
## OUT2
CH432T_MCR_OUT2 = (1 << 3)
## Enable loopback test mode
CH432T_MCR_LOOP_BIT = (1 << 4)
## Enable Hardware Flow control
CH432T_MCR_AFE = (1 << 5)


# LSR register bits
## Receiver data ready
CH432T_LSR_DR_BIT = (1 << 0)
## Overrun Error
CH432T_LSR_OE_BIT = (1 << 1)
## Parity Error
CH432T_LSR_PE_BIT = (1 << 2)
## Frame Error
CH432T_LSR_FE_BIT = (1 << 3)
## Break Interrupt
CH432T_LSR_BI_BIT = (1 << 4)
## TX holding register empty
CH432T_LSR_THRE_BIT = (1 << 5)
## Transmitter empty
CH432T_LSR_TEMT_BIT = (1 << 6)
## Fifo Error
CH432T_LSR_FIFOE_BIT = (1 << 7)
## BI, FE, PE, OE bits
CH432T_LSR_BRK_ERROR_MASK = 0x1E


# MSR register bits
## Delta CTS Clear To Send
CH432T_MSR_DCTS_BIT = (1 << 0)
## Delta DSR Data Set Ready
CH432T_MSR_DDSR_BIT = (1 << 1)
## Delta RI Ring Indicator
CH432T_MSR_DRI_BIT = (1 << 2)
## Delta CD Carrier Detect
CH432T_MSR_DCD_BIT = (1 << 3)
## CTS
CH432T_MSR_CTS_BIT = (1 << 4)
## DSR
CH432T_MSR_DSR_BIT = (1 << 5)
## RI
CH432T_MSR_RI_BIT = (1 << 6)
## CD
CH432T_MSR_CD_BIT = (1 << 7)
## Any of the delta bits!
CH432T_MSR_DELTA_MASK = 0x0F


## RS485 serial port 0
CH432T_PORT_1 = 0
## RS485 serial port 1
CH432T_PORT_2 = 1

# CH432T External input clock frequency or external crystal frequency
CH432T_CLOCK_FREQUENCY = 22118400

# Misc definitions
CH432T_FIFO_SIZE = 16
CH432T_REG_SHIFT = 2


class DFRobot_CH432T(SerialBase, object):
  '''!
    @brief Define DFRobot_CH432T basic class
    @details Drive ch432t Raspberry Pi SPI to modbus expansion board
  '''

  class INT_config_reg(Structure):
    '''
      @brief Interrupt enable register, including enhancing function control bit and serial port interrupt enabling
      @note Register struct:
      @n -------------------------------------------------------------------------------------
      @n |  b7   |    b6     |    b5    |    b4    |    b3    |    b2    |    b1   |   b0    |
      @n -------------------------------------------------------------------------------------
      @n | reset | low_power | slp_ck2x | reserved | ie_modem | ie_lines | ie_thre | ie_recv |
      @n -------------------------------------------------------------------------------------
      reset: When this bit is set to 1, soft reset the serial port, this bit will reset automatically without software
      low_power: When this bit is 1, close the internal reference clock of the serial port to make it enter low power status
      slp_ck2x: This bit makes different effects on serial port 0 and 1, serial port 0 is SLP, when the bit is 1, close the clock oscillator to make serial port 0 and 1 enter sleep mode;
          serial port 1 is CK2X, when the bit is 1, double frequency of external clock signal and take the result as internal reference clock for serial port 0 and 1
      ie_modem: when the bit is 1, allow modem to input status change interrupt
      ie_lines: when the bit is 1, allow receiving line status interrupt
      ie_thre: when the bit is 1, allow transmitting holding register empty interrupt
      ie_recv: when the bit is 1, allow receiving data interrupt
    '''
    _pack_ = 1
    _fields_ = [('reset', c_ubyte, 1),
          ('low_power',c_ubyte, 1),
          ('slp_ck2x', c_ubyte, 1),
          ('reserved', c_ubyte, 1),
          ('ie_modem', c_ubyte, 1),
          ('ie_lines', c_ubyte, 1),
          ('ie_thre', c_ubyte, 1),
          ('ie_recv', c_ubyte, 1)]
    def __init__(self):
      '''!
        @brief sensor_status structure init
      '''
      self.reset = 0
      self.low_power = 0
      self.slp_ck2x = 0
      self.ie_modem = 0
      self.ie_lines = 0
      self.ie_thre = 0
      self.ie_recv = 0

    def set_list(self, data):
      '''!
        @brief Assign the struct
        @param data The assigned uint8_t data
      '''
      buf = (c_ubyte * len(data))()
      for i in range(len(data)):
        buf[i] = data[i]
      memmove(addressof(self), addressof(buf), len(data))

    def get_list(self):
      '''!
        @brief Get the struct value
        @return Return the struct value, list type
      '''
      return list(bytearray(string_at( addressof(self), sizeof(self) )))

  class INT_status_reg(Structure):
    '''
      @brief Interrupt identity register, used to analyze and process the interrupt source
      @note Register struct:
      @n -----------------------------------------------------------------------------------
      @n |    b7    |    b6    |   b5    |   b4    |    b3    |    b2    |   b1   |   b0   |
      @n -----------------------------------------------------------------------------------
      @n |      fifo_ENS       |     reserved      |   IID3   |   IID2   |  IID1  | NOINT  |
      @n -----------------------------------------------------------------------------------
      fifo_ENS: This bit is for FIFO enabling status, 1: FIFO enabled
      IID3 + IID2 + IID1 + NOINT: 
        0001: priority: none; interrupt type: no interrupt occurs; 
          Interrupt source: no interrupt; method to clear interrupt: none;
        0110: priority: 1;  interrupt type: receive line status; 
          Interrupt source: OVERR, PARERR, FRAMEERR, BREAKINT; method to clear interrupt: read LSR;
        0100: priority: 2;  interrupt type: for receiving data; 
          Interrupt source: The number of received bytes reaches the trigger point of FIFO; method to clear interrupt: read RBR;
        1100: priority: 2;  interrupt type: receiving data timeout; 
          Interrupt source: next data has not been received for over 4 data periods; method to clear interrupt: read RBR;
        0010: priority: 3;  interrupt type: THR register empty; 
          Interrupt source: transmit holding register empty, re-enable interrupt when IETHRE changes from 0 to 1; method to clear interrupt: read IIR or write THR;
        0000: priority: 4;  interrupt type: MODEM input changes; 
          Interrupt source: △CTS、△DSR、△RI、△DCD; method to clear interrupt: read MSR;
    '''
    _pack_ = 1
    _fields_ = [
                ('int_type', c_ubyte, 4),
                ('reserved', c_ubyte, 2),
                ('fifo_ENS', c_ubyte, 2)]
    def __init__(self):
      '''!
        @brief sensor_status structure init
      '''
      self.fifo_ENS = 0
      self.int_type = 1

    def set_list(self, data):
      '''!
        @brief Assign the struct
        @param data The assigned uint8_t data
      '''
      buf = (c_ubyte * len(data))()
      for i in range(len(data)):
        buf[i] = data[i]
      memmove(addressof(self), addressof(buf), len(data))

    def get_list(self):
      '''!
        @brief Obtain the struct value
        @return Return the struct value, list type
      '''
      return list(bytearray(string_at( addressof(self), sizeof(self) )))

  class fifo_config_reg(Structure):
    '''
      @brief FIFO buffer FIFO control register, enable and reset FIFO
      @note Register struct:
      @n -------------------------------------------------------------------------------------
      @n |   b7    |    b6   |   b5   |   b4    |    b3    |    b2     |    b1     |    b0   |
      @n -------------------------------------------------------------------------------------
      @n |      recv_TG      |           reserved          | TFIFORST  | RFIFORST  | FIFOEN  |
      @n -------------------------------------------------------------------------------------
      recv_TG: Set the interrupt for receiving FIFO and the trigger point of hardware flow control, 
           00: For 1 byte, i.e., the interrupt for receiving data occur when 1 byte is received and pin RTS is automatically disabled when enabling the hardware flow control;
           01: for 4 bytes;
           10: for 8 bytes;
           11: for 14 bytes.
      t_fifo_rst: when this bit is set to 1, clear the transmitting data in FIFO (TSR is not included), this bit will reset automatically without software
      r_fifo_rst: when this bit is set to 1, clear the receiving data in FIFO (RSR is not included), this bit will reset automatically without software
      fifo_EN: when the bit is 1, enable FIFO, when the bit reset, disable FIFO,
           It switches to 16C450 compatible mode after disabling FIFO, equivalent to a FIFO with only one byte.
    '''
    _pack_ = 1
    _fields_ = [('fifo_EN', c_ubyte, 1),
                ('r_fifo_rst', c_ubyte, 1),
                ('t_fifo_rst', c_ubyte, 1),
                ('reserved', c_ubyte, 3),
                ('recv_TG', c_ubyte, 2)]
    def __init__(self):
      '''!
        @brief sensor_status structure init
      '''
      self.recv_TG = 0
      self.t_fifo_rst = 0
      self.t_fifo_rst = 0
      self.fifo_EN = 0

    def set_list(self, data):
      '''!
        @brief Assign the struct
        @param data The assigned uint8_t data
      '''
      buf = (c_ubyte * len(data))()
      for i in range(len(data)):
        buf[i] = data[i]
      memmove(addressof(self), addressof(buf), len(data))

    def get_list(self):
      '''!
        @brief Obtain the struct value
        @return Return the struct value, list type
      '''
      return list(bytearray(string_at( addressof(self), sizeof(self) )))

  class lines_config_reg(Structure):
    '''
      @brief Line control register, used to control the format of serial communication
      @note Register struct:
      @n -------------------------------------------------------------------------------------
      @n |   b7  |    b6    |   b5   |   b4   |    b3     |    b2      |    b1    |    b0    |
      @n -------------------------------------------------------------------------------------
      @n | DLAB  | break_EN |   parity_mode   | parity_EN |  stop_bit  |      word_size      |
      @n -------------------------------------------------------------------------------------
      DLAB: This bit is for enabling divisor latch access, DLL/DLM can be accessed only when it's 1, RBR/THR/IER can be accessed only when it's 0.
      break_EN: when this bit is 1, BREAK line interval is forced to occur.
      parity_mode: Set the format of parity check bit when PAREN is 1 (parity check bit): 
             00: odd parity check, 
             01: even parity check, 
             10: mark bit (MARK, set to 1), 
             11: space bit (SPACE, reset)
      parity_EN: When the bit is set to 1, it is allowed to generate parity check bit while transmitting and check that bit while receiving, 0 indicates no parity check bit.
      stop_bit: 1 indicates there are two stop bits and 0 indicates one
      word_size: Set word length, 
           00: 5 data bits, 
           01: 6 data bits, 
           10: 7 data bits, 
           11: 8 data bits.
    '''
    _pack_ = 1
    _fields_ = [('word_size', c_ubyte, 2),
                ('stop_bit', c_ubyte, 1),
                ('parity_EN', c_ubyte, 1),
                ('parity_mode', c_ubyte, 2),
                ('break_EN', c_ubyte, 1),
                ('DLAB', c_ubyte, 1)]
    def __init__(self):
      '''!
        @brief sensor_status structure init
      '''
      self.DLAB = 0
      self.break_EN = 0
      self.parity_mode = 0
      self.parity_EN = 0
      self.stop_bit = 0
      self.word_size = 0

    def set_list(self, data):
      '''!
        @brief Assign the struct
        @param data The assigned uint8_t data
      '''
      buf = (c_ubyte * len(data))()
      for i in range(len(data)):
        buf[i] = data[i]
      memmove(addressof(self), addressof(buf), len(data))

    def get_list(self):
      '''!
        @brief Obtain the struct value
        @return Return the struct value, list type
      '''
      return list(bytearray(string_at( addressof(self), sizeof(self) )))

  class modem_config_reg(Structure):
    '''
      @brief MODEM control register, control MODEM output
      @note Register struct:
      @n -------------------------------------------------------------------------------------
      @n |   b7  |  b6  |    b5    |    b4     |    b3     |    b2    |    b1    |     b0    |
      @n -------------------------------------------------------------------------------------
      @n |   reserved   |   AFE    |   loop    |   out2    |   out1   |   RTS    |    DTR    |
      @n -------------------------------------------------------------------------------------
      AFE: when the bit is set to 1, allow CTS & RTS hardware automatic flow control
      loop: when the bit is set to 1, enable test mode for internal loop
      out2: when the bit is set to 1, allow interrupt request output of the serial port, otherwise no actual interrupt request of the serial port will occur.
      out1: this bit is user-definable MODEM control bit and is not connected to the actual output pin.
      RTS: when the bit is set to 1, RTS pin output is valid (active low), otherwise it's invalid.
      DTR: when the bit is set to 1, DTR pin output is valid (active low), otherwise it's invalid.
    '''
    _pack_ = 1
    _fields_ = [('DTR', c_ubyte, 1),
                ('RTS', c_ubyte, 1),
                ('out1', c_ubyte, 1),
                ('out2', c_ubyte, 1),
                ('loop', c_ubyte, 1),
                ('AFE', c_ubyte, 1),
                ('reserved', c_ubyte, 2)]
    def __init__(self):
      '''!
        @brief sensor_status structure init
      '''
      self.AFE = 0
      self.loop = 0
      self.out2 = 0
      self.out1 = 0
      self.RTS = 0
      self.DTR = 0

    def set_list(self, data):
      '''!
        @brief Assign the struct
        @param data The assigned uint8_t data
      '''
      buf = (c_ubyte * len(data))()
      for i in range(len(data)):
        buf[i] = data[i]
      memmove(addressof(self), addressof(buf), len(data))

    def get_list(self):
      '''!
        @brief Obtain the struct value
        @return Return the struct value, list type
      '''
      return list(bytearray(string_at( addressof(self), sizeof(self) )))

  class lines_status_reg(Structure):
    '''
      @brief Line status register, used to analyze serial port status by query.
      @note Register struct:
      @n -----------------------------------------------------------------------------------------------
      @n |     b7     |   b6    |   b5   |    b4     |    b3     |     b2     |    b1     |     b0     |
      @n -----------------------------------------------------------------------------------------------
      @n | r_fifo_err | t_empty | THR_EN | break_INT | frame_err | parity_err | fifo_over | data_ready |
      @n -----------------------------------------------------------------------------------------------
      r_fifo_err: set to 1, indicates there is at least one parity_err, frame_err, or break_INT error in the received FIFO.
      t_empty: set to 1, indicates transmitting holding register THR and shift register TSR are both empty.
      THR_EN: set to 1, indicates transmitting holding register THR is empty.
      break_INT: set to 1, indicates BREAK line interval is detected.
      frame_err: set to 1, indicates the frame error of the data read from the received FIFO, lack of valid stop bit.
      parity_err: set to 1, indicates parity check error of the data read from the received FIFO occurred.
      fifo_over: set to 1, indicates the received FIFO buffer overflow
      data_ready: set to 1, indicates there is data received from the FIFO, after reading all the data in FIFO, the bit will automatically reset.
    '''
    _pack_ = 1
    _fields_ = [('data_ready', c_ubyte, 1),
                ('fifo_over', c_ubyte, 1),
                ('parity_err', c_ubyte, 1),
                ('frame_err', c_ubyte, 1),
                ('break_INT', c_ubyte, 1),
                ('THR_EN', c_ubyte, 1),
                ('t_empty', c_ubyte, 1),
                ('r_fifo_err', c_ubyte, 1)]
    def __init__(self):
      '''!
        @brief sensor_status structure init
      '''
      self.r_fifo_err = 0
      self.t_empty = 1
      self.THR_EN = 1
      self.break_INT = 0
      self.frame_err = 0
      self.parity_err = 0
      self.fifo_over = 0
      self.data_ready = 0

    def set_list(self, data):
      '''!
        @brief Assign the struct
        @param data The assigned uint8_t data
      '''
      buf = (c_ubyte * len(data))()
      for i in range(len(data)):
        buf[i] = data[i]
      memmove(addressof(self), addressof(buf), len(data))

    def get_list(self):
      '''!
        @brief Obtain the struct value
        @return Return the struct value, list type
      '''
      return list(bytearray(string_at( addressof(self), sizeof(self) )))

  class modem_status_reg(Structure):
    '''
      @brief MODEM Status register, for querying MODEM status
      @note Register struct:
      @n -------------------------------------------------------------------------------------
      @n |   b7  |   b6   |   b5  |  b4   |     b3     |    b2     |     b1     |     b0     |
      @n -------------------------------------------------------------------------------------
      @n |  DCD  |   RI   |  DSR  |  CTS  | DCD_change | RI_change | DSR_change | CTS_change |
      @n -------------------------------------------------------------------------------------
      DCD: This bit is bit flip of pin DCD, 1 indicates pin DCD is active (active low).
      RI: This bit is bit flip of pin RI, 1 indicates pin RI is active (active low).
      DSR: This bit is bit flip of pin DSR, 1 indicates pin DSR is active (active low).
      CTS: This bit is bit flip of pin CTS, 1 indicates pin CTS is active (active low).
      DCD_change: That the bit is set to 1 indicates that pin DCD input status has changed.
      RI_change: That the bit is set to 1 indicates that pin RI input status has changed.
      DSR_change: That the bit is set to 1 indicates that pin DSR input status has changed.
      CTS_change: That the bit is set to 1 indicates that pin CTS input status has changed.
    '''
    _pack_ = 1
    _fields_ = [('CTS_change', c_ubyte, 1),
                ('DSR_change', c_ubyte, 1),
                ('RI_change', c_ubyte, 1),
                ('DCD_change', c_ubyte, 1),
                ('CTS', c_ubyte, 1),
                ('DSR', c_ubyte, 1),
                ('RI', c_ubyte, 1),
                ('DCD', c_ubyte, 1)]
    def __init__(self):
      '''!
        @brief sensor_status structure init
      '''
      self.DCD = 0
      self.RI = 0
      self.DSR = 0
      self.CTS = 0
      self.DCD_change = 0
      self.RI_change = 0
      self.DSR_change = 0
      self.CTS_change = 0

    def set_list(self, data):
      '''!
        @brief Assign the struct
        @param data The assigned uint8_t data
      '''
      buf = (c_ubyte * len(data))()
      for i in range(len(data)):
        buf[i] = data[i]
      memmove(addressof(self), addressof(buf), len(data))

    def get_list(self):
      '''!
        @brief Obtain the struct value
        @return Return the struct value, list type
      '''
      return list(bytearray(string_at( addressof(self), sizeof(self) )))

  def __init__(self, port="CH432T_PORT_1", baudrate=115200, bytesize=8, parity='N', stopbits=1):
    '''!
      @brief Module init
      @param port serial number, "CH432T_PORT_1" or "CH432T_PORT_2"
      @param baudrate serial baud rate
      @param bytesize byte size
      @param parity check bit
      @param stopbits stop bit
    '''
    if port not in ["CH432T_PORT_1", "CH432T_PORT_2"]:
        raise SerialException("Invalid PORT, please select 'CH432T_PORT_1' or 'CH432T_PORT_2'")
    self.portnum = int(port[-1]) - 1
    logger.info("self.portnum = %d", self.portnum)
    self._spi = spidev.SpiDev()
    self._spi.open(0, 0)   # default to use spidev0.0
    self._spi.max_speed_hz = 1000000   # SPI communication frequency is default to be 1 MHz
    super(DFRobot_CH432T, self).__init__(port, baudrate, bytesize, parity, stopbits)

  def close(self):
    '''!
      @brief Close the serial port and the internal reference clock of it, so as to make the serial port enter low-power status
    '''
    self.set_low_power_mode(CH432T_LOW_POWER_MODE)   # close the internal reference clock of the serial port
    self.is_open = False

  def open(self):
    '''!
      @brief Initialize port
      @note Exceptions will be thrown when a hardware communication or config error occurs
    '''
    # Read the value of CH432T_IIR_REG and CH432T_LSR_REG register of port0 and port1
    iir = self._read_reg(CH432T_IIR_REG, 1)[0]
    logger.info( "CH432T_IIR_REG = %#x", iir)
    lsr = self._read_reg(CH432T_LSR_REG, 1)[0]
    logger.info( "CH432T_LSR_REG = %#x", lsr)

    # Test user register of port0 and port1
    self._write_reg(CH432T_SCR_REG, 0x66)
    scr = self._read_reg(CH432T_SCR_REG, 1)[0]
    logger.info( "CH432T_SCR_REG = %#x", scr)
    if 0x66 != scr:
      raise SerialException("Failed to open port! Check whether the expansion board \
                              is properly connected and whether spidev0.0 is occupied.")

    self.orig_attr = [0, 0]   # cflag and baudrate
    try:
      self._reconfigure_port(force_update=True)
    except:
      self.set_low_power_mode(CH432T_LOW_POWER_MODE)
    else:
      self.is_open = True

  def _reconfigure_port(self, force_update=False):
    '''!
      @brief Configure serial port
      @note Exceptions will be thrown when config error occurs
    '''

    # setup char len
    cflag = 0
    if self._bytesize == 8:
      cflag |= CH432T_LCR_WORD_LEN_8
    elif self._bytesize == 7:
      cflag |= CH432T_LCR_WORD_LEN_7
    elif self._bytesize == 6:
      cflag |= CH432T_LCR_WORD_LEN_6
    elif self._bytesize == 5:
      cflag |= CH432T_LCR_WORD_LEN_5
    else:
      raise ValueError('Invalid char len: {!r}'.format(self._bytesize))
    # setup stop bits
    if self._stopbits == STOPBITS_ONE:
      cflag |= CH432T_STOPBIT_1
    elif self._stopbits == STOPBITS_ONE_POINT_FIVE:
      cflag |= CH432T_STOPBIT_2  # XXX same as TWO.. there is no POSIX support for 1.5
    elif self._stopbits == STOPBITS_TWO:
      cflag |= CH432T_STOPBIT_2
    else:
      raise ValueError('Invalid stop bit specification: {!r}'.format(self._stopbits))
    # setup parity
    if self._parity == PARITY_NONE:
      cflag &= ~(CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_SPACE)
    elif self._parity == PARITY_EVEN:
      cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_EVEN)
    elif self._parity == PARITY_ODD:
      cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_ODD)
    elif self._parity == PARITY_MARK:
      cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_MARK)
    elif self._parity == PARITY_SPACE:
      cflag |= (CH432T_LCR_PARITY_EN_BIT | CH432T_CHECKBIT_SPACE)
    else:
      raise ValueError('Invalid parity: {!r}'.format(self._parity))

    logger.info("cflag = %d", cflag)
    if force_update or [cflag, self._baudrate] != self.orig_attr:
      self.orig_attr = [cflag, self._baudrate]
      # Now, initialize the UART
      self._write_reg(CH432T_LCR_REG, cflag)

      # Reset FIFOs, Enable FIFOs and configure interrupt & flow control levels to 8
      val = CH432T_FCR_FIFO_BIT | CH432T_FCR_RXRESET_BIT | CH432T_FCR_TXRESET_BIT | CH432T_FCR_RECVTG_LEN_8
      self._write_reg(CH432T_FCR_REG, val)

      # Enable RX, TX, CTS change interrupts
      # val = CH432T_IER_RDI_BIT | CH432T_IER_THRI_BIT | CH432T_IER_RLSI_BIT | CH432T_IER_MSI_BIT;
      val = CH432T_IER_RDI_BIT | CH432T_IER_RLSI_BIT | CH432T_IER_MSI_BIT
      self._write_reg(CH432T_IER_REG, val)

      # Enable Uart interrupts, and automatic flow control (automatically control pin RTS)
      self._write_reg(CH432T_MCR_REG, CH432T_MCR_RTS_BIT | CH432T_MCR_OUT2 | CH432T_MCR_AFE)

      # Set baud rate
      self.set_baudrate(self._baudrate)

      # self.set_sleep_mode(CH432T_STANDARD_MODE)

  def set_low_power_mode(self, mode):
    '''!
      @brief Close the internal reference clock of the serial port to make it enter low power status
      @param mode :
      @n     CH432T_LOW_POWER_MODE: LOW POWER mode
      @n     CH432T_STANDARD_MODE: STANDARD mode
    '''
    self._reg_bit_update(CH432T_IER_REG, CH432T_IER_LOWPOWER_BIT, mode)

  def set_baudrate(self, baud):
    '''!
      @brief Set baud rate
      @param baud baud rate range: 2~2764800
      @note It is recommended to use general communication baud rate to reduce communication errors such as: 9600, 19200, 115200, etc.
    '''
    # CK2X=0, internal 1/12 frequency division
    prescaler = 0
    clock_rate = CH432T_CLOCK_FREQUENCY / 12
    if baud > 115200:
      # CK2X=1, internal double frequency
      prescaler = CH432T_IER_CK2X_BIT
      clock_rate *= 24
    # Set prescaler
    if CH432T_PORT_1 == self.portnum:   # regular register
      reg = CH432T_IER_REG + 0x08
    else:
      reg = CH432T_IER_REG
    self._reg_bit_update(reg, CH432T_IER_CK2X_BIT, prescaler)

    # Save raw value of LCR register
    lcr = self._read_reg(CH432T_LCR_REG, 1)[0]
    logger.info("lcr = %#x", lcr)
    # Open the LCR divisors for configuration
    self._write_reg(CH432T_LCR_REG, CH432T_LCR_CONF_MODE_A)
    time.sleep(0.002)

    # Set new baud rate
    mode = int(clock_rate / 16 / baud)   # The set value corresponding to the baud rate in the current mode
    self._write_reg(CH432T_DLL_REG, mode & 0xFF)
    self._write_reg(CH432T_DLH_REG, (mode >> 8) & 0xFF)
    # logger.info( "CH432T_DLL_REG = %#x", self._read_reg(CH432T_DLL_REG, 1)[0])
    # logger.info( "CH432T_DLH_REG = %#x", self._read_reg(CH432T_DLH_REG, 1)[0])

    # Put LCR back to the normal mode
    self._write_reg(CH432T_LCR_REG, lcr)

  def reset_output_buffer(self):
    '''!
      @brief Clear the transmitting data in FIFO (TSR is not included)
    '''
    self._reg_bit_update(CH432T_FCR_REG, CH432T_FCR_TXRESET_BIT, CH432T_FCR_TXRESET_BIT)

  def reset_input_buffer(self):
    '''!
      @brief Clear the receiving data in FIFO (RSR is not included)
    '''
    self._reg_bit_update(CH432T_FCR_REG, CH432T_FCR_RXRESET_BIT, CH432T_FCR_RXRESET_BIT)

  def get_INT_status(self, INT_status):
    '''!
      @brief Interrupt identity register, used to analyze and process the interrupt source
      @param INT_status
      @n       fifo_ENS: This bit is for FIFO enabling status, 1: FIFO enabled
      @n       IID3 + IID2 + IID1 + NOINT: 
      @n         0001: priority: none; interrupt type: no interrupt occurs; 
      @n           Interrupt source: no interrupt; method to clear interrupt: none;
      @n         0110: priority:  1; interrupt type: receive line status; 
      @n           Interrupt source: OVERR, PARERR, FRAMEERR, BREAKINT; method to clear interrupt: read LSR;
      @n         0100: priority:  2; interrupt type: for receiving data; 
      @n           Interrupt source: The number of received bytes reaches the trigger point of FIFO; method to clear interrupt: read RBR;
      @n         1100: priority:  2; interrupt type: receiving data timeout; 
      @n           Interrupt source: next data has not been received for over 4 data periods; method to clear interrupt: read RBR;
      @n         0010: priority:  3; interrupt type: THR register empty; 
      @n           Interrupt source: transmit holding register empty, re-enable interrupt when IETHRE changes from 0 to 1; method to clear interrupt: read IIR or write THR;
      @n         0000: priority:  4; interrupt type: MODEM input changes; 
      @n           Interrupt source: △CTS, △DSR, △RI, △DCD; method to clear interrupt: read MSR;
    '''
    INT_status.set_list(self._read_reg(CH432T_IIR_REG, 1))

  def get_lines_status(self, lines_status):
    '''!
      @brief Line status register, used to analyze serial port status by query
      @param lines_status :
      @n       r_fifo_err: set to 1, indicates there is at least one parity_err, frame_err, or break_INT error in the received FIFO.
      @n       t_empty: set to 1, indicates transmitting holding register THR and shift register TSR are both empty.
      @n       THR_EN: set to 1, indicates transmitting holding register THR is empty.
      @n       break_INT: set to 1, indicates BREAK line interval is detected.
      @n       frame_err: set to 1, indicates the read data frame error from the received FIFO, lack of active stop bit.
      @n       parity_err: set to 1, indicates parity check error of data read from the received FIFO occurred.
      @n       fifo_over: set to 1, indicates the received FIFO buffer overflow
      @n       data_ready: set to 1, indicates there is data received from the FIFO, after reading all the data in FIFO, the bit will automatically reset.
    '''
    lines_status.set_list(self._read_reg(CH432T_LSR_REG, 1))

  def get_modem_status(self, modem_status):
    '''!
      @brief MODEM status register, for querying MODEM status
      @param modem_status :
      @n       DCD: This bit is bit flip of pin DCD, 1 indicates pin DCD is active (active low).
      @n       RI: This bit is bit flip of pin RI, 1 indicates pin RI is active (active low).
      @n       DSR: This bit is bit flip of pin DSR, 1 indicates pin DSR is active (active low).
      @n       CTS: This bit is bit flip of pin CTS, 1 indicates pin CTS is active (active low).
      @n       DCD_change: That the bit is set to 1 indicates that pin DCD input status has changed.
      @n       RI_change: That the bit is set to 1 indicates that pin RI input status has changed.
      @n       DSR_change: That the bit is set to 1 indicates that pin DSR input status has changed.
      @n       CTS_change: That the bit is set to 1 indicates that pin CTS input status has changed.
    '''
    modem_status.set_list(self._read_reg(CH432T_MSR_REG, 1))

  def read(self, size):
    '''!
      @brief Read serial data
      @param size Read length of serial data
      @return The read serial data
    '''
    return self.ch432t_port_irq(size)

  def write(self, data):
    '''!
      @brief Write serial data
      @param data The data to be written into serial port
    '''
    if not self.is_open:
      raise portNotOpenError
    if sys.version_info >= (3,0):
      d = list(data)
    else:
      d = map(ord, list(data))
    self._write_reg(CH432T_THR_REG, d)

  def ch432t_port_irq(self, size):
    '''!
      @brief Interrupt port handler function
      @param size Read length of serial data
      @return The read serial data
    '''
    timeout = Timeout(self._timeout)
    int_status = self.INT_status_reg()
    lines_status = self.lines_status_reg()
    while 1:
      int_status.set_list([0])
      self.get_INT_status(int_status)

      if CH432T_IIR_NO_INT_BIT == int_status.int_type:
        pass
      elif CH432T_IIR_RLSE_SRC == int_status.int_type:
        # Interrupt for receiving line status, priority: 1
        # Line status register, used to analyze serial port status by query
        lines_status.set_list([0])
        self.get_lines_status(lines_status)
        logger.info("Unknown LSR interrupt state")
        if lines_status.r_fifo_err:
          # logger.info("lines_status.r_fifo_err")
          # logger.info("lines_status.r_fifo_err(CH432T_RBR_REG)---%#x", self._read_reg(CH432T_RBR_REG, 1)[0])
          pass
        elif lines_status.fifo_over:
          # logger.info("lines_status.fifo_over")
          pass
        else:
          # logger.info("Unknown LSR interrupt state")
          pass
      elif int_status.int_type in [CH432T_IIR_RDI_SRC, CH432T_IIR_RTOI_SRC]:
        # Interrupt for receiving data, priority: 2; interrupt for receiving data timeout priority: 2
        # logger.info("CH432T_IIR_RTOI_SRC")
        return self.ch432t_handle_rx(size)
      # elif CH432T_IIR_THRI_SRC == int_status.int_type:
      #   # THR register empty interrupt, priority: 3
      #   self._write_reg(CH432T_THR_REG, 0x66)
      #   # break
      elif CH432T_IIR_MSI_SRC == int_status.int_type:
        # MODEM output change interrupt, priority: 4
        self.get_modem_status(CH432T_MSR_REG)
        logger.info("msr---%#x", self._read_reg(CH432T_MSR_REG, 1)[0])
        # pass
      else:
        # logger.info("Unknown interrupt state")
        pass

      if timeout.expired():
        return None

  def ch432t_handle_rx(self, size):
    '''!
      @brief Receive interrupt handler function
      @param size Read length of serial data
      @return The read serial data
    '''
    buf = []
    timeout = Timeout(self._timeout)
    lines_status = self.lines_status_reg()
    while 1:
      # Line status register, used to analyze serial port status by query
      lines_status.set_list([0])
      self.get_lines_status(lines_status)
      if lines_status.r_fifo_err:
        # logger.info("lines_status.r_fifo_err")
        pass
      if lines_status.data_ready:
        buf.append(self._read_reg(CH432T_RBR_REG, 1)[0])
      else:
        if size <= len(buf):
          logger.info(buf)
          return bytes(bytearray(buf))
      if timeout.expired():
        return None

  def _reg_bit_update(self, reg, mask, value):
    '''!
      @brief Update register specified bit
      @param reg register address
      @param mask mask, used to specify bit
      @param value written data
    '''
    temp = self._read_reg(reg, 1)[0]
    temp &= ~mask
    temp |= value & mask
    # logger.info("temp = %#x " % temp)
    self._write_reg(reg, temp)

  def _write_reg(self, reg, data):
    '''!
      @brief writes data to a register
      @param reg register address
      @param data written data
    '''
    with ch432t_spi_lock:
      if isinstance(data, int):
        data = [data]
      reg_addr = [0x02 | ( (reg + self.portnum * 0x08) << CH432T_REG_SHIFT )]
      logger.info("portnum = %d, reg = %d, reg_addr = %#x, data = %#x" % (self.portnum, reg , reg_addr[0], data[0]))
      data.insert(0, reg_addr[0])
      self._spi.xfer2(data)
      time.sleep(0.001)
  def _read_reg(self, reg, length):
    '''!
      @brief read the data from the register
      @param reg register address
      @param length read data length
      @return read data list
    '''
    with ch432t_spi_lock:
      reg_addr = [0xFD & ( (reg + self.portnum * 0x08) << CH432T_REG_SHIFT )]
      temp = reg_addr[0]
      # for k in range(0, length):
      reg_addr.append(0xFF)
      self._spi.xfer2(reg_addr)
      if not isinstance(reg_addr[0], int):
        reg_addr = list(map(int, reg_addr))
      reg_addr.pop(0)
      time.sleep(0.0001)
      return reg_addr

  def cancel_read(self):
    '''!
      @brief Except for the serial port and chip problem, SPI communication is likely to be fine
    '''
    if not self.is_open:
      raise portNotOpenError

  def cancel_write(self):
    '''!
      @brief Except for the serial port and chip problem, SPI communication is likely to be fine
    '''
    if not self.is_open:
      raise portNotOpenError

  def flush(self):
    '''!
      @brief Except for the serial port and chip problem, SPI communication is likely to be fine
    '''
    if not self.is_open:
      raise portNotOpenError

  @property
  def out_waiting(self):
    '''!
      @brief Except for the serial port and chip problem, SPI communication is likely to be fine
    '''
    return 0

  @property
  def in_waiting(self):
    '''!
      @brief Except for the serial port and chip problem, SPI communication is likely to be fine
    '''
    return 0


  def _reset_port(self):
    '''!
      @brief Soft reset the port
    '''
    self._reg_bit_update(CH432T_IER_REG, CH432T_IER_RESET_BIT, CH432T_IER_RESET_BIT)

  def set_sleep_mode(self, mode):
    '''!
      @brief Close the clock oscillator to make serial port 0 and 1 enter sleep mode
      @param mode :
      @n     CH432T_SLEEP_MODE: SLEEP mode
      @n     CH432T_STANDARD_MODE: STANDARD mode
    '''
    self._reg_bit_update((CH432T_IER_REG - self.portnum * 0x08), CH432T_IER_SLEEP_BIT, mode)

  def enable_ie_modem(self, mode):
    '''!
      @brief Allow modem to input status change interrupt
      @param mode :
      @n       True: enable the interrupt
      @n       False: disable the interrupt
    '''
    if mode:
      self._reg_bit_update(CH432T_IER_REG, CH432T_IER_MSI_BIT, CH432T_IER_MSI_BIT)
    else:
      self._reg_bit_update(CH432T_IER_REG, CH432T_IER_MSI_BIT, 0)

  def enable_ie_lines(self, mode):
    '''!
      @brief Allow receiving line status interrupt
      @param mode :
      @n       True: enable the interrupt
      @n       False: disable the interrupt
    '''
    if mode:
      self._reg_bit_update(CH432T_IER_REG, CH432T_IER_RLSI_BIT, CH432T_IER_RLSI_BIT)
    else:
      self._reg_bit_update(CH432T_IER_REG, CH432T_IER_RLSI_BIT, 0)

  def enable_ie_thre(self, mode):
    '''!
      @brief Allow transmitting holding register empty interrupt
      @param mode :
      @n       True: enable the interrupt
      @n       False: disable the interrupt
    '''
    if mode:
      self._reg_bit_update(CH432T_IER_REG, CH432T_IER_THRI_BIT, CH432T_IER_THRI_BIT)
    else:
      self._reg_bit_update(CH432T_IER_REG, CH432T_IER_THRI_BIT, 0)

  def enable_ie_recv(self, mode):
    '''!
      @brief Allow receiving data interrupt
      @param mode :
      @n       True: enable the interrupt
      @n       False: disable the interrupt
    '''
    if mode:
      self._reg_bit_update(CH432T_IER_REG, CH432T_IER_RDI_BIT, CH432T_IER_RDI_BIT)
    else:
      self._reg_bit_update(CH432T_IER_REG, CH432T_IER_RDI_BIT, 0)

  def set_fifo_recv_TG_mode(self, mode):
    '''!
      @brief Set the interrupt for receiving FIFO and the trigger point of hardware flow control
      @n     If it’s set to be 1 byte, i.e., the interrupt for receiving data occur when 1 byte is received and pin RTS is automatically disabled when enabling the hardware flow control.
      @param mode :
      @n       CH432T_FCR_RECVTG_LEN_1: for 1 byte;
      @n       CH432T_FCR_RECVTG_LEN_4: for 4 bytes;
      @n       CH432T_FCR_RECVTG_LEN_8: for 8 bytes;
      @n       CH432T_FCR_RECVTG_LEN_14: for 14 bytes
    '''
    self._reg_bit_update(CH432T_FCR_REG, CH432T_FCR_RXLVL_BIT, mode)

  def enable_fifo(self, mode):
    '''!
      @brief Enable fifo
      @param mode :
      @n       True: enable FIFO
      @n       False: disable FIFO (it switches to 16C450 compatible mode after disabling FIFO, equivalent to a FIFO with only one byte)
    '''
    if mode:
      self._reg_bit_update(CH432T_FCR_REG, CH432T_FCR_FIFO_BIT, CH432T_FCR_FIFO_BIT)
    else:
      self._reg_bit_update(CH432T_FCR_REG, CH432T_FCR_FIFO_BIT, 0)

  def enable_DLAB(self, mode):
    '''!
      @brief Enable divisor latch access
      @param mode :
      @n       True: access DLL and DLM
      @n       False: access RBR/THR/IER
    '''
    if mode:
      self._reg_bit_update(CH432T_LCR_REG, CH432T_LCR_DLAB_BIT, CH432T_LCR_CONF_MODE_A)
    else:
      self._reg_bit_update(CH432T_LCR_REG, CH432T_LCR_DLAB_BIT, 0)

  def enable_break(self, mode):
    '''!
      @brief BREAK line interval is forced to occur
      @param mode :
      @n       True: enable forcing to generate BREAK line interval
      @n       False: disable forcing to generate BREAK line interval
    '''
    if mode:
      self._reg_bit_update(CH432T_LCR_REG, CH432T_LCR_TXBREAK_BIT, CH432T_LCR_TXBREAK_BIT)
    else:
      self._reg_bit_update(CH432T_LCR_REG, CH432T_LCR_TXBREAK_BIT, 0)

  def set_parity_mode(self, mode):
    '''!
      @brief Set the format of parity check bit when PAREN is 1 (parity check bit)
      @param mode :
      @n       CH432T_CHECKBIT_ODD: odd parity check, 
      @n       CH432T_CHECKBIT_EVEN: even parity check, 
      @n       CH432T_CHECKBIT_MARK: mark bit (MARK, set to 1), 
      @n       CH432T_CHECKBIT_SPACE: space bit (SPACE, reset).
    '''
    self._reg_bit_update(CH432T_LCR_REG, CH432T_LCR_PARITY_MODE_BIT, mode)

  def enable_parity_bit(self, mode):
    '''!
      @brief Allow to generate parity check bit when transmitting and check that bit when receiving
      @param mode :
      @n       True: Allow to generate parity check bit when transmitting and check that bit when receiving
      @n       False: no parity check bit
    '''
    if mode:
      self._reg_bit_update(CH432T_LCR_REG, CH432T_LCR_PARITY_EN_BIT, CH432T_LCR_PARITY_EN_BIT)
    else:
      self._reg_bit_update(CH432T_LCR_REG, CH432T_LCR_PARITY_EN_BIT, 0)

  def set_stop_bit_mode(self, mode):
    '''!
      @brief Set stop bit number
      @param mode :
      @n     CH432T_STOPBIT_1: 1 stop bit
      @n     CH432T_STOPBIT_2: 1-1.5 stop bits if word length is 5, 2 stop bits otherwise
    '''
    self._reg_bit_update(CH432T_LCR_REG, CH432T_LCR_STOPLEN_BIT, mode)

  def set_word_size_mode(self, mode):
    '''!
      @brief Set word length
      @param mode :
      @n       CH432T_LCR_WORD_LEN_5: 5 data bit, 
      @n       CH432T_LCR_WORD_LEN_6: 6 data bit, 
      @n       CH432T_LCR_WORD_LEN_7: 7 data bit, 
      @n       CH432T_LCR_WORD_LEN_8: 8 data bit.
    '''
    self._reg_bit_update(CH432T_LCR_REG, CH432T_LCR_PARITY_MODE_BIT, mode)

  def enable_MCR_AFE(self, mode):
    '''!
      @brief Allow CTS & RTS hardware automatic flow control
      @param mode :
      @n       True: enable CTS & RTS hardware automatic flow control
      @n       False: disable CTS & RTS hardware automatic flow control
    '''
    if mode:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_AFE, CH432T_MCR_AFE)
    else:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_AFE, 0)

  def enable_MCR_loop(self, mode):
    '''!
      @brief enable test mode for internal loop
      @param mode :
      @n       True: enable test mode for internal loop
      @n       False: disable test mode for internal loop
    '''
    if mode:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_LOOP_BIT, CH432T_MCR_LOOP_BIT)
    else:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_LOOP_BIT, 0)

  def enable_MCR_out2(self, mode):
    '''!
      @brief Allow interrupt request output of the serial port, otherwise no actual interrupt request of the serial port will occur
      @param mode :
      @n       True: enable interrupt request output of the serial port 
      @n       False: no actual interrupt request of the serial port will occur
    '''
    if mode:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_OUT2, CH432T_MCR_OUT2)
    else:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_OUT2, 0)

  def enable_MCR_out1(self, mode):
    '''!
      @brief This bit is user-definable MODEM control bit and is not connected to the actual output pin
      @param mode :
      @n       True: definable
      @n       False: definable
    '''
    if mode:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_OUT1, CH432T_MCR_OUT1)
    else:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_OUT1, 0)

  def enable_MCR_RTS(self, mode):
    '''!
      @brief When the bit is set to 1, RTS pin output is valid (active low), otherwise it's invalid.
      @param mode :
      @n       True: RTS pin output is valid (active low)
      @n       False: RTS pin output is invalid
    '''
    if mode:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_RTS_BIT, CH432T_MCR_RTS_BIT)
    else:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_RTS_BIT, 0)

  def enable_MCR_DTR(self, mode):
    '''!
      @brief When the bit is set to 1, DTR pin output is valid (active low), otherwise it's invalid.
      @param mode :
      @n       True: DTR pin output is valid (active low)
      @n       False: DTR pin output is invalid
    '''
    if mode:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_DTR_BIT, CH432T_MCR_DTR_BIT)
    else:
      self._reg_bit_update(CH432T_MCR_REG, CH432T_MCR_DTR_BIT, 0)
