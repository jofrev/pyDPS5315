from __future__ import print_function

import dataclasses
import enum
import logging
import sys
import threading
from struct import pack, unpack
from time import sleep

import crcmod
import serial

logging.basicConfig()
logger = logging.getLogger(__name__)

crc16 = crcmod.mkCrcFun(poly=0x18005, rev=False, initCrc=0x800D, xorOut=0x0000)

STX = b"\x02"  # Start symbol
ETX = b"\x03"  # End symbol
ACK = b"\x06"


class Mode(enum.IntFlag):
    INITIAL = 0x00
    SERIES = 0x00
    DUAL = 0x01
    MASTER_SLAVE = 0x03
    SLAVE_STANDBY_ON = 0x04
    MASTER_STANDBY_ON = 0x08
    REMOTE = 0x10
    LOCK = 0x20
    CALIBRATION = 0x40  # not completely sure this is calibration
    ERROR = 0x80


class Control(enum.IntFlag):
    INITIAL = 0x00
    MASTER_VOLTAGE = 0x01
    MASTER_CURRENT = 0x02
    SLAVE_VOLTAGE = 0x04
    SLAVE_CURRENT = 0x08
    OVERTEMP_OUTPUT = 0x10
    OVERTEMP_TRAFO = 0x20


@dataclasses.dataclass
class TemperatureData:
    """Stores the temperature values of the device"""

    output: int = None
    trafo: int = None


@dataclasses.dataclass
class ChannelData:
    """Stores the output voltage and current of a channel"""

    voltage: int = None
    current: int = None


@dataclasses.dataclass()
class ChannelConfig:
    """Stores the configuration of a channel"""

    version: int = None
    standby: bool = None
    limits: ChannelData = dataclasses.field(default_factory=ChannelData)


class _Channel:
    """Base class for channels

    NOTE: This class must not be instantiated directly. Refer to :class:`MasterChannel` and :class:`SlaveChannel` instead.
    """

    VOLTAGE_CONTROL: Control
    CURRENT_CONTROL: Control
    STANDBY_ON: Mode

    def __new__(cls, *args, **kwargs):
        if cls is _Channel:
            raise TypeError(f"only children of '{cls.__name__}' may be instantiated")
        return super().__new__(cls)

    def __init__(self, dps: "Dps5315"):
        self.dps = dps
        self.output = ChannelData()
        self.config = ChannelConfig()

    def enable(self, en=True):
        """En-/Disable the channel's output"""
        self.config.standby = not en
        if en:
            self.dps.set_mode(self.dps.mode & ~Mode.STANDBY_ON)
        else:
            self.dps.set_mode(self.dps.mode | Mode.STANDBY_ON)

    def set_control_values(self, voltage, current):
        """Change the channel's output limits"""
        self.config.limits.voltage = voltage
        self.config.limits.current = current
        self.dps.set_control_values()


class MasterChannel(_Channel):
    """Class for the Master channel"""

    VOLTAGE_CONTROL = Control.MASTER_VOLTAGE
    CURRENT_CONTROL = Control.MASTER_CURRENT
    STANDBY_ON = Mode.MASTER_STANDBY_ON


class SlaveChannel(_Channel):
    """Class for the Slave channel"""

    VOLTAGE_CONTROL = Control.SLAVE_VOLTAGE
    CURRENT_CONTROL = Control.SLAVE_CURRENT
    STANDBY_ON = Mode.SLAVE_STANDBY_ON


class Dps5315:
    """This class provides control for ELV's DPS5315 digital power supply."""

    def __init__(self, com: serial.Serial):
        self.master = MasterChannel(self)
        self.slave = SlaveChannel(self)
        self.temperature = TemperatureData()
        self.mode = Mode.INITIAL
        self.control_mode = Control.INITIAL
        self.ser = com

    @staticmethod
    def at_port(port):
        """Create a control object using a certain serial port"""
        ser = serial.Serial(port, baudrate=115200, timeout=1)
        return Dps5315(ser)

    def send_instruction_and_receive_response(self, instruction: bytes):
        self.send_instruction(instruction)
        msg = self.receive_response()
        self.parse_response(msg)

    def send_instruction(self, instruction: bytes):
        logger.debug("TX: %s", instruction)
        instruction += pack("!H", crc16(instruction))
        instruction = instruction.replace(STX, b"\x10\x82").replace(ETX, b"\x10\x83")
        line = STX + instruction + ETX
        self.ser.write(line)

    def receive_response(self):
        sleep(0.1)
        frame = b""
        while True:
            ch = self.ser.read()
            frame += ch
            if ch == ETX:
                break

        msg = (
            frame.replace(STX, b"")
            .replace(ETX, b"")
            .replace(b"\x10\x82", STX)
            .replace(b"\x10\x83", ETX)
        )
        if msg.startswith(ACK):
            return msg
        else:
            crc = crc16(msg[:-2])
            crc_ok = pack("!H", crc) == msg[-2:]
            if not crc_ok:
                print(
                    "(CRC ERROR)",
                    raw2hexstring(msg),
                    ", %X %X" % (msg[-2], msg[-1]),
                    ", %X" % crc,
                )
                return b""
            return msg[:-2]

    def parse_response(self, msg):
        logger.debug("RX: %s", msg)

        msg_type = msg[0:1]
        if msg_type == b"x":  # Init
            pass
        elif msg_type == ACK:
            logger.debug("(ACK)")
            sleep(
                0.1
            )  # needed by the DPS, without this it doesn't respond to subsequent data anymore
        elif msg_type == b"c":  # control / limit values
            data = unpack("!BHHHH", msg[1:])
            self.mode = data[0]
            self.master.config.standby = bool(self.mode & Mode.MASTER_STANDBY_ON)
            self.slave.config.standby = bool(self.mode & Mode.SLAVE_STANDBY_ON)
            self.master.config.limits.voltage = data[1] * 0.01
            self.master.config.limits.current = data[2] * 0.001
            self.slave.config.limits.voltage = data[3] * 0.01
            self.slave.config.limits.current = data[4] * 0.001
        elif msg_type == b"i":  # status data
            data = unpack("!BBHHHHBB", msg[1:])
            self.mode = data[0]
            self.master.config.standby = bool(self.mode & Mode.MASTER_STANDBY_ON)
            self.slave.config.standby = bool(self.mode & Mode.SLAVE_STANDBY_ON)
            self.control_mode = data[1]
            self.master.output.voltage = data[2] * 0.01
            self.master.output.current = data[3] * 0.001
            self.slave.output.voltage = data[4] * 0.01
            self.slave.output.current = data[5] * 0.001
            self.temperature.output = data[6]
            self.temperature.trafo = data[7]
        elif msg_type == b"v":  # version
            self.master.config.version = msg[1]
            self.slave.config.version = msg[2]
        elif msg_type == b"m":  # mode
            self.mode = msg[1]
            self.master.config.standby = bool(self.mode & Mode.MASTER_STANDBY_ON)
            self.slave.config.standby = bool(self.mode & Mode.SLAVE_STANDBY_ON)
        else:
            print("unknown message: ", raw2hexstring(msg))

    def init(self):
        """Initialize the device

        Note: This method needs to be called before accessing any of the other methods or members
        """
        self.init_remote()
        self.get_control_values()
        self.get_version()
        self.get_data()
        self.set_mode(
            Mode.REMOTE
            | Mode.MASTER_SLAVE
            | Mode.MASTER_STANDBY_ON
            | Mode.SLAVE_STANDBY_ON
        )  # set mode

    def init_remote(self):
        """Initialize remote control"""
        self.send_instruction_and_receive_response(b"X")

    def get_version(self):
        """Read FW version of both channel's uCs"""
        self.send_instruction_and_receive_response(b"V")

    def get_control_values(self):
        """Read mode register and current and voltage limits of both channels"""
        self.send_instruction_and_receive_response(b"C")

    def set_control_values(
        self,
        master_voltage=None,
        master_current=None,
        slave_voltage=None,
        slave_current=None,
    ):
        """Set current and voltage limits of both channels"""
        if master_voltage is not None:
            self.master.config.limits.voltage = master_voltage
        if master_current is not None:
            self.master.config.limits.current = master_current
        if slave_voltage is not None:
            self.slave.config.limits.voltage = slave_voltage
        if slave_current is not None:
            self.slave.config.limits.current = slave_current
        master_voltage = int(self.master.config.limits.voltage * 100)
        master_current = int(self.master.config.limits.current * 1000)
        slave_voltage = int(self.slave.config.limits.voltage * 100)
        slave_current = int(self.slave.config.limits.current * 1000)
        self.send_instruction_and_receive_response(
            b"T"
            + pack(
                "!HHHH", master_voltage, master_current, slave_voltage, slave_current
            )
        )
        self.get_control_values()

    def get_mode(self):
        """Read the mode register"""
        self.send_instruction_and_receive_response(b"M")

    def set_mode(self, mode):
        """Write the mode register"""
        self.send_instruction_and_receive_response(b"N" + pack("b", mode))  # set mode
        self.get_mode()

    def set_master_slave_mode(self):
        """Switch to Master/Slave mode

        Note: This puts both channels into standby
        """
        if not (self.mode & Mode.MASTER_SLAVE):
            self.set_mode(
                Mode.REMOTE
                | Mode.MASTER_SLAVE
                | Mode.MASTER_STANDBY_ON
                | Mode.SLAVE_STANDBY_ON
            )

    def set_dual_mode(self):
        """Switch to dual/parallel mode

        Note: This puts both channels into standby
        """
        if not (self.mode & Mode.DUAL):
            self.set_mode(
                Mode.REMOTE | Mode.DUAL | Mode.MASTER_STANDBY_ON | Mode.SLAVE_STANDBY_ON
            )

    def set_series_mode(self):
        """Switch to series mode

        Note: This puts both channels into standby
        """
        if not (self.mode & Mode.SERIES):
            self.set_mode(
                Mode.REMOTE
                | Mode.SERIES
                | Mode.MASTER_STANDBY_ON
                | Mode.SLAVE_STANDBY_ON
            )

    def get_data(self):
        """Read mode and control mode register as well as output current and voltage of both channels and temperature"""
        self.send_instruction_and_receive_response(b"I")


class SerialReader(threading.Thread):
    def __init__(self):
        print("create thread")
        threading.Thread.__init__(self)

        self.exitFlag = False

    def run(self):
        global dps
        while True:
            dps.get_data()
            sleep(0.2)
            if self.exitFlag:
                break

    def exit(self):
        self.exitFlag = True


def raw2hexstring(bytestring: bytes, separator=b":"):
    return bytestring.hex(separator, 1)


def connect(port="COM6"):
    global ser, dps, thread
    # connect and disconnect (workaround for ch340 chip bug under linux)
    ser = serial.Serial(port, baudrate=9600)
    ser.close()
    ser = serial.Serial(port, baudrate=115200, timeout=1)
    dps = Dps5315(ser)
    dps.init()

    # dps.setControlValues(master_voltage=0.1, master_current=0.02, slave_voltage=0.3, slave_current=0.04)

    thread = SerialReader()
    thread.start()


def disconnect():
    thread.exit()
    sleep(0.1)  # would be nicer to check if thread has finished
    ser.close()


def print_data():
    global dps
    print(
        "mode: ",
        dps.mode,
        "control_mode: ",
        dps.control_mode,
        "mv: ",
        dps.master.output.voltage,
        "mi: ",
        dps.master.output.current,
        "sv: ",
        dps.slave.output.voltage,
        "si: ",
        dps.slave.output.current,
        "mv_limit: ",
        dps.master.config.limits.voltage,
        "mi_limit: ",
        dps.master.config.limits.current,
        "sv_limit: ",
        dps.slave.config.limits.voltage,
        "si_limit: ",
        dps.slave.config.limits.current,
        "temp_output: ",
        dps.temperature.output,
        "temp_trafo: ",
        dps.temperature.trafo,
    )


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1].startswith("-v"):
        logger.setLevel(logging.DEBUG)

    connect()  # adjust here if you use a none default port

    try:
        while True:
            print_data()
            sleep(0.5)
    except KeyboardInterrupt:
        disconnect()
