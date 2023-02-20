from __future__ import print_function
import enum
import crcmod
import serial
from time import sleep
from struct import pack, unpack
import threading


crc16 = crcmod.mkCrcFun(poly=0x18005, rev=False, initCrc=0x800D, xorOut=0x0000)

STX = b'\x02' # Start symbol
ETX = b'\x03' # End symbol
ACK = b'\x06'

class Mode(enum.IntFlag):
    SERIES              = 0x00
    DUAL                = 0x01
    MASTER_SLAVE        = 0x03
    SLAVE_STANDBY_ON    = 0x04
    MASTER_STANDBY_ON   = 0x08
    REMOTE              = 0x10
    LOCK                = 0x20
    CALIBRATION         = 0x40   # not completely sure this is calibration
    ERROR               = 0x80

class Control(enum.IntFlag):
    MASTER_VOLTAGE  = 0x01
    MASTER_CURRENT  = 0x02
    SLAVE_VOLTAGE   = 0x04
    SLAVE_CURRENT   = 0x08
    OVERTEMP_OUTPUT = 0x10
    OVERTEMP_TRAFO  = 0x20

master_version = None
slave_version = None
mode = None
control_mode = None
mv = 0
mi = 0
sv = 0
si = 0
mv_limit = 0
mi_limit = 0
sv_limit = 0
si_limit = 0
temp_endstufe = None
temp_trafo = None


class SerialReader(threading.Thread):
    def __init__(self):
        print('create thread')
        threading.Thread.__init__(self)
        
        self.exitFlag = False
        
    def run(self):
        while True:
            getData()
            sleep(0.2)
            if self.exitFlag:
                break

    def exit(self):
        self.exitFlag = True


def raw2hexstring(bytestring: bytes, separator=b':'):
    return bytestring.hex(separator, 1)

def sendInstruction(instruction: bytes):
    if DEBUG:
        print('TX: ', end='')
        print(instruction)
    instruction += pack('!H', crc16(instruction))
    instruction = instruction.replace(STX, b'\x10\x82').replace(ETX, b'\x10\x83')
    line = STX + instruction + ETX
    ser.write(line)

def receiveResponse():
    sleep(0.1)
    frame = b''
    while True:
        ch = ser.read()
        frame += ch
        if ch == ETX:
            break
    
    msg = frame.replace(STX, b'').replace(ETX, b'').replace(b'\x10\x82', STX).replace(b'\x10\x83', ETX)
    if msg.startswith(ACK):
        return msg
    else:
        crc = crc16(msg[:-2])
        crc_ok = (pack('!H', crc) == msg[-2:])
        if not crc_ok:
            print('(CRC ERROR)', raw2hexstring(msg), ', %X %X'%(msg[-2], msg[-1]), ', %X'%crc)
            return b''
        return msg[:-2]

def parseResponse(msg):
    global mode, control_mode, mv, mi, sv, si, mv_limit, mi_limit, sv_limit, si_limit, temp_endstufe, temp_trafo, master_version, slave_version

    if DEBUG:
        print('RX: ', msg)
                
    msg_type = msg[0:1]
    if msg_type == b'x': # Init
        pass
    elif msg_type == ACK:
        if DEBUG:
            print('(ACK)', end='')
        sleep(.1) # needed by the DPS, without this it doesn't respond to subsequent data anymore
    elif msg_type == b'c': # control / limit values
        data = unpack('!BHHHH', msg[1:])
        mode = data[0]
        mv_limit = data[1] * 0.01
        mi_limit = data[2] * 0.001
        sv_limit = data[3] * 0.01
        si_limit = data[4] * 0.001
    elif msg_type == b'i': # status data
        data = unpack('!BBHHHHBB', msg[1:])
        mode = data[0]
        control_mode = data[1]
        mv = data[2] * 0.01
        mi = data[3] * 0.001
        sv = data[4] * 0.01
        si = data[5] * 0.001
        temp_endstufe = data[6]
        temp_trafo = data[7]
    elif msg_type == b'v': # version
        master_version = msg[1]
        slave_version = msg[2]
    elif msg_type == b'm': # mode
        mode = msg[1]
    else: 
        print('unknown message: ', raw2hexstring(msg))

def setMode(m):
    sendInstructionAndReceiveResponse(b'N' + pack('b', m)) # set mode

def initRemote():
    sendInstructionAndReceiveResponse(b'X')

def getControlValues():
    sendInstructionAndReceiveResponse(b'C')

def setControlValues(mv_lim=mv_limit, mi_lim=mi_limit, sv_lim=sv_limit, si_lim=si_limit):
    mv_lim = int(mv_lim*100)
    mi_lim = int(mi_lim*1000)
    sv_lim = int(sv_lim*100)
    si_lim = int(si_lim*1000)
    sendInstructionAndReceiveResponse(b'T' + pack('!HHHH', mv_lim, mi_lim, sv_lim, si_lim))

def getVersion():
    sendInstructionAndReceiveResponse(b'V')
    return master_version, slave_version

def getData():
    sendInstructionAndReceiveResponse(b'I')
    return (mode,control_mode, mv, mi, sv, si, temp_endstufe, temp_trafo)

def getMode():
    sendInstructionAndReceiveResponse(b'M')
    return mode

def setMasterSlaveMode():
    if not (mode & Mode.MASTER_SLAVE):
        setMode(Mode.REMOTE | Mode.MASTER_SLAVE | Mode.MASTER_STANDBY_ON | Mode.SLAVE_STANDBY_ON)

def setDualMode():
    if not (mode & Mode.DUAL):
        setMode(Mode.REMOTE | Mode.DUAL | Mode.MASTER_STANDBY_ON | Mode.SLAVE_STANDBY_ON)

def setSeriesMode():
    if not (mode & Mode.SERIES):
        setMode(Mode.REMOTE | Mode.SERIES | Mode.MASTER_STANDBY_ON | Mode.SLAVE_STANDBY_ON)

def enableMaster():
    setMode(mode & ~Mode.MASTER_STANDBY_ON)

def disableMaster():
    setMode(mode | Mode.MASTER_STANDBY_ON)

def enableSlave():
    setMode(mode & ~Mode.SLAVE_STANDBY_ON)

def disableSlave():
    setMode(mode | Mode.SLAVE_STANDBY_ON)

def sendInstructionAndReceiveResponse(instruction: bytes):
    sendInstruction(instruction)
    msg = receiveResponse()
    parseResponse(msg)
    return msg

def init():
    initRemote()
    getControlValues()
    getVersion()
    setMode(Mode.REMOTE | Mode.MASTER_SLAVE | Mode.MASTER_STANDBY_ON | Mode.SLAVE_STANDBY_ON) # set mode

def connect(port='COM6'):
    global ser, thread
    # connect and disconnect (workaround for ch340 chip bug under linux)
    ser = serial.Serial(port, baudrate=9600)
    ser.close()
    ser = serial.Serial(port, baudrate=115200, timeout=1)
    init()
    thread = SerialReader()
    thread.start()

def disconnect():
    thread.exit()
    sleep(0.1) # would be nicer to check if thread has finished
    ser.close()

def printData():
    print('mode: ', mode, 'control_mode: ', control_mode,
          'mv: ',mv, 'mi: ', mi , 'sv: ', sv, 'si: ', si ,
          'mv_limit: ', mv_limit, 'mi_limit: ', mi_limit , 'sv_limit: ', sv_limit, 'si_limit: ', si_limit,
          'temp_endstufe: ', temp_endstufe , 'temp_trafo: ', temp_trafo)

DEBUG = True
DEBUG = False

# setControlValues(mv_lim=0.1, mi_lim=0.02, sv_lim=0.3, si_lim=0.04)

if __name__ == '__main__':
    connect()  # adjust here if you use a none default port
    try:
        while True:
            printData()
            sleep(0.5)
    except KeyboardInterrupt:
        disconnect()
