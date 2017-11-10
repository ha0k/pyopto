# -*- coding: utf-8 -*-
import serial
import time

class Opto(object):
    """
    # Pyopto

    ## example
        >>> from pyopto.pyopto import Opto
        >>> o = Opto("COM4")
        >>> o.mode('D')
        >>> o.current(100)
        >>> o.mode('C')
        >>> o.focul_power(5)
        >>> o.close()
    
    See Optotune Lens Driver 4 Manual for details.
    """

    def __init__(self, port=None):        
        self.ser = None
        self.is_connected = False
        self.max_current = 292.84
        self.crc_table = self._init_crc16ibm_table()
        self._mode = None # current mode
        self._firmware_type = None
        if port:
            self.connect(port)            

    def __exit__(self):
        self.close()

    def __repr__(self):
        if self.is_connected:
            return "<Opto port: {}>".format(self.ser.port)
        return "<Opto port: None>"
            
    def __str__(self):
        if self.is_connected:
            return "Opto object connected with serial port {}.".format(self.ser.port)
        return "Opto object without any connection."
        
    def connect(self, port):
        """Connect to the device via serial port.
        Note:
            You need to use this method if you don't put port name in initialization.
        Args:
            port: port name (check device manager).
        Todo:
            fix unstable get command.
        """
        try:
            self.ser = serial.Serial()
            self.ser.baudrate = 115200
            self.ser.port = port
            self.ser.timeout = 0.2
            self.ser.open()
        except serial.SerialException:
            raise serial.SerialException("Couldn't open port {}.".format(port))

        try:
            if self._handshake():
                self.is_connected = True
        except:
            raise Exception("Handshake failed.")
        self._mode = self.mode()

    def close(self):
        """Close port.
        """
        if self.ser:
            self.ser.close()
            del(self.ser)
            self.is_connected = False

    def connection_required(func):
        """Decorator for connection required methods.
        """
        def check_connection(self, *args, **kwargs):
            if self.is_connected:
                return func(self, *args, **kwargs)
            else:
                raise Exception("You need to connect first!")
        return check_connection

    def mode_required(required_mode):
        """Decorator checking if the device is in appropriate mode.
        """
        def inner_mode_required(func):
            def check_mode(self, *args, **kwargs):
                if self._mode in required_mode:
                    return func(self, *args, **kwargs)
                else:
                    print("You need to set mode: {}".format(required_mode))
            return check_mode
        return inner_mode_required

    def _send(self, cmd, has_response=False, with_crc=False):
        """Send a command with crc and return if there is a response.
        Note:
            CRC table is required. 
        Args:
            cmd: bytes command.
            has_response(bool): has response?
            with_crc(bool): requires crc?
        Returns:
            res: bytes response.
        """
        if not self.ser:
            raise Exception("Serail port is not connected. Please connect first.")
        if with_crc:
            cmd = cmd + self._crc(cmd)
        self.ser.write(cmd)
        if has_response:
            res = self._recv(with_crc)
            return res

    def _recv(self, with_crc):
        """Receive a response.
        Note:
            Response message has \n and \r at the end.
        Args:
            with_crc(bool): requires crc?
        Returns:
            res: bytes response.
        """
        if not self.ser:
            raise Exception("Serail port is not connected. Please connect first.")
        
        res = self.ser.readline()
        if with_crc:
            res_crc = res[-4:-2]
            res = res[:-4]
            if res_crc != self._crc(res):
                raise Exception("CRC validation failed.")
        if res[0] == b'E':
            raise Exception("Error in sent command.")
        return res
            
    def _handshake(self):
        """Conduct a handshake to confirm a connection.
        Note:
            The handshake command is used to check if the hardware is ready and running.
            It can also be used as a reset function as it will reset the current to zero.
            Other commands are also valid without sending this command after initialization, it is optional.
        Returns:
            True when handshake has done successfully.
        """
        res = self._send(b'Start', has_response=True)
        if res == b'Ready\r\n':
            return True
        return False

    def _crc(self, cmd):
        """Get CRC bytes.
        Note:
            reimplementation of sample in the manual.
        Args:
            command in bytes.
        Returns:
            2 crc bytes.
        """
        crc = 0
        for d in cmd:
             crc = (crc >> 8) ^ self.crc_table[(crc ^ d) & 0x00ff]
        return crc.to_bytes(2, byteorder='little')
        
    def _init_crc16ibm_table(self):
        """Initialize crc table.
        Returns:
            crc table (list length of 256).
        """
        polynomial = 0xA001
        table = []
        for i in range(256):
            value = 0
            tmp = i
            for j in range(8):
                if (value ^ tmp) & 0x0001 != 0:
                    value = value >> 1 ^ polynomial
                else:
                    value = value >> 1
                tmp = tmp >> 1
            table.append(value)
        return table

    @connection_required
    @mode_required(('D',))
    def current(self, value=None):
        """Set or Get current.
        Note:
            unit of the value is mA
            -292.84mA < value < 292.84mA
            Focal distance will be about 3m when value is set to 100mA.           
        Returns:
            value
        """
        if value is None:
            res = self._send(b'Ar\x00\x00', has_response=True, with_crc=True)
            cur = int.from_bytes(res[1:], byteorder='big',signed=True) / 4096 * self.max_current
        else:
            xi = int(value / self.max_current * 4096).to_bytes(2, byteorder='big', signed=True)
            res = self._send(b'Aw'+xi, has_response=False, with_crc=True)
        return value

    @connection_required
    @mode_required(('C',))
    def focal_power(self, value=None):
        """Set or Get focal power.
        Note:
            unit is diopter.
            When firmware type is 'A', you need to add 5 to value.
        Todo:
           Not working.
        """
        if value is None:
            res = self._send(b'PrDA\x00\x00\x00\x00', has_response=True, with_crc=True)
            fp = int.from_bytes(res[2:4], byteorder='big', signed=True) / 200 - 5
        else:
            data = int((value + 5) * 200).to_bytes(2, byteorder='big', signed=True)
            self._send(b'PwDA' + data + b'\x00\x00', with_crc=True)
            fp = value
        return fp

    @connection_required
    @mode_required(('S', 'Q', 'T'))
    def signal_generator_frequency(self, value=None):
        """Set or Get channel A frequency.
        Note:
            Command can only be used in sinusoidal, rectangular, and triangular mode.
        """
        if value is None:
            res = self._send(b'PrFA\x00\x00\x00\x00', has_response=True, with_crc=True)
            self._siggen_freq = int.from_bytes(res[3:7], byteorder='big')
        else:
            data = int(value * 1000)
            data = data.to_bytes(4, byteorder='big', signed=False)
            self._send(b'PwFA' + data, with_crc=True)
            self._siggen_freq = value
        return self._siggen_freq

    @connection_required
    @mode_required(('S', 'Q', 'T'))
    def signal_generator_lower(self, value=None):
        """Set or Get the lower current swing limit for channel A.
        Note:
            Command can only be used in sinusoidal, rectangular, and triangular mode.
        """
        if value is None:
            res = self._send(b'PrLA\x00\x00\x00\x00', has_response=True, with_crc=True)
            self._siggen_lower = (int.from_bytes(res[3:5], byteorder='big', signed=True) * self.max_current / 4095)
        else:
            data = int(value * 4095/ self.max_current).to_bytes(2, byteorder='big', signed=True)
            self._send(b'PwLA'+data+b'\x00\x00', with_crc=True)
            self._siggen_lower = value
        return self._siggen_lower

    @connection_required
    @mode_required(('S', 'Q', 'T'))
    def signal_generator_upper(self, value=None):
        """Set or Get the upper current swing limit for channel A.
        Note:
            Command can only be used in sinusoidal, rectangular, and triangular mode.
        """
        if value is None:
            res = self._send(b'PrUA\x00\x00\x00\x00', has_response=True, with_crc=True)
            self._siggen_lower = (int.from_bytes(res[3:5], byteorder='big', signed=True) * self.max_current / 4095)
        else:
            data = int(value * 4095/ self.max_current).to_bytes(2, byteorder='big', signed=True)
            self._send(b'PwUA'+data+b'\x00\x00', with_crc=True)
            self._siggen_lower = value
        return self._siggen_lower
    
    @connection_required
    def maximum_current_calibration(self, value=None):
        """Set or Get Max Output Current.
        """
        if value is None:
            res = self._send(b'CrMA\x00\x00', has_response=True, with_crc=True)
            self.max_current = int.from_bytes(res[3:5], byteorder='big', signed=True) / 100
        else:
            data = (value * 100).to_bytes(2, byteorder='big', signed=True)
            self._send(b'CwMA' + data)
            self.max_current = value
        return self.max_current

    @connection_required
    def calibration_factor(self, value=None):
        """Set or Get calibration factor.
        """
        if value is None:
            res = self._send(b'CrCA\x00\x00', has_response=True, with_crc=True)
            self.max_current = int.from_bytes(res[3:5], byteorder='big', signed=True) / 100
        else:
            data = (value * 100).to_bytes(2, byteorder='big', signed=True)
            self._send(b'CwCA' + data)
            self.max_current = value
        return self.max_current

    @connection_required
    def upper_current_limit(self, value=None):
        """Set or Get upper software current limit.
        """
        if value is None:
            res = self._send(b'CrUA\x00\x00', has_response=True, with_crc=True)
        else:
            data = int(value / self.max_current * 4096).to_bytes(2, byteorder='big', signed=True)
            res = self._send(b'CwUA' + data)
        self.upper_current = int.from_bytes(res[3:5], byteorder='big', signed=True) * self.max_current / 4096
        return self.upper_current

    @connection_required
    def lower_current_limit(self, value=None):
        """Set or Get lower software current limit.
        """
        if value is None:
            res = self._send(b'CrLA\x00\x00', has_response=True, with_crc=True)
        else:
            data = int(value / self.max_current * 4096).to_bytes(2, byteorder='big', signed=True)
            res = self._send(b'CwLA' + data)
        self.lower_current = int.from_bytes(res[3:5], byteorder='big', signed=True) * self.max_current / 4096
        return self.lower_current

    @connection_required
    def mode(self, mode_code=None):
        """Set or Get current mode.
        Note:
            current    : D
            sinusoidal : S
            trianglular: T
            rectangular: Q
            focal      : C
            analog     : A
 
        """
        if mode_code is None:
            modes = [
                'D', # 'current',
                'S', # 'sinusoidal',
                'T', # 'triangular',
                'Q', # 'rectangular',
                'C', # 'focal',
                'A', # 'analog',
                'P', # 'controlled'
            ]
            res = self._send(b'MMA', has_response=True, with_crc=True)
            self._mode = modes[res[3] - 1]
        else:
            if mode_code not in ['D', 'S', 'T', 'Q', 'C', 'A']:                
                raise ValueError('There is no mode labeled {}.'.format(mode_code))
            mode_code_b = bytes(mode_code, encoding='utf-8')
            res = self._send(b'Mw' + mode_code_b + b'A', has_response=True, with_crc=True)
            if res[1] != ord(mode_code_b):
                raise Exception("Failed to set mode.")
            self._mode = mode_code
        return self._mode

    @connection_required
    def temperature_limits(self, value=None):
        """Set or Get the upper and lower temperature limits.
        """
        if value is None:
            res = self._send(b'PrTA\x00\x00\x00\x00', has_response=True, with_crc=True)
            return (int.from_bytes(res[2:4], byteorder='big',signed=True) / 16,
                    int.from_bytes(res[4:6], byteorder='big',signed=True) / 16)
        else:
            # higher limit > lower limit
            if value[0] > value[1]:
                raise(ValueError)
            data = ((value[1] * 16).to_bytes(2, byteorder='big', signed=True) +
                    (value[0] * 16).to_bytes(2, byteorder='big', signed=True))
            res = self._send(b'PwTA' + data, has_response=True, with_crc=True)

    @connection_required
    def temperature(self):
        """Returns the temperature reading of the lens of channel A.
        """
        res = self._send(b'TCA', has_response=True, with_crc=True)
        return int.from_bytes(res[3:5], byteorder='big', signed=True) * 0.0625

    @connection_required
    @mode_required(('A',))
    def analog_input(self):
        res = self._send(b'GAA')
        return int.from_bytes(res[3:5], byteorder='big', signed=False)

    @connection_required
    def serial_number(self):
        """Returns the serial number of the connected lens.
        """
        res = self._send(b'X', has_response=True, with_crc=True)
        return res[1:]

    @connection_required
    def firmware_type(self):
        """Returns the firmware type of the compiled firmware.
        """
        res = self._send(b'H', has_response=True, with_crc=True)
        return res[1]

    @connection_required
    def firmware_branch(self):
        """Returns the firmware branch of the compiled firmware.
        """
        res = self._send(b'F', has_response=True, with_crc=True)
        return res[1]

    @connection_required
    def part_number(self):
        """Returns the part number of the compiled firmware.
        """
        res = self._send(b'J', has_response=True, with_crc=True)
        return res[1:4]

    @connection_required
    def firmware_version(self):
        """Get firmware version.
        """
        res = self._send(b'V', has_response=True, with_crc=True)
        return '{}.{}.{}.{}'.format(
            res[1],                                      # major
            res[2],                                      # minor
            int.from_bytes(res[3:5], byteorder='big'),   # build
            int.from_bytes(res[4:7], byteorder='big'))   # revision

    @connection_required
    def device_id(self, ):
        """Set or Get device id.
        Note:
            There is a set command but it is for debug mode.
        Returns:
            Device ID in bytes.
        """
        res = self._send(b'IR\x00\x00\x00\x00\x00\x00\x00\x00', has_response=True, with_crc=True)
        return res[2:]

    @connection_required
    def status(self):
        """Return the firmware status information.
           This commands was the status byte on the Temperature Reading command.
        """
        res = self._send(b'Sr', has_response=True, with_crc=True)
        return res[1:]

    @connection_required
    def eeprom_read(self, value):
        data = int(value).to_bytes(1, byteorder='big', signed=True)
        res = self._send(b'Zr' + data, has_response=True, with_crc=True)
        return res[1]

    @connection_required
    def eeprom_write(self, address, value):
        register_id = int(address).to_bytes(1, byteorder='big', signed=True)
        data = int(value).to_bytes(1, byteorder='big', signed=True)
        res = self._send(b'Zw' + register_id + data, has_response=True, with_crc=True)
        return res[1]

    @connection_required
    def eeprom_contents(self):
        res = self._send(b'D\x00\x00', has_response=True, with_crc=True)
        return res[1:]

    @connection_required
    def reset(self):
        """
        Restart the device in DFU mode.
        """
        self._send(b'RESET', with_crc=True)        
