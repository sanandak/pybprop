#!/usr/bin/env python

"""Parallax Propeller code uploader
Copyright (C) 2007 Remy Blank

This file is part of PropTools.

This program is free software; you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the
Free Software Foundation, version 2. 

License: http://www.gnu.org/licenses/gpl-2.0.html

This program is distributed in the hope that it will be useful, but 
WITHOUT ANY WARRANTY; without even the implied warranty of 
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General 
Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA

Modified January 2015 by Phil Howard <phil@pimoroni.com>
Modifications include support for the Raspberry Pi, mainly a
GPIO-based reset mechanism for GPIO-connected Propeller boards.

Various tweaks to the code have also been made both for general clarity
and to bring it closer to the Python PEP 8 style guidelines.

I have also extensively commented appropriate areas and attempted
to explain in sufficient detail the workings of the protocol.
"""

#import glob
import os
import sys
import time
import machine

# Processor constants
LFSR_REQUEST_LEN   = 250
LFSR_REPLY_LEN     = 250
LFSR_SEED          = ord("P")
CMD_SHUTDOWN       = 0
CMD_LOADRAMRUN     = 1
CMD_LOADEEPROM     = 2
CMD_LOADEEPROMRUN  = 3
EEPROM_SIZE        = 32768

class LoaderError(Exception): pass

def do_nothing(msg):
    """Do nothing progress callback."""
    pass

class Loader():
    """Propeller code uploader."""
    
    def __init__(self):
        self.serial = machine.UART(4, 115200, timeout=0, read_buf_len=300)
        self.reset_gpio = machine.Pin('X3', machine.Pin.OUT)
        self.gpio = None

    def _cleanup(self):
        self.serial.deinit()
    
    def __del__(self):
        self._cleanup()

    def __enter__(self):
        return self

    def __exit__(self, type, value, tb):
        self._cleanup()
        
    def _lfsr(self, seed):
        """Generate bits from 8-bit LFSR with taps at 0xB2."""
        while True:
            yield seed & 0x01
            seed = ((seed << 1) & 0xfe) | (((seed >> 7) ^ (seed >> 5) ^ (seed >> 4) ^ (seed >> 1)) & 1)

    # High-level functions
    def get_version(self, progress=do_nothing):
        """Connect to the Propeller and return its version."""
        self._open()
        print(self.reset_gpio)
        print(self.serial)
        try:
            version = self._connect()
            self._write_long(CMD_SHUTDOWN)
            time.sleep(0.010)
            self.reset()
            return version
        finally:
            self._close()
        
    def upload(self, code=None, path=None, eeprom=False, run=True, progress=do_nothing, terminal=False):
        """Connect to the Propeller and upload code to RAM or EEPROM."""
        print("reading ", path)
        if path is not None:
            f = open(path, "rb")
            try:
                code = f.read()
            finally:
                f.close()
        self._open()
        ver = self._connect()
        print("upload: ver = ", ver)
        time.sleep(0.010)

        try:
            code, code_len = self._prepare_code(code, eeprom)
        finally:
            if terminal:
                while True:
                    ser = self.serial.read()
                    if ser != None:
                        sys.stdout.write(ser)
                        #sys.stdout.flush()
            else:
                self._close()
    
    # Low-level functions
    def _open(self):
        pass
    def _close(self):
        pass
        
    def reset(self):
        """Reset the Propeller
        """
        #self.serial.flushOutput()

        self.reset_gpio.value(0)
        time.sleep(0.025)
        self.reset_gpio.value(1)
        time.sleep(0.090)
        
    def _calibrate(self):

        """Send calibration pulse to the Propeller

        This exploits the start bit in the RS232 Serial protocol
        which counts as one single pulse.

        0xf9 = 0b11111001

        So the actual pulse sent is ( LSB first ):

        _-__-----
        010011111

        Which is a single t ( one ) pulse followed by a
        2t ( two ) pulse in Propeller speak.

        This calibrates a single bit of the serial transmission as "t"
        and by using either 0xff ( 255 ) or 0xfe ( 254 ) we can send
        either a 1 ( t ) or 0 ( 2t ) like so:

        0xff = 0b11111111
        _--------
        011111111

        or

        0xfe = 0b11111110
        __-------
        001111111

        The amount of time between pulses, as long as it is less than 100ms,
        is irrelevant so the bits can be packed closer by clever manipulation
        of the serial protocol ( which probably wont happen here! )

        """
        self._write_byte(b'\xf9')
        
    def _connect(self):
        """Connect to the Propeller and send/receieve the 250 bit
        LFSR handshake and 250 bit response.
        """
        print("in connect")
        self.reset()
        self._calibrate()

        print("clear bus", self.serial.any())
        if self.serial.any() > 0:
            x=self.serial.read(self.serial.any())
            print("read", x)
            
        seq = []
    
        # Prime the LFSR sequence with 500 values. LFSR wraps at 256
        for (i, value) in zip(range(LFSR_REQUEST_LEN + LFSR_REPLY_LEN), self._lfsr(LFSR_SEED)):
            seq.append(value)

        # Send the first 250 values from the LFSR
        for each in seq[0:LFSR_REQUEST_LEN]:
            self.serial.write(bytes([each | 0xfe]))

        print("sent 250 lfsr")
    
        # Send 258 templates to "clock" the return values back to us
        # These are 250 bits of LFSR respone, plus
        # 8 bits denoting the Propeller version
        self.serial.write(b'\xf9' * (LFSR_REPLY_LEN + 8))

        print("sent 258 clocks for version")

        # Prop will return the last 6 values of the LFSR and then wrap
        # returning 244 additional values
        for i in range(LFSR_REQUEST_LEN, LFSR_REQUEST_LEN + LFSR_REPLY_LEN):
            x = self._read_bit(False, 0.100)
            if x != seq[i]:
                print("loader err")
                raise LoaderError("No hardware found")

        print("Successfully verified prop response")

        # Prop will return 8 bits denoting the Propeller version
        version = 0
        for i in range(8):
            version = ((version >> 1) & 0x7f) | ((self._read_bit(False, 0.050) << 7))
        return version

    def _bin_to_eeprom(self, code):
        if len(code) > EEPROM_SIZE - 8:
            raise LoaderError("Code too long for EEPROM (max %d bytes)" % (EEPROM_SIZE - 8))
        dbase = ord(code[0x0a]) + (ord(code[0x0b]) << 8)
        if dbase > EEPROM_SIZE:
            raise LoaderError("Invalid binary format")
        code += "".join(chr(0x00) * (dbase - 8 - len(code)))
        code += "".join(chr(each) for each in [0xff, 0xff, 0xf9, 0xff, 0xff, 0xff, 0xf9, 0xff])
        code += "".join(chr(0x00) * (EEPROM_SIZE - len(code)))
        return code

    def _prepare_code(self, code, eeprom=False):
        """Prepare code and send to prop"""
        print("preparing code...")
        if len(code) == 0:
            raise LoaderError("Empty file specified")

        if len(code) % 4 != 0:
            raise LoaderError("Invalid code size: must be a multiple of 4")

        if eeprom and len(code) < EEPROM_SIZE:
            code = self._bin_to_eeprom(code)

        print("checking checksum")
        checksum = 0x00
        for each in code:
            checksum += each
        #checksum = reduce(lambda a, b: a + b, (ord(each) for each in code))


        if not eeprom:
            checksum += 2 * (0xff + 0xff + 0xf9 + 0xff)

        checksum &= 0xff

        if checksum != 0:
            raise LoaderError("Code checksum error: 0x{:0>2x}".format(checksum))

        print("checksum OK", checksum&0xff)

        code_len = len(code)

        encoded_binary = bytearray(0)

        # I'm not comfortable with the implicit conversion of True/False to 1/0
        if eeprom:
            command = CMD_LOADEEPROMRUN #eeprom * 2 + 1 # run = 1
        else:
            command = CMD_LOADRAMRUN

        print("Sending command", command)
        self._write_long(command)

        # Send the total length of the upload in longs
        self._write_long(code_len // 4)
        print("Sending code ({} bytes)".format(code_len))

        for i in range(0, len(code), 4):
            x = self._encode_long(code[i] | (code[i + 1] << 8) | (code[i + 2] << 16) | (code[i + 3] << 24))
            #encoded_binary.extend(bytearray(x11))
            for b in x:
                self.serial.write(bytes([b]))
            #self.serial.write(bytearray(x))

        print("Done sending code", code_len)

        # Wait for achknowledge
        #x = self._read_bit(True, 0.25)
        #print("RAM checksum OK", x)
        if self._read_bit(True, 0.25) == 1:
            raise LoaderError("RAM checksum error")
        if eeprom:
            print("Programming EEPROM")
            if self._read_bit(True, 5) == 1:
                raise LoaderError("EEPROM programming error")
            print("Verifying EEPROM")
            if self._read_bit(True, 2.5) == 1:
                raise LoaderError("EEPROM verification error")

        print("Ack ok")
        return encoded_binary, code_len

    # Lowest-level functions
    def _write_byte(self, value):
        """Write a single byte of data to the serial port.
        The byte must be first transformed to represent one or more
        bits of data in the Propeller protocol.
        """
        self.serial.write(value)
        
    def _write_long(self, value):
        #print("write long", self._encode_long(value))
        self.serial.write(bytearray(self._encode_long(value)))

    def _encode_long(self, value):
        """Encode a 32-bit long as short/long pulses."""
        result = []
        for i in range(10):
            result.append(0x92 | (value & 0x01) | ((value & 2) << 2) | ((value & 4) << 4))
            value >>= 3
        result.append(0xf2 | (value & 0x01) | ((value & 2) << 2))
        return (result)
        
    def _read_bit(self, echo, timeout):
        """Read a single bit back from the Propeller."""
        start = time.time()
        while time.time() - start < timeout:
            if echo:
                self._write_byte(b'\xf9')
                time.sleep(0.025)
            c = self.serial.read(1)
            if c:
                if c in (b'\xfe', b'\xff'):
                    return ord(c) & 0x01
                else:
                    raise LoaderError("Bad reply")
        raise LoaderError("Timeout error")


def upload(path, eeprom=False, run=True, progress=do_nothing, terminal=False):
    """Upload file on given serial port and call the progress handler when done.

    Arguments:
    path -- File path to Propeller .eeprom or .binary file

    Keyword arguments:
    eeprom -- Boolean. Pass True to upload binary file at path to EEPROM otherwise
    it will be uploaded to RAM only.
    run -- Boolean. Pass True to run the uploaded binary when done.
    progress -- Progress handler, must accept a single message string.
    """
    with Loader() as loader:
        progress("Uploading {}".format(path))
        loader.upload(path=path, eeprom=eeprom, run=run, progress=progress, terminal=terminal)
        progress("Done")

def _action_get_version():
    """Get the version of the connected Propeller chip."""
    print("in get ver")
    try:
        with Loader() as loader:
            version = loader.get_version()
            if version > 0:
                return version
    except (LoaderError) as e:
        return -1

def _action_upload(filename, destination="RAM", progress=do_nothing, terminal=False):
    if filename.endswith(".eeprom"):
        destination = "EEPROM"
    else:
        destination = destination

    try:
        upload(filename, eeprom=(destination == "EEPROM"), progress=progress, terminal=terminal)
    except (SystemExit, KeyboardInterrupt):
        return 3
    except Exception as e:
        sys.stderr.write(str(e) + "\n")
        return 1

def print_status(msg):
    """Print status messages."""
    print(msg)

if __name__ == "__main__":
    """invoke with >>loader.py file[.eeprom|.binary] [-t]"""
    try:
        v = _action_get_version()
        print("version", v)
    except Exception as e:
        print("get version failed", e)

    if len(sys.argv) == 1: # get prop version and quit
        break

    file = sys.argv[1]
    print("Uploading ", file)
    
    # optional terminal echo
    if len(sys.argv) == 3 and sys.argv[2] == "-t"
        term = True
    else
        term = False

        try:
            _action_upload(file, progress=print_status, terminal=term)
        except Exception as e:
            print("upload failed", e)

