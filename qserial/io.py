# -*- coding: utf-8 -*-

# Copyright (C) 2015-2019 Alexey Naumov <rocketbuzzz@gmail.com>
# Copyright (C) 2001-2013 Chris Liechti <cliechti@gmx.net>
#
# This file is part of qserial.
#
# qserial is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or (at
# your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import os
import array
import fcntl
import select
import termios
import subprocess


class IOException(Exception):
    pass


class IO:
    PARITY_NONE, PARITY_EVEN, PARITY_ODD, PARITY_MARK, PARITY_SPACE = ('N', 'E', 'O', 'M', 'S')
    STOPBITS_ONE, STOPBITS_ONE_POINT_FIVE, STOPBITS_TWO = (1, 1.5, 2)
    DATABITS_FIVE, DATABITS_SIX, DATABITS_SEVEN, DATABITS_EIGHT = (5, 6, 7, 8)

    def __init__(self):
        self.__fd = None
        self.__isOpen = False
        self.__port = None
        self.__baudRate = None
        self.__byteSize = None
        self.__parity = None
        self.__stopBits = None

    def __isOpen(self):
        return self.__isOpen

    def __setPort(self, port):
        if self.isOpen:
            raise IOException("Can't reconfigure open port.")

        if not os.access(port, os.F_OK):
            raise IOException("No such file: {}.".format(port))

        if not os.access(port, os.R_OK|os.W_OK):
            raise IOException("No access rights to: {}.".format(port))

        self.__port = port

    def __port(self):
        return self.__port

    def __setBaudRate(self, baudRate):
        if self.isOpen:
            raise IOException("Can't reconfigure open port.")

        if not hasattr(termios, "B{}".format(baudRate)):
            raise IOException("Invalid baud rate: {}.".format(baudRate))

        self.__baudRate = baudRate

    def __baudRate(self):
        return self.__baudRate

    def __setByteSize(self, byteSize):
        if self.isOpen:
            raise IOException("Can't reconfigure open port.")

        self.__byteSize = byteSize

    def __byteSize(self):
        return self.__byteSize

    def __setParity(self, parity):
        if self.isOpen:
            raise IOException("Can't reconfigure open port.")

        self.__parity = parity

    def __parity(self):
        return self.__parity

    def __setStopBits(self, stopBits):
        if self.isOpen:
            raise IOException("Can't reconfigure open port.")

        self.__stopBits = stopBits

    def __stopBits(self):
        return self.__stopBits

    isOpen = property(__isOpen)
    port = property(__port, __setPort)
    baudRate = property(__baudRate, __setBaudRate)
    byteSize = property(__byteSize, __setByteSize)
    parity = property(__parity, __setParity)
    stopBits = property(__stopBits, __setStopBits)

    def __flushInput(self):
        if not self.isOpen:
            raise IOException("Can't flush closed port.")

        termios.tcflush(self.__fd, termios.TCIFLUSH)

    def __flushOutput(self):
        if not self.isOpen:
            raise IOException("Can't flush closed port.")

        termios.tcflush(self.__fd, termios.TCOFLUSH)

    def open(self):
        process = subprocess.Popen(['lsof', '-t', self.__port], stdout=subprocess.PIPE)
        pids, error = process.communicate()

        if error:
            raise IOException("Error opening port: {}.".format(error))

        if pids:
            pids = bytes(pids).decode().replace("\n", "").strip()

            raise IOException("Port is already open by a process/processes with PID(s): {}".format(pids))

        badOptions = []
        if self.port is None:
            badOptions.append("port")

        if self.baudRate is None:
            badOptions.append("baudRate")

        if self.byteSize is None:
            badOptions.append("byteSize")

        if self.parity is None:
            badOptions.append("parity")

        if self.stopBits is None:
            badOptions.append("stopBits")

        if badOptions:
            raise IOException("Invalid port settings: {}".format(badOptions))

        if self.isOpen:
            raise IOException("Port already open.")

        self.__fd = None

        try:
            self.__fd = os.open(self.port, os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)

        except IOError as exception:
            self.__fd = None
            raise IOException("Error opening port {} {}.".format(self.port, exception.strerror))

        try:
            iflag, oflag, cflag, lflag, ispeed, ospeed, cc = termios.tcgetattr(self.__fd)

        except termios.error as exception:
            raise IOException("Error reading port settings: {}.".format(exception))

        lflag &= ~(termios.ICANON|termios.ECHO|termios.ECHOE|termios.ECHOK|termios.ECHONL|termios.ISIG|termios.IEXTEN)
        oflag &= ~(termios.OPOST)
        iflag &= ~(termios.INLCR|termios.IGNCR|termios.ICRNL|termios.IGNBRK)

        if hasattr(termios, 'IUCLC'):
            iflag &= ~termios.IUCLC

        if hasattr(termios, 'PARMRK'):
            iflag &= ~termios.PARMRK

        # setup baud rate
        try:
            ispeed = ospeed = getattr(termios, 'B{}'.format(self.baudRate))

        except AttributeError:
            raise IOException("Invalid baud rate: {}.".format(self.baudRate))

        # setup char len
        cflag &= ~termios.CSIZE
        if self.byteSize == 8:
            cflag |= termios.CS8

        elif self.byteSize == 7:
            cflag |= termios.CS7

        elif self.byteSize == 6:
            cflag |= termios.CS6

        elif self.byteSize == 5:
            cflag |= termios.CS5

        else:
            raise IOException("Invalid byte size: {}".format(self.byteSize))

        # set up raw mode / no echo / binary
        cflag |= (termios.CLOCAL|termios.CREAD)

        cflag &= ~(termios.PARENB|termios.PARODD)
        if self.parity == IO.PARITY_EVEN:
            cflag |= termios.PARENB

        elif self.parity == IO.PARITY_ODD:
            cflag |= (termios.PARENB | termios.PARODD)

        # cflag |= termios.HUPCL
        cflag &= ~termios.CRTSCTS

        # setup stop bits
        if self.stopBits == IO.STOPBITS_ONE:
            cflag &= ~termios.CSTOPB

        elif self.stopBits == IO.STOPBITS_ONE_POINT_FIVE:
            cflag |= termios.CSTOPB  # there is no POSIX support for 1.5

        elif self.stopBits == IO.STOPBITS_TWO:
            cflag |= termios.CSTOPB

        else:
            raise IOException("Invalid stop bits: {}.".format(self.stopBits))

        cc[termios.VMIN] = 1
        cc[termios.VTIME] = 0

        try:
            termios.tcsetattr(self.__fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])

        except termios.error as error:
            raise IOException("Error applying port settings: {}.".format(error))

        self.__isOpen = True
        self.__flushInput()
        self.__flushOutput()

    def close(self):
        if self.isOpen:
            if self.__fd is not None:
                try:
                    os.close(self.__fd)
                except OSError:
                    pass
                finally:
                    self.__fd = None
            self.__isOpen = False

    def read(self):
        if not self.isOpen:
            raise IOException("Port not open.")
            # return
        try:
            ready, _, _ = select.select([self.__fd], [], [], 1)

        except select.error as error:
            raise IOException("Error reading data from port: {}.".format(error.message))

        if not ready:
            return None

        buffer = array.array('i', [0])
        if -1 == fcntl.ioctl(self.__fd, termios.FIONREAD, buffer, 1):
            raise IOException("Error getting number of bytes available for reading.")

        bytesAvailable = buffer[0]

        try:
            data = os.read(self.__fd, bytesAvailable)

        except OSError as error:
            raise IOException('Error reading data from port: {}.'.format(error.strerror))

        return data

    def write(self, data):
        if not self.isOpen:
            raise IOException("Port not open.")

        bytesToWrite = length = len(data)

        while bytesToWrite > 0:
            try:
                bytesWritten = os.write(self.__fd, data)

            except OSError as error:
                raise IOException('Error writing data to port: {}.'.format(error.strerror))

            try:
                _, ready, _ = select.select([], [self.__fd], [], 1)

            except select.error as error:
                raise IOException("Error writing data to port: {}.".format(error.message))

            if not ready:
                raise IOException("Error writing data to port.")

            data = data[bytesWritten:]
            bytesToWrite -= bytesWritten

        return length
