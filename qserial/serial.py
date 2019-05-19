# -*- coding: utf-8 -*-

# Copyright (C) 2015-2019 Alexey Naumov <rocketbuzzz@gmail.com>
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


from PyQt5.QtCore import pyqtSignal, pyqtSlot, QCoreApplication, QObject, QThread

from qserial.io import IO


class Serial(QObject):
    '''
    Simple serial communications class.
    NOTICE! RXD and TXD are the only pins used.

    Usage:
        import sys
        from PyQt5.QtCore import QCoreApplication
        from qserial.serial import Serial
        from qserial.io import IOException

        application = QCoreApplication(sys.argv)

        def onRead(data):
            print("Read {} byte(s): {}".format(len(data), data))

        serial = Serial()
        serial.port = "/dev/ttyUSB0"
        serial.baudRate = 19200
        serial.byteSize = Serial.DATABITS_SEVEN
        serial.parity = Serial.PARITY_EVEN
        serial.stopBits = Serial.STOPBITS_ONE
        serial.onRead = onRead

        try:
            serial.open()

        except IOException as exception:
            print(exception)
            serial.close()

        else:
            data = b"The quick brown fox jumps over the lazy dog"
            print("Write {} byte(s):{}".format(len(data), data))
            serial.write(data)

        application.exec_()
    '''


    PARITY_NONE, PARITY_EVEN, PARITY_ODD, PARITY_MARK, PARITY_SPACE = (
        IO.PARITY_NONE,
        IO.PARITY_EVEN,
        IO.PARITY_ODD,
        IO.PARITY_MARK,
        IO.PARITY_SPACE
    )

    STOPBITS_ONE, STOPBITS_ONE_POINT_FIVE, STOPBITS_TWO = (
        IO.STOPBITS_ONE,
        IO.STOPBITS_ONE_POINT_FIVE,
        IO.STOPBITS_TWO
    )

    DATABITS_FIVE, DATABITS_SIX, DATABITS_SEVEN, DATABITS_EIGHT = (
        IO.DATABITS_FIVE,
        IO.DATABITS_SIX,
        IO.DATABITS_SEVEN,
        IO.DATABITS_EIGHT
    )

    send = pyqtSignal(bytes)
    quit = pyqtSignal()

    def __init__(self, parent=None):
        QObject.__init__(self, parent)

        self.__io = IO()

        class Reader(QObject):
            '''
            Synchronous data reader.
            '''

            received = pyqtSignal(bytes)
            done = pyqtSignal()

            def __init__(self, io):
                QObject.__init__(self)
                self.io = io
                self.proceedToExit = None

            pyqtSlot()
            def read(self):
                '''
                Infinitely wait for incoming data in an infinite loop. All the incoming data is passed outside with the
                signal.

                :return data: bytes, incoming data
                '''

                self.proceedToExit = False

                while not self.proceedToExit:
                    data = self.io.read()

                    QCoreApplication.processEvents()  # need to process events, and delivered signals

                    if not data:
                        continue

                    self.received.emit(data)

                self.io.close()
                self.done.emit()

            @pyqtSlot()
            def quit(self):
                self.proceedToExit = True


        class Writer(QObject):
            '''
            Synchronous data writer.
            '''

            done = pyqtSignal()

            def __init__(self, io):
                QObject.__init__(self)
                self.io = io
                self.proceedToExit = False
                self.dataToWrite = []


            @pyqtSlot(bytes)
            def write(self, data):
                '''
                Write all the characters of <data> one by one.

                :param data: bytes, outgoing data
                :return: None
                '''

                self.proceedToExit = False

                self.dataToWrite.append(data)

                while len(self.dataToWrite):
                    chunk = self.dataToWrite.pop(0)

                    self.io.write(chunk)

                if self.proceedToExit:
                    self.done.emit()

            @pyqtSlot()
            def quit(self):
                self.proceedToExit = True


        self.__on_read = None  # on-read callback

        self.__readingThread = QThread(self)
        self.__writingThread = QThread(self)

        self.__reader = Reader(self.__io)
        self.__writer = Writer(self.__io)

        self.__reader.moveToThread(self.__readingThread)
        self.__writer.moveToThread(self.__writingThread)

        # TOUCH THE READER ONLY WITH SIGNALS !!
        self.__reader.received.connect(self.__onReadyRead)
        # self.__reader.done.connect(self.__readingThread.quit)
        self.__reader.done.connect(self.deleteLater)

        self.quit.connect(self.__reader.quit)

        self.__readingThread.started.connect(self.__reader.read)
        # self.__readingThread.finished.connect(self.deleteLater)

        #TOUCH THE WRITER ONLY WITH SIGNALS !!
        # self.__writer.done.connect(self.__writingThread.quit)
        self.__writer.done.connect(self.deleteLater)

        self.send.connect(self.__writer.write)
        self.quit.connect(self.__writer.quit)

        # self.__writingThread.finished.connect(self.deleteLater)


    def open(self):
        self.__io.open()

        self.__readingThread.start()
        self.__writingThread.start()

    def close(self):

        self.quit.emit()

        self.__writingThread.wait(2)
        self.__readingThread.wait(2)

        self.__writingThread.quit()
        self.__readingThread.quit()

    @pyqtSlot(bytes)
    def __onReadyRead(self, data):
        if self.__on_read:
            self.__on_read(data)

    def write(self, data):
        self.send.emit(data)

    @property
    def port(self):
        return self.__io.port

    @port.setter
    def port(self, port):
        self.__io.port = port

    @property
    def baudRate(self):
        return self.__io.baudRate

    @baudRate.setter
    def baudRate(self, baudRate):
        self.__io.baudRate = baudRate

    @property
    def byteSize(self):
        return self.__io.byteSize

    @byteSize.setter
    def byteSize(self, byteSize):
        self.__io.byteSize = byteSize

    @property
    def parity(self):
        return self.__io.parity

    @parity.setter
    def parity(self, parity):
        self.__io.parity = parity

    @property
    def stopBits(self):
        return self.__io.stopBits

    @stopBits.setter
    def stopBits(self, stopBits):
        self.__io.stopBits = stopBits

    @property
    def onRead(self):
        return self.__on_read

    @onRead.setter
    def onRead(self, callback):
        self.__on_read = callback

    @property
    def isOpen(self):
        return self.__io.isOpen


if __name__ == "__main__":
    import sys
    from PyQt5.QtCore import QCoreApplication
    from qserial.serial import Serial
    from qserial.io import IOException

    application = QCoreApplication(sys.argv)

    def onRead(data):
        print("Read {} byte(s): {}".format(len(data), data))

    serial = Serial()
    serial.port = "/dev/ttyUSB0"
    serial.baudRate = 19200
    serial.byteSize = Serial.DATABITS_SEVEN
    serial.parity = Serial.PARITY_EVEN
    serial.stopBits = Serial.STOPBITS_ONE
    serial.onRead = onRead

    try:
        serial.open()

    except IOException as exception:
        print(exception)
        serial.close()

    else:
        data = b"The quick brown fox jumps over the lazy dog"
        print("Write {} byte(s):{}".format(len(data), data))
        serial.write(data)

    application.exec_()
