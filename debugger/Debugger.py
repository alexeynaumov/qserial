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

import os
import termios
from distutils.util import strtobool

from PyQt5.QtCore import Qt, QTime, QSettings, QByteArray
from PyQt5.QtGui import QIcon
from PyQt5.QtWidgets import QDialog

from qserial.serial import Serial
from qserial.io import IOException
from qserial.utils import History, bytesToString, stringToBytes

from debugger.ui_Dialog import Ui_Dialog

ICON_ROCKET = os.path.dirname(__file__) + "/icons/rocket.svg"


class Dialog(QDialog, Ui_Dialog):
    def __init__(self, parent=None):
        QDialog.__init__(self, parent)
        
        self.setupUi(self)
        self.__initialize()

    def __del__(self):
        if self.__serial and self.__serial.isOpen:
            try:
                self.__serial.close()

            except Exception as error:
                self.__postText("E[?]: Error closing port.")

    def __initialize(self):
        self.__serial = None

        self.setWindowIcon(QIcon(ICON_ROCKET))

        # Find out all supported baud rates
        attrs = dir(termios)
        baudRates = sorted([int(attr[1:]) for attr in attrs if (("B" == attr[0].upper()) and attr[1:].isdigit())])
        if 0 in baudRates:
            baudRates.remove(0)
        
        baudRateList = []
        for rate in baudRates:
            baudRateList.append(str(rate))

        self.comboBoxBaudRate.addItems(baudRateList)

        self.__history = History()

        self.__loadSettings()

        self.pushButtonSend.clicked.connect(self.onPushButtonSendClicked)
        self.pushButtonOpenClose.clicked.connect(self.onPushButtonOpenCloseClicked)
        self.checkBoxRawText.stateChanged.connect(self.onCheckBoxRawTextStateChanged)
        self.lineEditData.keyPressed.connect(self.__keyPressed)

        self.pushButtonSend.setEnabled(False)
        self.lineEditData.setEnabled(False)

        self.__serialWidgets = list()
        self.__serialWidgets.append(self.lineEditDevice)
        self.__serialWidgets.append(self.comboBoxBaudRate)
        self.__serialWidgets.append(self.comboBoxDataBits)
        self.__serialWidgets.append(self.comboBoxParity)
        self.__serialWidgets.append(self.comboBoxStopBits)
        # self.__serialWidgets.append(self.textEditTraffic)
        
    def __keyPressed(self, key):
        if key in [Qt.Key_Enter, Qt.Key_Return]:
            self.pushButtonSend.click()

        if Qt.Key_Up == key:
            previous = self.__history.previous()
            if previous:
                self.lineEditData.setText(previous)

        if Qt.Key_Down == key:
            next = self.__history.next()
            if next:
                self.lineEditData.setText(next)

    def __enablePortSettings(self):
        for widget in self.__serialWidgets:
            widget.setEnabled(True)

    def __disablePortSettings(self):
        for widget in self.__serialWidgets:
            widget.setEnabled(False)

    def __postText(self, text):
        if self.checkBoxTimestamp.isChecked():
            time = QTime.currentTime().toString()
            self.textEditTraffic.append("{} - {}".format(time, text))

        else:
            self.textEditTraffic.append(text)

    def __saveSettings(self):
        settings = QSettings("Rocket Labs", "qserial-debugger")
        settings.setValue("device", self.lineEditDevice.text())

        settings.setValue("baudRate", self.comboBoxBaudRate.currentIndex())
        settings.setValue("dataBits", self.comboBoxDataBits.currentIndex())
        settings.setValue("parity", self.comboBoxParity.currentIndex())
        settings.setValue("stopBits", self.comboBoxStopBits.currentIndex())
        settings.setValue("format", self.comboBoxFormat.currentIndex())

        settings.setValue("leadingZeroes", self.checkBoxLeadingZeroes.isChecked())
        settings.setValue("timestamp", self.checkBoxTimestamp.isChecked())
        settings.setValue("rawText", self.checkBoxRawText.checkState())

    def __loadSettings(self):
        settings = QSettings("Rocket Labs", "qserial-debugger")
        self.lineEditDevice.setText(str(settings.value("device", "/dev/ttyS0")))

        self.comboBoxBaudRate.setCurrentIndex(int(settings.value("baudRate", 0)))
        self.comboBoxDataBits.setCurrentIndex(int(settings.value("dataBits", 0)))
        self.comboBoxParity.setCurrentIndex(int(settings.value("parity", 0)))
        self.comboBoxStopBits.setCurrentIndex(int(settings.value("stopBits", 0)))
        self.comboBoxFormat.setCurrentIndex(int(settings.value("format", 0)))

        self.checkBoxLeadingZeroes.setChecked(strtobool(settings.value("leadingZeroes", "False")))
        self.checkBoxTimestamp.setChecked(strtobool(settings.value("timestamp", "False")))

        checkBoxState = Qt.CheckState(int(settings.value("rawText", 0)))

        self.checkBoxRawText.setCheckState(checkBoxState)  # setting checkBox "checked" doesn't produce the event "stateChanged"
        self.onCheckBoxRawTextStateChanged(checkBoxState)  # so we call self.onCheckBoxRawTextStateChanged implicitly

    def closeEvent(self, event):
        self.__saveSettings()
        if self.__serial and self.__serial.isOpen:
            try:
                self.__serial.close()

            except Exception as error:
                self.__postText("E[?]: Error closing port.")

        super(Dialog, self).closeEvent(event)
        
    def onCheckBoxRawTextStateChanged(self, state):
        if state == Qt.Checked:
            self.labelFormat.setEnabled(False)
            self.comboBoxFormat.setEnabled(False)
            self.checkBoxLeadingZeroes.setEnabled(False)

        else:
            self.labelFormat.setEnabled(True)
            self.comboBoxFormat.setEnabled(True)
            self.checkBoxLeadingZeroes.setEnabled(True)

    def onRead(self, data):
        if self.checkBoxRawText.isChecked():
            dataFormat = "S"
            text = bytes(data).decode("utf-8")

        else:
            INDEX_BASE = {0: 2, 1: 8, 2: 10, 3: 16}
            index = self.comboBoxFormat.currentIndex()
            base = INDEX_BASE.get(index, None)
            if not base:
                self.__postText("E[?]: Invalid base of a number.")

            data = list(data)
            text = bytesToString(data, base, self.checkBoxLeadingZeroes.isChecked())

            INDEX_FORMAT = {0: "B", 1: "O", 2: "D", 3: "H"}
            dataFormat = INDEX_FORMAT.get(index, None)
            if not dataFormat:
                self.__postText("E[?]: Invalid data format.")

        self.__postText("R[%s:%s]: %s" % (dataFormat, len(data), text))

    def onPushButtonSendClicked(self):
        if not self.__serial.isOpen:
            self.__postText("E[?]: Port is not open.")
            return

        text = self.lineEditData.text().strip()
        if not text:
            self.__postText("E[?]: No input provided.")
            return

        self.__history.add(text)
        
        if self.checkBoxRawText.isChecked():
            data = str(text).encode("utf-8")
            dataFormat = "S"

        else:
            INDEX_BASE = {0: 2, 1: 8, 2: 10, 3: 16}
            index = self.comboBoxFormat.currentIndex()
            base = INDEX_BASE.get(index, None)
            if not base:
                self.__postText("E[?]: Invalid base of a number.")

            try:
                values = stringToBytes(str(text), base)
            except ValueError as error:
                self.__postText("E[?]: Incorrect input: <%s>." % str(error).capitalize())
                return

            data = bytes(values)

            text = bytesToString(values, base, self.checkBoxLeadingZeroes.isChecked())

            INDEX_FORMAT = {0: "B", 1: "O", 2: "D", 3: "H"}
            dataFormat = INDEX_FORMAT.get(index, None)
            if not dataFormat:
                self.__postText("E[?]: Invalid data format.")

        self.lineEditData.clear()
        self.__postText("T[%s:%s]: %s" % (dataFormat, len(data), text))
        self.__serial.write(data)

    def onPushButtonOpenCloseClicked(self):
        if self.__serial and self.__serial.isOpen:
            try:
                self.__serial.close()
                self.__enablePortSettings()
                self.pushButtonOpenClose.setText("Open")
                self.pushButtonSend.setEnabled(False)
                self.lineEditData.setEnabled(False)
            except Exception as error:
                self.__postText("E[?]: Error closing port. %s" % str(error).capitalize())
        else:
            try:
                self.__serial = Serial()
                self.__serial.port = self.lineEditDevice.text()
                self.__serial.baudRate = int(self.comboBoxBaudRate.currentText())
                self.__serial.byteSize = int(self.comboBoxDataBits.currentText())
                self.__serial.parity = [Serial.PARITY_NONE, Serial.PARITY_EVEN, Serial.PARITY_ODD, Serial.PARITY_MARK, Serial.PARITY_SPACE][self.comboBoxParity.currentIndex()]
                self.__serial.stopBits = [Serial.STOPBITS_ONE, Serial.STOPBITS_ONE_POINT_FIVE, Serial.STOPBITS_TWO][self.comboBoxStopBits.currentIndex()]
                self.__serial.onRead = self.onRead
                self.__serial.open()
                self.__disablePortSettings()
                self.pushButtonOpenClose.setText("Close")
                self.pushButtonSend.setEnabled(True)
                self.lineEditData.setEnabled(True)
            except IOException as exception:
                self.__postText(str(exception).capitalize())


if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication

    application = QApplication(sys.argv)

    dialog = Dialog()
    dialog.show()

    sys.exit(application.exec_())
