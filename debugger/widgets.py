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

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtWidgets import QLineEdit


class LineEdit(QLineEdit):
    keyPressed = pyqtSignal(int)

    def __init__(self, parent=None):
        QLineEdit.__init__(self, parent)

    def keyPressEvent(self, event):
        key = event.key()
        if key in [Qt.Key_Return, Qt.Key_Enter, Qt.Key_Up, Qt.Key_Down]:
            self.keyPressed.emit(key)

        QLineEdit.keyPressEvent(self, event)


if __name__ == "__main__":
    import sys
    from PyQt5.QtWidgets import QApplication

    application = QApplication(sys.argv)

    lineEdit = LineEdit("")
    lineEdit.show()

    sys.exit(application.exec_())
