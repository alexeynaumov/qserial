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

from collections import deque
from json import loads, dumps


def loadJSON(fileName):
    data_file = open(fileName, "r")
    text = data_file.read()
    if not text:
        return {}

    data = loads(text)
    return data


def saveJSON(fileName, data):
    old_data = loadJSON(fileName)
    old_data.update(data)

    text = dumps(old_data, sort_keys=True, indent=4, separators=(',', ': '))
    data_file = open(fileName, "w")
    data_file.write(text)
    data_file.close()


def bytesToString(values, base=10, pad=False):
    '''
    Convert list of integers into string of integers.

    :param values, list of int: list of integers to be converted into string of integers
    :param base, int (2, 8, 10, 16): base of integers in the output string
    :param pad, bool: whether use padding(zfill) or not
    :return: string, string of integers

    >>> bytesToString([0,1,254,255], 16, True)
    '00 01 FE FF'

    >>> bytesToString([0,1,254,255], 2, False)
    '0 1 11111110 11111111'

    '''

    # key: base, value: padding depth
    # base 2  (bin) -> values from 00000000 to 11111111: XXXXXXXX = padding 8 characters
    # base 3  (oct) -> values from 000 to 377: XXX = padding 3 characters
    # base 10 (dec) -> values from 000 to 255: XXX = padding 3 characters
    # base 16 (hex) -> values from 00 to FF: XX = padding 2 characters

    BASE_PADDING = {  # key:base, value:padding
        2: 8,
        8: 3,
        10: 3,
        16: 2
    }

    BASE_CODE = {  # key:base, value:code
         2: "b",
         8: "o",
        10: "d",
        16: "x"
    }

    if list != type(values):
        raise TypeError("object {} must be of type 'list' ".format(values))

    if base not in BASE_PADDING.keys():
        raise ValueError("base {} not in {}".format(base, BASE_PADDING.keys()))

    result = ""
    for value in values:
        if not isinstance(value, int):
            raise TypeError("object {} must be of type 'int' ".format(values))

        value = format(value, BASE_CODE[base]).upper()
        if pad:
            value = value.zfill(BASE_PADDING[base])

        result += value + " "

    return result.strip()


def stringToBytes(values, base=10, delimiter=" "):
    '''
    Convert string of integers into list of integers.

    :param intStr, str: string of integers to be converted into list of integers
    :param base, int (2, 8, 10, 16): base of integers in the string
    :param delimiter, str: delimiter of integers in the string
    :return: list of int: list of integers

    >>> stringToBytes("00 01 FE FF", 16, " ")
    [0, 1, 254, 255]

    >>> stringToBytes("0 1 11111110 11111111", 2, " ")
    [0, 1, 254, 255]

    '''

    if str != type(values):
        raise TypeError("object %s must be of type 'str' ".format(values))

    result = []

    items = values.strip().split(delimiter)
    for item in items:
        if not item:
            continue  # skip empty elements like ''

        value = int(item, base)
        if value < 0 or value > 255:
            raise ValueError("input out of bound:{}".format(value))

        result.append(value)

    return result


class History:
    def __init__(self):
        self.__previous = deque()
        self.__next = deque()

    def __str__(self):
        values = []
        for value in self.__previous:
            values.append(value)

        for value in self.__next:
            values.append(value)

        return " ".join([str(value) for value in values])

    def add(self, value):
        self.__previous.extend(self.__next)
        self.__previous.append(value)
        self.__next.clear()

    def previous(self):
        if self.__previous:
            value = self.__previous.pop()
            self.__next.appendleft(value)
            return value
        else:
            return None

    def next(self):
        if self.__next:
            value = self.__next.popleft()
            self.__previous.append(value)
            return value
        else:
            return None


if __name__ == "__main__":
    import doctest

    doctest.testmod()
