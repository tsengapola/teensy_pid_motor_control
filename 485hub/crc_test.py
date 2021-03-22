# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import crcmod

# this is a standard CCITT CRC even if it does not look like
# (crcmod applies xorOut to initCrc, so initCrc is in reality 0xffff, not 0)
_CRC_FUNC = crcmod.mkCrcFun(poly=0x18005, initCrc=0xffff, rev=0x4b37, xorOut=0x0000)

data = bytearray.fromhex("010320ab0002")
crc = _CRC_FUNC(data)
data.append(crc & 0xff)
data.append(((crc >> 8) & 0xff))

print (data.hex())
