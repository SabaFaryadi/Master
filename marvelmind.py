#!/usr/bin/env python
#
# marvelmind.py - small class for recieve and parse coordinates from Marvelmind mobile beacon by USB/serial port
# Written by Alexander Rudykh (awesomequality@gmail.com)
#
### Attributes:
#
#	tty - serial port device name (physical or USB/virtual). It should be provided as an argument: 
#   	/dev/ttyACM0 - typical for Linux / Raspberry Pi
#   	/dev/tty.usbmodem1451 - typical for Mac OS X
#
#	baud - baudrate. Should be match to baudrate of hedgehog-beacon
#		default: 9600
#
#	maxvaluescount - maximum count of measurements of coordinates stored in buffer
#		default: 3
#
#	lastValues - buffer of measurements
#
#	debug - debug flag which activate console output	
#		default: False
#
#	pause - pause flag. If True, class would not read serial data
#
#	terminationRequired - If True, thread would exit from main loop and stop
#
#
### Methods:
#
#	__init__ (self, tty="/dev/ttyACM0", baud=9600, maxvaluescount=3, debug=False) 
#		constructor
#
#	print_position(self)
#		print last measured data in default format
#
#	position(self)
#		return last measured data in array [adr, x, y, z, timestamp]
#
#	stop(self)
#		stop infinite loop and close port
#
### Needed libraries:
#
# To prevent errors when installing crcmod module used in this script, use the following sequence of commands:
#   sudo apt-get install python-pip
#   sudo apt-get update
#   sudo apt-get install python-dev
#   sudo pip install crcmod
#
###

import crcmod
import serial
import struct
import collections
from time import sleep
from threading import Thread


class MarvelmindHedge (Thread):
	def __init__ (self, tty="/dev/ttyACM0", baud=9600, maxvaluescount=3, debug=False, recieveDataCallback=None):
		self.tty = tty  # serial
		self.baud = baud  # baudrate
		self.debug = debug  # debug flag
		self._bufferSerialDeque = collections.deque(maxlen=255)  # serial buffer
		self.lastValues = collections.deque(maxlen=maxvaluescount) # meas. buffer
		
		self.recieveDataCallback = recieveDataCallback
		
		for x in range(0, 10):
			self.lastValues.append([0,0,0,0,0]) # last measured positions and timestamps; [adr, x, y, z, timestamp]
		self.pause = False
		self.terminationRequired = False
		
		self.serialPort = None
		
		Thread.__init__(self)

	def print_position(self):
		if (isinstance(self.position()[1], int)):
			print ("Hedge {:d}: X: {:d}, Y: {:d}, Z: {:d} at time T: {:.2f}".format(self.position()[0], self.position()[1], self.position()[2], self.position()[3], self.position()[4]/1000.0))
		else:
			print ("Hedge {:d}: X: {:.2f}, Y: {:.2f}, Z: {:.2f} at time T: {:.2f}".format(self.position()[0], self.position()[1], self.position()[2], self.position()[3], self.position()[4]/1000.0)	)

	def position(self):
		return list(self.lastValues)[-1];
	
	def stop(self):
		self.terminationRequired = True
		print ("stopping")

	def run(self):		
		while (not self.terminationRequired):
			if (not self.pause):
				try:
					if (self.serialPort is None):
						self.serialPort = serial.Serial(self.tty, self.baud, timeout=3)
					readChar = self.serialPort.read(1)
					while (readChar is not None) and (readChar is not '') and (not self.terminationRequired):
						self._bufferSerialDeque.append(readChar)
						readChar = self.serialPort.read(1)
						bufferList = list(self._bufferSerialDeque)
						# strbuf = ''.join(bufferList)
						strbuf = (b''.join(bufferList))
						pktHdrOffset = strbuf.find(b'\xff\x47')
						if (pktHdrOffset >= 0 and len(bufferList) > pktHdrOffset + 4 and pktHdrOffset<220):
# 							print(bufferList)
							isMmMessageDetected = False;
							isCmMessageDetected = False;
							pktHdrOffsetCm = strbuf.find(b'\xff\x47\x01\x00')
							pktHdrOffsetMm = strbuf.find(b'\xff\x47\x11\x00')
							if (pktHdrOffsetMm!=-1):
								isMmMessageDetected = True
								if (self.debug): print ('Message with mm-data was detected')
							elif (pktHdrOffsetCm!=-1):
								isCmMessageDetected = True
								if (self.debug): print ('Message with cm-data was detected')
							msgLen = ord(bufferList[pktHdrOffset + 4])
							if (self.debug): print ('Message length: ', msgLen)
							
							# offset+header+message+crc
							#print 'len of buffer: ', len(bufferList), '; len of msg: ', msgLen, '; offset: ', pktHdrOffset
							if (len(bufferList) > pktHdrOffset + 4 + msgLen + 2):
								usnCRC16 = 0
								if (isCmMessageDetected):
									usnTimestamp, usnX, usnY, usnZ, usnAdr, usnCRC16 = struct.unpack_from ('<LhhhxBxxxxH', strbuf, pktHdrOffset + 5)
								elif (isMmMessageDetected):
									usnTimestamp, usnX, usnY, usnZ, usnAdr, usnCRC16 = struct.unpack_from ('<LlllxBxxxxH', strbuf, pktHdrOffset + 5)
									usnX = usnX/10.0
									usnY = usnY/10.0
									usnZ = usnZ/10.0
								crc16 = crcmod.predefined.Crc('modbus')
								crc16.update(strbuf[ pktHdrOffset : pktHdrOffset + msgLen + 5 ])
								CRC_calc = int(crc16.hexdigest(), 16)

								if CRC_calc == usnCRC16:
									value = [usnAdr, usnX, usnY, usnZ, usnTimestamp]
									self.lastValues.append(value)
									if (self.recieveDataCallback is not None):
										self.recieveDataCallback()
								else:
									if self.debug:
										print ('\n*** CRC ERROR')


								if pktHdrOffset == -1:
									if self.debug:
										print ('\n*** ERROR: Marvelmind USNAV beacon packet header not found (check modem board or radio link)')
									continue
								elif pktHdrOffset >= 0:
									if self.debug:
										print ('\n>> Found USNAV beacon packet header at offset %d' % pktHdrOffset)
										self.print_position()
								for x in range(0, pktHdrOffset + msgLen + 7):
									self._bufferSerialDeque.popleft()

				except OSError:
					if self.debug:
						print ('\n*** ERROR: OS error (possibly serial port is not available)')
					sleep(1)
				except serial.SerialException:
					if self.debug:
						print ('\n*** ERROR: serial port error (possibly beacon is reset, powered down or in sleep mode). Restarting reading process...')
					self.serialPort = None
					sleep(1)
			else: 
				sleep(1)
	
		if (self.serialPort is not None):
			self.serialPort.close()
			