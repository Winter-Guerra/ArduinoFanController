#!/usr/bin/python
"""
	GPLv2 XtremD

	A basic munin-script for fetching data from hddtemp
Many thanks to Tassilo Schweyer for releasing his 2007 hddtemp script onto the internet under the GPLv2!
"""
import socket
import sys
import time

import serial
import glob

#Hddtemp should be running in daemon mode with its listening port set to the port number below before this script runs.
#sudo hddtemp -d $(ls /dev/ | egrep 'sd[a-z]$' | awk '{print "/dev/"$0}')
port = 7634
host = 'localhost'

arrayDrives = {"/dev/sda", "/dev/sdb", "/dev/sdc"}

lowTemp = 36
highTemp = 40

minSpeed = 0xA6
maxSpeed = 0xFF

baud = 9600

#Gets the port that the Arduino is connected to (usually /dev/ttyUSB0)
def getFanControllerPort():
	#Pick the first one
	return scanSerialPorts()[0]
	
def scanSerialPorts():
	# scan for available ports. return a list of device names.
	return glob.glob('/dev/ttyUSB*')
	
def sendFanPacket(fanNumber, temp):
	pwmSetting = calculateFanSetting(temp)
	print "Setting PWM to: %d" % pwmSetting
	
	packetLength = 2
	packetCMD = 0x10 + (fanNumber-1)
	#Send packet
	ser.write(chr(0xFF) + chr(0xFF) + chr(packetLength) + chr(packetCMD) + chr(pwmSetting))
	
	
	return

#Calculate the value (0x0-0xFF) to send to the fan controller
def calculateFanSetting(temp):
	pwmVal = maxSpeed
	if temp < lowTemp:
		pwmVal = 0x00
		print "Temp is very low"
	elif temp > highTemp:
		pwmVal = maxSpeed
		print "Temp is very high"
	else:
		print "Temp is in sweetspot"
		#We are somewhere in the middle, map the temps to the speed linearly
		pwmVal = scale(temp, (lowTemp, highTemp), (minSpeed, maxSpeed))
		
	return pwmVal
	
def scale(val, src, dst):
	"""
	Scale the given value from the scale of src to the scale of dst.
	"""
	return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]

#Fetches the data from hddtemp
#Outputs data formatted so:
#|/dev/sda|Hitachi HDP725050GLA360|34|C||/dev/sdb|WDC WD5000AACS-00ZUB0|33|C||/dev/sdc|ST31000528AS|34|C||/dev/sdd|ST9500325AS|32|C|
def fetchTemp():
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((host,port))
	#Wait for request to complete
	time.sleep(1)
	#Get temps
	data = s.recv(4096)
	s.close()
	return data
	
def print_data(data):
	for drive in data:
		print 'Drive: %s current temp: %s' % (drive, data[drive])


def parse(data):
	"""
|/dev/sda|Hitachi HDP725050GLA360|34|C||/dev/sdb|WDC WD5000AACS-00ZUB0|33|C||/dev/sdc|ST31000528AS|34|C||/dev/sdd|ST9500325AS|32|C|
Every HDD gets 5 parts
We want part 0 and part 2
We also want to sanity check part 3 and make sure that it is C, not F

		parses the data returned from hddtemp
		returns map of device => temp
	"""
	parts = data.split('|')
	if len(parts) < 4:
		# ERROR!
		
		data = fetchTemp()
		time.sleep(100)
		#Hmmmm.. Suspicious bees! This might recurse out of control!
		return parse(data)
	
	if (parts[4]) != 'C':
		#We have a problem
		print "Not celcius! ERROR!!!"
		
	ret = {}
	
	for idx in range((len(parts)+1)/5):
		driveIndex = idx*5
		drivePath = parts[driveIndex+1]
		
		ret[drivePath] = int(parts[driveIndex+3])
		
	return ret
	
#This will take the highest temperature of the drives we care about
def averageTemps(driveTemps):
	maxTemp = 0
	for drive, temp in driveTemps.iteritems():
		if temp > maxTemp:
			maxTemp = temp
	return maxTemp
	
	
def selectArrayDrives(driveTemps):
	ret = {}
	
	for disk in driveTemps:
		if disk in arrayDrives:
			ret[disk] = driveTemps[disk]
	return ret
	
if __name__ == '__main__':
	
	arduinoPort = getFanControllerPort()
	print "Arduino found at: " + arduinoPort
	
	ser = serial.Serial(arduinoPort, baud)
	#Wait for ready
	time.sleep(3)
	
	while True:
		data = parse(fetchTemp())
		data = selectArrayDrives(data)
		
		print_data(data)
		driveTemp = (averageTemps(data))
		print "Max temp: %d" % driveTemp
		
		sendFanPacket(1, driveTemp)
		time.sleep(60)
		