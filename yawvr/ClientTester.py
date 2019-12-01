import array
import time
import re
import struct
import socket
import keyboard

# Yaw VR TCP server port to request to
YAWVR_TCP_IP = '192.168.1.18' # Emulator, just launch the Unity Emulator app
#YAWVR_TCP_IP = '192.168.1.28' # Real YawVR device, start the Android app and start the device
YAWVR_TCP_PORT = 50020
# Yaw VR UDP server port to receive on (check you have a firewall rule : input UDP port 28067)
# 20191022 : YawVR Android app displays version 38, this version ignores the requested port and send always on UDP port 28067
YAWVR_UDP_IP = '0.0.0.0'
YAWVR_UDP_PORT = 28067

YAWVR_BUFFER_SIZE = 1024

# UDP port checkin request to send on TCP port in order to receive simulator device data on UDP port
YAWVR_UDP_PORT_CHECKIN_GAME_NAME = 'ClientTester'
YAWVR_UDP_PORT_CHECKIN_TCP_PACKET_REQUEST = bytearray([ 0x30 ] + list(struct.unpack('4B', struct.pack('>I', YAWVR_UDP_PORT))) + list(array.array('B', YAWVR_UDP_PORT_CHECKIN_GAME_NAME)) + [ 0x00 ])

# Yaw VR UDP packet formatting
# (SIMULATOR YAW) (SIMULATOR PITCH) (SIMULATOR ROLL) (GAME YAW) (GAME PITCH) (GAME ROLL) (1-2-3 MOTOR SPEED) (BATTERY LEVEL)
YAWVR_UDP_PACKET_SIMYAW = 'simYaw'
YAWVR_UDP_PACKET_SIMPITCH = 'simPitch'
YAWVR_UDP_PACKET_SIMROLL = 'simRoll'
YAWVR_UDP_PACKET_GAMEYAW = 'gameYaw'
YAWVR_UDP_PACKET_GAMEPITCH = 'gamePitch'
YAWVR_UDP_PACKET_GAMEROLL = 'gameRoll'
YAWVR_UDP_PACKET_MOTORSPD1 = 'motorSpd1'
YAWVR_UDP_PACKET_MOTORSPD2 = 'motorSpd2'
YAWVR_UDP_PACKET_MOTORSPD3 = 'motorSpd3'
YAWVR_UDP_PACKET_BATTERYLVL = 'batteryLvl'
YAWVR_UDP_PACKET_REGEX = re.compile('^(?:SY\[(?P<%s>[+-]?\d+[\.]?\d*)])?(?:SP\[(?P<%s>[+-]?\d+[\.]?\d*)])?(?:SR\[(?P<%s>[+-]?\d+[\.]?\d*)])?(?:GY\[(?P<%s>[+-]?\d+[\.]?\d*)])?(?:GP\[(?P<%s>[+-]?\d+[\.]?\d*)])?(?:GR\[(?P<%s>[+-]?\d+[\.]?\d*)])?(?:S1\[(?P<%s>[+-]?\d+[\.]?\d*)])?(?:S2\[(?P<%s>[+-]?\d+[\.]?\d*)])?(?:S3\[(?P<%s>[+-]?\d+[\.]?\d*)])?(?:U\[(?P<%s>\d+[\.]?\d*)])?.*$' % (YAWVR_UDP_PACKET_SIMYAW, YAWVR_UDP_PACKET_SIMPITCH, YAWVR_UDP_PACKET_SIMROLL, YAWVR_UDP_PACKET_GAMEYAW, YAWVR_UDP_PACKET_GAMEPITCH, YAWVR_UDP_PACKET_GAMEROLL, YAWVR_UDP_PACKET_MOTORSPD1, YAWVR_UDP_PACKET_MOTORSPD2, YAWVR_UDP_PACKET_MOTORSPD3, YAWVR_UDP_PACKET_BATTERYLVL))

class YawVRPacket:
	def __init__(self):
		self.simulatorYaw = 0.0
		self.simulatorPitch = 0.0
		self.simulatorRoll = 0.0
		self.gameYaw = 0.0
		self.gamePitch = 0.0
		self.gameRoll = 0.0
		self.motorSpeed1 = 0.0
		self.motorSpeed2 = 0.0
		self.motorSpeed3 = 0.0
		self.batteryLevel = 0.0
	def __init__(self, packet):
		firstLine = packet.split('\n')[0]
		m = YAWVR_UDP_PACKET_REGEX.match(firstLine)
		if m:
			self.simulatorYaw = float(m.group(YAWVR_UDP_PACKET_SIMYAW)) if YAWVR_UDP_PACKET_SIMYAW in m.groupdict() and m.group(YAWVR_UDP_PACKET_SIMYAW) else 0.0
			self.simulatorPitch = float(m.group(YAWVR_UDP_PACKET_SIMPITCH)) if YAWVR_UDP_PACKET_SIMPITCH in m.groupdict() and m.group(YAWVR_UDP_PACKET_SIMPITCH) else 0.0
			self.simulatorRoll = float(m.group(YAWVR_UDP_PACKET_SIMROLL)) if YAWVR_UDP_PACKET_SIMROLL in m.groupdict() and m.group(YAWVR_UDP_PACKET_SIMROLL) else 0.0
			self.gameYaw = float(m.group(YAWVR_UDP_PACKET_GAMEYAW)) if YAWVR_UDP_PACKET_GAMEYAW in m.groupdict() and m.group(YAWVR_UDP_PACKET_GAMEYAW) else 0.0
			self.gamePitch = float(m.group(YAWVR_UDP_PACKET_GAMEPITCH)) if YAWVR_UDP_PACKET_GAMEPITCH in m.groupdict() and m.group(YAWVR_UDP_PACKET_GAMEPITCH) else 0.0
			self.gameRoll = float(m.group(YAWVR_UDP_PACKET_GAMEROLL)) if YAWVR_UDP_PACKET_GAMEROLL in m.groupdict() and m.group(YAWVR_UDP_PACKET_GAMEROLL) else 0.0
			self.motorSpeed1 = float(m.group(YAWVR_UDP_PACKET_MOTORSPD1)) if YAWVR_UDP_PACKET_MOTORSPD1 in m.groupdict() and m.group(YAWVR_UDP_PACKET_MOTORSPD1) else 0.0
			self.motorSpeed2 = float(m.group(YAWVR_UDP_PACKET_MOTORSPD2)) if YAWVR_UDP_PACKET_MOTORSPD2 in m.groupdict() and m.group(YAWVR_UDP_PACKET_MOTORSPD2) else 0.0
			self.motorSpeed2 = float(m.group(YAWVR_UDP_PACKET_MOTORSPD3)) if YAWVR_UDP_PACKET_MOTORSPD3 in m.groupdict() and m.group(YAWVR_UDP_PACKET_MOTORSPD3) else 0.0
			self.batteryLevel = float(m.group(YAWVR_UDP_PACKET_BATTERYLVL)) if YAWVR_UDP_PACKET_BATTERYLVL in m.groupdict() and m.group(YAWVR_UDP_PACKET_BATTERYLVL) else 0.0

	def packet(self):
		return YAWVR_UDP_PACKET % (self.simulatorYaw, self.simulatorPitch, self.simulatorRoll)

	def __str__(self):
		return "%s:%.03f, %s:%.03f, %s:%.03f" % (YAWVR_UDP_PACKET_SIMYAW, self.simulatorYaw, YAWVR_UDP_PACKET_SIMPITCH, self.simulatorPitch, YAWVR_UDP_PACKET_SIMROLL, self.simulatorRoll)

quit = False

def onQuitKeyEvent():
	global quit
	print('Quitting ...')
	quit = True

keyboard.add_hotkey('esc', onQuitKeyEvent)

# No more need to request UDP port checkin on TCP port with emulator version : YawVREmulatorWin.x86_64WithUDPSupport
print 'Connecting to YawVR %s:%d' % (YAWVR_TCP_IP, YAWVR_TCP_PORT)
tcpsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcpsock.connect((YAWVR_TCP_IP, YAWVR_TCP_PORT))
print 'Requesting checkin on UDP port : %d' % YAWVR_UDP_PORT
print [hex(b) for b in YAWVR_UDP_PORT_CHECKIN_TCP_PACKET_REQUEST]
tcpsock.send(YAWVR_UDP_PORT_CHECKIN_TCP_PACKET_REQUEST)
yawVRTCPResponse = tcpsock.recv(YAWVR_BUFFER_SIZE)
print 'TCP response received : %s' % yawVRTCPResponse

print 'Receiving YawVR on %s:%d' % (YAWVR_UDP_IP, YAWVR_UDP_PORT)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((YAWVR_UDP_IP, YAWVR_UDP_PORT))
sock.setblocking(0)

while not quit:
	try:
		data, addr = sock.recvfrom(YAWVR_BUFFER_SIZE)
		print "Packet received : %s, %s" % (data, YawVRPacket(data))
	except socket.error:
		time.sleep(0.01)
		pass

print 'Closing connection to YawVR.'
if sock:
	sock.close()
if tcpsock:
	tcpsock.close()
