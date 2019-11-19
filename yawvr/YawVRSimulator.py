import time
import socket
import keyboard

# Yaw VR UDP server port
YAWVR_UDP_PORT = 28067
YAWVR_UDP_IP = '127.0.0.1'

# Yaw VR UDP packet formatting
# (SIMULATOR YAW) (SIMULATOR PITCH) (SIMULATOR ROLL) (GAME YAW) (GAME PITCH) (GAME ROLL) (1-2-3 MOTOR SPEED) (BATTERY LEVEL)
YAWVR_UDP_PACKET = 'SY[%.03f]SP[%.03f]SR[%.03f]GY[%.03f]GP[%.03f]GR[%.03f]S1[%0.f]S2[%0.f]S3[%0.f]U[%.03f]'

class YawVRPacket:
    def __init__(self):
        self.simulator_yaw = 0.0
        self.simulator_pitch = 0.0
        self.simulator_roll = 0.0
        self.game_yaw = 0.0
        self.game_pitch = 0.0
        self.game_roll = 0.0
        self.motor_speed_1 = 0.0
        self.motor_speed_2 = 0.0
        self.motor_speed_3 = 0.0
        self.battery_level = 0.0

    def packet(self):
        return YAWVR_UDP_PACKET % (self.simulator_yaw, self.simulator_pitch, self.simulator_roll, self.game_yaw, self.game_pitch, self.game_roll, self.motor_speed_1, self.motor_speed_2, self.motor_speed_3, self.battery_level)

quit = False
yawVRPacket = YawVRPacket()
sendPacket = True

def onQuitKeyEvent():
    global quit
    print('Quitting ...')
    quit = True

keyboard.add_hotkey('esc', onQuitKeyEvent)
TICK = 0.01
KEYS_ROTATION_SPEED_DEG_SEC = 3.0

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

while not quit:
    if keyboard.is_pressed('v'):
        yawVRPacket.simulator_yaw -= TICK*KEYS_ROTATION_SPEED_DEG_SEC
        sendPacket = True
    if keyboard.is_pressed('b'):
        yawVRPacket.simulator_yaw += TICK*KEYS_ROTATION_SPEED_DEG_SEC
        sendPacket = True
    if keyboard.is_pressed('t'):
        yawVRPacket.simulator_pitch -= TICK*KEYS_ROTATION_SPEED_DEG_SEC
        sendPacket = True
    if keyboard.is_pressed('g'):
        yawVRPacket.simulator_pitch += TICK*KEYS_ROTATION_SPEED_DEG_SEC
        sendPacket = True
    if keyboard.is_pressed('f'):
        yawVRPacket.simulator_roll -= TICK*KEYS_ROTATION_SPEED_DEG_SEC
        sendPacket = True
    if keyboard.is_pressed('h'):
        yawVRPacket.simulator_roll += TICK*KEYS_ROTATION_SPEED_DEG_SEC
        sendPacket = True
    if sendPacket:
        sendPacket = False
        packet = yawVRPacket.packet()
        sock.sendto(packet, (YAWVR_UDP_IP, YAWVR_UDP_PORT))
        print "Packet sent : %s" % packet
    time.sleep(0.01)
