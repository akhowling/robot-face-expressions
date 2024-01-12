#!/usr/bin/env python3
#
# *********     Sync Write Example      *********
#
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(STS/SMS), and an URT
#
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pkg_servo.msg import Num
import ast
import sys
import os


if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("/workspaces/ros2/servo/src/pkg_servo/src")
sys.path.append("/workspaces/ros2/servo/src/pkg_servo/src/pkg_servo")
from scservo_sdk import *                      # Uses SCServo SDK library
from dynamixel_sdk import *

# Dynamixel params
ADDR_TORQUE_ENABLE      = 64   
PROTOCOL_VERSION        = 2.0
DEVICENAME_D                 = '/dev/ttyUSB1'
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0 
ADDR_GOAL_POSITION      = 116

# Default setting
BAUDRATE                    = 1000000           # SCServo default baudrate : 1000000
DEVICENAME_F                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MINIMUM_POSITION_VALUE  = 0                 # SCServo will rotate between this value
# SCS_MAXIMUM_POSITION_VALUE  = 4095              
SCS_MOVING_SPEED            = 2400              # SCServo moving speed
SCS_MOVING_ACC              = 50                # SCServo moving acc

index = 0
# scs_goal_position = [SCS_MINIMUM_POSITION_VALUE, SCS_MAXIMUM_POSITION_VALUE]         # Goal position

portHandler_D = PortHandler(DEVICENAME_D)
packetHandler_D = PacketHandler(PROTOCOL_VERSION)

try:
   portHandler_D.openPort()
   print("Succeeded to open the Dynamixel port")
except:
    print("Failed to open the port")
    quit()
# Set port baudrate
try:
    portHandler_D.setBaudRate(BAUDRATE)
    print("Succeeded to change the baudrate")
except:
    print("Failed to change the baudrate")
    quit()


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME_F)

# Initialize PacketHandler instance
# Get methods and members of Protocol
packetHandler = sms_sts(portHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

dyn_servos = [40,41,42]
# dyn_servos = []
for id in dyn_servos:
    dxl_comm_result, dxl_error = packetHandler_D.write1ByteTxRx(portHandler_D, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)


def run_servo(res):
    for x in range(len(res)):
        # arm
        # checklist = [202]
        # head
        checklist = [23,101,30,31,102,13,10,11,14,15,16,12,20,22,21,25,24,103,104,1,3,4,0,5,6,105,106]
        if int(res[x][0]) in checklist:
            # print("fffffffffffffff",res)
            scs_addparam_result = packetHandler.SyncWritePosEx(int(res[x][0]), int(res[x][1]), SCS_MOVING_SPEED, SCS_MOVING_ACC)
            if scs_addparam_result != True:
                print("[ID:%03d] groupSyncWrite addparam failed" % scs_id)
        if int(res[x][0]) in dyn_servos:
            dxl_comm_result, dxl_error = packetHandler_D.write4ByteTxRx(portHandler_D, int(res[x][0]), ADDR_GOAL_POSITION, int(res[x][1]))
            
    # Syncwrite goal position
    scs_comm_result = packetHandler.groupSyncWrite.txPacket()
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    # Clear syncwrite parameter storage
    packetHandler.groupSyncWrite.clearParam()
    # portHandler.closePort()

def callback(msg):
    received_message = msg.data
    cleaned_str1 = received_message[0].replace("OrderedDict", "")
    res1 =ast.literal_eval(cleaned_str1)
    print('Result', res1)
    run_servo(res1)
    # print(f"Received message: {run_servo(res)}")
    # print(msg)
    

def main():
    rclpy.init()
    node = Node('Sub')
    subscriber = node.create_subscription(Num, 'motor_pos', callback, 1)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





