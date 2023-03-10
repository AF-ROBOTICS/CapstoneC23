#!/usr/bin/env python3
import rospy
import serial.tools.list_ports as port_list
import serial
import sys
import numpy as np
# TODO
# Remove Twist and add custom message
from geometry_msgs.msg import Twist
#from usafabot.msg import WheelVelocity

## USAFABOT
#
#  Connects to the MSP432 over serial and writes linear/angular velocity
#   to enable the USAFABOT to drive accordingly. The robot operates well around
#   with a linear velocity between 0.05 m/s and 0.25 m/s and an angular velocity of 
#   0.5 m/s and 1.5 m/s.
class USAFABOT:
    def __init__(self):
        #class variables to define the message structure for the arduino
        self.endByte = 126
        self.offsetDrive = 1520
        self.halfBandwidthDrive = 55
        self.offsetSteer = 1475
        self.halfBandwidthSteer = 300
        self.chooseSteer = 1
        self.chooseDrive = 2
        # read wheel speed from robot at 10 Hz
        #rospy.Timer(rospy.Duration(.1), self.callback_read)
        
        # Initialize a serial connection w/ the Arduino
        self.ser = self.find_usafabot()
        rospy.Subscriber('cmd_vel', Twist, self.callback_write)        

    # Subscriber function that gets linear/angular velocity
    # values from controller and writes to MSP432
    def callback_write(self, data):
        try:
            # arduino code wants byte array messages in format: (0:1) are either 1 or 2 for steer or drive, (2:3) are the microsecond pwm width, between 1000 and 2000, (4:5) are 126, which tells the arduino to terminate the byte array. After 126, the message structure repeats.
            inp = np.array([self.chooseSteer, (data.angular.z * self.halfBandwidthSteer + self.offsetSteer), self.endByte, self.chooseDrive, (data.linear.x * self.halfBandwidthDrive + self.offsetDrive), self.endByte], dtype = '>u2')
            input = inp.tobytes()

            #this if statement is to prevent early message termination from the second data byte taking the value of the end byte.
            if (input[3] == self.endByte):
                input[3] == (self.endByte - 1)
            self.ser.write(input)
            print(inp)
        except serial.SerialException:
            print("Serial port not open")
            
    # Helper function to connect to the Leaderbot. Checks each serial connection to NUC
    # until it finds the correct one (Arduino is running code to respond to a request)
    def find_usafabot(self):
        test = "start~".encode('ascii')
        ports = list(port_list.comports())
        for Port, desc, hwid in sorted(ports):
            print(desc)
            if('Lynxmotion Botboarduino' in desc):
                print(Port)
                ser = serial.Serial(port = Port,
                    baudrate = 9600,
                    parity = serial.PARITY_NONE,
                    stopbits = serial.STOPBITS_ONE,
                    bytesize = serial.EIGHTBITS,
                    timeout = 1)
                ser.write(test)
                response = ser.readline()
                
                print('response:')
                print(response)
                if(response.decode('ascii') == "start\n"):
                    print("Connected to Arduino", Port)   
                    return ser
                else:
                    ser.write(test)
                    response= ser.readline()
                    print(response)
                    if(response.decode('ascii') == "start\n"):
                        print("Connected to Arduino at", Port)   
                    return ser
                    
        print("USAFABOT not found...exiting")
        sys.exit()
        
    def handler(self):
        while not rospy.is_shutdown():
            pass
        input = "0.0,0.0\r\n"
        self.ser.write(input.encode())
        rospy.sleep(.5)
        input = "quit\n~"
        self.ser.write(input.encode('ascii'))
        response = self.ser.readline()
        while(len(response) > 0):
            print(response)
            response=self.ser.readline()
        self.ser.close()
        print("shutting down")
    
if __name__ == '__main__':
    rospy.init_node('leaderbot_serial', anonymous = True)
    
    # list available ports for that the MSP432 is using
    U = USAFABOT()
    U.handler()
