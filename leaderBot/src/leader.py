#!/usr/bin/env python3
import rospy, math, cv2, dlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Image
#from apriltag_ros.msg import AprilTagDetectionArray
from um7.srv import *
#from lab4.msg import Stop_Dist
# lambda function to convert rad to deg
RAD2DEG = lambda x: ((x)*180./math.pi)
# convert LaserScan degree from -180 - 180 degs to 0 - 360 degs
DEG_CONV = lambda deg: deg + 360 if deg < 0 else deg
#QUAD_CONV = lambda deg: abs(deg - 90) if deg < 180 else abs(deg - 270)
class leader:
        K1 = .7 # rotation constant for slope
        KTURN = 0.08 # gain constatnt for turning state
        MIN_ANG_Z = .8 # limit rad/s values sent to USAFABot
        MAX_ANG_Z_TURN = 2.7 # limit rad/s values sent to USAFABot
        MAX_ANG_Z_TD = .7 # limit rad/s values sent to
        MAX_ANG_Z = 1 # limit rad/s values sent to USAFABot
        DISTANCE = 2 # target distance from the wall
        # class initialization
        def __init__(self):
            # TODO: complete __init__() function
            self.intersectCount = 0
            self.tag_x = 0
            self.turning = False
            self.flip_side = False  # variable indicates true when following left wall\
            rospy.Subscriber('scan', LaserScan, self.callback_lidar)
            #rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
            
            self.cmd = Twist()
            #rospy.Subscriber('cmd_vel', Twist, self.callback_keyboard)
            self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
            rospy.Timer(rospy.Duration(0.01), self.callback_controller)
            self.ctrl_c = False
            
            self.CWall = 5
            self.LWall = 0
            self.RWall = 0
            ## State variable
            self.State = "Forward"
            self.HDG_TOL = 20
            ## wizrd math stuff

            rospy.on_shutdown(self.shutdownhook)

        """def tag_callback(self,data):
            if not self.ctrl_c and (self.State == "Forward" or self.State == "WaitForTag"):
                for tag in data.detections:
                    #print("Called")
                    #print(tag.pose.pose.pose.position.z)diagCount
                    #print(tag.id[0])
                    #print(tag.pose.pose.pose.position.x)"""


       # the cmd_vel topic

        def callback_lidar(self, scan):
            if not self.ctrl_c:
                degrees = []
                ranges = []
                maxRange = 100
                sum30Center = 0
                diagCountC = 0
                sumLeft = 0
                diagCountL = 0
                sumRight = 0
                diagCountR = 0
                # determine how many scans were taken during rotation
                scanCount = int((scan.angle_max-scan.angle_min) / scan.angle_increment)

                for i in range(scanCount):
                    # using min angle and incr data determine curr angle,
                    # convert to degrees, convert to 360 scale
                    degrees.append(int(DEG_CONV(RAD2DEG(scan.angle_min + scan.angle_increment*i))))
                    rng = scan.ranges[i]
                    # ensure range values are valid; set to 0 if not
                    if rng < 0.1:
                        ranges.append(0.0)
                    elif rng > maxRange:
                        ranges.append(maxRange)
                    else:
                        ranges.append(rng)


                # python way to iterate two lists at once!
                for deg, rng in zip(degrees, ranges):
                    if deg <= 110 and deg >= 70 and rng > 0.1:
                        sumLeft += rng
                        diagCountL += 1
                    elif deg <= 290 and deg >= 250 and rng > 0.1:
                        sumRight += rng
                        diagCountR += 1
                    elif deg >= 340 or deg <= 20 and rng > 0.1:
                        sum30Center += rng
                        diagCountC += 1
                    #print(self.LeftIntercept)
                if diagCountC != 0:
                    self.CWall = sum30Center/diagCountC
                if diagCountL != 0:
                    self.LWall = sumLeft / diagCountL
                if diagCountR != 0:
                    self.RWall = sumRight / diagCountR

        # callback that will run at 100 Hz
        # will be used to control the robot
        def callback_controller(self, event):
            # TODO: complete callback_controller() function
            # local variables do not need the self
            if self.ctrl_c:
                return
            #print(self.State)
            ang_z = 0
            lin_x = 0
            Walls = 3
            if self.State == "Forward" or self.State == "Stopping":
                lin_x = 0.5
                ang_z = self.cmd.angular.z + self.K1 * (self.RWall - self.LWall)
                if self.State == "Stopping":
                    lin_x = 0.15
                    ang_z = 0
                    self.State = "Stop"
                    
                if abs(ang_z) > self.MAX_ANG_Z: ang_z = self.MAX_ANG_Z*(abs(ang_z)/ang_z)
                if self.CWall < 0.8:
                    lin_x = 0.05
                    #ang_z *= 2
                if self.CWall < 6: Walls -= 1
                if self.LWall > 27: Walls -= 1
                if Walls <= 2:
                    self.intersectCount += 1
                if self.intersectCount == 5:
                    self.intersectCount = 0
                    lin_x = 0
                    ang_z = 0
                    self.State = "Turn"
                print(self.LWall)
                print(self.RWall)
                print(self.State)
                print("\n")

            elif self.State == "Stop":
                lin_x = 0
                ang_z = 0
                print("Stop")
                rospy.sleep(5.)
                self.State = "Stop"

            elif self.State == "Turn":
                lin_x = 0.15
                ang_z = self.MAX_ANG_Z_TURN
                print(self.LWall)
                print(self.RWall)
                print(self.State)
                print("\n")

                # check goal orientation
                if self.LWall < 3 or self.CWall < 8:
                    self.State = "Forward"
                    ang_z = 0
            else:
                lin_x = 0
                ang_z = 0


            # set Twist message and publish
            self.cmd.linear.x = lin_x
            self.cmd.angular.z = ang_z
            self.pub.publish(self.cmd)

        def shutdownhook(self):
            # TODO: complete shutdownhook() function
            print("Controller exiting. Halting robot.")
            self.ctrl_c = True
            self.cmd.linear.x = 0
            self.cmd.angular.z = 0
            self.pub.publish(self.cmd)

if __name__ == "__main__":
    rospy.init_node('leader')

    leader()
    rospy.spin()

"""#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist

class leader():
    def __init__(self):
        rospy.init_node('leader')
        rospy.loginfo("Subscribers set")
        self.sinPeriod = np.sin(np.linspace(-np.pi,np.pi, 201))
        self.index = 0
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.msg = Twist()
        rospy.loginfo("Publisher set")
    
    
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.msg.linear.x = self.sinPeriod[self.index]
            self.msg.angular.z = self.sinPeriod[self.index]
            self.pub.publish(self.msg)
            if self.index < (len(self.sinPeriod) - 1):
                self.index = self.index + 1
            else:
                self.index = 0
            r.sleep()
if __name__ == "__main__":
    rospy.init_node('leader')
    
    lead = leader()
    lead.run()   """         
