# Control a Ridgeback robot using an xbox controller and ROS

# https://github.com/tcstewar/xboxcontroller_linux
from xbox import XBox

import time
import sys
# Hack for finding rospy when running with sudo
sys.path.append('/opt/ros/indigo/lib/python2.7/dist-packages')
import rospy
from geometry_msgs.msg import Twist

# Threshold for 0 on joysticks for xbox controller
STICK_THRESHOLD = 7500#5000
TRIGGER_THRESHOLD = 10
STICK_MAX = 32767.0
TRIGGER_MAX = 255

class Controller(XBox):
    def __init__(self, topic='/cmd_vel', wireless_index=0, disable_signals=False):
        rospy.init_node('nengo_interface', anonymous=True, disable_signals=disable_signals)

        self.pub_twist = rospy.Publisher(topic, Twist)

        super(Controller, self).__init__(wireless_index=wireless_index)

    def send_command(self):
        
        msg = Twist()

        x = self.value['Y1']
        if abs(x) > STICK_THRESHOLD:
            x = x/STICK_MAX
        else:
            x = 0
        
        y = self.value['X1']
        if abs(y) > STICK_THRESHOLD:
            y = y/STICK_MAX
        else:
            y = 0
        
        zr = self.value['RT']
        if zr > TRIGGER_THRESHOLD:
            zr = zr/TRIGGER_MAX*2.2
        else:
            zr = 0
        
        zl = self.value['LT']
        if zl > TRIGGER_THRESHOLD:
            zl = zl/TRIGGER_MAX*2.2
        else:
            zl = 0

        #TODO: scale these appropriately
        msg.linear.x = x
        msg.linear.y = -y
        msg.angular.z = zl - zr
        
        # Speed limiting checks, so things don't break
        if msg.linear.x > 1:
            msg.linear.x = 1
        elif msg.linear.x < -1:
            msg.linear.x = -1

        if msg.linear.y > 1:
            msg.linear.y = 1
        elif msg.linear.y < -1:
            msg.linear.y = -1

        if msg.angular.z > 2.2:
            msg.angular.z = 2.2
        elif msg.angular.z < -2.2:
            msg.angular.z = -2.2

        self.pub_twist.publish(msg)


if __name__ == '__main__':
    c = Controller()
    c.start()
    while True:
        c.send_command()
        time.sleep(0.01)
