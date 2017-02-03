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
    def __init__(self, topic='/cmd_vel', wireless_index=0, disable_signals=False,
                 lin_limit=0.2, ang_limit=0.7):
        rospy.init_node('nengo_interface', anonymous=True, disable_signals=disable_signals)
        
        self.pub_twist = rospy.Publisher(topic, Twist)

        self.lin_limit = lin_limit
        self.ang_limit = ang_limit

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
        if msg.linear.x > self.lin_limit:
            msg.linear.x = self.lin_limit
        elif msg.linear.x < -self.lin_limit:
            msg.linear.x = -self.lin_limit

        if msg.linear.y > self.lin_limit:
            msg.linear.y = self.lin_limit
        elif msg.linear.y < -self.lin_limit:
            msg.linear.y = -self.lin_limit

        if msg.angular.z > self.ang_limit:
            msg.angular.z = self.ang_limit
        elif msg.angular.z < -self.ang_limit:
            msg.angular.z = -self.ang_limit

        self.pub_twist.publish(msg)


if __name__ == '__main__':
    c = Controller()
    c.start()
    while True:
        c.send_command()
        time.sleep(0.01)
