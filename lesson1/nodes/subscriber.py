#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Subscriber:
    def __init__(self):
        # 1. Read parameters
        # 2. Initialize internal variables
        # 3. Create publishers
        # 4. Create subscribers

        # Subscribers
        rospy.Subscriber('/message', String, self.message_callback)

    def message_callback(self, msg):
        print(msg.data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('subscriber')
    node = Subscriber()
    node.run()