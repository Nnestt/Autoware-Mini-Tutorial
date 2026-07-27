#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Publisher:
    def __init__(self):
        # Internal variables
        self.rate = rospy.Rate(2)

        # Publishers
        self.pub = rospy.Publisher('/message', String, queue_size=10)

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish("Hello world!")
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('publisher')
    node = Publisher()
    node.run()




# #!/usr/bin/env python3

# import rospy
# from std_msgs.msg import String

# rospy.init_node('publisher')
# rate = rospy.Rate(2)
# pub = rospy.Publisher('/message', String, queue_size=10)

# while not rospy.is_shutdown():
#     pub.publish("Hello world!")
#     rate.sleep()