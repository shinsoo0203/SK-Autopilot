#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sk_msgs.msg import VisionDetectCmd as msg

def talker():
    pub=rospy.Publisher('vision_detect',msg,queue_size=10)
    rospy.init_node('publisher',anonymous=True)
    rate=rospy.Rate(1) #10Hz
    while not rospy.is_shutdown():
        msg.isdetect = True
        msg.classifier = 0
        msg.data = 7
        rospy.loginfo(msg.isdetect)
        rospy.loginfo(msg.classifier)
        rospy.loginfo(msg.data)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
