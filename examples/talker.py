#!/usr/bin/env python
import rospy

from std_msgs.msg import String
from ros_g29_force_feedback.msg import ForceFeedback

def talker():
    rospy.init_node('talker')
    pub = rospy.Publisher('ff_target', ForceFeedback, queue_size=20)
#    pub = rospy.Publisher('g29test', String, queue_size=20)

    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
      g29msg = ForceFeedback()
      g29msg.position = 0.0
      g29msg.force = 0.6
      rospy.loginfo(g29msg)
      pub.publish(g29msg)
      r.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
