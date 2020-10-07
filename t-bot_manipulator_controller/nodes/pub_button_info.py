#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

def talker():
      pub = rospy.Publisher('/button_floor', Int32, queue_size=10)
      rospy.init_node('button_floor_publish', anonymous = True)
      rate = rospy.Rate(10) #10hz

      floor = int(input("input elevator floor: "))

      while not rospy.is_shutdown():
            rospy.loginfo(floor)
            pub.publish(floor)
            rate.sleep()

if __name__=='__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
