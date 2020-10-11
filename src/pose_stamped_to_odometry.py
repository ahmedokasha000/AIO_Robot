#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
odom_data = Odometry()

def poseCallback(msg):
    odom_data.header=msg.header
    odom_data.pose=msg.pose
    odom_data.child_frame_id="base_footprint"
    odometryPub.publish(odom_data)





if __name__ == '__main__':
    rospy.init_node('poseStamped_to_odometry', anonymous=True)
    poseSub = rospy.Subscriber("/poseupdate", PoseWithCovarianceStamped, poseCallback)
    odometryPub = rospy.Publisher('/odom', Odometry, queue_size=1)
    rospy.spin()
