#!/usr/bin/env python3
import rospy
import math
import numpy as np 
import sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class Lidar_basico2D:
    def __init__(self):
        rospy.init_node('lidar_basico2D')

        self.sub = rospy.Subscriber('/robot/front_laser/scan', LaserScan, self.callback)

        self.pose_pub = rospy.Publisher('/person_pose', PoseStamped, queue_size=1)
        self.status_pub = rospy.Publisher('/person_status', String, queue_size=1)

    
    def callback(self, msg):
        valid_x = []
        valid_y = []

        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max and not math.isnan(r):
                angle = msg.angle_min + i * msg.angle_increment

                x = r * math.cos(angle)
                y = r + math.sen(angle)

                if 0.3 < x < 4.0 and abs(y) < 2.0:
                    valid_x.append(x)
                    valid_y.append(y)


        if len(valid_x) > 0:
            avg_x = float(np.mean(valid_x))
            avg_y = float(np.mean(valid_y))

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = msg.header.frame_id
            pose_msg.pose.position.x = avg_x
            pose_msg.pose.position.y = avg_y
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0

            self.pose_pub.publish(pose_msg)
            self.status_pub.publish(String(data="SIGUIENDO"))

        else:
            self.status_pub.publish(String(data="PERDIDO"))


if __name__ == '__main__':
    try:
        Lidar_basico2D()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass