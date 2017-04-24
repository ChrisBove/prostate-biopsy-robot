#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
import dynamic_model
from std_msgs.msg import String



def move_needle(twist):
    """Move the needle foward"""
    global pub_movement
    global model
    dt = .1
    n = model.update(twist.linear.x, twist.angular.z,dt)
    pose = Pose()
    pose.position. x = n[0]
    pose.position. x = n[1]
    pose.position. x = n[2]
    pub_movement.publish(pose)

def reset(rest):
    """reset the model state"""
    global model
    model.rest()

if __name__ == '__main__':
    """ROS wrapper for the needle model"""
    global pub_movement
    global model
    model = dynamic_model.DynamicModel()

    rospy.init_node('Needle_Model', anonymous=True)
    pub_movement = rospy.Publisher('Needle_pose', Pose, queue_size=1)
    rospy.Subscriber("reset", String, reset)
    rospy.Subscriber("move_needle", Twist, move_needle)

    while not rospy.is_shutdown():
        rospy.spin()
