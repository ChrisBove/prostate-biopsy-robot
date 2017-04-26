#!/usr/bin/env python
"""@package docstring
    This is the model controller for the needle prostate bot
"""

import rospy, tf, numpy, math
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
import dynamic_model

def startCallback(data):
    """This callback will fire very frequently when the start marker is dragged"""
    global startPoint
    startPoint = data

def resetCallback(data):
    """This callback will occur when reset robot is request """
    rospy.loginfo(rospy.get_caller_id() + " got reset call %s", data)
    global doReset
    doReset = data

    global model
    model.reset()

    twist = Twist()
    twistCallback(twist)

    # TODO reset the dynamic model

def twistCallback(data):
    """This callback occurs with new twist updates going into the controller"""
    # rospy.loginfo(rospy.get_caller_id() + " got twist \n%s", data)
    global twist
    twist = data
    global posePub

    # access with twist.linear.x, twist.angular.x

    # update the dynamic model with the new velocity cmd
    global model
    dt = 0.1 # TODO change this to use the header
    n = model.update(twist.linear.x, twist.angular.x,dt)
    print n[0]
    pose = PoseStamped()
    # put the header info in
    pose.header.stamp = rospy.get_rostime()
    pose.header.frame_id = "gelatin"
#     pose.header.frame_id = data.header.frame_id

    pose.pose.position.x = n[2] + startPoint.point.x
    pose.pose.position.y = n[1] + startPoint.point.y
    pose.pose.position.z = n[0] + startPoint.point.z

    pose.pose.orientation.w = 1.0

    # TODO stuff with orientation data. Maybe need to
    # use quaternion_from_euler to fill orientation quaternion (include that)

    posePub.publish(pose)

# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('dynamic_model_node')

    global posePub

    global startPoint
    startPoint = PointStamped()

    # setup the dynamic model
    global model
    model = dynamic_model.DynamicModel()

    # this is for demanding the dynamic model to reset
    posePub = rospy.Publisher('needle_tip_pose', PoseStamped, None, queue_size=1)

    # setup the subscribers to the interactive marker Point topics
    rospy.Subscriber("reset_robot", Bool, resetCallback, queue_size=1)
    rospy.Subscriber("cmd_velocity", Twist, twistCallback, queue_size=1)

    rospy.Subscriber("start_point", PointStamped, startCallback, queue_size=1)
    
    # this just keeps the node alive, servicing the callbacks
    rospy.spin()

    rospy.loginfo(rospy.get_caller_id() + " terminated")
