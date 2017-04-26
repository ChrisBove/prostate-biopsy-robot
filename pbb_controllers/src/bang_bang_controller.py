#!/usr/bin/env python
"""@package docstring
    This is the bang bang controller for the needle prostate bot
"""

import rospy, tf, math
import numpy as np
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

def goalCallback(data):
    """This callback will occur once when the goal point is set"""
    rospy.loginfo(rospy.get_caller_id() + " got goal point \n%s", data.point)
    # do something with your goal point! (probably enter a loop or something)
    
    publishReset() # to clear out the dynamic model state (this is a new goal request)
    
    global goalPoint
    goalPoint = data
    # pull out info like data.point.x, data.point.y, etc.

def startCallback(data):
    """This callback will fire very frequently when the start marker is dragged"""
    # rospy.loginfo(rospy.get_caller_id() + " got start point %s", data.point)
    global startPoint
    startPoint = data
    # test
    # NOTE: if this callback does not happen before the goal, all will be OK:
    #  the startPoint is initialized to 0, which is also what it happens to be
    #  the actual system 

def poseFeedbackCallback(data):
    """This callback gets the pose feedback from the camera"""
    global feedbackPose
    feedbackPose = data
    #access with data.pose.position.x or quaternion from data.pose.orientation.x 
    quat = data.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    #convert to euler
    roll, pitch, yaw = euler_from_quaternion(q)
    #convert yaw to degrees
    # theta = math.degrees(yaw)

    # find the desired steering angle
    thetaTarget, error = SteeringAngle([goalPoint.point.x, goalPoint.point.y, goalPoint.point.z], [data.pose.position.x, data.pose.position.y, data.pose.position.z, roll, pitch, yaw])

    # How far off are we?
    thetaError = roll - thetaTarget

    # proportional gain
    Kp = 0.1
    Kd = 1

    # This is actually just a P controller right now
    publishTwist(0.005, Kp*thetaError)


def publishReset():
    """This is used to request a reset from the dynamic model node."""
    global resetPub
    msg = Bool()
    msg = True
    resetPub.publish(msg)
    
def publishTwist(lin_Vel, ang_Vel):
    """Send a movement (twist) message."""
    global twistPub
    msg = Twist()
    msg.linear.x = lin_Vel
    msg.angular.z = ang_Vel
    twistPub.publish(msg)

def SteeringAngle(targetPosition, needlePose):
    delta = np.array([needlePose[0] - targetPosition[0], needlePose[1] - targetPosition[1], needlePose[2] - targetPosition[2]])

    delta_rotated = np.dot(RotationMatrix(needlePose[4], needlePose[5]), delta.T)

    angle = math.atan2(delta_rotated[1], delta_rotated[2])

    error = [delta_rotated[1], delta_rotated[2]]
    return angle, error

def RotationMatrix(pitch, yaw):
    R_pitch = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]]).T

    R_yaw = np.array([[math.cos(yaw), math.sin(yaw), 0], [-math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])

    R = np.dot(R_yaw, R_pitch)
    return R

# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('bang_bang_controller')
    
    global twistPub
    global resetPub
    global startPoint 
    startPoint = PointStamped()
    global feedbackPose
    feedbackPose = PoseStamped()
    
    # setup the publisher for the twist msg to the dynamic controller
    twistPub = rospy.Publisher('cmd_velocity', Twist, None, queue_size=10)
    
    # this is for demanding the dynamic model to reset
    resetPub = rospy.Publisher('reset_robot', Bool, None, queue_size=1)
    
    # setup the subscribers to the interactive marker Point topics
    rospy.Subscriber("goal_point", PointStamped, goalCallback, queue_size=1)
    rospy.Subscriber("start_point", PointStamped, startCallback, queue_size=1)
    
    # subscriber for the pose feedback
    rospy.Subscriber("needle_tip_pose_noisy", PoseStamped, poseFeedbackCallback, queue_size=1)
    
    # this just keeps the node alive, servicing the callbacks
    rospy.spin()
    
    rospy.loginfo(rospy.get_caller_id() + " terminated")
