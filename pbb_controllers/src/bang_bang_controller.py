#!/usr/bin/env python
"""@package docstring
    This is the bang bang controller for the needle prostate bot
"""

import rospy, tf, numpy, math
from geometry_msgs.msg import Twist, PointStamped, PoseStamped
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

def goalCallback(data):
    """This callback will occur once when the """
    rospy.loginfo(rospy.get_caller_id() + " got goal point \n%s", data.point)
    # do something with your goal point! (probably enter a loop or something)
    
    publishReset() # to clear out the dynamic model state (this is a new goal request)
    
    # pull out info like data.point.x, data.point.y, etc.

def startCallback(data):
    """This callback will fire very frequently when the start marker is dragged"""
    rospy.loginfo(rospy.get_caller_id() + " got start point %s", data.point)
    global startPoint
    startPoint = data
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
    theta = math.degrees(yaw)


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