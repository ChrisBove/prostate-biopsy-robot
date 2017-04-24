#!/usr/bin/env python
"""@package docstring
    This is the model controller for the needle prostate bot
"""

import rospy, tf, numpy, math
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

def resetCallback(data):
    """This callback will occur when reset robot is request """
    rospy.loginfo(rospy.get_caller_id() + " got reset call %s", data) 
    global doReset
    doReset = data   

    # TODO reset the dynamic model

def twistCallback(data):
    """This callback occurs with new twist updates going into the controller"""
    # rospy.loginfo(rospy.get_caller_id() + " got twist \n%s", data)
    global twist
    twist = data
    
    # access with twist.linear.x, twist.angular.x
    
    # TODO update the dynamic model with the new velocity cmd
    
    pose = PoseStamped()
    # TODO stuff with new position and orientation data. Maybe need to 
    # use quaternion_from_euler to fill orientation quaternion (include that)
    
    posePub.publish(pose)

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


# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('dynamic_model_node')
    
    global posePub
        
    # this is for demanding the dynamic model to reset
    posePub = rospy.Publisher('needle_tip_pose', PoseStamped, None, queue_size=1)
    
    # setup the subscribers to the interactive marker Point topics
    rospy.Subscriber("reset_robot", Bool, resetCallback, queue_size=1)
    rospy.Subscriber("cmd_velocity", Twist, twistCallback, queue_size=1)
    
    # this just keeps the node alive, servicing the callbacks
    rospy.spin()
    
    rospy.loginfo(rospy.get_caller_id() + " terminated")