#!/usr/bin/env python
"""@package docstring
    This is the actual path drawing node for the needle prostate bot
"""

import rospy, tf, numpy, math
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool

def resetCallback(data):
    """This callback will occur when reset robot is request """
    rospy.loginfo(rospy.get_caller_id() + " got reset call %s", data) 
    global line
    del line.points[:]


def setupMarker():
    global line
    # create a line marker msg and fill it
    line = Marker()
    
    #header info
    line.header.frame_id = "gelatin"
    line.header.stamp = rospy.Time.now()
    
    #define marker type
    line.type = Marker.LINE_STRIP
    
    #same orientation for all (0,0,0,1 quat)
    line.pose.orientation.w = 1.0
    
    #coloring
    line.color.a = 0.8
    line.color.g = 1.0
    #g and b will be 0
    
    # just need x scale for a line width
    line.scale.x = 0.002

def poseCallback(data):
    """This callback will occur each time a new eef pose is sent """
    rospy.loginfo(rospy.get_caller_id() + " got eef pose \n%s", data.pose.position)
    # do something with your goal point! (probably enter a loop or something)
    # pull out info like data.point.x, data.point.y, etc.

    line.points.append(data.pose.position)
    
    markerPub.publish(line)
    

# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('actual_path_drawer')
    
    global markerPub
    global line

    setupMarker()
    
    # setup the publisher for the marker path
    markerPub = rospy.Publisher('actual_path', Marker, None, queue_size=10)
    
    # setup the subscribers to the interactive marker Point topics
    rospy.Subscriber("needle_tip_pose_noisy", PoseStamped, poseCallback, queue_size=10)
    rospy.Subscriber("reset_robot", Bool, resetCallback, queue_size=1)

    # this just keeps the node alive, servicing the callbacks
    rospy.spin()
    
    rospy.loginfo(rospy.get_caller_id() + " terminated")