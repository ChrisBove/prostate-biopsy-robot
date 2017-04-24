#!/usr/bin/env python
"""@package docstring
    This is the target path drawing node for the needle prostate bot
"""

import rospy, tf, numpy, math
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker


def goalCallback(data):
    """This callback will occur once when the """
    rospy.loginfo(rospy.get_caller_id() + " got goal point \n%s", data.point)
    # do something with your goal point! (probably enter a loop or something)
    # pull out info like data.point.x, data.point.y, etc.
    
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
    line.color.r = 1.0
    #g and b will be 0
    
    # just need x scale for a line width
    line.scale.x = 0.002
    
    # let's stuff some points into the thing
    p = Point()
    p = startPoint.point
    line.points.append(p)
    
    # TODO add lines below to keep adding intermediate points
    
    # the last point is the goal itself (so we get line from start to goal right now)
    line.points.append(data.point)
    
    markerPub.publish(line)
    

def startCallback(data):
    """This callback will fire very frequently when the start marker is dragged"""
    rospy.loginfo(rospy.get_caller_id() + " got start point %s", data.point)
    global startPoint
    startPoint = data
    # NOTE: if this callback does not happen before the goal, all will be OK:
    #  the startPoint is initialized to 0, which is also what it happens to be
    #  the actual system 


def publishTwist(lin_Vel, ang_Vel):
    """Send a movement (twist) message."""
    global twistPub
    msg = Twist()
    msg.linear.x = lin_Vel
    msg.angular.z = ang_Vel
    twistPub.publish(msg)

# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('target_path_drawer')
    
    global markerPub

    global startPoint 
    startPoint = PointStamped()

    # setup the publisher for the marker path
    markerPub = rospy.Publisher('target_path', Marker, None, queue_size=10)
    
    # setup the subscribers to the interactive marker Point topics
    rospy.Subscriber("goal_point", PointStamped, goalCallback, queue_size=1)
    rospy.Subscriber("start_point", PointStamped, startCallback, queue_size=1)

    # this just keeps the node alive, servicing the callbacks
    rospy.spin()
    
    rospy.loginfo(rospy.get_caller_id() + " terminated")