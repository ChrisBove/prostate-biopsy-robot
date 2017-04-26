#!/usr/bin/env python
"""@package docstring
    This is the target path drawing node for the needle prostate bot
"""

import rospy
import tf
import numpy as np
import math
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from scipy.linalg import expm

# Constants
l1 = 0.04  # Distance B to C
phi = 0.17767451785
l2 = 0.023775
k = math.tan(phi) / l1
r = 1 / k


def isomorphic(x):
    return np.array([[0, -x.item(2), x.item(1)],
		[x.item(2), 0, -x.item(0)],
		[-x.item(1), x.item(0), 0]])


def isFeasible(x0,y0,z0,x, y, z):
    d = math.sqrt(math.pow(x-x0, 2) + math.pow(y-y0, 2))
    if (math.pow(d-r, 2) <= math.pow(r, 2)):
    	z_tip = math.sqrt(math.pow(r, 2) - math.pow(d-r, 2))
    else:
        return False
    if (z_tip < z):
        return True
    return False


def kinematicModel(u1, u2, T, state):
    e1 = np.array([[1], [0], [0]])
    e3 = np.array([[0], [0], [1]])
    V1 = np.concatenate(([[e3], [k * e1]]), axis=0)
    V2 = np.concatenate(([[0], [0], [0]], e3), axis=0)
    V1_hat = np.concatenate((isomorphic(V1[1]), V1[0]), axis=1)
    V1_hat = np.vstack((V1_hat, np.array([0, 0, 0, 0])))
    V2_hat = np.concatenate((isomorphic(V2[3:6]), V2[0:3]), axis=1)
    V2_hat = np.vstack((V2_hat, np.array([0, 0, 0, 0])))

    new_state = np.array(state[:, :].dot(
        expm((u1 * V1_hat + u2 * V2_hat) * T)))
    n = (new_state[0:3, 0:3] * l2).dot(e3) + (new_state[0:3, 3]).reshape(3, 1)

    return (n, new_state)


def planPath(x0,y0,z0,x, y, z):
    path = []

    if (isFeasible(x0,y0,z0,x, y, z) is False):
        print "Not Feasible"
        return path
    else:
	    print "Feasible"
	# Calculate initial angle
    angle = math.atan2(y,x)
        # Create initial state which is rotated by 90 degrees
    state = np.array([[math.cos(angle+math.pi/2),-math.sin(angle+math.pi/2),0,x0],
			  [math.sin(angle+math.pi/2),math.cos(angle+math.pi/2),0,y0],
			  [0,	0,	1,	z0],
			  [0,	0,	0,	1]])
	# Calculate distance between the center of the circle and the goal
    d_c = math.sqrt(math.pow(abs(x-x0)-r,2)+math.pow(abs(y-y0)-r,2)+math.pow(z-z0,2))
	# Calculate length of line tangent to circle
    d_thres = math.sqrt(math.pow(d_c,2)-math.pow(r,2))
    t = 0.02
    distance = 1000000

    while (distance > d_thres):
        (n,state) = kinematicModel(0.01,0,t,state)
        path.append(n)
        distance = math.sqrt(math.pow(n[0]-x,2)+math.pow(n[1]-y,2)+math.pow(n[2]-z,2))

    path.append(np.array([[x],[y],[z]])) #list of arrays(x,y,z)
    return path

def goalCallback(data):
    """This callback will occur once when the """
    rospy.loginfo(rospy.get_caller_id() + " got goal point \n%s", data.point)
    # do something with your goal point! (probably enter a loop or something)
    # pull out info like data.point.x, data.point.y, etc.

    # create a line marker msg and fill it
    line = Marker()
    # header info
    line.header.frame_id = "gelatin"
    line.header.stamp = rospy.Time.now()

    # define marker type
    line.type = Marker.LINE_STRIP

    # same orientation for all (0,0,0,1 quat)
    line.pose.orientation.w = 1.0

    # coloring
    line.color.a = 0.8
    line.color.r = 1.0
    # g and b will be 0

    # just need x scale for a line width
    line.scale.x = 0.002

    # let's stuff some points into the thing
    p = Point()
    p = startPoint.point
    line.points.append(p)

    # add lines below to keep adding intermediate points

    path = planPath(p.z, p.y, p.x,data.point.z, data.point.y, data.point.x)

    if not (len(path) == 0):
        for i in path:
            p_local = Point(i[2],i[1],i[0])
            line.points.append(p_local)

        markerPub.publish(line)
        # pass along the goal point as reachable
        reachableGoalPub.publish(data)


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
    global reachableGoalPub

    global startPoint
    startPoint = PointStamped()

    # setup the publisher for the marker path
    markerPub = rospy.Publisher('target_path', Marker, None, queue_size=10)
    reachableGoalPub = rospy.Publisher('goal_point_reachable', PointStamped, None, queue_size=10)

    # setup the subscribers to the interactive marker Point topics
    rospy.Subscriber("goal_point", PointStamped, goalCallback, queue_size=1)
    rospy.Subscriber("start_point", PointStamped, startCallback, queue_size=1)

    # this just keeps the node alive, servicing the callbacks
    rospy.spin()

    rospy.loginfo(rospy.get_caller_id() + " terminated")
