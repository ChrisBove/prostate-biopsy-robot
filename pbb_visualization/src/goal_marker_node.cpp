// This is bad - the one marker node should be parameterized and configured at launch for whether
// its a goal or a start node.

/*
 * Modified from the below code:
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>

using namespace visualization_msgs;

ros::Publisher pointPub, trajPointPub, resetPub;

void processFeedback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
	std::ostringstream s;
	//	Eigen::Affine3d point;
	std_msgs::Bool reset_msg;
	geometry_msgs::PointStamped msg;
	msg.header.frame_id = "gelatin";
	msg.header.stamp = ros::Time::now();
	msg.point = feedback->pose.position;

	switch(feedback->event_type){
	case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
		//		ROS_INFO_STREAM(s.str() << data->marker_name << " is located at " << data->pose.position.x << ", "
		//				<< data->pose.position.y << ", "
		//				<< data->pose.position.z );
		//		tf::poseMsgToEigen(feedback->pose,point);
		//		std::cout << point.matrix() << std::endl;
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		switch(feedback->menu_entry_id){
		case 1:
			ROS_INFO_STREAM("Go to Point Selected!");
			pointPub.publish(msg);
			break;
		case 2:
			ROS_INFO_STREAM("Plan Trajectory selected!");
			trajPointPub.publish(msg);
			break;
		case 3:
			ROS_INFO_STREAM("Reset Robot Selected!");
			reset_msg.data = true;
			resetPub.publish(reset_msg);
			break;

		default:
			ROS_WARN_STREAM(s.str() << "Unknown menu option selected! Code: " << feedback->menu_entry_id);
			break;
		}
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_marker_node");
	ros::NodeHandle nh;

	pointPub = nh.advertise<geometry_msgs::PointStamped>( "goal_point", 1 );
	trajPointPub = nh.advertise<geometry_msgs::PointStamped>( "traj_goal_point", 1 );
	resetPub = nh.advertise<std_msgs::Bool>( "reset_robot", 1 );

	// create an interactive marker server on the topic namespace simple_marker
	interactive_markers::InteractiveMarkerServer server("goal_marker_node");

	// create an interactive marker for our server
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "gelatin";
	int_marker.header.stamp=ros::Time::now();
	int_marker.name = "goal_marker";
	int_marker.description = "Goal Marker";
	int_marker.scale = 0.05;

	interactive_markers::MenuHandler menuHandler;
	menuHandler.insert("Go to Point", &processFeedback);
	menuHandler.insert("Plan Trajectory", &processFeedback);
	menuHandler.insert("Reset Robot", &processFeedback);

	// create a grey box marker
	visualization_msgs::Marker box_marker;
	box_marker.type = visualization_msgs::Marker::SPHERE;
	box_marker.scale.x = 0.01;
	box_marker.scale.y = 0.01;
	box_marker.scale.z = 0.01;
	box_marker.color.r = 1.0;
	box_marker.color.g = 0.0;
	box_marker.color.b = 0.0;
	box_marker.color.a = 1.0;

	// create a non-interactive control which contains the box
	visualization_msgs::InteractiveMarkerControl box_control;
	box_control.always_visible = true;
	box_control.markers.push_back( box_marker );

	// add the control to the interactive marker
	int_marker.controls.push_back( box_control );

	visualization_msgs::InteractiveMarkerControl control;
	control.orientation.w = 1;
	control.orientation.x = 1;
	control.orientation.y = 0;
	control.orientation.z = 0;
	control.name = "move_x";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 1;
	control.orientation.z = 0;
	control.name = "move_z";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	control.orientation.w = 1;
	control.orientation.x = 0;
	control.orientation.y = 0;
	control.orientation.z = 1;
	control.name = "move_y";
	control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
	int_marker.controls.push_back(control);

	// add the control to the interactive marker
	int_marker.controls.push_back(control);

	int_marker.pose.position.x = -0.10;

	// add the interactive marker to our collection &
	// tell the server to call processFeedback() when feedback arrives for it
	server.insert(int_marker, &processFeedback);

	menuHandler.apply(server, int_marker.name);

	// 'commit' changes and send to all clients
	server.applyChanges();

	// start the ROS main loop
	ros::spin();
}
