/*
 * pose_feedback_noise_node.cpp
 *
 *  Created on: Apr 24, 2017
 *      Author: cpbove
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <cstdlib>
#include <random>
#include <ctime>

ros::Publisher posePub;

void poseCallback(geometry_msgs::PoseStamped data){

}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_feedback_noise_node");
	ros::NodeHandle nh;

	posePub = nh.advertise<geometry_msgs::PoseStamped>( "needle_tip_pose_noisy", 1 );
	ros::Subscriber statSub = nh.subscribe("needle_tip_pose", 1, poseCallback);

	ros::spin();
}
