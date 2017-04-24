/*
 * pose_feedback_noise_node.cpp
 *
 *  Created on: Apr 24, 2017
 *      Author: cpbove
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

#include <cstdlib>
#include <random>
#include <ctime>

ros::Publisher posePub;

// http://stackoverflow.com/questions/14638739/generating-a-random-double-between-a-range-of-values
//Mersenne Twister: Good quality random number generator
std::mt19937 rng;
std::normal_distribution<double> linearDist(-0.001, 0.001);
std::normal_distribution<double> angularDist(-0.0174533, 0.0174533); // 1 deg in rad

void poseCallback(geometry_msgs::PoseStamped data){
	geometry_msgs::PoseStamped noisyData = data;

	noisyData.pose.position.x += linearDist(rng);
	noisyData.pose.position.y += linearDist(rng);
	noisyData.pose.position.z += linearDist(rng);

	// this is kinda hacky - but I'm messing with the quaternion and then normalizing it
	tf::Quaternion quat;
	tf::quaternionMsgToTF(noisyData.pose.orientation, quat);
	quat.setX(quat.x() + angularDist(rng));
	quat.setY(quat.y() + angularDist(rng));
	quat.setY(quat.z() + angularDist(rng));
	quat.setY(quat.w() + angularDist(rng));

	quat.normalize();

	tf::quaternionTFToMsg(quat, noisyData.pose.orientation);

	posePub.publish(noisyData);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "pose_feedback_noise_node");
	ros::NodeHandle nh;

	rng.seed(std::random_device{}());

	posePub = nh.advertise<geometry_msgs::PoseStamped>( "needle_tip_pose_noisy", 1 );
	ros::Subscriber statSub = nh.subscribe("needle_tip_pose", 1, poseCallback);

	ros::spin();
}
