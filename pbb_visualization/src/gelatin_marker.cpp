#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// TODO make these ROS params - these are the dimensions of the box/gelatin
#define HALF_SIDES (0.10/2.0)
#define DEPTH (0.20)


int main( int argc, char** argv )
{
  ros::init(argc, argv, "gelatin");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("gelatin", 1);


  while (ros::ok())
  {
    visualization_msgs::Marker line_marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    line_marker.header.frame_id = "/gelatin";
    line_marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    line_marker.ns = "gelatin";
    line_marker.id = 0;

    // Set the marker type.
    line_marker.type = visualization_msgs::Marker::LINE_STRIP;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    line_marker.action = visualization_msgs::Marker::ADD;

    line_marker.scale.x = 0.001; // line width

    // Set the color -- be sure to set alpha to something non-zero!
    line_marker.color.r = 1.0f;
    line_marker.color.g = 1.0f;
    line_marker.color.b = 1.0f;
    line_marker.color.a = 0.3;

    line_marker.pose.orientation.w = 1.0; // others initialize to 0

    // draw the workspace with a line box... probably in the ugliest manner possible.
    // Totally hardcoded and just barf. I'm sorry for being a terrible person.
    geometry_msgs::Point p;
    p.x = 0; p.y = -HALF_SIDES; p.z = -HALF_SIDES;
    geometry_msgs::Point bot_left = p;
    line_marker.points.push_back(bot_left);
    p.y = HALF_SIDES;
    geometry_msgs::Point bot_right = p;
    line_marker.points.push_back(bot_right);
    p.z = HALF_SIDES;
    geometry_msgs::Point top_right = p;
    line_marker.points.push_back(top_right);
    p.y = -HALF_SIDES;
    geometry_msgs::Point top_left = p;
    line_marker.points.push_back(top_left);
    line_marker.points.push_back(bot_left);
    p = bot_left;
    p.x = -DEPTH;
    geometry_msgs::Point back_bot_left = p;
    line_marker.points.push_back(back_bot_left);
    p.z = HALF_SIDES;
    geometry_msgs::Point back_top_left = p;
    line_marker.points.push_back(back_top_left);
    line_marker.points.push_back(top_left);
    line_marker.points.push_back(back_top_left);

    p = back_top_left;
    p.y = HALF_SIDES;
    geometry_msgs::Point back_top_right = p;
    line_marker.points.push_back(back_top_right);
    line_marker.points.push_back(top_right);
    line_marker.points.push_back(back_top_right);

    p.z = -HALF_SIDES;
    geometry_msgs::Point back_bot_right = p;
    line_marker.points.push_back(back_bot_right);
    line_marker.points.push_back(bot_right);
    line_marker.points.push_back(back_bot_right);
    line_marker.points.push_back(back_bot_left);


    line_marker.lifetime = ros::Duration();

    marker_pub.publish(line_marker);

    r.sleep();
  }
}
