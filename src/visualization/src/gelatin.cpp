#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher pubTrackMarkers;
  pubTrackMarkers = n.advertise<visualization_msgs::Marker>("/track_markers",1, true);
  visualization_msgs::Marker markerMsg;

  markerMsg.header.frame_id  = "/trackMarkers_frame";
  markerMsg.ns = "trackMarkers";
  markerMsg.action = visualization_msgs::Marker::ADD;
  markerMsg.pose.orientation.w = 1.0;
  markerMsg.type = visualization_msgs::Marker::CUBE_LIST;
  markerMsg.scale.x = 0.8f;
  markerMsg.scale.y = 0.8f;
  markerMsg.scale.z = 0.8f;
  if (true){
      markerMsg.color.r = 0;
      markerMsg.color.g = 1;
      markerMsg.color.b = 0;
      markerMsg.color.a = 1;
  }

  for (int i=0; i<20; i++)
  {
    for(int j= -10; j < 10; j++  )
    {
      for (int k = -10; k < 10; k++)
      {
          geometry_msgs::Point temp;
          temp.x = i;
          temp.y = j;
          temp.z = k;
          markerMsg.points.push_back(temp);
          std_msgs::ColorRGBA c;
          c.r = 0;//(float)i/10.0;
          c.g = 0;
          c.b = 0;
          c.a = 0.5;
          markerMsg.colors.push_back(c);
      }
    }
  }

  pubTrackMarkers.publish(markerMsg);
  //markerMsg.points=points.clear();
  //markerMsg.colors=colors.clear();
  while (ros::ok()) {
      ros::spin();
  }
}
