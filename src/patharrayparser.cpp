#include <ros/ros.h>
#include <carplanner_msgs/PathArray.h>

ros::Publisher pub;

void callback(const carplanner_msgs::PathArray::ConstPtr msg)
{
  ROS_INFO("parsing %d paths",msg->paths.size());
  for(uint i=0; i < msg->paths.size(); i++)
  {
    pub.publish(msg->paths[i]);
  }
  ros::spinOnce();
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "patharrayparser");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<carplanner_msgs::PathArray>("path_array", 1, callback);
    pub = nh.advertise<nav_msgs::Path>("path",1);

    printf("%sInitialized.\n","[patharrayparser] ");

    ros::spin();
}
