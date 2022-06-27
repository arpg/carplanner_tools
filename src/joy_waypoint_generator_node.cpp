#include <ros/ros.h>
#include <carplanner_tools/joy_waypoint_generator.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "joy_waypoint_generator_node");

  ros::NodeHandle nh(""), nh_param("~");
  joy_waypoint_generator::JoyWaypointGenerator joy_waypoint_generator(&nh, &nh_param);

  ros::spin();
}
