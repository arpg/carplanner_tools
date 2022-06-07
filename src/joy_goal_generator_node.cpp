#include <ros/ros.h>
#include <carplanner_tools/joy_goal_generator.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "joy_goal_generator_node");

  ros::NodeHandle nh(""), nh_param("~");
  joy_goal_generator::JoyGoalGenerator jgg(&nh, &nh_param);

  ros::spin();
}
