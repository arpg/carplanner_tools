/********************************************************
 * This node subscribes to a ControlCommand (accel-based) 
 * and converts it to a DriveCommand (vel-based)
 * Author: Mike Miles
*********************************************************/

#include <ros/ros.h>
#include <carplanner_msgs/Command.h>
#include <drive_control/DriveCommand.h>

// const double max_speed = 5.; // m/s
// const double min_speed = -2.5; // m/s
// const double scale_speed = 5882.; // 5882 ERPM correspondes to 1 m/s

// const double scale_steer = -0.3; // 

// const double mass = 0.473368000;

const double servo_range = 500.;
const double accel_offset = 0.469000000;
const double steer_offset = 0.480091000;

ros::Publisher drive_pub;
drive_control::DriveCommand drive_msg;

void command_cb(const carplanner_msgs::Command::ConstPtr msg)
{
  // drive_msg.control_value = (msg->force-accel_offset*servo_range)/mass*msg->dt;
  drive_msg.control_value = msg->force/servo_range;
  
  drive_msg.front_steer_angle = -(msg->dphi-steer_offset*servo_range)/servo_range;
  
  drive_msg.rear_steer_angle = 0.0;

  drive_pub.publish(drive_msg);
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "controlcommand2drivecommand");

    ros::NodeHandle nh;
    ros::Subscriber command_sub = nh.subscribe<carplanner_msgs::Command>("control_command", 1, command_cb);
    drive_pub = nh.advertise<drive_control::DriveCommand>("drive_command",1);

    ROS_INFO("Initialized.");

    ros::spin();
}
