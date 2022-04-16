#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <carplanner_msgs/Command.h>
#include <algorithm>

const float max_lin_x = 0.25;
const float max_ang_z = 6.28/8;

const float servo_range = 500;
const float accel_offset = 0.469;
const float steer_offset = 0.480;

// const float force_rng = 0.469000000*500;
const float force_rng = 0.469000000*1000;

// const float phi_rng = 0.480091000*500;
const float phi_rng = 0.480091000*1000;

ros::Publisher cmd_vel_pub;

void command_cb(const carplanner_msgs::Command::ConstPtr msg)
{
  geometry_msgs::Twist cmd_vel;
  // cmd_vel->linear.x = std::min( (msg->force/mass)*msg->dt+prev_lin_x , max_lin_x );
  // cmd_vel.linear.x = (msg->force - accel_offset*servo_range)/100; 
  cmd_vel.linear.x = (msg->force)/servo_range; 
  cmd_vel.linear.y = 0;
  cmd_vel.linear.z = 0;
  // cmd_vel.linear.x = msg->force;
  // cmd_vel.linear.x = msg->target_vel[0];
  // cmd_vel.linear.y = msg->target_vel[1];
  // cmd_vel.linear.z = msg->target_vel[2];
  cmd_vel.angular.x = 0;
  cmd_vel.angular.y = 0;
  cmd_vel.angular.z = (msg->dphi - steer_offset*servo_range)/servo_range;
  // cmd_vel.angular.z = (msg->phi/phi_rng)*max_ang_z;
  // cmd_vel.angular.z = msg->phi;

  cmd_vel.linear.x = std::min((float)cmd_vel.linear.x, (float)max_lin_x);
  cmd_vel.linear.y = std::min((float)cmd_vel.linear.y, (float)max_lin_x);
  cmd_vel.linear.z = std::min((float)cmd_vel.linear.z, (float)max_lin_x);
  cmd_vel.angular.x = std::min((float)cmd_vel.angular.x, (float)max_ang_z);
  cmd_vel.angular.y = std::min((float)cmd_vel.angular.y, (float)max_ang_z);
  cmd_vel.angular.z = std::min((float)cmd_vel.angular.z, (float)max_ang_z);

  cmd_vel_pub.publish(cmd_vel);
  ros::spinOnce();
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "command2cmdvel");

    ros::NodeHandle nh;
    ros::Subscriber command_sub = nh.subscribe<carplanner_msgs::Command>("mochagui/command", 1, command_cb);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

    printf("%sInitialized.\n","[command2cmdvel] ");

    ros::spin();
}
