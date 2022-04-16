#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <carplanner_msgs/VehicleState.h>

ros::Publisher pose_pub;

void vehicle_state_cb(const carplanner_msgs::VehicleState::ConstPtr msg)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.frame_id = "world";
  pose_msg.header.stamp = ros::Time::now();
  pose_msg.pose.position.x =    msg->pose.transform.translation.x;
  pose_msg.pose.position.y =    msg->pose.transform.translation.y;
  pose_msg.pose.position.z =    msg->pose.transform.translation.z;
  pose_msg.pose.orientation.x = msg->pose.transform.rotation.x;
  pose_msg.pose.orientation.y = msg->pose.transform.rotation.y;
  pose_msg.pose.orientation.z = msg->pose.transform.rotation.z;
  pose_msg.pose.orientation.w = msg->pose.transform.rotation.w;
  pose_pub.publish(pose_msg);
  ros::spinOnce();
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "vehicestate2pose");

    ros::NodeHandle nh;
    ros::Subscriber vehicle_state_sub = nh.subscribe<carplanner_msgs::VehicleState>("mochagui/state", 5, vehicle_state_cb);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mochagui/pose_from_state",5);

    printf("%sInitialized.\n","[vehicestate2pose] ");

    ros::spin();
}
