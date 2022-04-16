#include <ros/ros.h>
#include <carplanner_msgs/MotionSample.h>
#include <nav_msgs/Path.h>

ros::Publisher pub;

void cb(const carplanner_msgs::MotionSample::ConstPtr msg_in)
{
  nav_msgs::Path msg_out;
  std::string frame_id = "map";//msg_in->states[0].header.frame_id;
  msg_out.header.frame_id = frame_id;
  msg_out.header.stamp = ros::Time::now();
  for (uint i=0; i<msg_in->states.size(); i++)
  {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = msg_in->states[i].header.stamp;
    pose_msg.header.frame_id = frame_id;
    pose_msg.pose.position.x =    msg_in->states[i].pose.transform.translation.x;
    pose_msg.pose.position.y =    msg_in->states[i].pose.transform.translation.y;
    pose_msg.pose.position.z =    msg_in->states[i].pose.transform.translation.z;
    pose_msg.pose.orientation.x = msg_in->states[i].pose.transform.rotation.x;
    pose_msg.pose.orientation.y = msg_in->states[i].pose.transform.rotation.y;
    pose_msg.pose.orientation.z = msg_in->states[i].pose.transform.rotation.z;
    pose_msg.pose.orientation.w = msg_in->states[i].pose.transform.rotation.w;
    msg_out.poses.push_back(pose_msg);
  }
  pub.publish(msg_out);
  ros::spinOnce();
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "motionsample2path");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<carplanner_msgs::MotionSample>("sample", 5, cb);
    pub = nh.advertise<nav_msgs::Path>("path",5);

    ROS_INFO("motionsample2path initialized.");

    ros::spin();
}
