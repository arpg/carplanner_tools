#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>

typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 7, 1> Vec7;

std::string pose_topic, odom_topic, twist_topic, child_frame_id;
ros::Publisher odom_pub, twist_pub;
double avg_window_length;

// std::vector<nav_msgs::Odometry> odoms;

std::vector<Vec7> past_vels;
geometry_msgs::PoseStamped* last_pose;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr pose_msg)
{
  // ROS_INFO("Got pose: %f %f %f %f %f %f %f",
  //   pose_msg->pose.position.x,
  //   pose_msg->pose.position.y,
  //   pose_msg->pose.position.z, 
  //   pose_msg->pose.orientation.x,
  //   pose_msg->pose.orientation.y,
  //   pose_msg->pose.orientation.z,
  //   pose_msg->pose.orientation.w
  // );

  // if pose is too old, skip
  if ( (ros::Time::now() - pose_msg->header.stamp).toSec() > avg_window_length )
    return;

  // if we already have at least one pose, we can use the previous poses to estimate velocity here
  Vec7 new_vels = Vec7::Zero();
  if (last_pose!=nullptr)
  {
    double dt = (pose_msg->header.stamp-last_pose->header.stamp).toSec();

    new_vels(0) = last_pose->header.stamp.toSec(); // first element is time

    // differential position for linear velocity estimation
    Vec3 dlin;
    dlin[0] = pose_msg->pose.position.x-last_pose->pose.position.x;
    dlin[1] = pose_msg->pose.position.y-last_pose->pose.position.y;
    dlin[2] = pose_msg->pose.position.z-last_pose->pose.position.z;
    new_vels.block<3,1>(1,0) = dlin/dt;

    // differential orientation for angular velocity estimation
    Eigen::Quaterniond this_quat(pose_msg->pose.orientation.w, pose_msg->pose.orientation.x, pose_msg->pose.orientation.y, pose_msg->pose.orientation.z);
    Eigen::Quaterniond last_quat(last_pose->pose.orientation.w, last_pose->pose.orientation.x, last_pose->pose.orientation.y, last_pose->pose.orientation.z);
    // Eigen::Quaterniond dquat = this_quat - last_quat;
    Eigen::Quaterniond dquat = last_quat.inverse() * this_quat;
    // differential orientation in euler
    Eigen::Vector3d dang;
    dang[0] = atan2(2*(dquat.w()*dquat.x()+dquat.y()*dquat.z()), 1-2*(pow(dquat.x(),2)+pow(dquat.y(),2)));
    dang[1] = asin(2*(dquat.w()*dquat.y()+dquat.z()*dquat.x()));
    dang[2] = atan2(2*(dquat.w()*dquat.z()+dquat.x()*dquat.y()), 1-2*(pow(dquat.y(),2)+pow(dquat.z(),2)));
    new_vels.block<3,1>(4,0) = dang/dt;
  }
  past_vels.push_back(new_vels);

  // go through the history of velocity estimations and make sure none are too old. if so, erase. if not, average.
  Vec6 vels = Vec6::Zero();
  // uint ii=0;
  for (auto i = past_vels.begin(); i != past_vels.end(); i++)
  {
    // std::cout << "past vel " << ii++ << std::endl;
    if ( (pose_msg->header.stamp.toSec()-(*i)(0)) > avg_window_length )
    {
      past_vels.erase(i--);
      continue;
    }
    vels += i->tail(6);
  }
  vels /= past_vels.size();

  // update last pose
  if (last_pose)
    delete last_pose;
  last_pose = new geometry_msgs::PoseStamped(*pose_msg);

  // set current velocity estimate in msg and publish
  nav_msgs::Odometry odom_msg;
  odom_msg.header = pose_msg->header;
  odom_msg.child_frame_id = child_frame_id;
  odom_msg.pose.pose = pose_msg->pose;
  odom_msg.twist.twist.linear.x = vels[0];
  odom_msg.twist.twist.linear.y = vels[1];
  odom_msg.twist.twist.linear.z = vels[2];
  odom_msg.twist.twist.angular.x = vels[3];
  odom_msg.twist.twist.angular.y = vels[4];
  odom_msg.twist.twist.angular.z = vels[5];

  // odoms.push_back(odom_msg);

  odom_pub.publish(odom_msg);

  twist_pub.publish(odom_msg.twist);
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "pose2odom");

    ros::NodeHandle pnh("~");

    pnh.param("pose_topic", pose_topic, std::string("pose"));
    pnh.param("odom_topic", odom_topic, std::string("odom"));
    pnh.param("twist_topic", twist_topic, std::string("twist"));
    pnh.param("child_frame_id", child_frame_id, std::string("base_link"));
    pnh.param("avg_window_length", avg_window_length, (double)1.f);

    ros::Subscriber pose_sub = pnh.subscribe<geometry_msgs::PoseStamped>(pose_topic, 5, pose_cb);
    odom_pub = pnh.advertise<nav_msgs::Odometry>(odom_topic, 5);
    twist_pub = pnh.advertise<geometry_msgs::TwistWithCovariance>(twist_topic, 5);

    ROS_INFO("Initialized.");

    ros::spin();
}
