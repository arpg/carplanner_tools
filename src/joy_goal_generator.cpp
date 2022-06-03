#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

typedef Eigen::Transform<double, 3, Eigen::TransformTraits::Affine> EigenTransform;

std::string goal_topic, odom_topic;
ros::Publisher goal_pub;
double path_length;
double path_angle;
float rate;
nav_msgs::Odometry odom_msg;

void odom_cb(const nav_msgs::Odometry::ConstPtr msg)
{
    odom_msg = *msg;
    ROS_INFO("Got odom x:%f y:%f z:%f",odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y,odom_msg.pose.pose.position.z);
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "joy_goal_generator");

    ros::NodeHandle nh, pnh("~");

    pnh.param("goal_topic", goal_topic, std::string("goal"));
    pnh.param("odom_topic", odom_topic, std::string("odom"));
    pnh.param("rate", rate, (float)10.0);
    pnh.param("path_length", path_length, 1.0);
    pnh.param("path_angle", path_angle, 0.0);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 5, odom_cb);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 5);

    ROS_INFO("Initialized.");

    uint last_odom_seq=0;
    while(ros::ok())
    {
      // if (goal_msg.header.seq>0)
      // {
      //   ROS_INFO("Publishing goal x:%f y:%f z:%f.", goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z);
      //   goal_pub.publish(goal_msg);
      // }

      /// if we are not ready to calculate the goal, skip
      // if (odom_msg.header.seq<=0 || odom_msg.header.seq==last_odom_seq)
      // {
      //   ROS_WARN_THROTTLE(5, "Haven't gotten odom yet");
      //   continue;
      // }

      EigenTransform start_transform = EigenTransform::Identity();
      start_transform.translate(Eigen::Vector3d(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z));
      start_transform.rotate(Eigen::Quaterniond(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z));

      EigenTransform diff_goal_transform = EigenTransform::Identity();
      diff_goal_transform.translate(Eigen::Vector3d(path_length, 0, 0));
      diff_goal_transform.rotate(Eigen::Quaterniond(1.0, 0, 0, 0));

      EigenTransform goal_transform = start_transform * diff_goal_transform;

      Eigen::Quaterniond quat(goal_transform.matrix().block<3,3>(0,0));

      // std::cout << "start\n" << start_transform.matrix() << "\ndiff\n" << diff_goal_transform.matrix() << "\ngoal\n" << goal_transform.matrix() << "\nquat\n" << quat.matrix() << std::endl; 

      geometry_msgs::PoseStamped goal_pose;
      goal_pose.header = odom_msg.header;
      goal_pose.pose.position.x = goal_transform.translation().x();
      goal_pose.pose.position.y = goal_transform.translation().y();
      goal_pose.pose.position.z = goal_transform.translation().z();
      goal_pose.pose.orientation.x = quat.x();
      goal_pose.pose.orientation.y = quat.y();
      goal_pose.pose.orientation.z = quat.z();
      goal_pose.pose.orientation.w = quat.w();

      ROS_INFO("Publishing goal px:%f py:%f pz:%f qx:%f qy:%f qz:%f qw:%f.", 
        goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z,
        goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);
      goal_pub.publish(goal_pose);

      ros::spinOnce();
      ros::Rate(rate).sleep();
    }
}
