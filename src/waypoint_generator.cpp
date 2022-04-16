#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <carplanner_msgs/OdometryArray.h>
// #include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

ros::Publisher pub;
ros::Timer loop;
std::string goal_topic, waypoints_topic;
std::string odom_topic;
// std::string map_frame, base_frame;
geometry_msgs::PoseStamped goal;
nav_msgs::Odometry odom;
float rate;

void goal_cb(const geometry_msgs::PoseStamped::ConstPtr msg)
{
  // ROS_INFO("Got goal: %f %f %f %f %f %f %f",
  //   msg->pose.position.x,
  //   msg->pose.position.y,
  //   msg->pose.position.z, 
  //   msg->pose.orientation.x,
  //   msg->pose.orientation.y,
  //   msg->pose.orientation.z,
  //   msg->pose.orientation.w
  // );

  goal = *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr msg)
{
  // ROS_INFO("Got odom: %f %f %f %f %f %f %f",
  //   msg->pose.pose.position.x,
  //   msg->pose.pose.position.y,
  //   msg->pose.pose.position.z, 
  //   msg->pose.pose.orientation.x,
  //   msg->pose.pose.orientation.y,
  //   msg->pose.pose.orientation.z,
  //   msg->pose.pose.orientation.w
  // );

  odom = *msg;
}

void loopFunc(const ros::TimerEvent& event)
{
    if (goal.header.seq==0) return;
    if (odom.header.seq==0) return;
    carplanner_msgs::OdometryArray odom_arr;

    // add start waypoint from tf lookup
    // {
    //   static tf::TransformListener tflistener;
    //   static tf::StampedTransform Twc;  
    //   try
    //   {
    //       tflistener.waitForTransform(map_frame, base_frame, ros::Time::now(), ros::Duration(0.5));
    //       tflistener.lookupTransform(map_frame, base_frame, ros::Time(0), Twc);
    //   } 
    //   catch (tf::TransformException ex)
    //   {
    //       ROS_ERROR("%s",ex.what());
    //       return;
    //   }

    //   ROS_INFO("Looked up start: %f %f %f %f %f %f %f",
    //     Twc.getOrigin().getX(),
    //     Twc.getOrigin().getY(),
    //     Twc.getOrigin().getZ(), 
    //     Twc.getRotation().getX(),
    //     Twc.getRotation().getY(),
    //     Twc.getRotation().getZ(),
    //     Twc.getRotation().getW()
    //   );

    //   nav_msgs::Odometry odom_msg;
    //   odom_msg.header.frame_id = map_frame;
    //   odom_msg.header.stamp = Twc.stamp_;
    //   odom_msg.child_frame_id = "start";
    //   odom_msg.pose.pose.position.x =    Twc.getOrigin().getX();
    //   odom_msg.pose.pose.position.y =    Twc.getOrigin().getY();
    //   odom_msg.pose.pose.position.z =    Twc.getOrigin().getZ();
    //   odom_msg.pose.pose.orientation.x = Twc.getRotation().getX();
    //   odom_msg.pose.pose.orientation.y = Twc.getRotation().getY();
    //   odom_msg.pose.pose.orientation.z = Twc.getRotation().getZ();
    //   odom_msg.pose.pose.orientation.w = Twc.getRotation().getW();
    //   odom_msg.twist.twist.linear.x =  1;
    //   odom_msg.twist.twist.linear.y =  0;
    //   odom_msg.twist.twist.linear.z =  0;
    //   odom_msg.twist.twist.angular.x = 0;
    //   odom_msg.twist.twist.angular.y = 0;
    //   odom_msg.twist.twist.angular.z = 0;

    //   odom_arr.odoms.push_back(odom);
    // }

    // add start waypoint from odom
    {
      odom_arr.odoms.push_back(odom);
    }

    // add end waypoint from goal
    {
      nav_msgs::Odometry odom_msg;
      // odom_msg.header.frame_id = map_frame;
      odom_msg.header.frame_id = goal.header.frame_id;
      // odom_msg.header.stamp = ros::Time::now();
      odom_msg.header.stamp = goal.header.stamp;
      odom_msg.child_frame_id = "goal";
      odom_msg.pose.pose.position.x =    goal.pose.position.x;
      odom_msg.pose.pose.position.y =    goal.pose.position.y;
      odom_msg.pose.pose.position.z =    goal.pose.position.z;
      odom_msg.pose.pose.orientation.x = goal.pose.orientation.x;
      odom_msg.pose.pose.orientation.y = goal.pose.orientation.y;
      odom_msg.pose.pose.orientation.z = goal.pose.orientation.z;
      odom_msg.pose.pose.orientation.w = goal.pose.orientation.w;
      odom_msg.twist.twist.linear.x =  1;
      odom_msg.twist.twist.linear.y =  0;
      odom_msg.twist.twist.linear.z =  0;
      odom_msg.twist.twist.angular.x = 0;
      odom_msg.twist.twist.angular.y = 0;
      odom_msg.twist.twist.angular.z = 0;

      odom_arr.odoms.push_back(odom_msg);
    }

    // ROS_INFO("Publishing...");
    pub.publish(odom_arr);
    ros::spinOnce();
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "waypoint_generator");

    ros::NodeHandle nh, pnh("~");

    pnh.param("goal_topic", goal_topic, std::string("goal"));
    pnh.param("waypoints_topic", waypoints_topic, std::string("waypoints"));
    pnh.param("odom_topic", odom_topic, std::string("odom"));
    // pnh.param("map_frame_id", map_frame, std::string("map"));
    // pnh.param("base_frame_id", base_frame, std::string("base_link"));
    pnh.param("rate", rate, (float)rate);

    // ROS_INFO("Params:\n goal=%s\n waypoints=%s\n map_frame=%s\n base_link_frame=%s",std::string(goal_topic).c_str(),std::string(waypoints_topic).c_str(),std::string(map_frame).c_str(),std::string(base_frame).c_str());

    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>(goal_topic, 5, goal_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 5, odom_cb);
    pub = nh.advertise<carplanner_msgs::OdometryArray>(waypoints_topic, 5);
    loop = nh.createTimer(ros::Duration(1.0f/rate), loopFunc);

    ROS_INFO("Initialized.");

    ros::spin();
}
