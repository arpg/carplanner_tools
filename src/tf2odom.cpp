#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Eigen>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf2odom");

  ros::NodeHandle node;
  ros::NodeHandle pnode("~");

  std::string parent_frame, child_frame, odom_topic;
  pnode.param("parent_frame", parent_frame, std::string("world"));
  pnode.param("child_frame", child_frame, std::string("base_link"));
  pnode.param("odom_topic", odom_topic, std::string("odometry"));
  double rate_;
  pnode.param("rate", rate_, (double)50);
  double cache_len_;
  pnode.param("cache_len", cache_len_, (double)0.1);

  ros::Publisher robot_odom_pub = node.advertise<nav_msgs::Odometry>(odom_topic, 10);

  ROS_INFO("Initialized.");

  tf::TransformListener listener;
  nav_msgs::Odometry odom_msg;
  geometry_msgs::TransformStamped tf_msg;
  geometry_msgs::Quaternion quat_msg;

  geometry_msgs::PointStamped point_msg;

  std::vector<nav_msgs::Odometry> odom_msg_cache;

  odom_msg.header.frame_id = parent_frame;
  odom_msg.child_frame_id = child_frame;
  ros::Rate rate(rate_);
  while (node.ok()){
    rate.sleep();
    ros::spinOnce();
    
    // look up pose from tf tree
    tf::StampedTransform transform;
    try{
      listener.lookupTransform(parent_frame.c_str(), child_frame.c_str(),
                               ros::Time(0), transform);
      odom_msg.header.stamp = transform.stamp_;
      odom_msg.pose.pose.position.x = transform.getOrigin().x();
      odom_msg.pose.pose.position.y = transform.getOrigin().y();
      odom_msg.pose.pose.position.z = transform.getOrigin().z();
      tf::quaternionTFToMsg(transform.getRotation().normalize(), odom_msg.pose.pose.orientation);
    }
    catch (tf::TransformException ex){
      ROS_WARN_THROTTLE(1.0,"%s",ex.what());
      // ros::Duration(0.1).sleep();
      continue;
    }

    // if we don't have any history saved, add the first one and move on
    if (odom_msg_cache.size()<=0)
    {
      odom_msg_cache.push_back(odom_msg);
      continue;
    }

    // if dt is negative, something is wrong. skip
    double dt = (odom_msg.header.stamp-odom_msg_cache[odom_msg_cache.size()-1].header.stamp).toSec();
    if (dt <= 0.f)
      continue;

    // estimate linear velocity via finite difference
    // if (odom_msg_cache.size() > 0)
    {
      odom_msg.twist.twist.linear.x = (odom_msg.pose.pose.position.x - odom_msg_cache[odom_msg_cache.size()-1].pose.pose.position.x)/dt;
      odom_msg.twist.twist.linear.y = (odom_msg.pose.pose.position.y - odom_msg_cache[odom_msg_cache.size()-1].pose.pose.position.y)/dt;
      odom_msg.twist.twist.linear.z = (odom_msg.pose.pose.position.z - odom_msg_cache[odom_msg_cache.size()-1].pose.pose.position.z)/dt;

      Eigen::Quaterniond this_quat(odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z);
      Eigen::Quaterniond last_quat(odom_msg_cache[odom_msg_cache.size()-1].pose.pose.orientation.w, odom_msg_cache[odom_msg_cache.size()-1].pose.pose.orientation.x, odom_msg_cache[odom_msg_cache.size()-1].pose.pose.orientation.y, odom_msg_cache[odom_msg_cache.size()-1].pose.pose.orientation.z);
      // Eigen::Quaterniond dquat = this_quat - last_quat;
      Eigen::Quaterniond dquat = last_quat.inverse() * this_quat;
      // differential orientation in euler
      Eigen::Vector3d dang;
      dang[0] = atan2(2*(dquat.w()*dquat.x()+dquat.y()*dquat.z()), 1-2*(pow(dquat.x(),2)+pow(dquat.y(),2)));
      dang[1] = asin(2*(dquat.w()*dquat.y()+dquat.z()*dquat.x()));
      dang[2] = atan2(2*(dquat.w()*dquat.z()+dquat.x()*dquat.y()), 1-2*(pow(dquat.y(),2)+pow(dquat.z(),2)));

      odom_msg.twist.twist.angular.x = atan2(2*(dquat.w()*dquat.x()+dquat.y()*dquat.z()), 1-2*(pow(dquat.x(),2)+pow(dquat.y(),2)))/dt;
      odom_msg.twist.twist.angular.y = asin(2*(dquat.w()*dquat.y()+dquat.z()*dquat.x()))/dt;
      odom_msg.twist.twist.angular.z = atan2(2*(dquat.w()*dquat.z()+dquat.x()*dquat.y()), 1-2*(pow(dquat.y(),2)+pow(dquat.z(),2)))/dt;
    }

    odom_msg_cache.push_back(odom_msg);

    // estimate current velocity from history
    geometry_msgs::Vector3 avg_vel;
    geometry_msgs::Vector3 avg_avel;
    for (auto cache_ptr = odom_msg_cache.begin(); cache_ptr != odom_msg_cache.end(); cache_ptr++)
    {
      // if message is too old for estimation, erase and continue
      if ((odom_msg.header.stamp-cache_ptr->header.stamp).toSec() > cache_len_)
      {
        odom_msg_cache.erase(cache_ptr--);
        continue;
      }
      // else, sum
      avg_vel.x += cache_ptr->twist.twist.linear.x;
      avg_vel.y += cache_ptr->twist.twist.linear.y;
      avg_vel.z += cache_ptr->twist.twist.linear.z;
      avg_avel.x += cache_ptr->twist.twist.angular.x;
      avg_avel.y += cache_ptr->twist.twist.angular.y;
      avg_avel.z += cache_ptr->twist.twist.angular.z;
    }
    // and average
    if (odom_msg_cache.size()>0)
    {
      avg_vel.x /= odom_msg_cache.size();
      avg_vel.y /= odom_msg_cache.size();
      avg_vel.z /= odom_msg_cache.size();
      avg_avel.x /= odom_msg_cache.size();
      avg_avel.y /= odom_msg_cache.size();
      avg_avel.z /= odom_msg_cache.size();
    } else {
      ROS_WARN_THROTTLE(5.f, "odom_msg_cache of size 0 detected");
    }
    odom_msg.twist.twist.linear = avg_vel;
    odom_msg.twist.twist.angular = avg_avel;
    
    robot_odom_pub.publish(odom_msg);


/*
    point_msg.header.stamp = ros::Time::now();
    point_msg.point.x = odom_msg.pose.pose.position.x;
    point_msg.point.y = odom_msg.pose.pose.position.y;
    point_msg.point.z = odom_msg.pose.pose.position.z;

    robot_point_pub.publish(point_msg);
*/
  }
  return 0;
};
