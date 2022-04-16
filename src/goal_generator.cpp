#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

std::string goal_topic, path_topic, odom_topic;
ros::Publisher goal_pub;
double path_intercept_length;
float rate;
nav_msgs::Odometry odom_msg;
nav_msgs::Path path_msg;
geometry_msgs::PoseStamped goal_msg;

void odom_cb(const nav_msgs::Odometry::ConstPtr msg)
{
    odom_msg = *msg;
    ROS_INFO("Got odom x:%f y:%f z:%f",odom_msg.pose.pose.position.x,odom_msg.pose.pose.position.y,odom_msg.pose.pose.position.z);
}

void path_cb(const nav_msgs::Path::ConstPtr msg)
{
    path_msg = *msg;
    ROS_INFO("Got path");
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "goal_generator");

    ros::NodeHandle nh, pnh("~");

    pnh.param("goal_topic", goal_topic, std::string("goal"));
    pnh.param("path_topic", path_topic, std::string("path"));
    pnh.param("odom_topic", odom_topic, std::string("odom"));
    pnh.param("path_intercept_length", path_intercept_length, 2.0);
    pnh.param("rate", rate, (float)2.0);
    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>(path_topic, 5, path_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic, 5, odom_cb);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>(goal_topic, 5);

    ROS_INFO("Initialized.");

    while(ros::ok())
    {
      // if (goal_msg.header.seq>0)
      // {
      //   ROS_INFO("Publishing goal x:%f y:%f z:%f.", goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z);
      //   goal_pub.publish(goal_msg);
      // }

      ros::spinOnce();
      ros::Rate(rate).sleep();

      // if we are not ready to calculate the goal, skip
      static uint last_path_seq=0;
      if (path_msg.header.seq<=0 || path_msg.poses.size()<2 || path_msg.header.seq==last_path_seq)
      {
        ROS_WARN_THROTTLE(5, "Haven't gotten path yet");
        continue;
      }
      static uint last_odom_seq=0;
      if (odom_msg.header.seq<=0 || odom_msg.header.seq==last_odom_seq)
      {
        ROS_WARN_THROTTLE(5, "Haven't gotten odom yet");
        continue;
      }

      // if we have already calculated the goal for this path, do not calc it again, just publish
      // bool calc_goal=true;
      // static uint last_path_seq=0;
      // if (last_path_seq==path_msg.header.seq)
      // {
      //   // we've already calculated the goal for this path
      //   calc_goal=false;
      //   // continue;
      // }
      // last_path_seq = path_msg.header.seq;

      // if (calc_goal)
      // {
        Eigen::Vector3d delta;
        uint start_idx = 0;
        float shortest_dist = FLT_MAX;
        for (uint i=0; i<path_msg.poses.size(); i++)
        {
            delta = Eigen::Vector3d(odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z) 
                    - Eigen::Vector3d(path_msg.poses[i].pose.position.x, path_msg.poses[i].pose.position.y, path_msg.poses[i].pose.position.z);
            
            // ROS_INFO("Path pose #%d/%d is %fm away.", i, path_msg.poses.size(), delta.norm());

            if (delta.norm() < shortest_dist)
            {
                shortest_dist = delta.norm();
                start_idx = i;
            }
        }
        // ROS_INFO("Closest point on path is pose #%d/%d x:%.1f y:%.1f z:%.1f at %fm away.",start_idx,path_msg.poses[start_idx].pose.position.x,path_msg.poses.size(),path_msg.poses[start_idx].pose.position.y,path_msg.poses[start_idx].pose.position.z,shortest_dist);
        double sum=0, new_sum=0;
        goal_msg.header.seq = -1;
        for (uint i=start_idx; i<path_msg.poses.size()-1; i++)
        {
            delta = Eigen::Vector3d(path_msg.poses[i+1].pose.position.x, path_msg.poses[i+1].pose.position.y, path_msg.poses[i+1].pose.position.z) 
                    - Eigen::Vector3d(path_msg.poses[i].pose.position.x, path_msg.poses[i].pose.position.y, path_msg.poses[i].pose.position.z);
            new_sum = sum + delta.norm();
            // ROS_INFO("Path len at pose #%d/%d is %fm long.", i+1, path_msg.poses.size(),new_sum);

            if (new_sum > path_intercept_length)
            {
              // ROS_INFO("Reached path intercept length at pose #%d/%d. Setting goal.",i,path_msg.poses.size());
              double delta_fraction = (path_intercept_length - sum)/delta.norm();
              delta *= delta_fraction;

              goal_msg.header = path_msg.poses[i].header;
              goal_msg.pose.position.x = path_msg.poses[i].pose.position.x + delta[0];
              goal_msg.pose.position.y = path_msg.poses[i].pose.position.y + delta[1];
              goal_msg.pose.position.z = path_msg.poses[i].pose.position.z + delta[2];
              goal_msg.pose.orientation.x = path_msg.poses[i+1].pose.orientation.x;
              goal_msg.pose.orientation.y = path_msg.poses[i+1].pose.orientation.y;
              goal_msg.pose.orientation.z = path_msg.poses[i+1].pose.orientation.z;
              goal_msg.pose.orientation.w = path_msg.poses[i+1].pose.orientation.w;
              break;
            }
            else
            {
              sum = new_sum;
            }
        }
        if (goal_msg.header.seq==-1)
        {
          // plan shorter than path intercept length, set goal to end of the path
          // ROS_INFO("Plan shorter than path intercept length, setting goal to end of path.");
          goal_msg.header = path_msg.header;
          goal_msg.pose.position.x = path_msg.poses[path_msg.poses.size()-1].pose.position.x + delta[0];
          goal_msg.pose.position.y = path_msg.poses[path_msg.poses.size()-1].pose.position.y + delta[1];
          goal_msg.pose.position.z = path_msg.poses[path_msg.poses.size()-1].pose.position.z + delta[2];
          goal_msg.pose.orientation.x = path_msg.poses[path_msg.poses.size()-1].pose.orientation.x;
          goal_msg.pose.orientation.y = path_msg.poses[path_msg.poses.size()-1].pose.orientation.y;
          goal_msg.pose.orientation.z = path_msg.poses[path_msg.poses.size()-1].pose.orientation.z;
          goal_msg.pose.orientation.w = path_msg.poses[path_msg.poses.size()-1].pose.orientation.w;
        }
      // } // if calc_goal

      // ROS_INFO("Publishing goal x:%f y:%f z:%f.", goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z);
      goal_pub.publish(goal_msg);
    }
}
