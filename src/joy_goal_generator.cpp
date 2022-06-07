#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nav_msgs/Odometry.h>
#include <carplanner_tools/joy_goal_generator.h>

#include <map>
#include <string>


namespace joy_goal_generator
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct JoyGoalGenerator::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendGoalMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);
  void sendDisableMsg(const sensor_msgs::Joy::ConstPtr& joy_msg);

  ros::Subscriber joy_sub;
  ros::Publisher goal_pub;

  int enable_button;
  int enable_turbo_button;

  std::map<std::string, int> axis_map;
  std::map< std::string, std::map<std::string, double> > scale_map;
  std::map< std::string, std::map<std::string, double> > offset_map;

  bool sent_disable_msg;

  std::string parent_frame_id;
};

/**
 * Constructs JoyGoalGenerator.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
JoyGoalGenerator::JoyGoalGenerator(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->goal_pub = nh->advertise<nav_msgs::Odometry>("goal", 1, true);
  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &JoyGoalGenerator::Impl::joyCallback, pimpl_);

  nh_param->param<std::string>("parent_frame_id", pimpl_->parent_frame_id, "map");

  nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

  if (nh_param->getParam("axis_length", pimpl_->axis_map["length"]))
  {
    nh_param->getParam("scale_length", pimpl_->scale_map["normal"]["length"]);
    nh_param->getParam("offset_length", pimpl_->offset_map["normal"]["length"]);
    nh_param->getParam("scale_length_turbo", pimpl_->scale_map["turbo"]["length"]);
    nh_param->getParam("offset_length_turbo", pimpl_->offset_map["turbo"]["length"]);
  }
  else
  {
    nh_param->param<int>("axis_length", pimpl_->axis_map["length"], 1);
    nh_param->param<double>("scale_length", pimpl_->scale_map["normal"]["length"], 1.0);
    nh_param->param<double>("offset_length", pimpl_->offset_map["normal"]["length"], 0.0);
    nh_param->param<double>("scale_length_turbo", pimpl_->scale_map["turbo"]["length"], 1.0);
    nh_param->param<double>("offset_length_turbo", pimpl_->offset_map["turbo"]["length"], 1.0);
  }

  if (nh_param->getParam("axis_angle", pimpl_->axis_map["angle"]))
  {
    nh_param->getParam("scale_angle", pimpl_->scale_map["normal"]["angle"]);
    nh_param->getParam("offset_angle", pimpl_->offset_map["normal"]["angle"]);
    nh_param->getParam("scale_angle_turbo", pimpl_->scale_map["turbo"]["angle"]);
    nh_param->getParam("offset_angle_turbo", pimpl_->offset_map["turbo"]["angle"]);
  }
  else
  {
    nh_param->param<int>("axis_angle", pimpl_->axis_map["angle"], 0);
    nh_param->param<double>("scale_angle", pimpl_->scale_map["normal"]["angle"], 0.7854);
    nh_param->param<double>("offset_angle", pimpl_->offset_map["normal"]["angle"], 0.0);
    nh_param->param<double>("scale_angle_turbo", pimpl_->scale_map["turbo"]["angle"], 1.5708);
    nh_param->param<double>("offset_angle_turbo", pimpl_->offset_map["turbo"]["angle"], 0.0);
  }

  ROS_INFO_NAMED("JoyGoalGenerator", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "JoyGoalGenerator",
      "Turbo on button %i.", pimpl_->enable_turbo_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_map.begin();
      it != pimpl_->axis_map.end(); ++it)
  {
    ROS_INFO_NAMED("JoyGoalGenerator", "Axis %s on %i at scale %f with offset %f.",
    it->first.c_str(), it->second, pimpl_->scale_map["normal"][it->first], pimpl_->offset_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "JoyGoalGenerator",
        "Turbo for axis %s is scale %f with offset %f.", it->first.c_str(), pimpl_->scale_map["turbo"][it->first], pimpl_->offset_map["turbo"][it->first]);
  }

  pimpl_->sent_disable_msg = false;
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::map<std::string, double>& offset_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      offset_map.find(fieldname) == offset_map.end() || 
      joy_msg->axes.size() <= axis_map.at(fieldname))
  {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname) + offset_map.at(fieldname);
}

void JoyGoalGenerator::Impl::sendDisableMsg(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Initializes with zeros by default.
  nav_msgs::Odometry goal_msg;

  goal_msg.header.stamp = joy_msg->header.stamp;
  goal_msg.header.frame_id = parent_frame_id;
  goal_msg.pose.pose.orientation.w = 1.0;

  goal_pub.publish(goal_msg);
  sent_disable_msg = true;
}

void JoyGoalGenerator::Impl::sendGoalMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  double length = getVal(joy_msg, axis_map, scale_map[which_map], offset_map[which_map], "length");
  double angle = getVal(joy_msg, axis_map, scale_map[which_map], offset_map[which_map], "angle"); // 1.0 -> 0.0 -> -1.0 = 45d -> 0d -> -45d

  ROS_INFO("Sending goal msg. Got length %f angle %f.", length, angle);

  double yaw = angle;
  double x, y;
  if (yaw == 0.f)
  {
    x = length;
    y = 0.f;
  }
  else if (length == 0.f)
  {
    x = 0.f;
    y = 0.f;
    yaw = 0.f;
  }
  else
  {
    double r2 = abs(length/yaw);
    double r1 = abs(r2*tan(yaw));
    // double r1 = abs(2*r2*sin(yaw/2.f));
    // x = sqrt(pow(r2,2) - pow(r1,4)/4.f/pow(r2,2));
    x = sqrt(pow(r1,2) - pow(r1,4)/4.f/pow(r2,2));
    y = sgn(yaw)*(r2 - sqrt(pow(r2,2) - pow(x,2)));
    ROS_INFO("Calculated r2 %f r1 %f x %f y %f yaw %f.", r2, r1, x, y, yaw);
    // double l = sqrt(pow(x,2)+pow(y,2)); 
 }

  // Initializes with zeros by default.
  nav_msgs::Odometry goal_msg;

  goal_msg.header.stamp = joy_msg->header.stamp;
  goal_msg.header.frame_id = parent_frame_id;
  // goal_msg.child_frame_id = child_frame_id;
  goal_msg.pose.pose.position.x    = x; 
  goal_msg.pose.pose.position.y    = y; 
  // goal_msg.pose.pose.position.z    = 0.0; 
  // goal_msg.pose.pose.orientation.x = 0.0;
  // goal_msg.pose.pose.orientation.y = 0.0;
  goal_msg.pose.pose.orientation.z = sin(yaw/2);
  goal_msg.pose.pose.orientation.w = cos(yaw/2); // sqrt(1.f - pow(goal_msg.pose.pose.orientation.x,2) - pow(goal_msg.pose.pose.orientation.y,2) - pow(goal_msg.pose.pose.orientation.z,2));
  // goal_msg.pose.covariance = 
  goal_msg.twist.twist.linear.x = 1.0;
  // goal_msg.twist.twist.angular.x =
  // goal_msg.twist.covariance = 

  goal_pub.publish(goal_msg);
  sent_disable_msg = false;
}

void JoyGoalGenerator::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (joy_msg->buttons.size() > enable_button &&
      joy_msg->buttons[enable_button]&&
      enable_turbo_button >= 0 &&
      joy_msg->buttons.size() > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button])
  {
    sendGoalMsg(joy_msg, "turbo");
  }
  else if (joy_msg->buttons.size() > enable_button &&
           joy_msg->buttons[enable_button])
  {
    sendGoalMsg(joy_msg, "normal");
  }
  else
  {
    // When enable button is released, immediately send a single no-motion command
    // in order to stop the robot.
    if (!sent_disable_msg)
    {
      sendDisableMsg(joy_msg);
    }
  }
}

}  // namespace joy_goal_generator
