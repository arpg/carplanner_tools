#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <carplanner_tools/joy_waypoint_generator.h>

#include <map>
#include <string>


namespace joy_waypoint_generator
{

/**
 * Internal members of class. This is the pimpl idiom, and allows more flexibility in adding
 * parameters later without breaking ABI compatibility, for robots which link TeleopTwistJoy
 * directly into base nodes.
 */
struct JoyWaypointGenerator::Impl
{
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  void sendWaypointsMsg(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::string& which_map);

  ros::Subscriber joy_sub;
  ros::Publisher waypoints_pub;

  int add_waypoint_button;
  int remove_waypoint_button;
  int increase_speed_waypoint_button;
  int decrease_speed_waypoint_button;

  double speed_delta;
  double nearby_waypoint_radius;

  int enable_button;
  int enable_turbo_button;

  std::map<std::string, int> axis_map;
  std::map< std::string, std::map<std::string, double> > scale_map;

  bool sent_disable_msg;
};

/**
 * Constructs JoyWaypointGenerator.
 * \param nh NodeHandle to use for setting up the publisher and subscriber.
 * \param nh_param NodeHandle to use for searching for configuration parameters.
 */
JoyWaypointGenerator::JoyWaypointGenerator(ros::NodeHandle* nh, ros::NodeHandle* nh_param)
{
  pimpl_ = new Impl;

  pimpl_->joy_sub = nh->subscribe<sensor_msgs::Joy>("joy", 1, &JoyWaypointGenerator::Impl::joyCallback, pimpl_);
  pimpl_->waypoints_pub = nh->advertise<carplanner_msgs::WayPoints>("waypoints", 1, true);

  nh_param->param<int>("add_waypoint_button", pimpl_->add_waypoint_button, -1);
  nh_param->param<int>("remove_waypoint_button", pimpl_->remove_waypoint_button, -1);
  nh_param->param<int>("increase_speed_waypoint_button", pimpl_->increase_speed_waypoint_button, -1);
  nh_param->param<int>("decrease_speed_waypoint_button", pimpl_->decrease_speed_waypoint_button, -1);

  nh_param->param<double>("speed_delta", pimpl_->speed_delta, 0.0);
  nh_param->param<double>("nearby_waypoint_radius", pimpl_->nearby_waypoint_radius, 0.0);

  nh_param->param<int>("enable_button", pimpl_->enable_button, 0);
  nh_param->param<int>("enable_turbo_button", pimpl_->enable_turbo_button, -1);

  if (nh_param->getParam("axis_control_value", pimpl_->axis_map["control_value"])) {
    nh_param->getParam("scale_control_value", pimpl_->scale_map["normal"]["control_value"]);
    nh_param->getParam("scale_control_value_turbo", pimpl_->scale_map["turbo"]["control_value"]);
  }
  else {
    nh_param->param<int>("axis_control_value", pimpl_->axis_map["control_value"], 1);
    nh_param->param<double>("scale_control_value", pimpl_->scale_map["normal"]["control_value"], 0.5);
    nh_param->param<double>("scale_control_value_turbo", pimpl_->scale_map["turbo"]["control_value"], 1.0);
  }

  if (nh_param->getParam("axis_steer", pimpl_->axis_map["steer"])) {
    nh_param->getParam("scale_steer", pimpl_->scale_map["normal"]["steer"]);
    nh_param->getParam("scale_steer_turbo", pimpl_->scale_map["turbo"]["steer"]);
  }
  else {
    nh_param->param<int>("axis_steer", pimpl_->axis_map["steer"], 1);
    nh_param->param<double>("scale_steer", pimpl_->scale_map["normal"]["steer"], 0.5);
    nh_param->param<double>("scale_steer_turbo", pimpl_->scale_map["turbo"]["steer"], 1.0);
  }

  ROS_INFO_NAMED("JoyWaypointGenerator", "Teleop enable button %i.", pimpl_->enable_button);
  ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "JoyWaypointGenerator",
      "Turbo on button %i.", pimpl_->enable_turbo_button);

  for (std::map<std::string, int>::iterator it = pimpl_->axis_map.begin();
      it != pimpl_->axis_map.end(); ++it) {
    ROS_INFO_NAMED("JoyWaypointGenerator", "Axis %s on %i at scale %f.",
    it->first.c_str(), it->second, pimpl_->scale_map["normal"][it->first]);
    ROS_INFO_COND_NAMED(pimpl_->enable_turbo_button >= 0, "JoyWaypointGenerator",
        "Turbo for axis %s is scale %f.", it->first.c_str(), pimpl_->scale_map["turbo"][it->first]);
  }

  pimpl_->sent_disable_msg = false;
}

double getVal(const sensor_msgs::Joy::ConstPtr& joy_msg, const std::map<std::string, int>& axis_map,
              const std::map<std::string, double>& scale_map, const std::string& fieldname)
{
  if (axis_map.find(fieldname) == axis_map.end() ||
      scale_map.find(fieldname) == scale_map.end() ||
      joy_msg->axes.size() <= axis_map.at(fieldname)) {
    return 0.0;
  }

  return joy_msg->axes[axis_map.at(fieldname)] * scale_map.at(fieldname);
}

void JoyWaypointGenerator::Impl::sendDisableMsg(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  // Initializes with zeros by default.
  drive_control::DriveCommand drive_msg;

  drive_pub.publish(drive_msg);
  sent_disable_msg = true;
}

void JoyWaypointGenerator::Impl::sendDriveMsg(const sensor_msgs::Joy::ConstPtr& joy_msg,
                                         const std::string& which_map)
{
  // Initializes with zeros by default.
  drive_control::DriveCommand drive_msg;

  drive_msg.control_value = getVal(joy_msg, axis_map, scale_map[which_map], "control_value");
  drive_msg.front_steer_angle = getVal(joy_msg, axis_map, scale_map[which_map], "steer");
  drive_msg.rear_steer_angle = 0.0;

  drive_pub.publish(drive_msg);
  sent_disable_msg = false;
}

void JoyWaypointGenerator::Impl::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  if (joy_msg->buttons.size() > enable_button &&
      joy_msg->buttons[enable_button] &&
      enable_turbo_button >= 0 &&
      joy_msg->buttons.size() > enable_turbo_button &&
      joy_msg->buttons[enable_turbo_button])
  {
    sendDriveMsg(joy_msg, "turbo");
  }
  else if (joy_msg->buttons.size() > enable_button &&
           joy_msg->buttons[enable_button])
  {
    sendDriveMsg(joy_msg, "normal");
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

}  // namespace joy_waypoint_generator
