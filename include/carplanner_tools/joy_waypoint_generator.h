#ifndef TELEOP_DRIVE_JOY_TELEOP_DRIVE_JOY_H
#define TELEOP_DRIVE_JOY_TELEOP_DRIVE_JOY_H

namespace ros { class NodeHandle; }

namespace teleop_drive_joy
{

/**
 * Class implementing a basic Joy -> Drive translation.
 */
class TeleopDriveJoy
{
public:
  TeleopDriveJoy(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

private:
  struct Impl;
  Impl* pimpl_;
};

}  // namespace teleop_drive_joy

#endif
