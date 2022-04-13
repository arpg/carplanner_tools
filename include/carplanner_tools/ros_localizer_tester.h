#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>

class Tester
{
public:
  Tester(ros::NodeHandle& nh_, ros::NodeHandle& pnh_);
  ~Tester();

private:
  void loopFunc(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh, pnh;
  ros::Timer loop;
  std::string parent_frame_id, child_frame_id;
  float rate;
  uint iteration;
  tf::TransformBroadcaster tfcaster;
};



