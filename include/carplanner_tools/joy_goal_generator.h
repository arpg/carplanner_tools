#ifndef JOY_GOAL_GENERATOR_H
#define JOY_GOAL_GENERATOR_H

namespace ros { class NodeHandle; }

namespace joy_goal_generator
{

template <typename T> static int sgn(T val) { return (T(0)<val)-(val<T(0)); }

/**
 * Class implementing a basic Joy -> Goal translation.
 */
class JoyGoalGenerator
{
public:
  JoyGoalGenerator(ros::NodeHandle* nh, ros::NodeHandle* nh_param);

private:
  struct Impl;
  Impl* pimpl_;
};

}  // namespace joy_goal_generator

#endif
