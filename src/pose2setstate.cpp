#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <carplanner_msgs/VehicleState.h>
#include <carplanner_msgs/SetStateAction.h>

int num_worlds;
float server_timeout;
std::string pose_topic, set_state_service;
actionlib::SimpleActionClient<carplanner_msgs::SetStateAction>* client;

void pose_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
{
  ROS_INFO("Got pose: %f %f %f %f %f %f %f",
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z, 
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );

  if (!client)
  {
    ROS_ERROR("Client not set. Aborting.");
    return;
  }

  for (uint i=0; i<num_worlds; i++)
  {
    carplanner_msgs::SetStateGoal goal;

    goal.world_id = i; // int8

    // goal.state_in.header = ; // std_msgs/Header
    goal.state_in.pose.transform.translation.x = msg->pose.pose.position.x; // geometry_msgs/TransformStamped
    goal.state_in.pose.transform.translation.y = msg->pose.pose.position.y;
    goal.state_in.pose.transform.translation.z = msg->pose.pose.position.z;
    goal.state_in.pose.transform.rotation.x = msg->pose.pose.orientation.x;
    goal.state_in.pose.transform.rotation.y = msg->pose.pose.orientation.y;
    goal.state_in.pose.transform.rotation.z = msg->pose.pose.orientation.z;
    goal.state_in.pose.transform.rotation.w = msg->pose.pose.orientation.w;
    // goal.state_in.wheel_poses // geometry_msgs/TransformStamped[]
    // goal.state_in.wheel_contacts // bool[]
    goal.state_in.chassis_collision = false; // bool
    goal.state_in.lin_vel.x = 0.f; // geometry_msgs/Vector3
    goal.state_in.lin_vel.y = 0.f;
    goal.state_in.lin_vel.z = 0.f;
    goal.state_in.ang_vel.x = 0.f; // geometry_msgs/Vector3
    goal.state_in.ang_vel.y = 0.f;
    goal.state_in.ang_vel.z = 0.f;
    goal.state_in.curvature = 0.f; // float64
    goal.state_in.steering = 0.f; // float64

    goal.raycast = true; // bool

    client->sendGoal(goal);

    // VehicleState stateOut;
    carplanner_msgs::SetStateResultConstPtr result;
    bool finished_before_timeout = client->waitForResult(ros::Duration(server_timeout));
    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = client->getState();
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
          result = client->getResult();
          carplanner_msgs::VehicleState state_out_msg = result->state_out;
          ROS_INFO("Set state to: %f %f %f %f %f %f %f", 
            state_out_msg.pose.transform.translation.x, 
            state_out_msg.pose.transform.translation.y, 
            state_out_msg.pose.transform.translation.z, 
            state_out_msg.pose.transform.rotation.x, 
            state_out_msg.pose.transform.rotation.y, 
            state_out_msg.pose.transform.rotation.z, 
            state_out_msg.pose.transform.rotation.w);
          // stateOut.fromROS(state_out_msg);
      }
      else
      {
          ROS_ERROR("SetState action failed.");
      }
    }
    else
      ROS_ERROR("SetState action did not finish before the time out.");
  }
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "pose2setstate");

    ros::NodeHandle nh, pnh("~");

    pnh.param("num_worlds", num_worlds, 10);
    pnh.param("server_timeout", server_timeout, 1.f);
    pnh.param("pose_topic", pose_topic, std::string("pose"));
    pnh.param("set_state_service", set_state_service, std::string("set_state"));

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(pose_topic, 5, pose_cb);
    client = new actionlib::SimpleActionClient<carplanner_msgs::SetStateAction>(set_state_service, true);

    ROS_INFO("Initialized.");

    ros::spin();

    delete client;
}
