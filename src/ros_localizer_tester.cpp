#include <carplanner_tools/ros_localizer_tester.h>

Tester::Tester(ros::NodeHandle& nh_, ros::NodeHandle& pnh_) : nh(nh_), pnh(pnh_) {
  pnh.param("parent_frame_id", parent_frame_id, std::string("map"));
  pnh.param("child_frame_id", child_frame_id, std::string("base_link"));
  pnh.param("rate", rate, (float)100.);

  loop = nh.createTimer(ros::Duration(1./rate), &Tester::loopFunc, this);

  ROS_INFO("Initialized.");
};

Tester::~Tester(){
  ROS_INFO("Destroyed.");
};
  
void Tester::loopFunc(const ros::TimerEvent& event)
{
    float offset[3] = {0, 0, 0.1};
    float radius = 1.;
    float speed = 1.;
    float dist_per_step = speed/rate;
    float total_dist = 2*M_PI*radius;
    uint total_steps = round(total_dist/dist_per_step);
    float s = (float)iteration++/total_steps; // 0-1 fraction of trajectory
    iteration = iteration % total_steps; // cap to less than total_steps

    geometry_msgs::TransformStamped T;
    T.header.stamp = ros::Time::now();
    T.header.frame_id = parent_frame_id;
    T.child_frame_id = child_frame_id;
    T.transform.translation.x = radius*cos(2*M_PI*s)+offset[0];
    T.transform.translation.y = radius*sin(2*M_PI*s)+offset[1];
    T.transform.translation.z = offset[2];
    T.transform.rotation.x = 0.f;
    T.transform.rotation.y = 0.f;
    T.transform.rotation.z = sin(M_PI*s+0.785);
    T.transform.rotation.w = cos(M_PI*s+0.785);

    tfcaster.sendTransform(T);

    ROS_INFO("Sent transform: %s->%s %f %f %f %f %f %f %f", parent_frame_id.c_str(), child_frame_id.c_str(), T.transform.translation.x, T.transform.translation.y, T.transform.translation.z, T.transform.rotation.x, T.transform.rotation.y, T.transform.rotation.z, T.transform.rotation.w);
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "ros_localizer_tester");

    ros::NodeHandle nh, pnh("~");

    Tester tester(nh,pnh);

    ros::spin();
}



