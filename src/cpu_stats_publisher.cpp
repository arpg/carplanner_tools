#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <carplanner_tools/cpu_stats.h>

ros::Timer loop;
ros::Publisher total_cpu_usage_pub;

void loopFunc(const ros::TimerEvent& event)
{
    double total_cpu_usage = sys_utils::getTotalCpuUsage();
    std_msgs::Float64 total_cpu_usage_msg;
    total_cpu_usage_msg.data = total_cpu_usage;
    total_cpu_usage_pub.publish(total_cpu_usage_msg);
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "cpu_stats_publisher");

    ros::NodeHandle nh, pnh("~");

    float rate;
    pnh.param("rate", rate, (float)10.);

    sys_utils::init();

    total_cpu_usage_pub = pnh.advertise<std_msgs::Float64>("total_cpu_usage", 5);
    loop = nh.createTimer(ros::Duration(1./rate), &loopFunc);

    ROS_INFO("Initialized.");

    ros::spin();
}



