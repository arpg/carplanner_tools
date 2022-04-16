#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <tf/transform_listener.h>

typedef Eigen::Matrix<double, 3, 1> Vec3;
typedef Eigen::Matrix<double, 1, 3> RVec3;
typedef Eigen::Matrix<double, 4, 1> Vec4;
typedef Eigen::Matrix<double, 1, 4> RVec4;
typedef Eigen::Matrix<double, 6, 1> Vec6;
typedef Eigen::Matrix<double, 7, 1> Vec7;
typedef Eigen::Quaternion<double> Quat;
typedef Eigen::Transform<double, 3, Eigen::TransformTraits::Affine> Tform;

std::string odom_in_topic, odom_out_topic, frame_id, child_frame_id;
ros::Publisher odom_pub;
tf2_ros::Buffer tfbuffer;

bool lookup_transform(Tform& tform_out, std::string current_frame, std::string new_frame, ros::Time stamp=ros::Time(0))
{
  geometry_msgs::TransformStamped gtform;
  try
  {
    gtform = tfbuffer.lookupTransform(new_frame, current_frame, stamp);
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR_THROTTLE(1.0,"%s",ex.what());
    return false;
  }

  tform_out = Tform::Identity();
  tform_out.translate(Vec3(gtform.transform.translation.x, gtform.transform.translation.y, gtform.transform.translation.z));
  tform_out.rotate(Quat(gtform.transform.rotation.w, gtform.transform.rotation.x, gtform.transform.rotation.y, gtform.transform.rotation.z));
}

bool transform(Quat& in, std::string current_frame, std::string new_frame, ros::Time stamp=ros::Time(0))
{
  Tform tform;
  if (!lookup_transform(tform, current_frame, new_frame, stamp))
    return false;

  in = tform.rotation() * in;

  return true;
}

bool pretransform(Vec3& in, std::string current_frame, std::string new_frame, ros::Time stamp=ros::Time(0))
{
  Tform tform;
  if (!lookup_transform(tform, current_frame, new_frame, stamp))
    return false;

  in = tform * in;

  return true;
}

// bool posttransform(Vec3& in, std::string current_frame, std::string new_frame, ros::Time stamp=ros::Time(0))
// {
//   Tform tform;
//   if (!lookup_transform(tform, current_frame, new_frame, stamp))
//     return false;

//   in = in * tform;

//   return true;
// }

bool pretransform(Tform& in, std::string current_frame, std::string new_frame, ros::Time stamp=ros::Time(0))
{
  Tform tform;
  if (!lookup_transform(tform, current_frame, new_frame, stamp))
    return false;

  in = tform * in;

  return true;
}

bool posttransform(Tform& in, std::string current_frame, std::string new_frame, ros::Time stamp=ros::Time(0))
{
  Tform tform;
  if (!lookup_transform(tform, current_frame, new_frame, stamp))
    return false;

  in = in * tform;

  return true;
}

void times(Eigen::Matrix<double,3,1>& out, Eigen::Transform<double,3,Eigen::TransformTraits::Affine>& in)
{
  Eigen::Matrix<double,4,1> this_temp;
  this_temp.head(3) = out;
  this_temp[3] = (double)1.f;
  Eigen::Matrix<double,1,4> out_temp;
  out_temp = this_temp.transpose() * in.matrix();
  out = out_temp.head(3);
}

void odom_cb(const nav_msgs::Odometry::ConstPtr input_msg)
{
  ROS_INFO("Got odom (%s->%s): p: %f %f %f, q: %f %f %f %f, v: %f %f %f",
    input_msg->header.frame_id.c_str(),
    input_msg->child_frame_id.c_str(),
    input_msg->pose.pose.position.x,
    input_msg->pose.pose.position.y,
    input_msg->pose.pose.position.z, 
    input_msg->pose.pose.orientation.x,
    input_msg->pose.pose.orientation.y,
    input_msg->pose.pose.orientation.z,
    input_msg->pose.pose.orientation.w,
    input_msg->twist.twist.linear.x,
    input_msg->twist.twist.linear.y,
    input_msg->twist.twist.linear.z
  );

  Vec3 pose_vec(input_msg->pose.pose.position.x, input_msg->pose.pose.position.y, input_msg->pose.pose.position.z);
  Quat quat_vec(input_msg->pose.pose.orientation.w, input_msg->pose.pose.orientation.x, input_msg->pose.pose.orientation.y, input_msg->pose.pose.orientation.z);
  Vec3 linvel_vec(input_msg->twist.twist.linear.x, input_msg->twist.twist.linear.y, input_msg->twist.twist.linear.z);
  Vec3 angvel_vec(input_msg->twist.twist.angular.x, input_msg->twist.twist.angular.y, input_msg->twist.twist.angular.z);

  Tform tform = Tform::Identity();
  tform.translate(pose_vec);
  tform.rotate(quat_vec);

  // Vec4 linvel_temp;
  // linvel_temp.head(3) = linvel_vec;
  // linvel_temp[3] = (double)1.f;
  // RVec4 linvel_temp2;
  // linvel_temp2 = linvel_temp.transpose() * tform.matrix();
  // linvel_vec = linvel_temp2.head(3);
  times(linvel_vec, tform);
  times(angvel_vec, tform);

  // if (!transform(pose_vec, input_msg->header.frame_id, frame_id, input_msg->header.stamp))
  //   return;
  // if (!transform(quat_vec, input_msg->header.frame_id, frame_id, input_msg->header.stamp))
  //   return;
  // if (!pretransform(tform, input_msg->header.frame_id, frame_id, input_msg->header.stamp))
  //   return;
  // if (!posttransform(tform, child_frame_id, input_msg->child_frame_id, input_msg->header.stamp))
  //   return;
  // if (!transform_vec(pose_vec, input_msg->child_frame_id, child_frame_id, input_msg->header.stamp))
  //   return;

  // if (!transform_vec(linvel_vec, input_msg->header.frame_id, frame_id, input_msg->header.stamp))
  //   return;

  // Tform Twv;
  // if (!lookup_transform(Twv, current_frame, new_frame, stamp))
  //   return false;

  pose_vec = tform.translation();
  quat_vec = tform.rotation();
  // linvel_vec = 

  ROS_INFO("New odom (%s->%s): p: %f %f %f, q: %f %f %f %f, v: %f %f %f",
    input_msg->header.frame_id.c_str(),
    input_msg->child_frame_id.c_str(),
    pose_vec[0],
    pose_vec[1],
    pose_vec[2],
    quat_vec.w(),
    quat_vec.x(),
    quat_vec.y(),
    quat_vec.z(),
    linvel_vec[0],
    linvel_vec[1],
    linvel_vec[2]
  );

  nav_msgs::Odometry output_msg = *input_msg;
  // output_msg.header.stamp = input_msg->header.stamp;
  // output_msg.header.frame_id = frame_id;
  // output_msg.child_frame_id = child_frame_id;
  // output_msg.pose.pose.position.x = pose_vec[0];
  // output_msg.pose.pose.position.y = pose_vec[1];
  // output_msg.pose.pose.position.z = pose_vec[2];
  // output_msg.pose.pose.orientation.w = quat_vec.w();
  // output_msg.pose.pose.orientation.x = quat_vec.x();
  // output_msg.pose.pose.orientation.y = quat_vec.y();
  // output_msg.pose.pose.orientation.z = quat_vec.z();
  output_msg.twist.twist.linear.x = linvel_vec[0];
  output_msg.twist.twist.linear.y = linvel_vec[1];
  output_msg.twist.twist.linear.z = linvel_vec[2];
  output_msg.twist.twist.angular.x = angvel_vec[0];
  output_msg.twist.twist.angular.y = angvel_vec[1];
  output_msg.twist.twist.angular.z = angvel_vec[2];
  odom_pub.publish(output_msg);
}

int main( int argc, char* argv[] )
{
    ros::init(argc, argv, "odom_reframe");

    ros::NodeHandle pnh("~");

    pnh.param("odom_in_topic", odom_in_topic, std::string("odom"));
    pnh.param("odom_out_topic", odom_out_topic, std::string("new_odom"));
    pnh.param("frame_id", frame_id, std::string("world"));
    pnh.param("child_frame_id", child_frame_id, std::string("base_link"));

    ros::Subscriber odom_sub = pnh.subscribe<nav_msgs::Odometry>(odom_in_topic, 5, odom_cb);
    odom_pub = pnh.advertise<nav_msgs::Odometry>(odom_out_topic, 5);

    tf2_ros::TransformListener tflistener(tfbuffer);

    ROS_INFO("Initialized.");

    ros::spin();
}
