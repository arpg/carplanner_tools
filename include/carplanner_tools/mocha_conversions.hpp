#ifndef _MOCHA_CONVERSIONS
#define	_MOCHA_CONVERSIONS

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <carplanner_msgs/Command.h>
#include <carplanner_msgs/VehicleState.h>
#include <carplanner_msgs/PathArray.h>

// #include <CarPlanner/BulletCarModel.h>
// #include <CarPlanner/ApplyVelocitiesFunctor.h>

namespace carplanner_msgs{
struct MarkerArrayConfig
{
    std::string ns;
    float scale_x;
    float scale_y;
    float scale_z;
    float color_r;
    float color_g;
    float color_b;
    float color_a;

    MarkerArrayConfig() :
        ns(""), scale_x(1.0), scale_y(1.0), scale_z(1.0), color_r(1.0), color_g(1.0), color_b(1.0), color_a(1.0) {}

    MarkerArrayConfig(std::string _ns, float _sx, float _sy, float _sz, float _cr, float _cg, float _cb, float _ca) :
        ns(_ns), scale_x(_sx), scale_y(_sy), scale_z(_sz), color_r(_cr), color_g(_cg), color_b(_cb), color_a(_ca) {}
};
} // namespace carplanner_msgs

inline void convertPathArrayMsg2PathMsg(carplanner_msgs::PathArray& patharr_msg, nav_msgs::Path* path_msg_out)
{
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = patharr_msg.header.frame_id;
    path_msg.header.stamp = patharr_msg.header.stamp;

    if (patharr_msg.paths.empty())
    {
        ROS_ERROR("%s: Empty path array.",__FUNCTION__);
        (*path_msg_out) = path_msg;
        return;
    }
    
    for (uint iPath=0; iPath<patharr_msg.paths.size(); iPath++)
    {
        for (uint iPose=0; iPose<patharr_msg.paths[iPath].poses.size(); iPose++)
        {
            path_msg.poses.push_back(patharr_msg.paths[iPath].poses[iPose]);
        }
    }

    (*path_msg_out) = path_msg;
}

inline void convertPathArrayMsg2LineStripArrayMsg(carplanner_msgs::PathArray& patharr_msg, visualization_msgs::MarkerArray* markarr_msg_out, const carplanner_msgs::MarkerArrayConfig& config = carplanner_msgs::MarkerArrayConfig(), const Eigen::Vector3d offset = Eigen::Vector3d::Zero())
{
    visualization_msgs::MarkerArray markarr_msg;

    if (patharr_msg.paths.empty())
    {
        ROS_ERROR("%s: Empty path array.",__FUNCTION__);
        (*markarr_msg_out) = markarr_msg;
        return;
    }

    markarr_msg.markers.resize(patharr_msg.paths.size());
    for (unsigned iPath=0; iPath<markarr_msg.markers.size(); ++iPath)
    {
        markarr_msg.markers[iPath].header.frame_id = patharr_msg.paths[iPath].header.frame_id;
        markarr_msg.markers[iPath].header.stamp = patharr_msg.paths[iPath].header.stamp;
        markarr_msg.markers[iPath].ns = config.ns;
        markarr_msg.markers[iPath].id = iPath;
        markarr_msg.markers[iPath].type = visualization_msgs::Marker::LINE_STRIP;
        markarr_msg.markers[iPath].scale.x = config.scale_x;
        markarr_msg.markers[iPath].color.r = config.color_r;
        markarr_msg.markers[iPath].color.g = config.color_g;
        markarr_msg.markers[iPath].color.b = config.color_b;
        markarr_msg.markers[iPath].color.a = config.color_a;

        markarr_msg.markers[iPath].pose.orientation.w = 1.f;

        markarr_msg.markers[iPath].points.resize(patharr_msg.paths[iPath].poses.size());

        if (markarr_msg.markers[iPath].points.size() > 0)
        {
            markarr_msg.markers[iPath].action = visualization_msgs::Marker::ADD;
        }
        else
        {
            markarr_msg.markers[iPath].action = visualization_msgs::Marker::DELETE;
            break;
        }

        for (unsigned iPose=0; iPose<markarr_msg.markers[iPath].points.size(); ++iPose)
        {
            markarr_msg.markers[iPath].points[iPose] = patharr_msg.paths[iPath].poses[iPose].pose.position;
            markarr_msg.markers[iPath].points[iPose].x += offset[0];
            markarr_msg.markers[iPath].points[iPose].y += offset[1];
            markarr_msg.markers[iPath].points[iPose].z += offset[2];
        }
    }

    (*markarr_msg_out) = markarr_msg;
}

inline void convertSomePath2PathArrayMsg(std::list<std::vector<VehicleState> *>& path, carplanner_msgs::PathArray* patharr_msg_out, std::string frame_id="map")
{
    Sophus::SE3d rot_180_x(Eigen::Quaterniond(0,1,0,0),Eigen::Vector3d(0,0,0));
    carplanner_msgs::PathArray patharr_msg;
    patharr_msg.header.frame_id = frame_id;
    patharr_msg.header.stamp = ros::Time::now();
    for(std::list<std::vector<VehicleState> *>::iterator i_seg=path.begin(); i_seg!=path.end(); advance(i_seg,1))
    {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = frame_id;
        path_msg.header.stamp = ros::Time::now();
        for(uint i_state=0; i_state < (*i_seg)->size(); i_state++)
        {
            VehicleState state = (**i_seg)[i_state];
            state.m_dTwv = rot_180_x * state.m_dTwv;
            carplanner_msgs::VehicleState state_msg = state.toROS();
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id = frame_id;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.pose.position.x = state_msg.pose.transform.translation.x;
            pose_msg.pose.position.y = state_msg.pose.transform.translation.y;
            pose_msg.pose.position.z = state_msg.pose.transform.translation.z;
            pose_msg.pose.orientation.x = state_msg.pose.transform.rotation.x;
            pose_msg.pose.orientation.y = state_msg.pose.transform.rotation.y;
            pose_msg.pose.orientation.z = state_msg.pose.transform.rotation.z;
            pose_msg.pose.orientation.w = state_msg.pose.transform.rotation.w;

            path_msg.poses.push_back(pose_msg);
        }
        patharr_msg.paths.push_back(path_msg);
    }
    (*patharr_msg_out) = patharr_msg;
}

inline void convertSomePath2PathMsg(std::list<std::vector<VehicleState> *>& path, nav_msgs::Path* path_msg_out, std::string frame_id="map")
{
    carplanner_msgs::PathArray patharr_msg;
    convertSomePath2PathArrayMsg(path, &patharr_msg, frame_id);
    nav_msgs::Path path_msg;
    convertPathArrayMsg2PathMsg(patharr_msg, &path_msg);
    (*path_msg_out) = path_msg;
}

inline void convertSomePath2PathMsg(Eigen::Vector3dAlignedVec& path, nav_msgs::Path* path_msg_out, std::string frame_id="map")
{   
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = frame_id;
    path_msg.header.stamp = ros::Time::now();
    for(Eigen::Vector3dAlignedVec::iterator i_seg=path.begin(); i_seg!=path.end(); advance(i_seg,1))
    {
        // carplanner_msgs::VehicleState state_msg = i_seg[i_state].FlipCoordFrame().toROS();
        
        tf::Transform rot_180_x(tf::Quaternion(1,0,0,0),tf::Vector3(0,0,0));
        tf::Transform pose(tf::Quaternion(0,0,0,1),tf::Vector3((*i_seg)[0],(*i_seg)[1],(*i_seg)[2]));
        pose = rot_180_x * pose;

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = frame_id;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x = pose.getOrigin().getX();
        pose_msg.pose.position.y = pose.getOrigin().getY();
        pose_msg.pose.position.z = pose.getOrigin().getZ();

        path_msg.poses.push_back(pose_msg);
    }
    (*path_msg_out) = path_msg;
}

inline void convertSomePath2PathMsg(MotionSample& sample, nav_msgs::Path* path_msg_out, std::string frame_id="map")
{
    path_msg_out->poses.clear();
    path_msg_out->header.frame_id = frame_id;
    path_msg_out->header.stamp = ros::Time::now();
    for(uint i_state=0; i_state < sample.m_vStates.size(); i_state++)
    {
        carplanner_msgs::VehicleState state_msg = sample.m_vStates[i_state].toROS();
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id = frame_id;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.pose.position.x =      state_msg.pose.transform.translation.x;
        pose_msg.pose.position.y =      state_msg.pose.transform.translation.y;
        pose_msg.pose.position.z =      state_msg.pose.transform.translation.z;
        pose_msg.pose.orientation.x =   state_msg.pose.transform.rotation.x;
        pose_msg.pose.orientation.y =   state_msg.pose.transform.rotation.y;
        pose_msg.pose.orientation.z =   state_msg.pose.transform.rotation.z;
        pose_msg.pose.orientation.w =   state_msg.pose.transform.rotation.w;

        path_msg_out->poses.push_back(pose_msg);
    }
}

inline void convertSomePath2PathArrayMsg(std::vector<MotionSample>& path, carplanner_msgs::PathArray* patharr_msg_out, std::string frame_id="map")
{   
    carplanner_msgs::PathArray patharr_msg;
    patharr_msg.header.frame_id = frame_id;
    patharr_msg.header.stamp = ros::Time::now();
    for(std::vector<MotionSample>::iterator it_seg=path.begin(); it_seg!=path.end(); advance(it_seg,1))
    {
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = frame_id;
        path_msg.header.stamp = ros::Time::now();
        for(uint i_state=0; i_state < it_seg->m_vStates.size(); i_state++)
        {
            carplanner_msgs::VehicleState state_msg = it_seg->m_vStates[i_state].FlipCoordFrame().toROS();
            geometry_msgs::PoseStamped pose_msg;
            pose_msg.header.frame_id = frame_id;
            pose_msg.header.stamp = ros::Time::now();
            pose_msg.pose.position.x = state_msg.pose.transform.translation.x;
            pose_msg.pose.position.y = state_msg.pose.transform.translation.y;
            pose_msg.pose.position.z = state_msg.pose.transform.translation.z;
            pose_msg.pose.orientation.x = state_msg.pose.transform.rotation.x;
            pose_msg.pose.orientation.y = state_msg.pose.transform.rotation.y;
            pose_msg.pose.orientation.z = state_msg.pose.transform.rotation.z;
            pose_msg.pose.orientation.w = state_msg.pose.transform.rotation.w;

            path_msg.poses.push_back(pose_msg);
        }
        patharr_msg.paths.push_back(path_msg);
    }
    (*patharr_msg_out) = patharr_msg;
}

inline void convertSomePath2PathMsg(std::vector<MotionSample>& path, nav_msgs::Path* path_msg_out, std::string frame_id="map")
{   
    carplanner_msgs::PathArray patharr_msg;
    convertSomePath2PathArrayMsg(path, &patharr_msg, frame_id);
    nav_msgs::Path path_msg;
    convertPathArrayMsg2PathMsg(patharr_msg, &path_msg);
    (*path_msg_out) = path_msg;
}

inline void convertSomePath2LineStripArrayMsg(std::vector<MotionSample>& path, visualization_msgs::MarkerArray* markarr_msg_out, std::string frame_id="map", const carplanner_msgs::MarkerArrayConfig& config = carplanner_msgs::MarkerArrayConfig())
{
    carplanner_msgs::PathArray patharr_msg;
    convertSomePath2PathArrayMsg(path, &patharr_msg, frame_id);
    convertPathArrayMsg2LineStripArrayMsg(patharr_msg, markarr_msg_out, config);
}

// } // namespace carplanner_msgs

#endif