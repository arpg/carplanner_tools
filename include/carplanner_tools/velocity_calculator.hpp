#ifndef VELOCITY_CALCULATOR_H
#define VELOCITY_CALCULATOR_H

#include <Eigen/Eigen>
#include <iostream>
#include <boost/format.hpp>

namespace carplanner_tools 
{

template <typename Derived>
static void TransformVector(Eigen::MatrixBase<Derived>& out, Eigen::Transform<double,3,Eigen::TransformTraits::Affine>& in)
{
  Eigen::Matrix<double,4,1> this_temp;
  this_temp.head(3) = out;
  this_temp[3] = (double)1.f;
  Eigen::Matrix<double,1,4> out_temp;
  out_temp = this_temp.transpose() * in.matrix();
  out = out_temp.head(3);
}

static void TransformVelocity(Eigen::Vector3d pos, Eigen::Quaterniond quat, Eigen::Matrix<double,6,1>& vel)
{
    Eigen::Transform<double,3,Eigen::TransformTraits::Affine> tform = Eigen::Transform<double,3,Eigen::TransformTraits::Affine>::Identity();
    tform.translate(pos);
    tform.rotate(quat);
    auto lv = vel.block<3,1>(0,0);
    auto av = vel.tail(3);
    TransformVector(lv, tform);
    TransformVector(av, tform);
}

class VelocityCalculator
{
public:
    typedef Eigen::Matrix<double,6,1> Velocity;
    typedef std::tuple<Eigen::Vector3d&, Eigen::Quaterniond&, Velocity&> CacheElement;
    typedef std::map<double,CacheElement> CacheMap;

    enum CalculateVelocityStatus { UNDEFINED=0, INITIALIZED, ABORTED, ERROR, SUCCESS};

    inline VelocityCalculator(bool vel_in_chassis_frame=false, size_t cache_length=20, double cache_duration=0.1f, bool monotonic_time=true) 
        : vel_in_chassis_frame_(vel_in_chassis_frame), cache_length_(cache_length), cache_duration_(cache_duration), monotonic_time_(monotonic_time)
        {}; 
    // inline ~VelocityCalculator() {};

private:
    CacheMap cache_;
    bool vel_in_chassis_frame_, monotonic_time_;
    size_t cache_length_;
    double cache_duration_;

    inline bool Insert
        (double& time, Eigen::Vector3d& pos, Eigen::Quaterniond& quat, Velocity& vel) 
    { 
        // cache_.insert(std::make_pair(time, std::make_tuple(pos, quat, vel)));
        cache_.emplace(std::piecewise_construct, std::forward_as_tuple(time), std::forward_as_tuple(pos, quat, vel)); // most efficient (?)
    }

public:
    inline CalculateVelocityStatus CalculateVelocity(double time, Eigen::Vector3d pos, Eigen::Quaterniond quat, Velocity& vel)
    {
        // if we don't have any history saved, add the first one and move on
        if (cache_.empty())
        {
            if (Insert(time, pos, quat, vel))
                return INITIALIZED;
            else
                return ERROR;
        }

        // if we already have a cached velocity estimate for the requested time, just return that instead of calculating
        CacheMap::iterator existing_element = cache_.find(time);
        if (existing_element!=cache_.end())
        {
            vel = std::get<2>(existing_element);
            return ABORTED;
        }

        double last_time = cache_.end()->first;
        double dt = time - last_time;

        // if newest received time is not newer than the latest in the cache, something may be wrong. skip if monotonic_time is set
        if (dt<=0.f && monotonic_time_)
            return ABORTED;

        // calculate velocity estimate for this step
        CacheElement last_element = cache_.end()->second;

        double vx = (pos.x() - std::get<0>(last_element)[0])/dt;
        double vy = (pos.y() - std::get<0>(last_element)[1])/dt;
        double vz = (pos.z() - std::get<0>(last_element)[2])/dt;

        Eigen::Quaterniond dquat = std::get<1>(last_element).inverse() * quat;
        double wx = atan2(2*(dquat.w()*dquat.x()+dquat.y()*dquat.z()), 1-2*(pow(dquat.x(),2)+pow(dquat.y(),2)))/dt;
        double wy = asin(2*(dquat.w()*dquat.y()+dquat.z()*dquat.x()))/dt;
        double wz = atan2(2*(dquat.w()*dquat.z()+dquat.x()*dquat.y()), 1-2*(pow(dquat.y(),2)+pow(dquat.z(),2)))/dt;

        Velocity this_vel; this_vel << vx,vy,vz,wx,wy,wz;
        if (!Insert(time, pos, quat, this_vel))
                return ERROR;

        double new_time = cache_.end()->first;
        CacheElement* new_element = &(cache_.end()->second);

        // estimate current velocity from history
        Velocity avg_vel;
        for (auto cache_ptr = cache_.begin(); cache_ptr != cache_.end(); cache_ptr++)
        {
            // starting at the oldest (first) element, if element is too old or cache is full, erase element and continue
            if ((new_time-cache_ptr->first)>cache_duration_ || cache_.size()>cache_length_)
            {
                cache_.erase(cache_ptr--);
                continue;
            }
            // else, sum velocities
            avg_vel += std::get<2>(cache_ptr->second);
        }
        // and average velocities
        size_t cache_size = cache_.size();
        if (cache_size>0)
            avg_vel /= cache_size;
        else
            return ERROR;
        
        // update new element with velocity estimate
        std::get<2>(*new_element) = avg_vel;
        vel = std::get<2>(*new_element);

        if (vel_in_chassis_frame_)
            TransformVelocity(pos, quat, vel);

        Print(cache_);

        return SUCCESS;
    }

    inline static std::string ToString(const CacheMap cache, bool header=true)
    {
        std::string str;
        if (header)
            str += "Time\t\tPos\t\tQuat\t\tVel\n";
        for (auto cache_ptr = cache.begin(); cache_ptr != cache.end(); cache_ptr++)
        {
            str += boost::str(boost::format{"%.2f\t\t%.2f,%.2f,%.2f\t\t%.2f,%.2f,%.2f,%.2f\t\t%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n"} 
                % cache_ptr->first 
                % std::get<0>(cache_ptr->second).x() 
                % std::get<0>(cache_ptr->second).y()
                % std::get<0>(cache_ptr->second).z()
                % std::get<1>(cache_ptr->second).x()
                % std::get<1>(cache_ptr->second).y()
                % std::get<1>(cache_ptr->second).z()
                % std::get<1>(cache_ptr->second).w()
                % std::get<2>(cache_ptr->second)[0]
                % std::get<2>(cache_ptr->second)[1]
                % std::get<2>(cache_ptr->second)[2]
                % std::get<2>(cache_ptr->second)[3]
                % std::get<2>(cache_ptr->second)[4]
                % std::get<2>(cache_ptr->second)[5]
            );
        }
        return str;
    }

    inline static void Print(const CacheMap cache, bool header=true)
    {
        std::string str = ToString(cache, header);
        std::cout << str;
    }
};

} // namespace carplanner_tools

#endif // VELOCITY_CALCULATOR_HPP