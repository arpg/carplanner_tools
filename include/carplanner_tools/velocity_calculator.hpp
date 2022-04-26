#ifndef VELOCITY_CALCULATOR_H
#define VELOCITY_CALCULATOR_H

#include <Eigen/Eigen>
#include <iostream> 
#include <boost/format.hpp>

namespace carplanner_tools 
{

static Eigen::Vector3d& Vee(Eigen::Matrix3d mat)
{
    Eigen::Vector3d vec1; vec1 << mat(2,1), mat(0,2), mat(1,0);
    Eigen::Vector3d vec2; vec2 << -mat(1,2), -mat(2,0), -mat(0,1);
    if(vec1!=vec2)
    {
        std::cout << "ERROR: Matrix is not skew symmetric." << std::endl;
        assert(false);
    }
    return vec1;
}

static Eigen::Matrix3d& Hat(Eigen::Vector3d vec)
{
    Eigen::Matrix3d mat; mat << 0.f, -vec[2],  vec[1],
                        vec[2],     0.f, -vec[0],
                       -vec[1],  vec[0],     0.f;
    return mat;
}

template <typename Derived>
static void TransformVector(Eigen::MatrixBase<Derived>& out, const Eigen::Matrix4d& in)
{
    Eigen::Matrix<double,4,1> this_temp;
    this_temp.head(3) = out;
    this_temp[3] = (double)1.f;
    std::cout << "this_temp\n" << this_temp.format(Eigen::IOFormat(2)) << std::endl;
    std::cout << "in\n" << in.format(Eigen::IOFormat(2)) << std::endl;
    Eigen::Matrix<double,1,4> out_temp;
    out_temp = this_temp.transpose() * in;
    std::cout << "out_temp\n" << out_temp.format(Eigen::IOFormat(2)) << std::endl;
    out = out_temp.head(3);
}

static void TransformVelocity(Eigen::Vector3d pos, Eigen::Quaterniond quat, Eigen::Matrix<double,6,1>& vel)
{
    Eigen::Affine3d tform = Eigen::Affine3d::Identity();
    tform.translate(pos);
    tform.rotate(quat);
    auto lv = vel.head(3);
    auto av = vel.tail(3);
    TransformVector(lv, tform.matrix()); // lv_c = lv_w * Twc
    // TransformVector(av, tform.matrix());
    // av = Vee(quat.toRotationMatrix().transpose().inverse() * Hat(av) * quat.toRotationMatrix().transpose());
    av = (Eigen::Affine3d(Eigen::Quaterniond(Eigen::AngleAxisd(av[0],Eigen::Vector3d::UnitX())*Eigen::AngleAxisd(av[1],Eigen::Vector3d::UnitY())*Eigen::AngleAxisd(av[2],Eigen::Vector3d::UnitZ()))).matrix() * tform.matrix()).block<3,3>(0,0).eulerAngles(0,1,2);
}

class VelocityCalculator
{
public:
    typedef const double Time;
    typedef Eigen::Vector3d Position;
    typedef Eigen::Quaterniond Orientation;
    typedef Eigen::Matrix<double,6,1> Velocity;
    // typedef Velocity InstantaneousVelocity;
    // typedef Velocity AveragedVelocity;
    // typedef Velocity FilteredVelocity;
    typedef std::tuple<Position, Orientation, Velocity, Velocity, Velocity> CacheElement;
    typedef std::pair<Time,CacheElement> CachePair;
    typedef std::map<CachePair::first_type, CachePair::second_type> CacheMap;

    enum CalculateVelocityStatus { UNDEFINED_STATUS=0, INITIALIZED, ABORTED, ERROR, SUCCESS};
    enum VelocityType {UNDEFINED_TYPE=0, INSTANTANEOUS=2, AVERAGED=3, FILTERED=4};

    inline VelocityCalculator(bool vel_in_chassis_frame=true, size_t cache_length=20, double cache_duration=0.1f, bool monotonic_time=true) 
        : vel_in_chassis_frame_(vel_in_chassis_frame), cache_length_(cache_length), cache_duration_(cache_duration), monotonic_time_(monotonic_time)
        {
            cache_.clear();
        }; 
    // inline ~VelocityCalculator() {};

private:
    CacheMap cache_;
    bool vel_in_chassis_frame_, monotonic_time_;
    size_t cache_length_;
    double cache_duration_;

    inline bool Insert
        (Time time, Position pos, Orientation quat, Velocity vel, Velocity avg_vel, Velocity fil_vel) 
    { 
        // cache_.insert(std::make_pair(time, std::make_tuple(pos, quat, vel)));
        try {
            cache_.emplace(std::piecewise_construct, std::forward_as_tuple(time), std::forward_as_tuple(pos, quat, vel, avg_vel, fil_vel)); // most efficient (?)
            return true;
        } catch (std::exception& e) {
            return false;
        }
    }

    inline CalculateVelocityStatus PopulatePose(Time time, Position pos, Orientation quat)
    {
        if (!Insert(time, pos, quat, Velocity::Zero(), Velocity::Zero(), Velocity::Zero()))
        {   
            std::cout << "VelocityCalculator: Failed to insert new element." << std::endl;
            Print(cache_); 
            return ERROR;   
        }

        // Print("Done populating pose.\n");
        // Print(cache_);

        return SUCCESS;
    }

    inline CalculateVelocityStatus PopulateInstantaneousVelocity()
    {
        auto last_pair_ptr = GetNewestPair(1);
        Time last_time = last_pair_ptr->first;

        auto new_pair_ptr = GetNewestPair();
        Time new_time = new_pair_ptr->first;

        Time dt = new_time - last_time;

        CacheElement last_element = last_pair_ptr->second;
        Position last_pos = std::get<0>(last_element);
        Orientation last_quat = std::get<1>(last_element);

        CacheElement& new_element = new_pair_ptr->second;
        Position new_pos = std::get<0>(new_element);
        Orientation new_quat = std::get<1>(new_element);

        Print(boost::str(boost::format{"VelocityCalculator: Last pose: %f %f %f %f %f %f %f %f.\n"} 
            % last_time % last_pos.x() % last_pos.y() % last_pos.z() % last_quat.x() % last_quat.y() % last_quat.z() % last_quat.w()));

        // calculate velocity estimate for this step
        double vx = (new_pos.x() - last_pos.x())/dt;
        double vy = (new_pos.y() - last_pos.y())/dt;
        double vz = (new_pos.z() - last_pos.z())/dt;

        Orientation dquat = last_quat.inverse() * new_quat;

        // Eigen::Vector3d a = last_pos.cross(new_pos);
        // double w = sqrt(pow(last_pos.norm(),2)*pow(new_pos.norm(),2))+last_pos.dot(new_pos);
        // Orientation dquat(w, a.x(), a.y(), a.z());
        // dquat.normalize();

        double wx = atan2(2*(dquat.w()*dquat.x()+dquat.y()*dquat.z()), 1-2*(pow(dquat.x(),2)+pow(dquat.y(),2)))/dt;
        double wy = asin(2*(dquat.w()*dquat.y()+dquat.z()*dquat.x()))/dt;
        double wz = atan2(2*(dquat.w()*dquat.z()+dquat.x()*dquat.y()), 1-2*(pow(dquat.y(),2)+pow(dquat.z(),2)))/dt;

        Velocity inst_vel; inst_vel << vx,vy,vz,wx,wy,wz;

        Print(boost::str(boost::format{"VelocityCalculator: This vel: %f %f %f %f %f %f %f.\n"} 
            % new_time % inst_vel[0] % inst_vel[1] % inst_vel[2] % inst_vel[3] % inst_vel[4] % inst_vel[5]));

        std::get<2>(new_element) = inst_vel;

        // Print("Done populating inst vel.\n");
        // Print(cache_);

        return SUCCESS;
    }

    inline CalculateVelocityStatus PopulateAveragedVelocity()
    {
        auto new_pair_ptr = GetNewestPair();
        Time new_time = new_pair_ptr->first;
        CacheElement& new_element = new_pair_ptr->second;
        Position new_pos = std::get<0>(new_element);
        Orientation new_quat = std::get<1>(new_element);

        Print(boost::str(boost::format{"VelocityCalculator: New pose: %f %f %f %f %f %f %f %f.\n"} 
            % new_time % new_pos.x() % new_pos.y() % new_pos.z() % new_quat.x() % new_quat.y() % new_quat.z() % new_quat.w()));

        // estimate current velocity from history
        Velocity avg_vel = Velocity::Zero();
        for (auto cache_ptr = cache_.begin(); cache_ptr != cache_.end(); )
        {
            Print("Ptr now at "+std::to_string(cache_ptr->first)+".\n");
            // starting at the oldest (first) element, if element is too old or cache is full, erase element and continue
            if ((new_time-cache_ptr->first)>cache_duration_)
            {
                Print("Message at "+std::to_string(cache_ptr->first)+" is too old ("+std::to_string(cache_duration_)+"). Removing from cache...\n");
                cache_ptr = cache_.erase(cache_ptr);
                Print(cache_);
                continue;
            }
            if (cache_.size()>cache_length_)
            {
                Print("Message at "+std::to_string(cache_ptr->first)+" is too many ("+std::to_string(cache_length_)+"). Removing from cache...\n");
                cache_ptr = cache_.erase(cache_ptr);
                Print(cache_);
                continue;
            }
            // else, sum velocities
            avg_vel += std::get<2>(cache_ptr->second);
            ++cache_ptr;
            Print("Message at "+std::to_string(cache_ptr->first)+" summed. new sum "+std::to_string(avg_vel[0])+" "+std::to_string(avg_vel[1])+" "+std::to_string(avg_vel[2])+" "+std::to_string(avg_vel[3])+" "+std::to_string(avg_vel[4])+" "+std::to_string(avg_vel[5])+" new size "+std::to_string(cache_.size())+".\n");
        }
        // and average velocities
        size_t cache_size = cache_.size();
        if (cache_size>0)
        {
            Print("Cache sum "+std::to_string(avg_vel[0])+" "+std::to_string(avg_vel[1])+" "+std::to_string(avg_vel[2])+" "+std::to_string(avg_vel[3])+" "+std::to_string(avg_vel[4])+" "+std::to_string(avg_vel[5])+" size "+std::to_string(cache_size)+".\n");
            avg_vel /= cache_size;
        } else {
            std::cout << "VelocityCalculator: Cache size of zero detected." << std::endl;
            // Print(cache_);
            return ERROR;
        }
        
        // update new element with velocity estimate
        std::get<3>(new_element) = avg_vel;

        Print("Done populating avg vel.\n");
        // Print(cache_);

        return SUCCESS;
    }

    inline CalculateVelocityStatus PopulateFilteredVelocity()
    {
        auto new_pair_ptr = GetNewestPair();
        Time new_time = new_pair_ptr->first;
        CacheElement& new_element = new_pair_ptr->second;

        Velocity fil_vel = std::get<3>(new_element);

        if (vel_in_chassis_frame_)
        {
            auto last_pair_ptr = GetNewestPair(1);
            Time last_time = last_pair_ptr->first;
            CacheElement last_element = last_pair_ptr->second;

            const double dt = new_time - last_time;
            const Position last_pos = std::get<0>(last_element);
            const Orientation last_quat = std::get<1>(last_element);
            const Position new_pos = std::get<0>(new_element);
            const Orientation new_quat = std::get<1>(new_element);

            Print("Transforming linear vel...\n");

            fil_vel.head(3) = (fil_vel.head(3).transpose() * new_quat.toRotationMatrix()).transpose();

            Print("Transforming angular vel...\n");

            Eigen::Vector3d a = last_pos.cross(new_pos);
            double w = sqrt(pow(last_pos.norm(),2)*pow(new_pos.norm(),2))+last_pos.dot(new_pos);
            Orientation dquat(w, a.x(), a.y(), a.z());
            dquat.normalize();

            double wx = atan2(2*(dquat.w()*dquat.x()+dquat.y()*dquat.z()), 1-2*(pow(dquat.x(),2)+pow(dquat.y(),2)))/dt;
            double wy = asin(2*(dquat.w()*dquat.y()+dquat.z()*dquat.x()))/dt;
            double wz = atan2(2*(dquat.w()*dquat.z()+dquat.x()*dquat.y()), 1-2*(pow(dquat.y(),2)+pow(dquat.z(),2)))/dt;

            fil_vel.tail(3) << wx,wy,wz;
        }

        std::get<4>(new_element) = fil_vel;

        // Print("Done populating fil vel.\n");
        // Print(cache_);

        return SUCCESS;
    }

public:
    inline CachePair* GetNewestPair(int dist_from_end=0) { return &*(std::next(cache_.rbegin(),dist_from_end)); }
    inline CachePair* GetOldestPair(int dist_from_start=0) { return &*(std::next(cache_.begin(),dist_from_start)); }

    inline const CachePair* GetPair(Time time=DBL_MAX)
    {
        const CachePair* pair_ptr;

        if (time==DBL_MAX)
        {
            // if time is max, get last element
            pair_ptr = GetNewestPair();
        }
        else
        {
            // else, try to get element of the given time
            pair_ptr = &*(cache_.find(time));
            if (pair_ptr==&*(cache_.end()))
            {
                std::cout << "VelocityCalculator: Element at time " << time << " does not exist. Returning zeros." << std::endl;
                return nullptr;
            }
        }

        return pair_ptr;
    }

    inline Velocity GetVelocity(Time time=DBL_MAX, const VelocityType type=FILTERED)
    {
        const CachePair* pair_ptr = GetPair(time);

        Velocity vel = Velocity::Zero();
        
        switch(type)
        {
            case INSTANTANEOUS:
                vel = std::get<2>(pair_ptr->second);
                break;
            case AVERAGED:
                vel = std::get<3>(pair_ptr->second);
                break;
            case FILTERED:
                vel = std::get<4>(pair_ptr->second);
                break;
            default:
                break;
        }

        return vel;
    }

    inline CalculateVelocityStatus CalculateVelocity(Time time, Position pos, Orientation quat, Velocity& vel)
    {
        Print(boost::str(boost::format{"VelocityCalculator: Got new pose: %f %f %f %f %f %f %f %f. Calculating velocity...\n"} 
            % time % pos.x() % pos.y() % pos.z() % quat.x() % quat.y() % quat.z() % quat.w()));

        // init output velocity as all zeros
        vel.setZero();

        // if we don't have any history saved, add the first one and move on
        if (cache_.empty())
        {
            if (Insert(time, pos, quat, Velocity::Zero(), Velocity::Zero(), Velocity::Zero()))
            {   
                std::cout << "VelocityCalculator: Initialized." << std::endl;
                Print(cache_);
                return INITIALIZED;
            } else {
                std::cout << "VelocityCalculator: Failed to initialize." << std::endl;
                Print(cache_);
                return ERROR;
            }
        }

        // if we already have a cached velocity estimate for the requested time, just return that instead of calculating
        CacheMap::iterator existing_element = cache_.find(time);
        if (existing_element!=cache_.end())
        {
            std::cout << "VelocityCalculator: Element at time " << time << " already exists." << std::endl;
            vel = std::get<3>(existing_element->second);
            return ABORTED;
        }

        auto last_pair_ptr = GetNewestPair();
        Time last_time = last_pair_ptr->first;
        Time dt = time - last_time;

        // if newest received time is not newer than the latest in the cache, something may be wrong. skip if monotonic_time is set
        if (dt<=0.f && monotonic_time_)
        {
            std::cout << "VelocityCalculator: Time not monotonic." << std::endl;
            Print(cache_);
            return ABORTED;
        }

        CalculateVelocityStatus status = SUCCESS;

        status = PopulatePose(time, pos, quat);
        if (status!=SUCCESS)
            return status;

        status = PopulateInstantaneousVelocity();
        if (status!=SUCCESS)
            return status;

        PopulateAveragedVelocity();
        if (status!=SUCCESS)
            return status;

        PopulateFilteredVelocity();
        if (status!=SUCCESS)
            return status;
        
        vel = std::get<4>(GetNewestPair()->second);

        // std::cout << "VelocityCalculator succeeded." << std::endl;
        Print(cache_);
        // std::cout << "vel\navg " << GetVelocity(DBL_MAX,AVERAGED).transpose().format(Eigen::IOFormat(2)) << " \nfil " << GetVelocity(DBL_MAX,FILTERED).transpose().format(Eigen::IOFormat(2)) << "\n-----------------------" << std::endl;

        return status;
    }

    inline static std::string ToString(const Position pos, bool header=false)
    {
        std::string str;

        if (header)
            str += "Pos\n";

        str += boost::str(boost::format{"%.2f,%.2f,%.2f\n"} 
            % pos.x() 
            % pos.y()
            % pos.z()
        );

        return str;
    }

    inline static std::string ToString(const Orientation quat, bool header=false)
    {
        std::string str;

        if (header)
            str += "Quat\n";

        str += boost::str(boost::format{"%.2f,%.2f,%.2f,%.2f\n"} 
            % quat.x() 
            % quat.y()
            % quat.z()
            % quat.w()
        );

        return str;
    }

    inline static std::string ToString(const Velocity vel, bool header=false)
    {
        std::string str;

        if (header)
            str += "Vel\n";

        str += boost::str(boost::format{"%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n"} 
            % vel[0]
            % vel[1]
            % vel[2]
            % vel[3]
            % vel[4]
            % vel[5]
        );

        return str;
    }

    inline static std::string ToString(const CacheElement elem, bool header=false)
    {
        std::string str;

        if (header)
            str += "Pos\t\tQuat\t\t\tIVel\t\t\tAVel\t\t\tFVel\n";

        str += boost::str(boost::format{"%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\t%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n"} 
            % std::get<0>(elem).x() 
            % std::get<0>(elem).y()
            % std::get<0>(elem).z()
            % std::get<1>(elem).x()
            % std::get<1>(elem).y()
            % std::get<1>(elem).z()
            % std::get<1>(elem).w()
            % std::get<2>(elem)[0]
            % std::get<2>(elem)[1]
            % std::get<2>(elem)[2]
            % std::get<2>(elem)[3]
            % std::get<2>(elem)[4]
            % std::get<2>(elem)[5]
            % std::get<3>(elem)[0]
            % std::get<3>(elem)[1]
            % std::get<3>(elem)[2]
            % std::get<3>(elem)[3]
            % std::get<3>(elem)[4]
            % std::get<3>(elem)[5]
            % std::get<4>(elem)[0]
            % std::get<4>(elem)[1]
            % std::get<4>(elem)[2]
            % std::get<4>(elem)[3]
            % std::get<4>(elem)[4]
            % std::get<4>(elem)[5]
        );

        return str;
    }

    inline static std::string ToString(const CachePair pair, bool header=false)
    {
        std::string str;

        if (header)
            str += "Time"+std::string(14+1-4,' ')+"| Pos"+std::string(7*3-1+1-3,' ')+"| Quat"+std::string(7*4-1+1-4,' ')+"| IVel"+std::string(7*6-1+1-4,' ')+"| AVel"+std::string(7*6-1+1-4,' ')+"| FVel\n";

        std::string format = "%6.2f";
        str += boost::str(boost::format{"%14.2f | "+format+","+format+","+format+" | "+format+","+format+","+format+","+format+" | "+format+","+format+","+format+","+format+","+format+","+format+" | "+format+","+format+","+format+","+format+","+format+","+format+" | "+format+","+format+","+format+","+format+","+format+","+format+"\n"} 
            %             pair.first 
            % std::get<0>(pair.second).x() 
            % std::get<0>(pair.second).y()
            % std::get<0>(pair.second).z()
            % std::get<1>(pair.second).x()
            % std::get<1>(pair.second).y()
            % std::get<1>(pair.second).z()
            % std::get<1>(pair.second).w()
            % std::get<2>(pair.second)[0]
            % std::get<2>(pair.second)[1]
            % std::get<2>(pair.second)[2]
            % std::get<2>(pair.second)[3]
            % std::get<2>(pair.second)[4]
            % std::get<2>(pair.second)[5]
            % std::get<3>(pair.second)[0]
            % std::get<3>(pair.second)[1]
            % std::get<3>(pair.second)[2]
            % std::get<3>(pair.second)[3]
            % std::get<3>(pair.second)[4]
            % std::get<3>(pair.second)[5]
            % std::get<4>(pair.second)[0]
            % std::get<4>(pair.second)[1]
            % std::get<4>(pair.second)[2]
            % std::get<4>(pair.second)[3]
            % std::get<4>(pair.second)[4]
            % std::get<4>(pair.second)[5]
        );

        return str;
    }

    inline static std::string ToString(const CacheMap cache, bool header=false)
    {
        std::string header_str = "Time"+std::string(14+1-4,' ')+"| Pos"+std::string(7*3-1+1-3,' ')+"| Quat"+std::string(7*4-1+1-4,' ')+"| IVel"+std::string(7*6-1+1-4,' ')+"| AVel"+std::string(7*6-1+1-4,' ')+"| FVel"+std::string(7*6-1-4,' ')+"\n";

        std::string header_line = std::string(header_str.size(),'-')+"\n";

        std::string str;

        str += header_line;

        if (header)
            str += header_str;

        for (auto cache_ptr = cache.begin(); cache_ptr != cache.end(); cache_ptr++)
        {
            str += "---\n";
            str += ToString(*cache_ptr, false);
        }

        str += header_line;

        return str;
    }

    inline static void Print(const CacheMap cache, bool header=true)
    {
        Print(ToString(cache, header));
    }

    inline void PrintCache(bool header=true)
    {
        Print(ToString(cache_, header));
    }

    inline static void Print(const std::string str)
    {
        if (0)
            std::cout << str << std::flush;
    }
};

} // namespace carplanner_tools

#endif // VELOCITY_CALCULATOR_HPP