#ifndef _APPLY_VELOCITIES_MANAGER
#define	_APPLY_VELOCITIES_MANAGER

typedef actionlib::SimpleActionServer<carplanner_msgs::ApplyVelocitiesAction> SimpleServer;

struct ApplyVelocitiesManager
{
  std::vector<SimpleServer> simple_servers_; 

  void schedule(const carplanner_msgs::ApplyVelocitiesGoalConstPtr &goal)
  {
    
    simple_servers_.push_back();
  }
}

#endif
