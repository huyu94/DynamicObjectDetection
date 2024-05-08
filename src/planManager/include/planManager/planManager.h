#ifndef _PLAN_MANAGER_H_
#define _PLAN_MANAGER_H_




#include <ros/ros.h>
#include <Eigen/Eigen>

#include <plan_env/env_manager.h>

using Vector3d = Eigen::Vector3d;
namespace fast_planner
{

class PlanManager
{
    public:
        PlanManager();
        PlanManager();

        bool kinodynamicReplan(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                                Vector3d end_pos, Vector3d end_vel);
        bool planGlobalTraj(const Eigen::Vector3d& start_pos);
        bool topoReplan(bool Collide);

        // void planYaw(const Vector3d& start_yaw);

        void initPlanModules(ros::NodeHandle& nh);
        // void setGlobalWaypoints(vector<Vector3d>& waypoints);


    private:
        EnvManager::Ptr env_manger_;




public:
    typedef unique_ptr<PlanManager> Ptr;

};


}






#endif