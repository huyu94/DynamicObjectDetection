#ifndef _PLAN_MANAGER_H_
#define _PLAN_MANAGER_H_




#include <ros/ros.h>
#include <Eigen/Eigen>

#include <plan_env/env_manager.h>
#include <plan_manager/plan_container.hpp>
#include <plan_manager/path_searching/astar.h>
#include <plan_manager/path_searching/kinodynamic_astar.h>
#include <plan_manager/uniform_bspline_opt/bspline_optimizer.h>
#include <plan_manager/path_searching/topo_prm.h>


using Vector3d = Eigen::Vector3d;
using MatrixXd = Eigen::MatrixXd;

namespace fast_planner
{

class PlanManager
{
public:
    PlanManager();
    PlanManager();

    void initPlanModules(ros::NodeHandle& nh);

    bool kinodynamicReplan(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                            Vector3d end_pos, Vector3d end_vel);
    bool planGlobalTraj(const Eigen::Vector3d& start_pos);
    bool topoReplan(bool Collide);
    // void planYaw(const Vector3d& start_yaw);

    // void setGlobalWaypoints(vector<Vector3d>& waypoints);

    bool checkTrajCollision();



    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;

private:
    EnvManager::Ptr env_manger_;

    // unique_ptr<Astar> geo_path_finder_;
    unique_ptr<KinodynamicAstar> kino_path_finder_;
    unique_ptr<TopoPRM> topo_prm_;
    vector<BsplineOptimizer::Ptr> bspline_optimizers_;

    void updateTrajInfo();
    void reparamBspline(UniformBspline& bspline, vector<Vector3d>& start_end_derivative, double ratio, MatrixXd& ctrl_pts, double& dt, double& time_inc);
    bool refineTraj(UniformBspline& traj, vector<Vector3d>& start_end_derivatives, double ratio, double& ts, MatrixXd& optimal_control_points);


public:
    typedef unique_ptr<PlanManager> Ptr;

};


}






#endif