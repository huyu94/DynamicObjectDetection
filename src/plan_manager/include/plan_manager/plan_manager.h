#ifndef _PLAN_MANAGER_H_
#define _PLAN_MANAGER_H_




#include <ros/ros.h>
#include <Eigen/Eigen>

#include <env_manager/env_manager.h>
#include <plan_manager/plan_container.hpp>
#include <plan_manager/path_searching/astar.h>
#include <plan_manager/path_searching/kinodynamic_astar.h>
#include <plan_manager/uniform_bspline_opt/bspline_optimizer.h>
#include <plan_manager/path_searching/topo_prm.h>
#include <visualization_utils/traj_visualizer.h>
#include <trajectories/uniform_bspline.h>
#include <trajectories/polynomial_traj.h>


using Vector3d = Eigen::Vector3d;
using MatrixXd = Eigen::MatrixXd;

namespace fast_planner
{

class PlanManager
{
public:
    PlanManager(); 
    ~PlanManager();

    void initPlanModules(ros::NodeHandle& nh);

    // void setGlobalWaypoints(vector<Vector3d>& waypoints);



    bool kinodynamicReplan(const Vector3d& start_pos, const Vector3d& start_vel, const Vector3d& start_acc,
                            const Vector3d& end_pos, const Vector3d& end_vel);
    bool planGlobalTraj(const Vector3d& start_pos, const Vector3d& start_vel, const Vector3d& start_acc,
                        const Vector3d& end_pos, const Vector3d& end_vel, const Vector3d& end_acc);
    bool planGlobalTrajWaypoints(const Vector3d& start_pos, const Vector3d& start_vel, const Vector3d& start_acc,
                                 std::vector<Vector3d> &waypoints, const Vector3d& end_vel, const Vector3d& end_acc);
    
    bool emergencyStop(Vector3d& pos);

    bool checkTrajCollision();
    // bool topoReplan(bool Collide);





    PlanParameters pp_;
    LocalTrajData local_data_;
    GlobalTrajData global_data_;
    MidPlanData plan_data_;

    TrajVisualizer::Ptr traj_visual_ptr_;
    EnvManager::Ptr env_manager_ptr_;

private:

    // unique_ptr<Astar> geo_path_finder_;
    unique_ptr<KinodynamicAstar> kino_path_finder_ptr_;
    unique_ptr<TopoPRM> topo_prm_ptr_;
    vector<BsplineOptimizer::Ptr> bspline_optimizer_ptrs_;


    int continuous_failures_count_{0};

    void updateTrajInfo(const UniformBspline& position_traj, const ros::Time time_now);
    MatrixXd reparamLocalTraj(double start_t, double& dt, double& duration);                // according to the radius, reparam the local traj
    MatrixXd reparamLocalTraj(double start_t, double duration, int seg_num, double& dt);    // according to the duration, reparam the local traj
    /**
     * @brief
     * @param bspline 
     * @param start_end_derivative: output
     * @param ratio: input
     * @param ctrl_pts : output ,
     * @param dt : output , B样条的时间间隔
     * @param time_inc: output, B样条的时间增量
    */
    void reparamBspline(UniformBspline& bspline, vector<Vector3d>& start_end_derivative, double ratio, MatrixXd& ctrl_pts, double& dt, double& time_inc); // 拉长b样条时间，重新参数化，生成新的控制点ctrl_pts

    bool refineTraj(UniformBspline& traj, vector<Vector3d>& start_end_derivatives, double ratio, double& ts, MatrixXd& optimal_control_points);

    vector<Vector3d> sampleTraj(Eigen::MatrixXd& ctrl_pts, double ts);


public:
    typedef unique_ptr<PlanManager> Ptr;

};


}






#endif