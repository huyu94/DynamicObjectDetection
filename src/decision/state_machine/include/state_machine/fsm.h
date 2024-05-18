#ifndef _FSM_H_
#define _FSM_H_

#include <string>
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Empty.h>

// #include <trajectories/non_uniform_bspline.h>
// #include <state_machine/plan_container.hpp>
#include <plan_manager/plan_manager.h>
#include <trajectories/Bspline>


using string = std::string;
using Vector3d = Eigen::Vector3d;

namespace fast_planner
{
class FSM
{
private:
    /* data */
public:
    FSM(/* args */);
    ~FSM();
    void init(ros::NodeHandle& nh);

private:
    bool searchForTraj(Vector3d start_pos, Vector3d start_vel, Vector3d start_acc,
                       Vector3d end_pos, Vector3d end_vel, Vector3d end_acc,
                       double search_time, const Vector3d& normal, const Vector3d& dire, bool need_comsistancy);

    // void sendTrajServer(const Trajectory& poly_traj);
    void sendStopToServer();
    void reachGoal(double radius);
    bool needReplan();

    // Eigen::VectorXd geReplanStateFromPath(double t, const Trajectory& traj);

    /*
    * replan in t second from current state
    */
    bool replanOnce(double t);
    bool optimize();

    /* -------------- flag ------------ */
    enum MACHINE_STATE{
        INIT,
        WAIT_GOAL,
        GENERATE_TRAJ,
        FOLLOW_TRAJ,
        REPLAN_TRAJ,
        EMERGENCY_STOP
    };

    enum TARGET_TYPE{
        MANUAL_TARGET = 1,
        PRESET_TARGET = 2,
        REFERENCE_PATH = 3  
    };


    // function classes
    // EnvManager::Ptr env_manager_ptr_;
    PlanManager::Ptr plan_manager_ptr_;


    // params
    // int 
    // bool new_goal_, started_, use_optimizatino_, replan_;

    MACHINE_STATE machine_state_;
    bool have_odom_, have_goal_, trigger_;
    Eigen::Vector3d odom_pos_, odom_vel_;
    Eigen::Quaterniond odom_orient_;


    Vector3d start_pos_, start_vel_, start_acc_, start_yaw_;
    Vector3d end_pos_, end_vel_;
    // Eigen::Vector3d last_goal_pos_;
    // double replan_time_;
    // Eigen::Vector3d start_pos_, start_vel_, start_acc_, end_pos_, end_vel_, end_acc_;
    // ros::Time cur_traj_start_time_;
    


    // ros
    ros::NodeHandle node_;
    ros::Timer exec_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_, goal_sub_;
    ros::Publisher bspline_pub_;
    
    // void goalCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg);
    void execCallback(const ros::TimerEvent& event);


    void changeState(MACHINE_STATE new_state, string pos_call);
    void printState();

    bool callKinoDynamicReplan();
    boool callEmergencyStop(Vector3d stop_pos);






    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
#endif 