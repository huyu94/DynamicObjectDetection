#ifndef _REPLAN_FSM_H_
#define _REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <plan_manager/uniform_bspline_opt/bspline_optimizer.h>
#include <plan_manager/plan_manager.h>
#include <trajectories/Bspline.h>

#include <mutex>
using std::vector;
using namespace fast_planner;
class TopoReplanFSM
{
private:
    enum FSM_EXEC_STATE
    {
        INIT,
        WAIT_TARGET,
        GEN_NEW_TRAJ,
        REPLAN_TRAJ,
        EXEC_TRAJ,
        EMERGENCY_STOP
    };
    enum TARGET_TYPE
    {
        MANUAL_TARGET = 1,
        PRESET_TARGET = 2,
        REFENCE_PATH = 3
    };

    /* planning utils */
    PlanManager::Ptr planner_manager_;
    // PlanningVisualization::Ptr visualization_;
    // traj_utils::DataDisp data_disp_;

    /* parameters */
    int target_type_; // 1 manual target, 2 preset target
    double no_replan_thresh_, replan_thresh_;
    double waypoints_[50][3];
    int waypoints_num_, wp_id_;
    double planning_horizen_, planning_horizen_time_;
    double emergency_time_;
    bool flag_realworld_experiment_;

    /* planning data */
    bool have_trigger_, have_target_, have_odom_, have_new_target_;
    FSM_EXEC_STATE exec_state_;
    int continously_called_times_{0};


    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> wps_;
    int current_wp_;

    bool flag_escape_emergency_;

    // mutex
    std::mutex mutex_call_rebound_replan_;

    /* ROS utils */
    ros::NodeHandle node_;
    ros::Timer exec_timer_, safety_timer_;
    ros::Subscriber waypoint_sub_, odom_sub_,trigger_sub_;
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_;

    /* functions */
    bool callTopoReplan();
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj);
    bool callEmergencyStop(Eigen::Vector3d stop_pos);
    bool planFromCurrentTraj();

    /* state machine change */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pose_call);
    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    std::pair<int,TopoReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    void planNextWaypoint(const Eigen::Vector3d next_wp);
    void getLocalTarget();

    /* ROS FUNCTIONs*/
    void execFSMCallback(const ros::TimerEvent &event);
    void checkCollisionCallback(const ros::TimerEvent &event);
    void triggerCallback(const geometry_msgs::PoseStampedPtr &msg);
    void waypointCallback(const geometry_msgs::PoseStampedPtr &msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg);

    bool checkCollision();

public:
    TopoReplanFSM(/* args */) {}
    ~TopoReplanFSM() {}

    void init(ros::NodeHandle &nh);

    void readGivenWps();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

#endif