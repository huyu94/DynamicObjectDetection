#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <bspline/non_uniform_bspline.h>
// #include <poly_traj/polynomial_traj.h>
#include <path_searching/topo_prm.h>


namespace fast_planner
{
    struct LocalTrajData
    {
        int traj_id_;
        double duration_;
        ros::Time start_time_;
        Eigen::Vector3d start_pos_;
        NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_;
        NonUniformBspline yaw_traj_, yaw_rate_traj_, yawdotdot_traj_;
    }
};




#endif