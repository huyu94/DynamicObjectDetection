#include <state_machine/fsm.h>

namespace fast_planner
{  
    FSM::FSM()
    {
    }

    FSM::~FSM()
    {
    }


    void FSM::init(ros::NodeHandle &nh)
    {

        // // ros
        // node_ = nh;
        // current_wp_ = 0;
        // machine_state_ = INIT;
        // have_goal_ = false;
        // have_odom_ = false;
        // /*  fsm param  */
        // node_.param("fsm/flight_type", target_type_, -1);
        // node_.param("fsm/thresh_replan", replan_thresh_, -1.0);
        // node_.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
        // node_.param("fsm/planning_horizon", planning_horizon_, -1.0);
        // node_.param("fsm/planning_horizen_time", planning_horizon_time_, -1.0);
        // node_.param("fsm/emergency_time_", emergency_time_, 1.0);

        // node_.param("fsm/waypoint_num", waypoint_num_, -1);
        // for (int i = 0; i < waypoint_num_; i++)
        // {
        //     node_.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
        //     node_.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
        //     node_.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
        // }

        // function classes
        plan_manager_ptr_.reset(new PlanManager);
        plan_manager_ptr_->initPlanModules(nh);

        // /* callback */
        // exec_timer_ = nh.createTimer(ros::Duration(0.01), &FSM::execCallback, this);
        // safety_timer_ = nh.createTimer(ros::Duration(0.05), &FSM::safetyCallback, this);

        // odom_sub_ = nh.subscribe("odom", 1, &FSM::odomCallback, this);

        // bspline_pub_ = nh.advertise<trajectories::Bspline>("bspline", 10);
        // // data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);

        // if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
        //     waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &FSM::waypointCallback, this);
        // else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        // {
        //     ROS_ERROR("not implemeneted!!");
        //     // ros::Duration(1.0).sleep();
        //     // while (ros::ok() && !have_odom_)
        //     //     ros::spinOnce();
        //     // planGlobalTrajbyGivenWps();
        // }
        // else
        //     cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
    }

    void FSM::changeState(FSM::MACHINE_STATE state, string pos_call)
    {
        string state_str[6] = {"INIT", "WAIT_GOAL", "GENERATE_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ", "EMERGENCY_STOP"};
        ROS_INFO_STREAM("[FSM]: change from " << state_str[int(machine_state_)] << " to " << state_str[int(state)] << " by " << pos_call);
    }

    void FSM::printState()
    {
        string state_str[6] = {"INIT", "WAIT_GOAL", "GENERATE_TRAJ", "EXEC_TRAJ", "REPLAN_TRAJ", "EMERGENCY_STOP"};
        ROS_INFO_STREAM("[FSM]: state : " << state_str[int(machine_state_)]);
    }

    void FSM::execCallback(const ros::TimerEvent &event)
    {
        static int fsm_num = 0;
        fsm_num++;
        // check state state
        if (fsm_num == 100)
        {
            printState();
            if (!have_odom_)
                ROS_INFO_STREAM("[FSM]: no odom");
            if (!trigger_)
                ROS_INFO_STREAM("[FSM]: no trigger");
            fsm_num = 0;
        }

        switch (machine_state_)
        {
        case INIT:
        {
            if (!have_odom_)
            {
                return;
            }
            if (!trigger_)
            {
                return;
            }
            changeState(WAIT_GOAL, "FSM");
            break;
        }

        case WAIT_GOAL:
        {
            if (!have_goal_)
            {
                return;
            }
            else
            {
                changeState(GENERATE_TRAJ, "FSM");
            }
            break;
        }

        case GENERATE_TRAJ:
        {
            start_pos_ = odom_pos_;
            start_vel_ = odom_vel_;
            start_acc_.setZero();

            // Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
            // start_yaw_(0) = atan2(rot_x(1), rot_x(0));
            // start_yaw_(1) = start_yaw_(2) = 0.0;

            bool success = callKinoDynamicReplan();

            if (success)
            {
                changeState(EXEC_TRAJ, "FSM");
            }
            else
            {
                changeState(GENERATE_TRAJ, "FSM");
            }
            break;
        }

        case REPLAN_TRAJ:
        {
            if (planFromCurrentTraj())
            {
                changeState(EXEC_TRAJ, "FSM");
            }
            else
            {
                changeState(REPLAN_TRAJ, "FSM");
            }

            break;
        }

        case EXEC_TRAJ:
        {
            LocalTrajData *info = &plan_manager_ptr_->local_data_;
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - info->start_time_).toSec();
            t_cur = min(info->duration_, t_cur);

            Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

            if (t_cur >= info->duration_ - 1e-2)
            {
                have_goal_ = false;
                changeState(WAIT_GOAL, "FSM");
                return;
            }
            else if ((end_pos_ - pos).norm() < no_replan_thresh_)
            {
                ROS_INFO_STREAM("[FSM]: near local traj end");
                return;
            }
            else if ((info->start_pos_ - pos).norm() < replan_thresh_)
            {
                ROS_INFO_STREAM("near start");
                return;
            }
            else
            {
                changeState(REPLAN_TRAJ, "FSM");
            }
            break;
        }
        }
    }

    void FSM::odomCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        odom_pos_(0) = msg->pose.pose.position.x;
        odom_pos_(1) = msg->pose.pose.position.y;
        odom_pos_(2) = msg->pose.pose.position.z;

        odom_vel_(0) = msg->twist.twist.linear.x;
        odom_vel_(1) = msg->twist.twist.linear.y;
        odom_vel_(2) = msg->twist.twist.linear.z;

        odom_orient_.w() = msg->pose.pose.orientation.w;
        odom_orient_.x() = msg->pose.pose.orientation.x;
        odom_orient_.y() = msg->pose.pose.orientation.y;
        odom_orient_.z() = msg->pose.pose.orientation.z;

        have_odom_ = true;
    }

    void FSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
    {
        if (msg->poses[0].pose.position.z < -0.1)
        {
            return;
        }

        ROS_INFO("Triggered");
        trigger_ = true;
        bool success = false;
        end_pos_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;

        success = plan_manager_ptr_->planGlobalTraj(odom_pos_);

        if (success)
        {
            constexpr double step_size_t = 0.1;
            int i_end = floor(plan_manager_ptr_->global_data_.global_duration_ / step_size_t);
            vector<Eigen::Vector3d> gloabl_traj(i_end);
            for (int i = 0; i < i_end; i++)
            {
                gloabl_traj[i] = plan_manager_ptr_->global_data_.global_traj_.evaluate(i * step_size_t);
            }


            end_vel_.setZero();
            have_goal_ = true;
            // have_new_target_ = true;

             /*** FSM ***/
            if (machine_state_ == WAIT_GOAL)
            {
                changeState(GENERATE_TRAJ, "TRIG");
            }
            else if (machine_state_ == EXEC_TRAJ)
            {
                changeState(REPLAN_TRAJ, "TRIG");
            }
            // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
            // visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
        }
    }

    void FSM::safetyCallback(const ros::TimerEvent &event)
    {
        LocalTrajData *info = &plan_manager_ptr_->local_data_;
        auto env_manager = plan_manager_ptr_->env_manager_ptr_;

        if(machine_state_ == WAIT_GOAL || info->start_time_.toSec() < 1e-5)
        {
            return;
        }

        constexpr double time_step = 0.01;
        double t_cur = (ros::Time::now() - info->start_time_).toSec();
        double t_2_3 = info->duration_ * 2 / 3;
        bool occ = false;
        for(double t = t_cur; t < info->duration_; t += time_step)
        {
            if(t_cur < t_2_3 && t >= t_2_3)
            {
                break;
            }

            if(env_manager->checkCollisionInGridMap(info->position_traj_.evaluateDeBoorT(t)))
            {
                ROS_ERROR("Collision detected in grid map!");
                occ = true;
            }
            int collision_id;
            ros::Time check_time = info->start_time_ + ros::Duration(t);
            if(env_manager->checkCollisionInTrackerPool(info->position_traj_.evaluateDeBoorT(t),check_time,collision_id))
            {
                ROS_ERROR("Collision detected in tracker pool!");
                occ = true;
            }

            if(occ)
            {
                if(planFromCurrentTraj())
                {
                    changeState(EXEC_TRAJ, "SAFETY");
                    return ;
                }
                else
                {
                    if(t - t_cur < emergency_time_)
                    {
                        ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
                        changeState(EMERGENCY_STOP, "SAFETY");
                    }
                    else
                    {
                        changeState(REPLAN_TRAJ, "SAFETY");
                    }
                    return ;
                
                }
                break;
            }
        }
        
    }

    bool FSM::callKinoDynamicReplan()
    {
        getLocalTarget();

        bool plan_success = plan_manager_ptr_->kinodynamicReplan(start_pos_, start_vel_, start_acc_, local_target_pos_, local_target_vel_);

        ROS_INFO_STREAM("final plan success");

        if (plan_success)
        {
            publishBspline();
        }
    }

    bool FSM::callEmergencyStop(Vector3d stop_pos)
    {

        plan_manager_ptr_->emergencyStop(stop_pos);

        publishBspline();

        return true;
    }

    bool FSM::planFromCurrentTraj()
    {
        LocalTrajData *info = &plan_manager_ptr_->local_data_;
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - info->start_time_).toSec();

        start_pos_ = info->position_traj_.evaluateDeBoorT(t_cur);
        start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
        start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

        bool success = callKinoDynamicReplan();

        if (!success)
        {
            success = callKinoDynamicReplan();
            if (!success)
            {
                return false;
            }
        }
        return true;
    }

    // void FSM::planGlobalTrajbyGivenWps()
    // {
    //     std::vector<Eigen::Vector3d> wps(waypoint_num_);
    //     for (int i = 0; i < waypoint_num_; i++)
    //     {
    //         wps[i](0) = waypoints_[i][0];
    //         wps[i](1) = waypoints_[i][1];
    //         wps[i](2) = waypoints_[i][2];

    //         end_pos_ = wps.back();
    //     }
    //     plan_manager_ptr_->setGlobalWaypoints(wps);
    //     bool success = plan_manager_ptr_->planGlobalTraj(odom_pos_);

    //     // for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    //     // {
    //     //     visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    //     //     ros::Duration(0.001).sleep();
    //     // }

    //     if (success)
    //     {

    //         /*** display ***/
    //         constexpr double step_size_t = 0.1;
    //         int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    //         std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    //         for (int i = 0; i < i_end; i++)
    //         {
    //             gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    //         }

    //         end_vel_.setZero();
    //         have_target_ = true;
    //         have_new_target_ = true;

    //         /*** FSM ***/
    //         // if (exec_state_ == WAIT_TARGET)
    //         changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    //         // else if (exec_state_ == EXEC_TRAJ)
    //         //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

    //         // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
    //         ros::Duration(0.001).sleep();
    //         visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    //         ros::Duration(0.001).sleep();
    //     }
    //     else
    //     {
    //         ROS_ERROR("Unable to generate global trajectory!");
    //     }
    // }

    void FSM::getLocalTarget()
    {
        double t;

        double t_step = planning_horizon_ / 20 / plan_manager_ptr_->pp_.max_vel_;
        double dist_min = 9999, dist_min_t = 0.0;
        for (t = plan_manager_ptr_->global_data_.last_progress_time_; t < plan_manager_ptr_->global_data_.global_duration_; t += t_step)
        {
            Eigen::Vector3d pos_t = plan_manager_ptr_->global_data_.getPosition(t);
            double dist = (pos_t - start_pos_).norm();

            if (t < plan_manager_ptr_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizon_)
            {
                // todo
                ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
                ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
                ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
                ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
                ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
                return;
            }
            if (dist < dist_min)
            {
                dist_min = dist;
                dist_min_t = t;
            }
            if (dist >= planning_horizon_)
            {
                local_target_pos_ = pos_t;
                plan_manager_ptr_->global_data_.last_progress_time_ = dist_min_t;
                break;
            }
        }
        if (t > plan_manager_ptr_->global_data_.global_duration_) // Last global point
        {
            local_target_pos_ = end_pos_;
        }

        if ((end_pos_ - local_target_pos_).norm() < (plan_manager_ptr_->pp_.max_vel_ * plan_manager_ptr_->pp_.max_vel_) / (2 * plan_manager_ptr_->pp_.max_acc_))
        {
            // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
            // cout << "A" << endl;
            local_target_vel_ = Eigen::Vector3d::Zero();
        }
        else
        {
            local_target_vel_ = plan_manager_ptr_->global_data_.getVelocity(t);
            // cout << "AA" << endl;
        }
    }

    void FSM::publishBspline()
    {
        auto info = &plan_manager_ptr_->local_data_;

        /* publish traj */
        trajectories::Bspline bspline;
        bspline.order = 3;
        bspline.start_time = info->start_time_;
        bspline.traj_id = info->traj_id_;

        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        bspline.pos_pts.reserve(pos_pts.cols());
        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info->position_traj_.getKnot();
        bspline.knots.reserve(knots.rows());
        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i));
        }

        bspline_pub_.publish(bspline);
    }
}