// #include <plan_manage/topo_replan_fsm.h>
#include <state_machine/replan_fsm.h>

void TopoReplanFSM::init(ros::NodeHandle &nh)
{
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
    nh.param("fsm/planning_horizon_time", planning_horizen_time_, -1.0);
    nh.param("fsm/emergency_time_", emergency_time_, 1.0);
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);
    nh.param("fsm/waypoint_num", waypoints_num_, -1);

    have_trigger_ = !flag_realworld_experiment_;


    for (int i = 0; i < waypoints_num_; i++)
    {
        nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
        nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    // visualization_.reset(new PlanningVisualization(nh));
    planner_manager_.reset(new PlanManager);
    planner_manager_->initPlanModules(nh);

    /* callback */
    exec_timer_ = nh.createTimer(ros::Duration(0.05), &TopoReplanFSM::execFSMCallback, this);
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &TopoReplanFSM::checkCollisionCallback, this);

    odom_sub_ = nh.subscribe("odom", 1, &TopoReplanFSM::odometryCallback, this);

    bspline_pub_ = nh.advertise<trajectories::Bspline>("bspline", 10);

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
    {
        waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &TopoReplanFSM::waypointCallback, this);
        // ROS_WARN("TARGET_TYPE::MANUAL_TARGET");
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
        trigger_sub_ = nh.subscribe("traj_start_trigger", 1, &TopoReplanFSM::triggerCallback, this);

        ROS_INFO("wait for 1 second.");
        int count = 0;
        while(ros::ok() && count++ < 1000)
        {
            ros::spinOnce();
            ros::Duration(0.001).sleep();
        }

        ROS_WARN("Waiting for trigger from [n3ctrl] from RC");    

        while(ros::ok() && (!have_odom_ || !have_trigger_))
        {
            ros::spinOnce();
            ros::Duration(0.001).sleep();
        }

        readGivenWps();
    }
    else
    {
        cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
    }
}

void TopoReplanFSM::readGivenWps()
{
    if(waypoints_num_ <= 0)
    {
        ROS_ERROR("Wrong waypoint_num_ = %d", waypoints_num_);
        return;
    }


    wps_.resize(waypoints_num_);
    for (int i = 0; i < waypoints_num_; i++)
    {
        wps_[i](0) = waypoints_[i][0];
        wps_[i](1) = waypoints_[i][1];
        wps_[i](2) = waypoints_[i][2];

      // end_pt_ = wps_.back();
    }

    for(size_t i = 0; i < (size_t) waypoints_num_; i++)
    {
        // visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0,0.5,0.5,1), 0.3, i);
        ros::Duration(0.001).sleep();
    }

    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);
}


void TopoReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
{
    bool success = false;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), next_wp, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
        end_pt_ = next_wp;
        /*** display ***/
        constexpr double step_size_t = 0.1;
        int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
        vector<Eigen::Vector3d> gloabl_traj(i_end);
        for (int i = 0; i < i_end; i++)
        {
            gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
        }

        end_vel_.setZero();
        have_target_ = true;
        have_new_target_ = true;

        /*** FSM ***/
        if (exec_state_ == WAIT_TARGET) // 如果是等待目标状态，那么直接生成新轨迹
        {
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
        }
        else if (exec_state_ == EXEC_TRAJ) // 如果是执行状态，就转换到重新规划状态
        {
            changeFSMExecState(REPLAN_TRAJ, "TRIG");
        }

        // visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
        ROS_ERROR("Unable to generate global trajectory!");
    }

}

void TopoReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
}

void TopoReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    if (msg->pose.position.z < -0.1)
    {
        return;
    }

    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;

    Eigen::Vector3d end_wp(msg->pose.position.x,msg->pose.position.y, 1.0);
    // visualization_->displayGoalPoint(end_wp,Eigen::Vector4d(1,0,0,1),0.5,100);
    planNextWaypoint(end_wp);

}

void TopoReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
{
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    // odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
}

void TopoReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
{
    if (new_state == exec_state_)
    {
        continously_called_times_++;
    }
    else
    {
        continously_called_times_ = 1;
    }

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
}

std::pair<int, TopoReplanFSM::FSM_EXEC_STATE> TopoReplanFSM::timesOfConsecutiveStateCalls()
{
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
}

void TopoReplanFSM::printFSMExecState()
{
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
}

void TopoReplanFSM::execFSMCallback(const ros::TimerEvent &e)
{
    static int fsm_num = 0;
    fsm_num++;

    if (fsm_num == 100)
    {
        printFSMExecState();
        if (!have_odom_)
            cout << "no odom." << endl;
        if (!have_target_)
            cout << "wait for goal." << endl;
        fsm_num = 0;
    }
    switch (exec_state_)
    {
    case INIT:
    {
        if (!have_odom_)
        {
            return;
        }
        changeFSMExecState(WAIT_TARGET, "FSM");
        break;
    }

    case WAIT_TARGET:
    {
        if (!have_target_)
        {
            return;
        }
        else
        {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
    }

    case GEN_NEW_TRAJ:
    {
        start_pt_ = odom_pos_;
        start_vel_ = odom_vel_;
        start_acc_.setZero();

        bool flag_random_poly_init;
        if (timesOfConsecutiveStateCalls().first == 1)
        {
            flag_random_poly_init = false;
        }
        else
        {
            flag_random_poly_init = true;
        }


        bool success = callReboundReplan(true,flag_random_poly_init);


        if (success)
        {
            changeFSMExecState(EXEC_TRAJ, "FSM");
            flag_escape_emergency_ = true;
        }
        else
        {
            changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        }
        break;
    }

    case REPLAN_TRAJ:
    {

        if(planFromCurrentTraj())
        {
            changeFSMExecState(EXEC_TRAJ, "FSM");
        }
        else
        {
            ROS_INFO("plan failed in [replan_traj]");
            changeFSMExecState(REPLAN_TRAJ,"FSM");
        }
        break;
    }

    case EXEC_TRAJ:
    {
        LocalTrajData *info = &planner_manager_->local_data_;
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - info->start_time_).toSec();
        t_cur = min(info->duration_, t_cur);

        Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

        /* && (end_pt_ - pos).norm() < 0.5 */
        if (t_cur > info->duration_ - 1e-1)
        {
            have_target_ = false;

            changeFSMExecState(WAIT_TARGET, "FSM");
            return;
        }
        else if ((end_pt_ - pos).norm() < no_replan_thresh_) // 接近终点
        {
            return;
        }
        else if ((info->start_pos_ - pos).norm() < replan_thresh_) // 接近起点
        {
            // cout << "near start" << endl;
            return;
        }
        else
        {
            ROS_INFO("from state [EXEC_TRAJ]");
            changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
        break;
    }

    case EMERGENCY_STOP:
    {
        if (flag_escape_emergency_)
        {
            callEmergencyStop(odom_pos_);
        }
        else
        {
            if (odom_vel_.norm() < 0.1)
            {
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
        }
        flag_escape_emergency_ = false;
        break;
    }

        // data_disp_.header.stamp = ros::Time::now();
        // data_disp_pub_.publish(data_disp_);
    }
}

bool TopoReplanFSM::planFromCurrentTraj()
{

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    // cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false); // 在原轨迹基础制上rebound

    if (!success)
    {
        // success = callReboundReplan(true, false); // 初始化一条新轨迹rebound， 不生成中间路径点
        // // changeFSMExecState(EXEC_TRAJ, "FSM");
        // if (!success)
        // {
        //     success = callReboundReplan(true, true); // 初始化一条新轨迹rebound, 生成中间路径点
        //     if (!success)
        //     {
        //         return false;
        //     }
        // }
        return false;
    }

    return true;
}

void TopoReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
{
    LocalTrajData *info = &planner_manager_->local_data_;
    auto env_manager = planner_manager_->env_manager_ptr_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
    {
        return;
    }
    static int fail_times = 0;
    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    // int traj_risk_step = 0;
    double traj_risk = 0;
    /*
    1. if in inflate map, emergency stop
    2. if traj risk > thresh, replan
    */
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
        // traj_risk_step++;
        if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
            break;
        
        Eigen::Vector3d pt = info->position_traj_.evaluateDeBoorT(t);
        
        // if(traj_risk_step >= 10)
        // {
        //     traj_risk_step = 0;
        // }
        // traj_risk += map->getVoxelFutureRisk(pt);


        if(env_manager->checkCollisionInGridMap(pt))
        {
            if(planFromCurrentTraj())
            {
                changeFSMExecState(EXEC_TRAJ, "SAFETY");
                return ;
            } 
            else
            {
                /* 不紧急停止，直接重新规划轨迹 */
                if (t - t_cur < emergency_time_) // 0.8s of emergency time
                {
                    ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
                    fail_times ++;
                    ROS_ERROR("fail_times: %d",fail_times);
                    changeFSMExecState(EMERGENCY_STOP, "SAFETY");           
                }
                else
                {
                    ROS_WARN("current traj in collision, replan.");
                    changeFSMExecState(REPLAN_TRAJ, "SAFETY");
                }
                return ;
            }
            break;
        }
        // // ROS_INFO("traj_risk=%f",traj_risk);
        // if(traj_risk > planner_manager_->pp_.traj_risk_thresh_)
        // {
        //     ROS_WARN("current traj risk is too high, replan.");
        //     changeFSMExecState(REPLAN_TRAJ, "SAFETY");
        //     return ;
        // }
    }
}



bool TopoReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
{
    if(mutex_call_rebound_replan_.try_lock())
    {
        getLocalTarget();

        bool plan_success =
            planner_manager_->kinodynamicReplan(start_pt_,start_vel_,start_acc_,local_target_pt_,local_target_vel_);
            // planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
        have_new_target_ = false;

        cout << "final_plan_success=" << plan_success << endl;

        if (plan_success)
        {

            auto info = &planner_manager_->local_data_;

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
            // ROS_INFO("publish bspline");
            // visualization_->displayOptimalList(info->position_traj_.getControlPoint(), 0);
        }
        mutex_call_rebound_replan_.unlock();
        return plan_success;
    }
    else
    {
        return false;
    }
    

}

bool TopoReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
{

    planner_manager_->emergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

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

    return true;
}

void TopoReplanFSM::getLocalTarget()
{
    double t;
    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    double dist_min = 9999, dist_min_t = 0.0;

    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
        Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
        double dist = (pos_t - start_pt_).norm();
        if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
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
        if (dist >= planning_horizen_)
        {
            local_target_pt_ = pos_t;
            planner_manager_->global_data_.last_progress_time_ = dist_min_t;
            break;
        }
    }

    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
        local_target_pt_ = end_pt_;
    }

    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
        // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
        // cout << "A" << endl;
        local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
        local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
        // cout << "AA" << endl;
    }
}