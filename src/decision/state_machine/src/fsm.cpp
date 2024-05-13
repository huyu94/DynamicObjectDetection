#include <state_machine/fsm.h>


namespace fast_planner
{
FSM::FSM(/* args */)
{}

FSM::~FSM()
{}

void FSM::init(ros::NodeHandle& nh)
{
 
    // ros
    node_ = nh;

    // params 

    // function classes 




}


void FSM::changeState(FSM::MACHINE_STATE state, string pos_call)
{
    string state_str[6] = { "INIT", "WAIT_GOAL", "GENERATE_TRAJ", "FOLLOW_TRAJ", "REPLAN_TRAJ","EMERGENCY_STOP" };
    ROS_INFO_STREAM("[FSM]: change from " << state_str[int(machine_state_)] << " to " << state_str[int(state)] << " by " << pos_call);

}


void FSM::printState()
{
    string state_str[6] = { "INIT", "WAIT_GOAL", "GENERATE_TRAJ", "FOLLOW_TRAJ", "REPLAN_TRAJ","EMERGENCY_STOP" };
    ROS_INFO_STREAM("[FSM]: state : " << state_str[int(machine_state_)]);    
}

void FSM::execCallback(const ros::TimerEvent& event)
{
    static int fsm_num = 0;
    fsm_num++;
    // check state state
    if(fsm_num == 100)
    {
        printState();
        if(!have_odom_) ROS_INFO_STREAM("[FSM]: no odom");
        if(!trigger_) ROS_INFO_STREAM("[FSM]: no trigger");
        fsm_num = 0;
    }

    switch (machine_state_)
    {
        case INIT:
        {
            if(!have_odom_)
            {
                return ;
            }
            if(!trigger_)
            {
                return ;
            }
            changeState(WAIT_GOAL, "FSM");
            break;
        }

        case WAIT_GOAL: 
        {
            if(!have_goal_)
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

            Vector3d rot_x = odom_orient_.toRotationMatrix().block(0,0,3,1);
            start_yaw_(0)  = atan2(rot_x(1), rot_x(0));
            start_yaw_(1) = start_yaw_(2) = 0.0;

            bool success = callKinoDynamicReplan();

            if(success)
            {
                changeState(FOLLOW_TRAJ, "FSM");
            }
            else
            {
                changeState(GENERATE_TRAJ, "FSM");
            }
            break;
        }

        // case FOLLOW_TRAJ:
        // {
        //     LocalTrajData* info = &state_machine->local_data_;
        //     ros::Time       time_now = ros::Time::now();
        //     double          t_cur    = (time_now - info->start_time_).toSec();
        //     t_cur                    = min(info->duration_, t_cur);

        //     Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);


        //     if(t_cur >= info->duration_ - 1e-2)
        //     {
        //         have_goal_ = false;
        //         changeState(WAIT_GOAL, "FSM");
        //         return ;
        //     }
        //     else if((end_pos_ - pos).norm() < no_replan_thresh_)
        //     {
        //         ROS_INFO_STREAM("[FSM]: near local traj end");
        //         return ;
        //     }
        //     else if((info->start_pos_ - pos).norm() < replan_thresh_)
        //     {
        //         ROS_INFO_STREAM("near start");
        //         return ;
        //     }
        //     else
        //     {
        //         changeState(REPLAN_TRAJ, "FSM");
        //     }
        //     break;
        // }

        // case REPLAN_TRAJ:
        // {
        //     LocalTrajData*  info        = &state_machine->local_data_;
        //     ros::Time       time_now    = ros::Time::now();
        //     double          t_cur       = (time_now - info->start_time_).toSec();

        //     start_pos_ = info->position_traj_.evaluateDeBoorT(t_cur);
        //     start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
        //     start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

        //     start_yaw(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
        //     start_yaw(1) = info->yaw_rate_traj_.evaluateDeBoorT(t_cur)[0];
        //     start_yaw(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];

        //     // std_msgs::Empty replan_msg;
        //     // replan_pub_.publish(replan_msg);

        //     bool success = callKinodynamicReplan();
        //     if(success)
        //     {
        //         changeState(FOLLOW_TRAJ,"FSM");
        //     }
        //     else
        //     {
        //         changeState(GENERATE_TRAJ, "FSM");
        //     }

        //     break;
    }

}

bool FSM::callKinoDynamicReplan()
{
    // bool plan_success = 
}




}