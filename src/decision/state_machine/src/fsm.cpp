#include <state_machine/fsm.h>


namespace my_planner
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

void FSM::execCallback(const ros::TimeEvent& event)
{
    static int fsm_num = 0;
    fsm_num++;
    // check state state
    if(fsm_num == 100)
    {
        printState();
        if(!have_odom_)) ROS_INFO_STREAM("[FSM]: no odom");
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
            start_pt_ = odom_pos_;
        }





}




}