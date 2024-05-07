#include <state_machine/fsm.h>
#include <ros/ros.h>



using namespace my_planner;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_machine_node");
    ros::NodeHandle nh("~");

    FSM fsm;
    fsm.init(nh);

    FSM kino_replan;
    kino_replan.init(nh);


    ros::Duration(1.0).sleep();
    ros::spin();

    return 0;
}
