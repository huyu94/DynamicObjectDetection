#include <state_machine/fsm.h>
#include <state_machine/replan_fsm.h>


#include <ros/ros.h>
#include <plan_manager/plan_manager.h>


using namespace fast_planner;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "state_machine_node");
    ros::NodeHandle nh("~");

    // FSM::Ptr fsm_ptr;
    // fsm_ptr.reset(new FSM());
    // fsm_ptr->init(nh);
    // PlanManager::Ptr plan_manager_ptr;
    // plan_manager_ptr.reset(new PlanManager());
    // plan_manager_ptr->initPlanModules(nh);

    // FSM kino_replan;
    // kino_replan.init(nh);
    TopoReplanFSM topo_replan;
    topo_replan.init(nh);


    ros::Duration(1.0).sleep();
    // ros::MultiThreadedSpinner spinnner(4);
    // spinnner.spin();
    ros::spin();

    return 0;
}
