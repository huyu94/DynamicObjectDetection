#include <ros/ros.h>
#include <plan_manager/plan_manager.h>
#include <geometry_msgs/PoseStamped.h>

using namespace fast_planner;
PlanManager::Ptr plan_manager_ptr_;



void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    Vector3d goal(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);

    plan_manager_ptr_->kinodynamicReplan(Vector3d(0,0,0), Vector3d(0,0,0), Vector3d(0,1,1), goal, Vector3d(0,0,0));


}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "plan_manager_test_ndoe");

    ros::NodeHandle nh("~");

    ROS_INFO_STREAM("plan_manager_test_node start!");

    plan_manager_ptr_.reset(new PlanManager);
    plan_manager_ptr_->initPlanModules(nh);

    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 1000, goalCallback);
    


    ros::spin();


    return 0;
}