#include <ros/ros.h>
#include <plan_manager/plan_manager.h>
#include <geometry_msgs/PoseStamped.h>

using namespace fast_planner;

void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{

}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "plan_manager_test_ndoe");

    ros::NodeHandle nh("~");

    ROS_INFO_STREAM("plan_manager_test_node start!");

    PlanManager plan_manager;

    ros::Subscriber sub = nh.subscribe("pose_topic", 1000, goalCallback);

    ros::spin();


    return 0;
}
