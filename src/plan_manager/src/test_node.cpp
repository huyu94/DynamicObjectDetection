#include <ros/ros.h>
#include <plan_manager/plan_manager.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace fast_planner;
PlanManager::Ptr plan_manager_ptr_;
nav_msgs::Odometry odom;



void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    Vector3d goal(msg->pose.position.x,msg->pose.position.y,0.6);
    Vector3d pos(odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z);
    plan_manager_ptr_->kinodynamicReplan(pos, Vector3d(0,0,0), Vector3d(0,1,1), goal, Vector3d(0,0,0));

    //2. global planning
    // vector<Vector3d> waypoints;
    // waypoints.push_back(Vector3d(4,4,9));
    // waypoints.push_back(Vector3d(10,9,8));
    // waypoints.push_back(Vector3d(15,15,10));

    // plan_manager_ptr_->setGlobalWaypoints(waypoints);
    // Vector3d zero(0,0,0);
    // plan_manager_ptr_->planGlobalTraj(pos);

    // double duration = plan_manager_ptr_->global_data_.global_duration_;
    // ROS_INFO_STREAM("global duration: " << duration);

    // vector<Vector3d> traj;
    // for(double t = 0.0; t < duration - 1e-4; t += 0.1)
    // {
    //     traj.push_back(plan_manager_ptr_->global_data_.getPosition(t));
    // }
    // ROS_INFO_STREAM("traj size : " << traj.size());

    // plan_manager_ptr_->traj_visual_ptr_->visualizeKinodynamicTraj(traj,false,true);

}

void odomcallback(const nav_msgs::OdometryConstPtr& msg)
{
    odom = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc,argv, "plan_manager_test_ndoe");

    ros::NodeHandle nh("~");

    ROS_INFO_STREAM("plan_manager_test_node start!");

    plan_manager_ptr_.reset(new PlanManager);
    plan_manager_ptr_->initPlanModules(nh);

    ros::Subscriber sub = nh.subscribe("/move_base_simple/goal", 1000, goalCallback);
    ros::Subscriber odom_sub = nh.subscribe("/drone_0/odometry",1,odomcallback);


    ros::spin();


    return 0;
}
