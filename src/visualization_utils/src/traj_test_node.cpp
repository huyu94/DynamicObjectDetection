#include <ros/ros.h>
#include <ros/subscriber.h>
#include <Eigen/Eigen>

#include <visualization_utils/traj_visualizer.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

static int id = 0;

ros::Publisher pub;
ros::Subscriber sub;

using Vector3d = Eigen::Vector3d;
TrajVisualizer::Ptr visualizer;

Vector3d start(0,0,0);
Vector3d goal(5,5,5);
std::vector<Vector3d> kino_paths;
std::vector<std::vector<Vector3d>> multi_topo_paths;
std::vector<bool> success;


void simcallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    goal.x() = msg->pose.position.x;
    goal.y() = msg->pose.position.y;
    goal.z() = msg->pose.position.z;
    // visualizer->visualizeStartAndGoal(start,goal,ros::Time::now());
    visualizer->visualizeCollision(goal,ros::Time::now());
    visualizer->visualizeKinodynamicTraj(kino_paths,ros::Time::now());
    visualizer->visualizeMultiTopoTrajs(multi_topo_paths,success,ros::Time::now());
}   


int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "test_node"); 
    ros::NodeHandle nh("~");
    visualizer.reset(new TrajVisualizer);
    visualizer->init(nh);

    kino_paths.push_back(Vector3d(0,0,0));
    kino_paths.push_back(Vector3d(1,1,1));
    kino_paths.push_back(Vector3d(2,2,3));
    kino_paths.push_back(Vector3d(4,5,6));

    multi_topo_paths.push_back(kino_paths);
    success.push_back(true);

    std::vector<Vector3d> topo_path;
    topo_path.push_back(Vector3d(0,0,0));
    topo_path.push_back(Vector3d(-1,-1,-1));
    topo_path.push_back(Vector3d(-2,-2,-3));
    multi_topo_paths.push_back(topo_path);
    success.push_back(true);

    topo_path.clear();
    topo_path.push_back(Vector3d(0,0,0));
    topo_path.push_back(Vector3d(-1,1,-1));
    topo_path.push_back(Vector3d(-2,3,-3));
    topo_path.push_back(Vector3d(-3,4,-6));
    multi_topo_paths.push_back(topo_path);
    success.push_back(true);
    topo_path.clear();

    sub = nh.subscribe("/move_base_simple/goal", 1, simcallback);

    
    
    ros::spin();

    return 0;
}