#include <ros/ros.h>
#include <visualization_utils/map_visualizer.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;


ros::Publisher pub;
ros::Subscriber sub;    
MapVisualizer::Ptr map_visualizer;


void simcallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    vector<VisualBox> boxes;
    static int count = 0;
    if(count == 0)
    {
        boxes.push_back(VisualBox( Vector3d(2,2,0), Vector3d(2,2,2), Vector3d(2,2,2), 0));
        boxes.push_back( VisualBox( Vector3d(8,8,0), Vector3d(2,2,2), Vector3d(2,2,2), 0));
        boxes.push_back(VisualBox( Vector3d(3,3,0), Vector3d(2,2,2), Vector3d(2,2,2), 0));
        boxes.push_back(VisualBox( Vector3d(6,6,0), Vector3d(2,2,2), Vector3d(2,2,2), 0));
        map_visualizer->visualizeFutureBox(boxes);
        count ++;
    }
    else
    {
        boxes.push_back(VisualBox( Vector3d(-2,-2,0), Vector3d(2,2,2), Vector3d(2,2,2), 0));
        boxes.push_back( VisualBox( Vector3d(-8,-8,0), Vector3d(2,2,2), Vector3d(2,2,2), 0));
        boxes.push_back(VisualBox( Vector3d(-3,-3,0), Vector3d(2,2,2), Vector3d(2,2,2), 0));
        boxes.push_back(VisualBox( Vector3d(-6,-6,0), Vector3d(2,2,2), Vector3d(2,2,2), 0));
        map_visualizer->visualizeFutureBox(boxes);
    }


}   


int main(int argc, char** argv)
{
    ros::init(argc,argv, "map_test_node");

    ros::NodeHandle nh("~");

    map_visualizer.reset(new MapVisualizer());

    map_visualizer->init(nh);



    


    sub = nh.subscribe("/move_base_simple/goal", 1, simcallback);

    
    ros::spin();
    return 0;
}
