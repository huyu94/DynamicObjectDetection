#include <ros/ros.h>
#include <ros/subscriber.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

static int id = 0;

ros::Publisher pub;
ros::Subscriber sub;

using Vector3d = Eigen::Vector3d;
using MatrixXd = Eigen::MatrixXd;

void simcallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    ROS_INFO("I heard: [%f]", msg->pose.position.x);
    

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.header.stamp = ros::Time::now();
    marker.id = id++;

    marker.pose.position.x = msg->pose.position.x;
    marker.pose.position.y = msg->pose.position.y;
    marker.pose.position.z = msg->pose.position.z;


    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    
    marker.pose.orientation.w = 1.0;

    pub.publish(marker);
}


int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "test_node"); 
    ros::NodeHandle nh("~");

    sub = nh.subscribe("/move_base_simple/goal", 1, simcallback);
    pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    
    MatrixXd list(3,10);
    list << 1,2,3,
            4,5,6,
            7,8,9,
            10,11,12,
            13,14,15,
            16,17,18,
            19,20,21,
            22,23,24,
            25,26,27,
            28,29,30;
    for(int i=0;i<list.cols();i++)
    {
        for(int j=0;j < 3 ; j++)
        {
            ROS_INFO("list[%d]: %f", i, list.col(i)[j]);
        }
    }
    
    ros::spin();

    return 0;
}