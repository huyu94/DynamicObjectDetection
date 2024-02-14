#include "plan_env/map_visualizer.h"
#include <queue>
#include <string>

MapVisualizer::MapVisualizer(const ros::NodeHandle &nh) : node_(nh)
{
    cluster_result_pub_ = node_.advertise<visualization_msgs::MarkerArray>("cluster_result", 1);
    segmentation_result_pub_ = node_.advertise<visualization_msgs::MarkerArray>("segmentation_result", 1);
    km_result_pub_ = node_.advertise<visualization_msgs::MarkerArray>("km_result", 1);
    moving_object_box_pub_ = node_.advertise<visualization_msgs::MarkerArray>("moving_object_box", 1);
    moving_object_traj_pub_ = node_.advertise<visualization_msgs::MarkerArray>("moving_object_traj", 1);
}


void MapVisualizer::visualizeClusterResult(std::vector<VisualCluster> &visual_cluster)
{
    visualization_msgs::MarkerArray marker_array;
    for (int i = 0; i < visual_cluster.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "cluster";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = visual_cluster[i].centroid(0);
        marker.pose.position.y = visual_cluster[i].centroid(1);
        marker.pose.position.z = visual_cluster[i].centroid(2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = visual_cluster[i].length(0);
        marker.scale.y = visual_cluster[i].length(1);
        marker.scale.z = visual_cluster[i].length(2);
        marker.color = Color::Green();
        marker_array.markers.push_back(marker);
    }
    cluster_result_pub_.publish(marker_array);
}