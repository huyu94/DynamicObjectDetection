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
    receive_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("received_cloud",1);
}



void MapVisualizer::visualizeReceiveCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    receive_cloud_pub_.publish(cloud_msg);
}

void MapVisualizer::visualizeClusterResult(std::vector<VisualCluster> &visual_clusters)
{
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < visual_clusters.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.header.stamp = ros::Time::now();
        marker.ns = "cluster";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = visual_clusters[i].centroid(0);
        marker.pose.position.y = visual_clusters[i].centroid(1);
        marker.pose.position.z = visual_clusters[i].centroid(2);
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = visual_clusters[i].length(0);
        marker.scale.y = visual_clusters[i].length(1);
        marker.scale.z = visual_clusters[i].length(2);
        marker.color = Color::Green();
        marker_array.markers.push_back(marker);
    }
    cluster_result_pub_.publish(marker_array);
}
void MapVisualizer::visualizeSegmentationResult(std::vector<VisualCluster> &visual_clusters)
{

    visualization_msgs::MarkerArray bboxs_and_arrows;

    for(size_t i = 0; i < visual_clusters.size(); i++)
    {
        Eigen::Vector3d p, v, s, box_min, box_max;
        p = visual_clusters[i].centroid;
        v = visual_clusters[i].velocity;
        s = visual_clusters[i].length;
        box_min = visual_clusters[i].min_bound;
        box_max = visual_clusters[i].max_bound;


        // 创建立方体框
        bboxs_and_arrows.markers.push_back(generateBBox(box_min,box_max,i));
        // 创建箭头
        bboxs_and_arrows.markers.push_back(generateArrows(p,v,i));
        

    }

    segmentation_result_pub_.publish(bboxs_and_arrows);
}

visualization_msgs::Marker generateArrows(const Vector3d &pos,const Vector3d &vel,int id)
{
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = ros::Time::now();
    arrow.ns = "arrow";
    arrow.id = id;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point start,end;
    start.x = pos.x();
    start.y = pos.y();
    start.z = pos.z();

    end.x = pos.x() + vel.x();
    end.y = pos.y() + vel.y();
    end.z = pos.z() + vel.z();

    arrow.points.push_back(start);
    arrow.points.push_back(end);
    arrow.pose.orientation.x = 0.0;
    arrow.pose.orientation.y = 0.0;
    arrow.pose.orientation.z = 0.0;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.1; // Shaft diameter
    arrow.scale.y = 0.2; // Head diameter
    arrow.scale.z = 0.5; // Head length
    arrow.color = Color::Green();

    return arrow;
}

visualization_msgs::Marker generateBBox(const Eigen::Vector3d &min_point, const Eigen::Vector3d &max_point,int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    marker.ns = "bbox";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.scale.x = 0.1;
    marker.color = Color::Orange();

    geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
    p1.x = min_point.x(); p1.y = min_point.y(); p1.z = min_point.z();
    p2.x = max_point.x(); p2.y = min_point.y(); p2.z = min_point.z();
    p3.x = max_point.x(); p3.y = max_point.y(); p3.z = min_point.z();
    p4.x = min_point.x(); p4.y = max_point.y(); p4.z = min_point.z();
    p5.x = min_point.x(); p5.y = min_point.y(); p5.z = max_point.z();
    p6.x = max_point.x(); p6.y = min_point.y(); p6.z = max_point.z();
    p7.x = max_point.x(); p7.y = max_point.y(); p7.z = max_point.z();
    p8.x = min_point.x(); p8.y = max_point.y(); p8.z = max_point.z();
    // Lines along x axis
    marker.points.push_back(p1); marker.points.push_back(p2);
    marker.points.push_back(p4); marker.points.push_back(p3);
    marker.points.push_back(p5); marker.points.push_back(p6);
    marker.points.push_back(p8); marker.points.push_back(p7);

    // Lines along y axis
    marker.points.push_back(p1); marker.points.push_back(p4);
    marker.points.push_back(p2); marker.points.push_back(p3);
    marker.points.push_back(p5); marker.points.push_back(p8);
    marker.points.push_back(p6); marker.points.push_back(p7);

    // Lines along z axis
    marker.points.push_back(p1); marker.points.push_back(p5);
    marker.points.push_back(p2); marker.points.push_back(p6);
    marker.points.push_back(p3); marker.points.push_back(p7);
    marker.points.push_back(p4); marker.points.push_back(p8);

    return marker;
}