#include <visualization_utils/map_visualizer.h>
#include <queue>
#include <string>

MapVisualizer::MapVisualizer(const ros::NodeHandle &nh) : node_(nh)
{
    cluster_result_pub_ = node_.advertise<visualization_msgs::MarkerArray>("cluster_result", 1);
    segmentation_result_pub_ = node_.advertise<visualization_msgs::MarkerArray>("segmentation_result", 1);
    static_point_pub_ = node_.advertise<sensor_msgs::PointCloud2>("static_point", 1);
    km_result_pub_ = node_.advertise<visualization_msgs::MarkerArray>("km_result", 1);
    kalman_tracker_pub_ = node_.advertise<visualization_msgs::MarkerArray>("kalman_filter",1);
    slide_box_pub_ = node_.advertise<visualization_msgs::MarkerArray>("slide_box",1);
    moving_object_box_pub_ = node_.advertise<visualization_msgs::MarkerArray>("moving_object_box", 1);
    moving_object_traj_pub_ = node_.advertise<visualization_msgs::MarkerArray>("moving_object_traj", 1);
    receive_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("received_cloud",1);
}

void MapVisualizer::visualizeStaticPoint(std::vector<Eigen::Vector3d> &static_points)
{
    if(static_points.size() == 0)
    {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 cloud_msg;
    for(size_t i=0;i<static_points.size();i++)
    {
        pcl::PointXYZ p;
        p.x = static_points[i].x();
        p.y = static_points[i].y();
        p.z = static_points[i].z();
        cloud->points.push_back(p);
    }
    cloud->width = static_points.size();
    cloud->height = 1;
    cloud->header.frame_id = "world";

    pcl::toROSMsg(*cloud, cloud_msg);
    static_point_pub_.publish(cloud_msg);

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
    ROS_INFO_STREAM("finish publisher");
}
void MapVisualizer::visualizeSegmentationResult(std::vector<VisualCluster> &visual_clusters)
{

    visualization_msgs::MarkerArray bboxs;

    for(size_t i = 0; i < visual_clusters.size(); i++)
    {
        Eigen::Vector3d p, v, s, box_min, box_max;
        p = visual_clusters[i].centroid;
        v = visual_clusters[i].velocity;
        s = visual_clusters[i].length;
        box_min = visual_clusters[i].min_bound;
        box_max = visual_clusters[i].max_bound;


/* segmentation result does not have vel infor */
        // 创建立方体框
        bboxs.markers.push_back(generateBBox(box_min,box_max,i));
        // 创建箭头
        // bboxs_and_arrows.markers.push_back(generateArrows(p,v,i));
    }

    segmentation_result_pub_.publish(bboxs);
}

void MapVisualizer::visualizeKalmanTracker(std::vector<VisualKalmanTracker> &visual_trackers)
{
    visualization_msgs::MarkerArray ellipses_and_arrows;
    for(size_t i=0; i<visual_trackers.size();i++)
    {
        Eigen::Vector3d p,v,l;
        int id;
        p = visual_trackers[i].pos;
        v = visual_trackers[i].vel;
        l = visual_trackers[i].len;
        id = visual_trackers[i].id;
        visualization_msgs::Marker ellipse,arrow;
        ellipse = generateEllipse(p,l,id);
        arrow = generateArrows(p,v,id);

        ellipses_and_arrows.markers.push_back(ellipse);
        ellipses_and_arrows.markers.push_back(arrow);
    }

    kalman_tracker_pub_.publish(ellipses_and_arrows);

}


void MapVisualizer::visualizeSlideBox(std::vector<VisualizeSlideBox> &visual_slideboxes)
{
    visualization_msgs::MarkerArray slide_boxes;
    for(size_t i=0; i<visual_slideboxes.size();i++)
    {
        visualization_msgs::Marker box = generateCube(visual_slideboxes[i].center,visual_slideboxes[i].length,visual_slideboxes[i].rotation,visual_slideboxes[i].id);
        slide_boxes.markers.push_back(box);
    }
    slide_box_pub_.publish(slide_boxes);
}

visualization_msgs::Marker generateEllipse(const Vector3d &pos, const Vector3d &len, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "tracker";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = pos.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = len.x();
    marker.scale.y = len.y();
    marker.scale.z = len.z();
    marker.color = Color::Yellow();
    return marker;
}
visualization_msgs::Marker generateArrows(const Vector3d &pos, const Vector3d &vel, int id)
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
    arrow.scale.y = 0.1; // Head diameter
    arrow.scale.z = 0.1; // Head length
    arrow.color = Color::Blue();

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

visualization_msgs::Marker generateCube(const Eigen::Vector3d &center, const Eigen::Vector3d &length, const Eigen::Matrix3d &rotation, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();

    marker.ns = "SlideBox";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();

    Eigen::Quaterniond q(rotation);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = length.x() * 2;
    marker.scale.y = length.y() * 2;
    marker.scale.z = length.z() * 2;


    marker.color = Color::Purple();

    return marker;
}

