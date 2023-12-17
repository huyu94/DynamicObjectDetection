#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <munkres.h>
#include <dynamic_tracker/dbscan_cluster.hpp>

// TODO: Include necessary headers for clustering methods

using namespace std;

int cluster_type_;
double epsilon_;
int min_points_;
float sensing_rate_;
bool need_update_{false};
bool use_adaptive_dbscan_{false};

string frame_id;
Eigen::Vector3d position_;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

ros::Publisher aabb_pub_;


struct ClusterFeature{
    Eigen::Vector3d centroid;
    Eigen::Vector3d velocity;   
};
vector<ClusterFeature> cluster_features_last_;

void dbscan_cluster(std::vector<pcl::PointIndices> &cluster_indices)
{
    cluster_indices.clear();
    pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    for(auto & p : cloud_->points)
    {
        pcl::PointXYZI p_i;
        p_i.x = p.x;
        p_i.y = p.y;
        p_i.z = p.z;
        p_i.intensity = (Eigen::Vector3d(p.x,p.y,p.z) - position_).norm();
        latest_cloud->points.push_back(p_i);
    }


    DBSCANCluster dbscan_cluster;
    dbscan_cluster.setInputCloud(latest_cloud);
    dbscan_cluster.run(cluster_indices,use_adaptive_dbscan_);


}

void euclidean_cluster(std::vector<pcl::PointIndices> &cluster_indices)
{
    cluster_indices.clear();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (epsilon_);
    ec.setMinClusterSize (5);
    ec.setMaxClusterSize (10000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_);
    ec.extract (cluster_indices);
}

void generate_AABB(std::vector<pcl::PointIndices> &cluster_indices,vector<pair<pcl::PointXYZ,pcl::PointXYZ>> & AABB_list)
{
    ROS_INFO("clustet_indices size : %zu" , cluster_indices.size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    for(size_t i = 0; i <cluster_indices.size();i++){
        cloud_cluster->clear();
        for(auto &index : cluster_indices[i].indices)
        {
            cloud_cluster->points.push_back(cloud_->points[index]);
        }
        pcl::PointXYZ minPt,maxPt;
        pcl::getMinMax3D(*cloud_cluster,minPt,maxPt);
        AABB_list.emplace_back(make_pair(minPt,maxPt));
    }
}

void publsihAABB(vector<pair<pcl::PointXYZ,pcl::PointXYZ>> & AABB_list)
{
    visualization_msgs::MarkerArray aabb_array_;

    aabb_array_.markers.clear();
    aabb_array_.markers.reserve(AABB_list.size());
    for(size_t i = 0; i < AABB_list.size();i++)
    {
        visualization_msgs::Marker box;
        box.header.stamp = ros::Time::now();
        box.header.frame_id = frame_id;
        box.type = visualization_msgs::Marker::CUBE;
        box.action = visualization_msgs::Marker::ADD;

        box.pose.position.x = (AABB_list[i].first.x + AABB_list[i].second.x)/2;
        box.pose.position.y = (AABB_list[i].first.y + AABB_list[i].second.y)/2;
        box.pose.position.z = (AABB_list[i].first.z + AABB_list[i].second.z)/2;

        box.pose.orientation.x = 0;
        box.pose.orientation.y = 0;
        box.pose.orientation.z = 0;
        box.pose.orientation.w = 1;

        box.scale.x = (AABB_list[i].second.x - AABB_list[i].first.x);
        box.scale.y = (AABB_list[i].second.y - AABB_list[i].first.y);
        box.scale.z = (AABB_list[i].second.z - AABB_list[i].first.z);
        box.color.r = 1.0;
        box.color.g = 0.0;
        box.color.b = 1.0;
        box.color.a = 0.5;
        box.id = i;
        aabb_array_.markers.push_back(box);
    }

    aabb_pub_.publish(aabb_array_);

}


void velocity_estimation(std::vector<pcl::PointIndices> &cluster_indices)
{
    vector<ClusterFeature> cluster_features;
    for(const auto & cluster_indices : cluster_indices)
    {
        ClusterFeature cluster_feature;
        cluster_feature.centroid = Eigen::Vector3d::Zero();
        cluster_feature.velocity = Eigen::Vector3d::Zero();
        for(const auto & index : cluster_indices.indices)
        {
            cluster_feature.centroid += Eigen::Vector3d(cloud_->points[index].x,cloud_->points[index].y,cloud_->points[index].z);
        }
        cluster_feature.centroid /= cluster_indices.indices.size();
        cluster_features.push_back(cluster_feature);
    }

    if(cluster_features_last_.empty() || cluster_features.empty())
    {
        cluster_features_last_ = cluster_features;
        return ;
    }
    double distance_gate_ = 5.f;
    Matrix<double> matrix_cost(cluster_features.size(),cluster_features_last_.size());
    Matrix<bool> matrix_gate(cluster_features.size(),cluster_features_last_.size());
    for(size_t i = 0; i < cluster_features.size();i++)
    {
        for(size_t j = 0; j < cluster_features_last_.size();j++)
        {
            double feaureDistance = (cluster_features[i].centroid - cluster_features_last_[j].centroid).norm();
            matrix_cost(i,j) = feaureDistance < distance_gate_ ? feaureDistance : distance_gate_ * 5000;
            matrix_gate(i,j) = matrix_cost(i,j) < distance_gate_;
        }
    }

    Munkres<double> munkres_solver;
    munkres_solver.solve(matrix_cost);

    for(size_t i = 0; i < cluster_features.size();i++)
    {
        for(size_t j = 0; j < cluster_features_last_.size();j++)
        {
            if(matrix_cost(i,j) == 0.0f && matrix_gate(i,j))
            {
                cluster_features[i].velocity = cluster_features[i].centroid - cluster_features_last_[j].centroid / sensing_rate_;
                if(cluster_features[i].velocity.norm() > 10.0f)
                {
                    cluster_features[i].velocity = cluster_features_last_[j].velocity;
                }
                break;
            }
        }
    }
    cluster_features_last_ = cluster_features;

    for(auto & cluster : cluster_features_last_)
    {
        ROS_INFO("cluster velocity : %f %f %f",cluster.velocity.x(),cluster.velocity.y(),cluster.velocity.z());
    }



}

void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const nav_msgs::OdometryConstPtr& odom_msg)
{
    // ROS_INFO("here");
    // 1. convert ros msg to pcl pointcloud && preprocess
    pcl::fromROSMsg(*cloud_msg, *cloud_);
    position_ = Eigen::Vector3d(odom_msg->pose.pose.position.x,odom_msg->pose.pose.position.y,odom_msg->pose.pose.position.z);
    // ROS_INFO("cloud size : %d ",cloud_->points.size()   );
    // 2. 去除地面点
    pcl::ExtractIndices<pcl::PointXYZ> cliper;
    cliper.setInputCloud(cloud_);
    pcl::PointIndices indices;
    for (size_t i = 0; i < cloud_->points.size(); i++)
    {
        if (cloud_->points[i].z < 0.1)
        {
            indices.indices.push_back(i);
        }
    }
    /* 若setNegative(true)，filter输出不在索引里面的点,也可以用getRemoveIndices取得索引的补集。*/
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);
    cliper.filter(*cloud_);
    
    need_update_ = true;

}

void updateCallback(const ros::TimerEvent &)
{  
    if(!need_update_)
    {
        return ;
    }

    // 3. cluster
    vector<pcl::PointIndices> cluster_indices;
    if(cluster_type_ == 1)
    {
        dbscan_cluster(cluster_indices);
    }
    else if(cluster_type_ == 2)
    {
        euclidean_cluster(cluster_indices);
    }

    // 4. velocity estimation
    velocity_estimation(cluster_indices);

    // 5. generate AABB
    vector<pair<pcl::PointXYZ,pcl::PointXYZ>> AABB_list;
    generate_AABB(cluster_indices,AABB_list);

    // 6. publish AABB
    publsihAABB(AABB_list);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cluster_test_node");
    ros::NodeHandle nh("~");

    string odom_topic_, cloud_topic_;


    nh.param("cluster_type", cluster_type_, 1);
    nh.param("epsilon", epsilon_, 0.5); 
    nh.param("min_points", min_points_, 5);  
    nh.param("odom_topic", odom_topic_, string("/odom_topic"));
    nh.param("cloud_topic", cloud_topic_, string("/cloud_topic"));
    nh.param("sensing_rate", sensing_rate_, 10.0F);
    nh.param("use_adaptive_dbscan",use_adaptive_dbscan_,false);
    nh.param("frame_id",frame_id, string("world")); 

    cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub(nh,cloud_topic_, 1);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh,odom_topic_, 1);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), point_cloud_sub, odom_sub);
    sync.registerCallback(boost::bind(&cloudOdomCallback, _1, _2));

    ros::Timer update_timer_ = nh.createTimer(ros::Duration(1.0/sensing_rate_), &updateCallback);

    aabb_pub_ =  nh.advertise<visualization_msgs::MarkerArray>("cluster", 1);
    
    ros::spin();
    
    return 0;
}
