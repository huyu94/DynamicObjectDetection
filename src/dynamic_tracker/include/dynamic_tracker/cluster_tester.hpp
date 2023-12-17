#ifndef _CLUSTER_TESTER_H

#define _CLUSTER_TESTER_H

#include <iostream>
#include <memory>
#include <vector>
#include <queue>
#include <cmath>
#include <random>
#include <string>
#include <algorithm>
#include "dynamic_tracker/dbscan_cluster.hpp"
#include <ros/ros.h>
#include <ros/console.h>
#include <munkres.h>
#include "dynamic_tracker/tracking_pool.hpp"
#include "dynamic_tracker/typedef.hpp"




#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>



#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>


using Cluster = dbscan::DBSCANCluster;

namespace cluster_tester
{
    class ClusterTester
    {

        // pose type : 1 : pose ; 2: odom;
        // cluster type : 1: DBSCAN; 2: Euclidean 


        private:
            ros::NodeHandle node_; // 句柄
            int cluster_type_;
            int pose_type_;
            bool tracking_need_update_; // 是否需要更新，有点云和odom数据之后为true，更新完后为false
            /* 点云滤波 */
            double clip_height_;
            double euclidean_cluster_tolerance_;

            /* 空间变换 参数 */
            Eigen::Matrix4d lidar2body_; // 雷达到机体
            Eigen::Vector3d lidar_pos_; // 雷达在全局坐标系下的位置
            Eigen::Matrix3d lidar_r_m_; // 雷达在全局坐标系下的旋转矩阵
            pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud; // 当前帧的点云数据,用来cluster
            ros::Time last_update_time_;

            void cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::Indices &cloud_ids, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &observed_clusters);

            bool checkCloudOdomNeedUpdate();
            void publishAABB(std::vector<Vector6d> &pos, std::vector<std::pair<Vector3d, Vector3d>> &minmax);

            void updateCallback(const ros::TimerEvent &event);


            static bool generateAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vector6d &obs_state, Vector3d& vmin,Vector3d &vmax);

            static bool generateEllipsoid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vector6d &obs_state, Vector3d &obs_axis);

            static void transferToColorCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &input,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output,const std::vector<int> clusterIds);
            /* callback， */
            void cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                    const geometry_msgs::PoseStampedConstPtr& pose);
            void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                    const nav_msgs::OdometryConstPtr& odom);

        private:
            std::vector<std::vector<double>> color_table_;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyCloudOdom;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> SyncPolicyCloudPose;

            typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;
            typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

            std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
            std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
            std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
            SynchronizerCloudOdom sync_cloud_odom_;
            SynchronizerCloudPose sync_cloud_pose_;
            ros::Publisher aabb_pub_;
            ros::Timer update_timer_;
            std::string cloud_topic_,odom_topic_,pose_topic_;


            Cluster::Ptr cluster_ptr_; // cluster base on dbscan algorithm 
            int minPts_;
            double eps_;
            bool cluster_adaptive_;


        public:
            ClusterTester(){};
            ~ClusterTester(){};
            void init(const ros::NodeHandle &nh);
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            typedef std::unique_ptr<ClusterTester> Ptr; 
    };


}



#endif