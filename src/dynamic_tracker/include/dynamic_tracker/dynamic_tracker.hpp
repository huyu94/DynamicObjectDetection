#ifndef _DYNAMIC_TRACKER_H
#define _DYNAMIC_TRACKER_H

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




#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>


using Cluster = dbscan::DBSCANCluster;

namespace dynamic_tracker
{

    class DynamicTracker
    {

            enum
            {
                POSE_STAMPED = 1,
                ODOMETRY = 2,
                INVALID_IDX = -10000
            };

        private:
            
            /* 聚类 */
            /**
             * @brief cluster origin point cloud to a lot of point clusters
             * @param cloud 总的帧点云
             * @param cloud_ids 点云聚类后对应的簇标记，该点属于哪个类
             * @param clusters 传出，用来存放各个簇的点云指针
             * 
            */
            void cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<pcl::PointIndices> &cluster_indices,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);
            
            // /**
            //  * @brief 生成移动椭圆 
            //  * @param point_clusters, 点云簇
            //  * @param ellpsoids, 生成对应椭圆的信息,[x,y,z,axis]
            //  */  
            // void generateEllipsoids(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& point_clusters,std::vector<Vector6d>& ellpsoids);

            /** 
             * @brief match observed points and tracking points
             * @param clusters, 传进来的点云簇 
             * 1. 传入聚类好的点云簇，计算对应的点云质心。
             * 2. 用最近邻匹配算法来匹配当前帧和之前前向传播过来的簇 
             */
            void match(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& observed_clusters); 

            /* 投票函数，决定簇的速度 */
            void vote();
            /* 核心更新函数，更新所有 */
            void update(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& observed_clusters); 



        private:
            /* callback， */
            void cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                    const geometry_msgs::PoseStampedConstPtr& pose);
            void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                    const nav_msgs::OdometryConstPtr& odom);
            void odomCallback(const nav_msgs::OdometryConstPtr& odom);

            void updateDynamicTrackingCallback(const ros::TimerEvent&);

            void visCallback(const ros::TimerEvent&);
            void publishCloud();
            void publishColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

            bool checkCloudOdomNeedUpdate();
            /* 发布跟踪的 */
            void publishDynamicObject();
            void publishDynamicEllipsoid();

            void publishAABB(std::vector<Vector6d> &pos, std::vector<std::pair<Vector3d, Vector3d>> &minmax);



            static bool generateAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vector6d &obs_state, Vector3d& vmin,Vector3d &vmax);

            static bool generateEllipsoid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vector6d &obs_state, Vector3d &obs_axis);

            static void transferToColorCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &input,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output,const std::vector<int> clusterIds);



        private:   

            ros::NodeHandle node_; // 句柄
            int pose_type_; // 


            /* timeout */
            double tracking_update_timeout_;
            ros::Time last_tracking_update_time_;
            bool tracking_need_update_; // 是否需要更新，有点云和odom数据之后为true，更新完后为false
            int tracking_disappear_remain_frame_threshold; // tracking 丢失，保留的帧数
            double tracking_update_rate_; // 更新周期 

            /* 点云滤波 */
            double clip_height_;

            /* 聚类器 */
            Cluster::Ptr cluster_ptr_; // cluster base on dbscan algorithm 
            int minPts_;
            double eps_;
            bool cluster_adaptive_;

            /* 空间变换 参数 */
            Eigen::Matrix4d lidar2body_; // 雷达到机体
            Eigen::Vector3d lidar_pos_; // 雷达在全局坐标系下的位置
            Eigen::Matrix3d lidar_r_m_; // 雷达在全局坐标系下的旋转矩阵
            pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud; // 当前帧的点云数据,用来cluster

            /* color table */
            std::vector<std::vector<double>> color_table_;
            /* ros publisher subscribe timer */
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyCloudOdom;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> SyncPolicyCloudPose;

            typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;
            typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudPose>> SynchronizerCloudPose;

            std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
            std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
            std::shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
            SynchronizerCloudOdom sync_cloud_odom_;
            SynchronizerCloudPose sync_cloud_pose_;
            
            ros::Subscriber indep_cloud_sub_,indep_odom_sub_;
            void indepCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
            void indepOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg);
            ros::Publisher dynamic_object_pub_,dynamic_cloud_pub_,dynamic_traj_pub_;
            ros::Publisher temp_cloud_pub_,color_cloud_pub_;
            ros::Publisher ellipsoid_pub_,aabb_pub_;
            ros::Timer update_timer_, vis_timer_;
            std::string cloud_topic_,odom_topic_,pose_topic_;

            


            /* system */            
            double ts_; // 系统帧的时间

            // std::vector<int> observed_cluster_ids; 
            /* 新一版所需要的变量 */
            // std::vector<EllipsoidFeature> tracking_clusters_; // 维持跟踪的椭球
            TrackingPool::Ptr tracking_pool_ptr_;

            
        public:
            DynamicTracker(){};
            ~DynamicTracker(){};
            void init(const ros::NodeHandle& nh);
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            typedef std::unique_ptr<DynamicTracker> Ptr; 

    };

}





#endif