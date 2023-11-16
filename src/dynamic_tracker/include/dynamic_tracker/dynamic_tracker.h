#ifndef _DYNAMIC_TRACKER_H
#define _DYNAMIC_TRACKER_H

#include <iostream>
#include <memory>
#include <vector>
#include <queue>
#include <cmath>
#include <random>
#include "dbscan_cluster.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>


#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

namespace dynamic_tracker
{
    using VectorXd = Eigen::VectorXd;
    using MatrixXd = Eigen::MatrixXd;
    using Matrix4d = Eigen::Matrix4d;
    using Quaterniond = Eigen::Quaterniond;
    using Vector3d = Eigen::Vector3d;
    using ClusterPtr = dbscan::DBSCANCluster::Ptr;
    using Vector4d = Eigen::Vector4d;
    using Vector2d = Eigen::Vector2d;
    using Vector6d = Eigen::Matrix<double,6,1>;

    class KalmanFilter
    {
        Vector4d state_last_; // 4x1
        Matrix4d A_; // 4x4
        Matrix4d P_last_; // 4x4 
        Matrix4d Rc_; // 4x4
        Matrix4d Qc_; // 4x4
        Matrix4d H_; // 4x4
        double ts_;
        
        /* 初始化 */
        KalmanFilter(Vector4d& init_state,double ts);
    public: 
        /* 更新 */
        Vector4d update(const Vector4d& obs);
        /* 预测 */
        Vector4d predict(); // 预测一步
        Vector4d predict(int timestep); // 预测timestep步

    };
    
    class DynamicTracker
    {



        public:
            DynamicTracker();
            ~DynamicTracker();
            void init(const ros::NodeHandle& nh);

        private:
            /**
             * @brief 根据卡尔曼滤波融合当前帧的簇信息
             * @param predicted_clusters, 之前跟踪的簇传播到当前帧，[x,y,vx,vy]
             */
            void forwardPropagation(std::vector<Vector4d>& predicted_clusters);
            
            /* 聚类 */
            /**
             * @brief cluster origin point cloud to a lot of point clusters
             * @param cloud 总的帧点云
             * @param cloud_ids 点云聚类后对应的簇标记，该点属于哪个类
             * @param clusters 传出，用来存放各个簇的点云指针
             * 
            */
            void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::Indices& cloud_ids,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);
            
            /**
             * @brief 生成移动椭圆 
             * @param point_clusters, 点云簇
             * @param ellpsoids, 生成对应椭圆的信息,[x,y,z,axis]
             */  
            void generateEllipsoids(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& point_clusters,std::vector<Vector6d>& ellpsoids);

            /** 
             * @brief match observed points and tracking points
             * @param clusters, 传进来的点云簇 
             * 1. 传入聚类好的点云簇，计算对应的点云质心。
             * 2. 用最近邻匹配算法来匹配当前帧和之前前向传播过来的簇 
             */
            void match(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters); 

            /* 投票函数，决定簇的速度 */
            void vote();
            /* 核心更新函数，更新所有 */
            void update(); 


        private:
            /* callback， */
            void cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                    const geometry_msgs::PoseStampedConstPtr& pose);
            void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                    const nav_msgs::OdometryConstPtr& odom);
            void odomCallback(const nav_msgs::OdometryConstPtr& odom);

            void updateDynamicTrackingCallback(const ros::TimerEvent&);

            bool checkCloudOdomNeedUpdate();
            /* 发布跟踪的*/
            
            void publishDynamicObject();
            void publishDynamicEllipsoid();


        private:   

            ros::NodeHandle node_; // 句柄

            /* timeout */
            double tracking_update_timeout_;
            ros::Time last_tracking_update_time_;
            bool tracking_need_update_; // 是否需要更新，有点云和odom数据之后为true，更新完后为false
            int tracking_disappear_remain_frame_threshold; // tracking 丢失，保留的帧数
            double tracking_update_rate_; // 更新周期 

            /* 聚类器 */
            ClusterPtr cluster_ptr_; // cluster base on dbscan algorithm 
            int minPts_;
            double eps_;

            /* 空间变换 参数 */
            Eigen::Matrix4d lidar2body_; // 雷达到机体
            Eigen::Vector3d lidar_pos_; // 雷达在全局坐标系下的位置
            Eigen::Matrix3d lidar_r_m_; // 雷达在全局坐标系下的旋转矩阵
            pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud; // 当前帧的点云数据,用来cluster


            /* ros publisher subscribe timer */
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> syncPolicyCloudOdom;
            typedef std::unique_ptr<message_filters::Synchronizer<syncPolicyCloudOdom>> synchronizerCloudOdom;
            std::unique_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub;
            std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub;
            synchronizerCloudOdom sync_cloud_odom_;
            
            ros::Publisher dynamic_object_pub_,dynamic_cloud_pub_,dynamic_traj_pub_;
            ros::Timer update_timer_;


            /* system */            
            double ts_; // 系统帧的时间

            // std::vector<int> observed_cluster_ids; 
            std::vector<int> tracking_cluster_ids; // 追踪的cluster当前id
            std::vector<std::queue<Vector2d>> tracking_clusters_traj;  // 追踪簇的状态[x,y,vx,vy], 第一维簇序号，第二维簇历史
            
            std::vector<KalmanFilter> tracking_clusters;
            std::vector<pcl::PointCloud<pcl::PointXYZ>> tracking_clusters_cloud; // 追踪的动态物体的点云
            std::vector<Vector3d> tracking_ellipsoids; // 对应簇的三个轴的长度
            std::vector<int> disappear_frames;

            typedef std::unique_ptr<DynamicTracker> Ptr; // 单例模式
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    };





}





#endif