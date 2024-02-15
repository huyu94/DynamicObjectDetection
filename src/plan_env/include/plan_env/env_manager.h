#ifndef _ENV_MANAGER_H
#define _ENV_MANAGER_H

#include <mutex>
#include <thread>

#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <plan_env/grid_map.h>


#include <munkres.h>
#include "plan_env/dynamic/tracker_pool.h"
#include "plan_env/ikd_Tree.h"
#include "plan_env/static/grid_map.h"
#include "plan_env/map_visualizer.h"

using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;


struct ClusterFeature
{
    VectorXd state; // [pos,vel] 
    Vector3d length;
    Vector3d min_bound,max_bound;
    int match_id{-1}; // 与tracker_pool中的tracker匹配的id, -1 表示没有匹配中
    pcl::PointIndices cluster_indices;
    double gamma_1; // global_average_minimum_distance
    double gamma_2; // normalized average variance of distance
    int motion_type; // 0:moving; 1:static; 2:Unkown 
    typedef std::shared_ptr<ClusterFeature> Ptr;   
};


class EnvManager
{
private:
    ros::NodeHandle node_;
    // KD_TREE<PointType>::Ptr kd_tree_ptr_;
    

    TrackerPool::Ptr tracker_pool_ptr_;
    GridMap::Ptr grid_map_ptr_;

    shared_ptr<PointVector> current_cloud_ptr_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr_;
    
/* multi-threads */
    std::mutex slide_window_mtx_;
    
/* data */
    nav_msgs::Odometry current_odom_;
    ros::Time current_time_, last_update_time_;

/* visualizer */
    MapVisualizer::Ptr map_vis_ptr_;

/*  cluster : */ 
    pcl::KdTreeFLANN<pcl::PointXYZ> cluster_kdtree_; //pcl kdtree used for DBSCAN
    // KD_TREE<PointType>::Ptr cluster_kdtree_ptr_;
    double dbscan_eps_;
    int dbscan_min_ptn_;
    vector<ClusterFeature::Ptr> cluster_features_;

/* segmentation */
    KD_TREE<PointType>::Ptr segmentation_kdtree_ptr_;
    queue<shared_ptr<PointVector>> cloud_slide_window_;
    int slide_window_size_; 
    bool ikd_tree_built_;
    bool cloud_window_ready_;
    double gamma1_threshold_,gamma2_threshold_;

/* synchronizer */
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyCloudOdom;
    typedef std::shared_ptr<message_filters::Synchronizer<SyncPolicyCloudOdom>> SynchronizerCloudOdom;
    SynchronizerCloudOdom sync_cloud_odom_;


/* publisher && subscriber && timer*/
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> cloud_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    ros::Timer update_timer_, vis_timer_;

    ros::Publisher vis_pub_;

/* params */
    double ts_; // 更新时间间隔
    

public:



public:
    EnvManager(){};
    ~EnvManager(){};
    void init(const ros::NodeHandle& nh);
    void setGridMap();
    void setTrackerPool();

private:

    /*
    1. cluster (\u2713)[v]
    2. segmentation 
    3. data-association  
    */
    void update();

    /**
     * @brief DBSCAN cluster
     * @param cloud 总的帧点云
     * @param cloud_ids 点云聚类后对应的簇标记，该点属于哪个类
     * @param clusters 传出，用来存放各个簇的点云指针
     * 
    */
    void cluster();
    void calClusterFeatureProperty(ClusterFeature::Ptr cluster_ptr);
    // void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    //              std::vector<pcl::PointIndices> &cluster_indices,
    //              std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

    void segmentation();

    void match();

/* callback */
    void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                    const nav_msgs::OdometryConstPtr& odom);
    void odomCallback(const nav_msgs::OdometryConstPtr &odom);
    void updateCallback(const ros::TimerEvent&);
    void visCallback(const ros::TimerEvent&);
    void indepCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void indepOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

    

/* toolkit  */
    void checkReady();


public:
typedef std::shared_ptr<EnvManager> Ptr;

EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};




#endif