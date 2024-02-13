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
#include "plan_env/ikd-Tree/ikd_Tree.h"
#include "plan_env/static/grid_map.h"

using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;


struct ClusterFeature
{
    VectorXd state;
    Vector3d length;
    pcl::PointIndices cluster_indices;
};


class EnvManager
{
private:
    ros::NodeHandle node_;
    // KD_TREE<PointType>::Ptr kd_tree_ptr_;
    

    TrackerPool::Ptr tracker_pool_ptr_;
    GridMap::Ptr grid_map_ptr_;

    shared_ptr<PointVector> current_cloud_ptr_;
    
/* data */
    nav_msgs::Odometry current_odom_;

/*  cluster : */ 
    pcl::KdTreeFLANN<pcl::PointXYZ> dbscan_kdtree_; //pcl kdtree used for DBSCAN
    double dbscan_eps_;
    int dbscan_min_ptn_;
    vector<ClusterFeature> cluster_features_;

/* segmentation */
    KD_TREE<PointType>::Ptr ikd_tree_ptr_;
    queue<shared_ptr<PointVector>> cloud_slide_window_;
    std::mutex slide_window_mtx_;
    bool ikd_tree_built_;
    bool cloud_window_ready_;

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
    void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
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



EIGEN_MAKE_ALIGNED_OPERATOR_NEW
typedef std::shared_ptr<EnvManager> Ptr;


};




#endif