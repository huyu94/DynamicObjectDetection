#ifndef _ENV_MANAGER_H
#define _ENV_MANAGER_H

#include <mutex>
#include <thread>
#include <fstream>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>

#include <ros/package.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
// #include <pcl/kdtree/kdtree_flann.h>



#include <munkres.h>
#include "env_manager/dynamic/tracker_pool.h"
#include "env_manager/ikd-Tree/ikd_Tree.h"
#include "env_manager/static/grid_map.h"
#include <visualization_utils/map_visualizer.h>

using PointType = ikdTree_PointType;
using PointVector = KD_TREE<PointType>::PointVector;
typedef shared_ptr<PointVector> PointVectorPtr;
typedef shared_ptr<nav_msgs::Odometry> OdomPtr;
using std::pair;
struct ClusterFeature
{
    VectorXd state; // [pos,vel] 
    Vector3d length;
    Vector3d min_bound,max_bound;
    int match_id{-1}; // 与tracker_pool中的tracker匹配的id, -1 表示没有匹配中
    pcl::PointIndices cluster_indices;
    double gamma_1; // global_average_minimum_distance
    double gamma_2; // normalized average variance of distance
    // int motion_type; // 0:moving; 1:static; 2:Unkown ; -1:undefine
    enum MotionType
    {
        MOVING = 0,
        STATIC = 1,
        UNKOWN = 2,
        UNDEFINE = -1
    } motion_type;
    typedef std::shared_ptr<ClusterFeature> Ptr;   
};


class EnvManager
{
private:
    ros::NodeHandle node_;
    // KD_TREE<PointType>::Ptr kd_tree_ptr_;
    

    TrackerPool::Ptr tracker_pool_ptr_;
    GridMap::Ptr grid_map_ptr_;

    
/* record */
    std::ofstream update_time_record_;
    bool record_;

/* multi-threads */
    std::mutex slide_window_mtx_;
    
/* data */
    nav_msgs::Odometry current_odom_;
    ros::Time last_update_time_, odom_time_; // odom_time_ is used to record perception cloud time
    double tracking_update_timeout_;
    bool need_update_;
    queue<pair<PointVectorPtr,OdomPtr>> cloud_odom_slide_window_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud_ptr_;
    int slide_window_size_; 
    bool cloud_odom_window_ready_;
    vector<Vector3d> static_points_;


/* visualizer */
    MapVisualizer::Ptr map_vis_ptr_;

/*  cluster : */ 
    // KD_TREE<PointType>::Ptr cluster_ikdtree_ptr_;
    // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr cluster_kdtree_ptr_;
    double dbscan_eps_;
    int dbscan_min_ptn_;
    int dbscan_min_cluster_size_,dbscan_max_cluster_size_;
    vector<ClusterFeature::Ptr> cluster_features_;

/* segmentation */
    KD_TREE<PointType>::Ptr segmentation_ikdtree_ptr_;

    double gamma1_threshold_,gamma2_threshold_;

/* inflation */
    double dynamic_object_inflation_;


/* poschecker */
    bool enable_virtual_walll_;
    double virtual_ceil_,virtual_ground_;



/* match */
    double distance_gate_;
    double cluster_max_height_;

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
/* poschecker */
    void getLineGrids(const Vector3d &start, const Vector3d &end, vector<Vector3d> &grids);

    bool checkCollision(const Vector3d& pos, ros::Time check_time, int &collision_type, int &collision_id);

    bool checkCollisionInGridMap(const Vector3d& pos);

    bool checkCollisionInTrackerPool(const Vector3d& pos, const ros::Time& pos_time, int &collision_id);

    bool checkCollisionInSlideBox(const Vector3d& pos, int &collision_id);

    void generateSlideBox(double forward_time, vector<SlideBox>& slide_boxes);

    // inline bool isInInfMap(const Vector3d& pos)
    // {
    //     return grid_map_ptr_->isInInfBuf(pos);
    // }
    bool isInInfMap(const Vector3d& pos);

    inline double getResolution()
    {
        return grid_map_ptr_->getResolution();
    }

    


public:
    EnvManager();
    ~EnvManager();
    void init(const ros::NodeHandle& nh);
    void setGridMap();
    void setTrackerPool();

    GridMap::Ptr getGridMap(){return grid_map_ptr_;};
    TrackerPool::Ptr getTrackerPool(){return tracker_pool_ptr_;};
    MapVisualizer::Ptr getMapVisualizer(){return map_vis_ptr_;};


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
    /**
     * @brief calculate the feature of the cluster
    */
    void calClusterFeatureProperty(ClusterFeature::Ptr cluster_ptr);
    // void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    //              std::vector<pcl::PointIndices> &cluster_indices,
    //              std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters);

    void segmentation();

    void match();

    /**
     * @brief used to check if there are new cloud and odom data
    */
    bool checkNeedUpdate();

    /**
     * @brief add cloud and odom to slide window
    */
    void addCloudOdomToSlideWindow(PointVectorPtr &cloud, OdomPtr &odom);
    /* callback */
    void cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                    const nav_msgs::OdometryConstPtr& odom);
    void odomCallback(const nav_msgs::OdometryConstPtr &odom);
    void updateCallback(const ros::TimerEvent&);
    void visCallback(const ros::TimerEvent &);
    void indepCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    void indepOdomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

    

/* toolkit  */
    /**
     * @brief record data 
    */
    void record(int update_count, int point_size, double cluster_time, double segmentation_time, double match_time, double total_time);


public:
typedef std::shared_ptr<EnvManager> Ptr;

EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};






#endif