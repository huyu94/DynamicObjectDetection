#ifndef _DSP_MAP_H_
#define _DEP_MAP_H_

#include <iostream>
#include <random>
#include <queue>
#include <array>


#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/search>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>


#define logix(x) (log((x) / (1 - (x))))
#define GRID_MAP_OBS_FLAG 32767
#define GRID_MAP_NEW_PLATFORM_TEST false
#define LIMIT_MOVEMENT_IN_XY_PLANE 1 // 是否限制障碍物预测在XY平面的运动
#define O_MAKE_VALID 1
#define O_MAKE_INVALID 0


using namespace std;
using Vector3d = Eigen::Vector3d;
using Vector3i = Eigen::Vector3i;
using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;
using Quaterniond = Eigen::Quaterniond;
namespace particle_map{

struct Particle
{
    Vector3d position;
    Vector3d velocity;
    float weight;
    int voxel_index;
};

struct ClusterFeature{
    Vector3d center = {0.0,0.0,0.0};
    int point_num = 0;
    int match_cluster_seq = -1;
    Vector3d velocity = {-10000.0f,-10000.0f,-10000.0f};
    float velocityNorm = 0.f;
    float intensity = 0.f;
};

struct MappingParamters
{
    bool have_initialized_ = false;
    Vector3d local_update_range3d_; // half of update range
    Vector3i local_update_range3i_;

    int inf_grid_;
    double obstacles_inflation_;
    string frame_id_;
    int pose_type_;
    bool enable_virtual_wall_; // 是否允许虚拟墙体出现
    double virtual_ceil_, virtual_ground_; // 虚拟天花板、虚拟地面
    /* time out */
    double odom_lidar_timeout_;

    /* visualization  and computation time display */
    bool show_occ_time_;

    /* particle map */
    /* const */
    float voxel_resolution_;// 体素分辨率
    float voxel_resolution_inv_;
    float angle_resolution_; // 角度分辨率
    float angle_resolution_inv_;
    // const float angle_resolution_rad_ = 
    int pyramid_neighbor_;// 金字塔空间邻居数量
    int pyramid_neighbor_num_;
    int max_particle_num_in_voxel_; // 每个体素子空间中粒子的最大数量
    int prediction_time_; // 预测时间区间
    
    /* velocity estimation */
    int dynamic_cluster_max_point_num_;
    float dynamic_cluster_max_center_height_;

    /* non const */
    float voxel_filter_resolution_; // 体素滤波的分辨率
    std::vector<float> prediction_future_time_;
    int observation_pyramid_num_horizontal_; 
    int observation_pyramid_num_vertical_;
    double half_fov_horizontal_;
    double half_fov_vertical_;
    int voxel_num_;
    int observation_pyramid_num_;
    int pyramid_num_;
    int safe_particle_num_;
    int safe_particle_num_in_voxel_;
    int safe_particle_num_in_pyramid_;

    /* particle output */
    bool if_record_particle_csv;
    string particle_save_folder;

    /* map upadte parameters */
    float position_prediction_stddev; //位置估计方差
    float velocity_prediction_stddev; //速度估计方差
    float kappa;
    float sigma_ob;
    float P_detection; //被检测到的概率


    int observation_max_points_num_one_pyramid;
    int GAUSSIAN_RANDOM_NUM;

    int voxel_objects_number_dimension;

    vector<float> p_gaussian_randoms;
    vector<float> v_gaussian_randoms;
    vector<float> standard_gaussian_pdf;
    int standard_gaussian_pdf_num_;
    const int guassian_random_num_ = 1000000;


    const int max_point_num_ = 5000;


    /* camera parameters */
    // double cx_, cy_, fx_, fy_;
    /* occupancy param */
    double occupancy_thresh_;  // 占据阈值


};


struct MappingData
{
    /* center and origin */
    Vector3i center_last3i_;
    /* ringbuffer */
    Vector3i ringbuffer_origin3i_;
    Vector3d ringbuffer_lowbound3d_;
    Vector3i ringbuffer_lowbound3i_;
    Vector3d ringbuffer_upbound3d_;
    Vector3i ringbuffer_upbound3i_;
    Vector3i ringbuffer_size3i_;
    /* inflation */
    Vector3i ringbuffer_inf_origin3i_;
    Vector3d ringbuffer_inf_lowbound3d_;
    Vector3i ringbuffer_inf_lowbound3i_;
    Vector3d ringbuffer_inf_upbound3d_;
    Vector3i ringbuffer_inf_upbound3i_;
    Vector3i ringbuffer_inf_size3i_;

    /* particle map */
    //根据体素存储对应粒子，[VOXEL_NUM]体素数量，SAFE_PARTICLE_NUM_VOXEL体素内最大粒子数。
    // 1. flag: 0.0f invalid, 1.0f valid but not newborn 3.0f valid newborn 7.0f recently predicted 
    // 2.vx 3.vy 4.vz 5.px 6.py 7.pz
    // 8.weight 9.update time
    // flag 要不要改？ 不确定
    vector<vector<vector<float>>> voxels_with_particles;
    //1. objects sum weights; 2-4. Avg vx, vy, vz; 5-9. Future objects number  10. 是否这个子空间后来被更新过，也就是mapmove出去的地图空间要置0.f
// static float voxels_objects_number[VOXEL_NUM][voxels_objects_number_dimension];
    vector<vector<float>> voxels_objects_number;
    vector<vector<vector<float>>> pyramids_in_fov;
    vector<vector<int>> observation_pyramid_neighbours;
    vector<vector<float>> future_status;
    vector<float> input_points_;
    float delt_t_from_last_observation;

    vector<double> occupancy_buffer; // 
    vector<uint16_t> occupancy_buffer_inflate_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_current_view_rotated;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud_with_velocity; 


    /* variables */
    int position_guassian_random_seq_;
    int velocity_gaussian_random_seq_;
    float kappa_;
    float update_time_;
    int update_counter_;
    float expected_new_born_objects_;
    float new_born_particle_weight_;
    int new_born_particle_number_each_point_;
    float new_born_each_object_weight_;

    vector<vector<vector<float>>> point_cloud_in_pyramid;
    vector<int> point_num_in_pyramid;

    // camera position and pose data
    Vector3d camera_position_, last_camera_position_;
    Matrix3d camera_rotation_, last_camera_rotation_;
    Matrix4d camera2body_;

    // flags of map state
    bool occ_need_update_, local_updated_;
    bool has_first_depth_;
    bool has_odom_;

    // odom_depth_timeout
    ros::Time last_occ_update_time_;
    bool flag_lidar_odom_timeout_;
    bool flag_lidar_ever_received_depth_;

    // std::vector<short> count_hit_, count_hit_and_miss_;
    // std::vector<char> flag_traverse_, flag_rayend_;
    // char raycast_num_;

    // vector<Vector3i> cache_voxel_;
    // int cache_voxel_cnt_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

class ParticleMap
{
public: 
    ParticleMap(){}
    ~ParticleMap() {}

    void initMap(ros::NodeHandle &nh);
    inline int getOccupancy(const Vector3d& pos);
    inline int getInflateOccupancy(const Vector3d& pos);
    inline double getResolution();
    bool getOdomLidarTimeout(){return md_.flag_lidar_odom_timeout_;}

    typedef std::shared_ptr<ParticleMap> Ptr;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    MappingParamters mp_;
    MappingData md_;

    enum 
    {
        POSE_STAMPED = 1,
        ODOMETRY = 2,
        INVALID_IDX = -10000
    };

    inline Vector3d globalIdx2Pos(const Vector3i& idx);
    inline Vector3i pos2GlobalIdx(const Vector3d& pos);
    inline int globalIdx2BufIdx(const Vector3i& id);
    inline int globalIdx2InfBufIdx(const Vector3i& id);
    inline Vector3i bufIdx2GlobalIdx(size_t address);
    inline Vector3i infBufIdx2GlobalIdx(size_t address);
    inline bool isInBuf(const Vector3d &pos);
    inline bool isInBuf(const Vector3i &idx);
    inline bool isInInfBuf(const Vector3d &pos);
    inline bool isInInfBuf(const Vector3i &idx);

    void publishMap();
    void publishMapInflate();

    /* receive callback */
    void lidarOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                           const nav_msgs::OdometryConstPtr &odom_msg);
    void lidarPoseCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                           const geometry_msgs::PoseStampedConstPtr &pose_msg);
    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);
    void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg);
    /* publish callback */
    void visCallback(const ros::TimerEvent &e);
    void updateOccupancyCallback(const ros::TimerEvent &e);


    /* ring buffer */
    void clearBuffer(char casein, int bound);
    void moveRingBuffer();
    
    /* particle core */
    bool addAParticle(const Particle &p, const int &voxel_index) const;
    int moveAParticle(const int& new_voxel_index, const int current_v_index, int current_v_inner_index, float *ori_particle_flag_ptr);
    
    void mapPrediction();
    void mapUpdate();
    void mapAddNewBornParticleByObservation();
    void mapOccupancyCalculationAndResample();
    // generate random number;
    void generateGaussianRandomsVectorZeroCenter() const;
    float getPositionGaussianZeroCenter();
    float getVelocityGaussianZeroCenter();
    bool ifInPyramidsArea(float x,float y,float z);
    int findPointPyramidHorizontalIndexInSensorFrame(float x,float y,float z);
    int findPointPyramidVerticalIndexInSensorFrame(float x,float y,float z);
    void getKMClusterResult(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cluster_cloud);

    /**********************************/

    inline void changeInBuf(const bool dir, const int inf_buf_idx, const Vector3i global_idx);
    inline int setCacheOccupancy(Vector3d pos, int occ);
    Vector3d closestPointInMap(const Vector3d &pt, const Vector3d &center);
    void testIndexingCost();
    bool checkLidarOdomNeedUpdate();
    void initMapBoundary();

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyLidarOdom;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> SyncPolicyLidarPose;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyLidarOdom>> SynchronizerLidarOdom;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyLidarPose>> SynchronizerLidarPose;

    ros::NodeHandle node_;
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> lidar_sub_;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
    SynchronizerLidarOdom sync_lidar_odom_;
    SynchronizerLidarPose sync_lidar_pose_;

    ros::Subscriber indep_cloud_sub_, indep_odom_sub_, indep_pose_sub_;
    ros::Publisher map_pub_, map_inflate_pub_;
    ros::Timer occ_update_timer_, vis_timer_;

    uniform_real_distribution<double> rand_noise_;
    normal_distribution<double> rand_noise2_;
    default_random_engine rand_engine_;

    void velocityEstimationThread();
    void setOriginVoxelFilterResolution(float res);
    void findPyramidNeighborIndexInFOV(int index, vector<vector<float>> &neighobrs_list);
    static void removeParticle(float *ori_particle_flag_ptr);
    float standardNormalPDF(float value);
    void calculateNormalPDFBuffer();
    float queryNormalPDF(float &x,float &mu, float &sigma);
    static void transformParticleToSensorFrame(Vector3d& transformPoint,const Vector3d &oriPoint, const Vector3d &position,const Quaterniond &rotation);
    static float clusterDistance(ClusterFeature &c1, ClusterFeature &c2);

    static float generateRandomFloat(float min, float max);

};

/* =================================================== definition of inline function 
 * =============================================*/

// inline int ParticleMap::setCacheOccupancy(Vector3d pos, int occ)
// {
//     if(occ != 1 && occ != 0)
//         return INVALID_IDX;
    
//     Vector3i id = pos2GlobalIdx(pos); //  pos -> global index
//     int idx_ctns = globalIdx2BufIdx(id); // global index -> buffer address 
    
//     md_.count_hit_and_miss_[idx_ctns] += 1;
//     if(md_.count_hit_and_miss_[idx_ctns] == 1)
//     {
//         md_.cache_voxel_[md_.cache_voxel_cnt_++] = id;
//     }
    
//     if(occ == 1)
//     {
//         md_.count_hit_[idx_ctns] += 1;
//     }
//     return idx_ctns;
// }

inline void ParticleMap::changeInBuf(const bool dir, const int inf_buf_idx, const Vector3i global_idx)
{
    int inf_grid = mp_.inf_grid_;
    if(dir)
    {
        md_.occupancy_buffer_inflate_[inf_buf_idx] += GRID_MAP_OBS_FLAG;
    }
    else{
        md_.occupancy_buffer_inflate_[inf_buf_idx] -= GRID_MAP_OBS_FLAG;
    }

    for(int x_inf = -inf_grid; x_inf <= inf_grid; ++x_inf)
    {
        for(int y_inf = -inf_grid; y_inf <= inf_grid; ++y_inf)
        {
            for(int z_inf = -inf_grid; z_inf <= inf_grid; ++z_inf)
            {
                Vector3i idx_inf = global_idx + Vector3i(x_inf, y_inf, z_inf);
                int id_inf_buf = globalIdx2InfBufIdx(idx_inf);
                if(dir)
                {
                    ++md_.occupancy_buffer_inflate_[id_inf_buf];
                }
                else
                {
                    --md_.occupancy_buffer_inflate_[id_inf_buf];
                    if(md_.occupancy_buffer_inflate_[id_inf_buf] > 65000) // an error case
                    {
                        ROS_ERROR("A negative value of nearby obstacle number ! reset the map");
                        // fill(md_.occupancy_buffer.begin(),md_.occ_occupancy_buffer.end(),mp_.)
                        // fill(md_.occupancy_buffer_inflate_.begin(),md_.occupancy_buffer_inflate_.end(),0);
                    }
                }
            }
        }
    }
}

/* global idx ------> buffer idx*/
inline int ParticleMap::globalIdx2BufIdx(const Vector3i &id)
{
    int x_buffer = (id(0) - md_.ringbuffer_origin3i_(0)) % md_.ringbuffer_size3i_(0);
    int y_buffer = (id(1) - md_.ringbuffer_origin3i_(0)) % md_.ringbuffer_size3i_(1);
    int z_buffer = (id(2) - md_.ringbuffer_origin3i_(0)) % md_.ringbuffer_size3i_(2);
    if(x_buffer < 0){
        x_buffer += md_.ringbuffer_size3i_(0);
    }
    if(y_buffer < 0){
        y_buffer += md_.ringbuffer_size3i_(1);
    }
    if(z_buffer < 0){
        z_buffer += md_.ringbuffer_size3i_(2);
    }

    return md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1) * z_buffer + md_.ringbuffer_size3i_(0) * y_buffer + x_buffer;
}

/* global idx ------> inflate buffer idx */
inline int ParticleMap::globalIdx2InfBufIdx(const Vector3i &id)
{
    int x_buffer = (id(0) - md_.ringbuffer_inf_origin3i_(0)) % md_.ringbuffer_inf_size3i_(0);
    int y_buffer = (id(1) - md_.ringbuffer_inf_origin3i_(0)) % md_.ringbuffer_inf_size3i_(1);
    int z_buffer = (id(2) - md_.ringbuffer_inf_origin3i_(0)) % md_.ringbuffer_inf_size3i_(2);
    if(x_buffer < 0){
        x_buffer += md_.ringbuffer_inf_size3i_(0);
    }
    if(y_buffer < 0){
        y_buffer += md_.ringbuffer_inf_size3i_(1);
    }
    if(z_buffer < 0){
        z_buffer += md_.ringbuffer_inf_size3i_(2);
    }

    return md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1) * z_buffer + md_.ringbuffer_inf_size3i_(0) * y_buffer + x_buffer;
}

inline Vector3i ParticleMap::bufIdx2GlobalIdx(size_t address)
{
    const int ringbuffer_xysize = md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1);
    int zid_in_buffer = address / ringbuffer_xysize;
    address %= ringbuffer_xysize;
    int yid_in_buffer = address / md_.ringbuffer_size3i_(0);
    int xid_in_buffer = address / md_.ringbuffer_size3i_(0);

    int xid_global = xid_in_buffer + md_.ringbuffer_origin3i_(0);
    if(xid_global > md_.ringbuffer_upbound3i_(0))
    {
        xid_global -= md_.ringbuffer_size3i_(0);
    }
    int yid_global = yid_in_buffer + md_.ringbuffer_origin3i_(1);
    if(yid_global > md_.ringbuffer_upbound3i_(1))
    {
        yid_global -= md_.ringbuffer_size3i_(1);
    }
    int zid_global = xid_in_buffer + md_.ringbuffer_origin3i_(2);
    if(zid_global > md_.ringbuffer_upbound3i_(2))
    {
        zid_global -= md_.ringbuffer_size3i_(2);
    }
    
    return Vector3i(xid_global,yid_global,zid_global);
}


inline Vector3i ParticleMap::infBufIdx2GlobalIdx(size_t address)
{
    const int ringbuffer_xysize = md_.ringbuffer_inf_size3i_(0) * md_.ringbuffer_inf_size3i_(1);
    int zid_in_buffer = address / ringbuffer_xysize;
    address %= ringbuffer_xysize;
    int yid_in_buffer = address / md_.ringbuffer_inf_size3i_(0);
    int xid_in_buffer = address / md_.ringbuffer_inf_size3i_(0);

    int xid_global = xid_in_buffer + md_.ringbuffer_inf_origin3i_(0);
    if(xid_global > md_.ringbuffer_inf_upbound3i_(0))
    {
        xid_global -= md_.ringbuffer_inf_size3i_(0);
    }
    int yid_global = yid_in_buffer + md_.ringbuffer_inf_origin3i_(1);
    if(yid_global > md_.ringbuffer_inf_upbound3i_(1))
    {
        yid_global -= md_.ringbuffer_inf_size3i_(1);
    }
    int zid_global = zid_in_buffer + md_.ringbuffer_inf_origin3i_(2);
    if(zid_global > md_.ringbuffer_inf_upbound3i_(2))
    {
        zid_global -= md_.ringbuffer_inf_size3i_(2);
    }

    return Vector3i(xid_global,yid_global,zid_global);
}

inline int ParticleMap::getOccupancy(const Vector3d &pos)
{
    if(!isInBuf(pos))
    {
        return 0;
    }

    if(mp_.enable_virtual_wall_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    {
        return -1;
    }

    return md_.occupancy_buffer[globalIdx2BufIdx(pos2GlobalIdx(pos))] > mp_.occupancy_thresh_ ? 1 : 0;
}

inline int ParticleMap::getInflateOccupancy(const Vector3d &pos)
{
    if(!isInBuf(pos))
    {
        return 0;
    }

    if(mp_.enable_virtual_wall_ && (pos(2) >= mp_.virtual_ceil_ || pos(2) <= mp_.virtual_ground_))
    {
        return -1;
    }

    return int(md_.occupancy_buffer_inflate_[globalIdx2InfBufIdx(pos2GlobalIdx(pos))]) ;
}

inline bool ParticleMap::isInBuf(const Vector3d &pos)
{
    if(pos(0) < md_.ringbuffer_lowbound3d_(0) || pos(1) < md_.ringbuffer_lowbound3d_(1) || pos(2) < md_.ringbuffer_lowbound3d_(2))
    {
        return false;
    } 
    if(pos(0) > md_.ringbuffer_upbound3d_(0) || pos(1) > md_.ringbuffer_upbound3d_(1) || pos(2) > md_.ringbuffer_upbound3d_(2))
    {
        return false;
    }
    return true;
}


inline bool ParticleMap::isInBuf(const Vector3i &idx)
{
    if(idx(0) < md_.ringbuffer_lowbound3i_(0) || idx(1) < md_.ringbuffer_lowbound3i_(1) || idx(2) < md_.ringbuffer_lowbound3i_(2))
    {
        return false;
    }
    if(idx(0) > md_.ringbuffer_upbound3i_(0) || idx(1) > md_.ringbuffer_upbound3i_(1) || idx(2) > md_.ringbuffer_upbound3i_(2))
    {
        return false;
    }
    return true;
}


inline bool ParticleMap::isInInfBuf(const Vector3d &pos)
{
    if(pos(0) < md_.ringbuffer_inf_lowbound3d_(0) || pos(1) < md_.ringbuffer_inf_lowbound3d_(1) || pos(2) < md_.ringbuffer_inf_lowbound3d_(2))
    {
        return false;
    }
    if(pos(0) > md_.ringbuffer_inf_upbound3d_(0) || pos(1) > md_.ringbuffer_inf_upbound3d_(2) || pos(2) > md_.ringbuffer_inf_upbound3d_(2))
    {
        return false;
    }
    return true;
}

inline bool ParticleMap::isInInfBuf(const Vector3i &idx)
{
    if(idx(0) < md_.ringbuffer_inf_lowbound3i_(0) || idx(1) < md_.ringbuffer_inf_lowbound3i_(1) || idx(2) < md_.ringbuffer_inf_lowbound3i_(2))
    {
        return false;
    }
    if(idx(0) > md_.ringbuffer_inf_upbound3i_(0) || idx(1) > md_.ringbuffer_inf_upbound3i_(1) || idx(2) > md_.ringbuffer_inf_upbound3i_(2))
    {
        return false;
    }
    return true;
}

inline Vector3d ParticleMap::globalIdx2Pos(const Vector3i &idx)
{
    return Vector3d((idx(0) + 0.5) * mp_.voxel_resolution_, (idx(1) + 0.5) * mp_.voxel_resolution_, (idx(2) + 0.5) * mp_.voxel_resolution_);
}

inline Vector3i ParticleMap::pos2GlobalIdx(const Vector3d &pos)
{
    return (pos * mp_.voxel_resolution_inv_).array().floor().cast<int>(); // more than twice faster than std::floor()
}


inline double ParticleMap::getResolution()
{
    return mp_.voxel_resolution_;
}

#endif

}; // end namespace particle_map 