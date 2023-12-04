/**************************************************************************

Copyright <2022> <Gang Chen>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


Author: Gang Chen

Date: 2021/8/19

Description: This is the head file for the DSP map with constant velocity model. In this file, Parameter ANGLE_RESOLUTION can be as small as the real sensor angle resolution. A (2*PYRAMID_NEIGHBOR_N+1)^2 neighborhood pyramid space will be considered in the update. This is ideal to handle occlusion when very tiny obstacles exist but is less efficient.

**************************************************************************/

#ifndef _DSP_MAP_H
#define _DSP_MAP_H

#include <ctime>
#include <cmath>
#include <fstream>
#include "iostream"
#include <random>
#include <fstream>
#include "Eigen/Eigen"
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/voxel_grid.h>
#include <thread>
#include "munkres.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>


#include "plan_env/Typdefs.hpp"
#include "plan_env/dsp_map_math.hpp"

using namespace std;
#define LIMIT_MOVEMENT_IN_XY_PLANE 1 // 是否现在运动在XY平面
#define O_MAKE_VALID 1
#define O_MAKE_INVALID 0
#define MAP_LENGTH_VOXEL_NUM 50 // 地图长 10
#define MAP_WIDTH_VOXEL_NUM 50 //地图宽 10
#define MAP_HEIGHT_VOXEL_NUM 15 // 地图高 3
#define VOXEL_RESOLUTION 0.2 // subspace1 体素分辨率
#define ANGLE_RESOLUTION 1 // subspace2 角度分辨率
#define PYRAMID_NEIGHBOR_N 2 //subspace2 邻近空间
#define MAX_PARTICLE_NUM_VOXEL 30 // 单个体素中最大粒子数
#define PREDICTION_TIMES 6

/** Parameters for the map **/
// const static int MAP_LENGTH_VOXEL_NUM = 75;// 地图长 50
// const static int MAP_WIDTH_VOXEL_NUM = 75; //地图宽 50
// const static int MAP_HEIGHT_VOXEL_NUM = 30;// 地图高 30
// const static float VOXEL_RESOLUTION = 0.2;// subspace1 体素分辨率
// const static int ANGLE_RESOLUTION = 1; //角度分辨率
// const static int PYRAMID_NEIGHBOR_N = 2;// subspace2 角度分辨率
// const static int MAX_PARTICLE_NUM_VOXEL = 30; // 单个体素中最大粒子数
// const static int PREDICTION_TIMES = 6;
static const float prediction_future_time[PREDICTION_TIMES] = {0.05f, 0.2f, 0.5f, 1.f, 1.5f, 2.f}; //unit: second. The first value is used to compensate the delay caused by the map.


// 意思是42度， 27度， horizontal angle & vertical angle 
// avia 35 34
// mid360 180 30
const int half_fov_h = 180;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make half_fov_h a smaller value than the real FOV angle
const int half_fov_v = 27;  // can be divided by ANGLE_RESOLUTION. If not, modify ANGLE_RESOLUTION or make half_fov_h a smaller value than the real FOV angle

// const static int DYNAMIC_CLUSTER_MAX_POINT_NUM = 200;// Pre-velocity estimation parameter. Cluster with too many points will be allocated with a zero velocity.
// const static int DYNAMIC_CLUSTER_MAX_CENTER_HEIGHT = 1.5;// Pre-velocity estimation parameter. Cluster with too high center will be allocated with a zero velocity.

#define DYNAMIC_CLUSTER_MAX_POINT_NUM 200// Pre-velocity estimation parameter. Cluster with too many points will be allocated with a zero velocity.
#define DYNAMIC_CLUSTER_MAX_CENTER_HEIGHT 1.5// Pre-velocity estimation parameter. Cluster with too high center will be allocated with a zero velocity.


const static string particle_save_folder = ".";
/** END **/


static const int observation_pyramid_num_h = (int)half_fov_h * 2 / ANGLE_RESOLUTION; // subspace2 的水平方向有多少个金字塔格子
static const int observation_pyramid_num_v = (int)half_fov_v * 2 / ANGLE_RESOLUTION; // subsapce2 的垂直方向有多少个金字塔格子
// FOV空间中能有多少金字塔子空间，取决于分辨率的大小和传感器的fov
static const int observation_pyramid_num = observation_pyramid_num_h * observation_pyramid_num_v;

// 37500
static const int VOXEL_NUM = MAP_LENGTH_VOXEL_NUM*MAP_WIDTH_VOXEL_NUM*MAP_HEIGHT_VOXEL_NUM; // subspace1 总的体素数量
static const int PYRAMID_NUM = 360*180/ANGLE_RESOLUTION/ANGLE_RESOLUTION; // Na 360度，180度的理论金字塔空间？
static const int SAFE_PARTICLE_NUM = VOXEL_NUM * MAX_PARTICLE_NUM_VOXEL + 1e5; // subspace1中理论的粒子数
static const int SAFE_PARTICLE_NUM_VOXEL = MAX_PARTICLE_NUM_VOXEL * 2; //  
static const int SAFE_PARTICLE_NUM_PYRAMID = SAFE_PARTICLE_NUM/PYRAMID_NUM * 2; // 

//An estimated number. If the observation points are too dense (over 100 points in one pyramid), the overflowed points will be ignored. It is suggested to use a voxel filter to the original point cloud.
static const int observation_max_points_num_one_pyramid = 100; // 一个金字塔空间中最大的粒子数

static const int GAUSSIAN_RANDOMS_NUM = 10000000;

// static const int O_MAKE_VALID = 1;
// static const int O_MAKE_INVALID = 0;


using namespace std;
/** Struct for an individual particle**/
struct Particle{
    Position position;
    Velocity velocity;
    float weight;
    int voxel_index; //所在的体素下标
};

// 聚类特征，用来做实际速度估计
struct ClusterFeature{
    Position centerPosition = {0.f,0.f,0.f};
    int point_num = 0;
    int match_cluster_seq = -1;
    Velocity velocity = {-10000.0f,-10000.0f,-10000.0f};
    float velocityNorm = 0.f;
    float intensity = 0.f;
};

// 根据体素存储对应粒子，[VOXEL_NUM]体素数量，SAFE_PARTICLE_NUM_VOXEL体素内最大粒子数。
// 1. flag: 0.0f invalid, 1.0f valid but not newborn 3.0f valid newborn 7.0f recently predicted 
// 2.vx 3.vy 4.vz 5.px 6.py 7.pz
// 8.weight 9.update time
// flag 要不要改？ 不确定
static float voxels_with_particle[VOXEL_NUM][SAFE_PARTICLE_NUM_VOXEL][9]; 

// 体素对应信息量。
static const int voxels_objects_number_dimension = 4 + PREDICTION_TIMES + 1;
// 每个体素对应的信息 
//1. objects sum weights; 2-4. Avg vx, vy, vz; 5-9. Future objects number  10. 是否这个子空间后来被更新过，也就是mapmove出去的地图空间要置0.f
static float voxels_objects_number[VOXEL_NUM][voxels_objects_number_dimension];

/// Container for pyramids
// 存储fov视角中的金字塔子空间
// observation_pyramid_num fov视角里金字塔子空间的数量，SAFE_PARTICLE_NUM_PYRAMID 每个金字塔子空间中粒子的最大数量
// 0.flag:{0 invalid;1:valid}
// 1.particle voxel index  2.particle index inside voxel，在金字塔子空间中的粒子个数
static int pyramids_in_fov[observation_pyramid_num][SAFE_PARTICLE_NUM_PYRAMID][3];

// 1.neighbors num 2-:neighbor indexes
static const int pyramid_neighbors_num = (2 * PYRAMID_NEIGHBOR_N + 1) * (2 * PYRAMID_NEIGHBOR_N + 1); //(2n+1)^2
// 金字塔子空间的邻居数组，1维是子空间下标，2维{0:邻居数，1-end:邻居下标}
static int observation_pyramid_neighbors[observation_pyramid_num][pyramid_neighbors_num+1]{};

// static float current_position[3] = {0.f, 0.f, 0.f}; //传感器当前位置
static float voxel_filtered_resolution = 0.15; // 体素滤波分辨率
static float delt_t_from_last_observation = 0.f; // 与上一帧点云之间的时间差


/** Storage for Gaussian randoms and Gaussian PDF**/
static float p_gaussian_randoms[GAUSSIAN_RANDOMS_NUM];
static float v_gaussian_randoms[GAUSSIAN_RANDOMS_NUM];
static float standard_gaussian_pdf[20000];

static const int MAX_POINT_NUM = 5000;
static float input_points[MAX_POINT_NUM*3];
static float future_status[VOXEL_NUM][PREDICTION_TIMES];


/// Variables for velocity estimation
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_current_view_rotated;
static pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud_with_velocity;
// static float point_cloud[observation_pyramid_num][observation_max_points_num_one_pyramid][5];
// static int observation_num_each_pyramid[observation_pyramid_num]{};
// static float point_cloud_max_length[observation_pyramid_num];



namespace dsp_map
{

class DspMap
{
private:


    /** Parameters **/
    float voxelResolution_; // 体素分辨率

    int pyramid_num_h; // 金字塔空间水平方向体素数量
    int pyramid_num_v; // 金字塔空间垂直方向体素数量
    int angleResolution_; // 角度分辨率
    float angle_resolution_half; //角度分辨率的一半

    float angle_resolution_rad; //角度分辨率-弧度制
    float angle_resolution_rad_half; //角度分辨率的一一半-弧度制

    int voxels_total_num; //体素子空间数量
    int pyramid_total_num; //金字塔子空间总数量


    Position currentSensorPosition_; // 当前传感器的位置
    Position lastSensorPosition_; // 上一次更新，传感器的位置
    Rotation currentSensorRotation_; // 当前传感器的姿态
    Position mapPosition_; // 地图中心位置
    Size mapSize_; // 地图的体素长度int
    Length mapLength_; // 地图的实际长度float
    Index mapStartIndex_; // 循环local map的起始位置
    Length mapMinBoundary_;
    Length mapMaxBoundary_;

    double timestamp_; // 本次更新地图的时间戳
    double last_timestamp_; // 上次更新地图的时间戳
    bool first_call_;

    int max_particle_num_voxel; //体素子空间的最大粒子数
    int max_input_points; // pointcloud callback中最大可接受的点云数
    float position_prediction_stddev; //位置估计方差
    float velocity_prediction_stddev; //速度估计方差

    float sigma_ob; 

    float P_detection; //被检测到的概率

    int if_record_particle_csv; //是否记录粒子到csv
    float record_time; //记录时间


    /** Variables **/
    int position_gaussian_random_seq; 
    int velocity_gaussian_random_seq;

    float kappa;

    float update_time;
    int update_counter;

    float expected_new_born_objects;

    float new_born_particle_weight; //新出生粒子的权重
    int new_born_particle_number_each_point; 
    float new_born_each_object_weight;  

    // 每个金字塔空间中的观测点的情况，即为观测点，point cloud 
    // 1.px, 2.py, 3.pz 4.acc 5.length for later usage
    // row:金字塔体素数量，column:每个金字塔空间最大观测点数
    float point_cloud[observation_pyramid_num][observation_max_points_num_one_pyramid][5];

    // 1.point_num，每个金字塔空间中观测到的点数
    int observation_num_each_pyramid[observation_pyramid_num]{};

    // 传感器 姿态四元数
    Eigen::Quaternionf rotate_quaternion; // sensor2map
    float sensor_rotation_quaternion[4];

    // Normal vectors for pyramids boundary planes when sensor has no rotation，金字塔空间平面的法线向量
    // float pyramid_BPnorm_params_ori_h[observation_pyramid_num_h+1][3]; // x, y, z
    // float pyramid_BPnorm_params_ori_v[observation_pyramid_num_v+1][3];

    // Normal vectors for pyramids boundary planes when sensor rotated，如果传感器相对于机器人位置有旋转
    // float pyramid_BPnorm_params_h[observation_pyramid_num_h+1][3]; 
    // float pyramid_BPnorm_params_v[observation_pyramid_num_v+1][3];


    // Max length, used to judge if occlusion happens
    float point_cloud_max_length[observation_pyramid_num];

    float half_fov_h_rad; //fov空间水平角-弧度制
    float half_fov_v_rad; //fov空间垂直角-弧度制

    double total_time;
    unsigned int update_times;

    /* 功能类的一些变量*/
    ros::NodeHandle node_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> SyncPolicyPointCloudOdom;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> SyncPolicyPointCloudPose;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyPointCloudOdom>> SynchronizerPointCloudOdom;
    typedef shared_ptr<message_filters::Synchronizer<SyncPolicyPointCloudPose>> SynchronizerPointCloudPose;
    
    shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> pointcloud_sub_;
    shared_ptr<message_filters::Subscriber<geometry_msgs::PoseStamped>> pose_sub_;
    shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    SynchronizerPointCloudOdom sync_pointcloud_odom_;
    SynchronizerPointCloudPose sync_pointcloud_pose_;
    ros::Timer occ_timer_;
    ros::Timer vis_timer_;
    ros::Publisher map_pub_,inflate_map_pub_,future_map_pub_,cube_marker_pub_;
    ros::Subscriber indep_pose_sub_,indep_pointcloud_sub_;

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;



    

public:

    DspMap();
    ~DspMap(){
        cout << "\n See you ;)" <<endl;
    }
    typedef std::shared_ptr<DspMap> Ptr;
    enum {POSE_STAMPED = 1, ODOMETRY = 2, INVALID_IDX = -10000};

    void initMap(ros::NodeHandle& nh);

    int update(int point_cloud_num, int size_of_one_point, float *point_cloud_ptr,
                const Position& sensorPosition, double time_stamp_second,
                const Rotation& sensorRotation);  /// also requires point cloud and velocity
    

    void setPredictionVariance(float p_stddev, float v_stddev);
    void setObservationStdDev(float ob_stddev);
    void setNewBornParticleWeight(float weight);
    void setNewBornParticleNumberofEachPoint(int num);
    void setParticleRecordFlag(int record_particle_flag, float record_csv_time);





private:
    /* original core function*/
    void setInitParameters();
    void addRandomParticles(int particle_num, float avg_weight);
    void mapPrediction(float odom_delt_px, float odom_delt_py, float odom_delt_pz, float delt_t);
    void mapUpdate();
    void mapAddNewBornParticlesByObservation();
    void mapOccupancyCalculationAndResample();
    // int isParticleInMap(const Particle &p) const;
    // int isParticleInMap(const float &px, const float &py, const float &pz) const;
    void generateGaussianRandomsVectorZeroCenter() const;
    float getPositionGaussianZeroCenter();
    float getVelocityGaussianZeroCenter();
    int ifInPyramidsAreaInSensorFrame(float& x, float& y, float& z);
    int findPointPyramidHorizontalIndexInSensorFrame(float& x,float& y,float& z);
    int findPointPyramidsVerticalIndexInSensorFrame(float& x,float& y,float& z);
    void getKMClusterResult(pcl::PointCloud<pcl::PointXYZINormal> &cluster_cloud);

    /*callback*/
    void pointcloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud,
                                const geometry_msgs::PoseStampedConstPtr &pose);
    void pointcloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &pointcloud,
                                const nav_msgs::OdometryConstPtr &odom);
    void updateMapCallback(const ros::TimerEvent& /*event*/);
    void visCallback(const ros::TimerEvent& /*event*/);
    void indepPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud);
    void indepPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose);


    /* mainly index functions*/

    /*待修改函数*/
    // int getParticleVoxelsIndex(const Particle &p, int &index);
    // int getParticleVoxelsIndex(const float &px, const float &py, const float &pz, int & index);
    // void getVoxelPositionFromIndex(const int &index, float &px, float &py, float &pz) const;
    // void updateMapPosition(const Eigen::Vector3f& position);

    bool addAParticle(const Particle &p, const int &voxel_index) const;
    int moveParticle(const int& new_voxel_index, const int& current_v_index, const int& current_v_inner_index, float *ori_particle_flag_ptr);
    void getOccupancyMap(int &obstacles_num, pcl::PointCloud<pcl::PointXYZ> &cloud, const float threshold);
    void getOccupancyMapWithFutureStatus(int &obstacles_num, pcl::PointCloud<pcl::PointXYZ> &cloud, float *future_status, const float threshold);
    void clearOccupancyMapPrediction();
    // pos <-> index
    // float localIndexToGlobalPosition(const Eigen::Vector3i& index) const; // 实现不了
    // get single occupancy





    // int getOccupancyAtPosition(const Position& position,const float threshold = 0.7) const;
    // int getOccupancyAtLocalIndex(const Index& localIndex,const float threshold = 0.7) const;

    //放在了x_math.cpp里实现
    // bool getPositionShiftFromIndexShift(Eigen::Vector3f& position_shift,const Eigen::Vector3i& index_shift);
    // bool getIndexShiftFromPositionShift(Eigen::Vector3i& index_shift, const Eigen::Vector3f& position_shift);
    // void wrapIndexInMap();
    // Eigen::Vector3i getIndexFromBufferIndex(const Eigen::Vector3i& buffer_index); 
    // Eigen::Vector3i getBufferIndexFromIndex(const Eigen::Vector3i& index);
    // bool checkIfStartIndexAtDefaultPosition() const; 


    // bool indexInMap(const Index& index) const;  // 暂时不需要，因为都是先判断positioninmap以后，再转index，这时index就是肯定在map内的了

    // void boundIndexInMap(Eigen::Vector3i& index);
    // void globalPositionToLocalIndex(const Eigen::Vector3f& globalPosition,Eigen::Vector3i& localIndex) const;
    // int localIndexToAddress(const Eigen::Vector3i& localIndex) const;
    // int globalPositionToAddress(const Eigen::Vector3f& globalPosition) const;
    // Eigen::Vector3i getIndexFromPosition(const Eigen::Vector3f& position) const;


public:

    int getInflateOccupancy(const Eigen::Vector3d& pos) const;

    float getResolution() const;



    /*已完成函数*/
    const Size& getMapSize() const;
    const Length& getMapLength() const;
    const Position& getMapPosition() const;
    const float getVoxelResolution() const;
    const int getAngleResolution() const;
    void setMapStartIndex(const Index& startIndex);
    Index getMapStartIndex() const;
    bool ifPositionInMap(const Position& position) const;
    bool ifPositionInMap(const float& x,const float& y,const float& z) const;

    bool ifParticleInMap(const Particle& p) const; 
    bool getIndex(Index& index,const Position& position) const;
    bool getPosition(Position& position,const Index& index) const;
    int getAddress(const Position& position) const;
    int getAddress(const Index& index) const;
    bool mapMove(const Position& position);
    void clearAll();
    void clearXAxis(const int startIndex,const int indexShift);
    void clearYAxis(const int startIndex,const int indexShift);
    void clearZAxis(const int startIndex,const int indexShift);
    bool getParticleAddress(const float& x, const float& y, const float& z,int& address);
    bool getParticleAddress(const Particle& particle, int& address);

    /*static utils function */
    static void setOriginalVoxelFilterResolution(float res);
    static void findPyramidNeighborIndexInFOV(const int &index_ori, int &neighbor_spaces_num, int *neighbor_spaces_index);
    static void removeParticle(float *ori_particle_flag_ptr);
    static float standardNormalPDF(float value);
    static void calculateNormalPDFBuffer();
    static float queryNormalPDF(float &x, float &mu, float &sigma);
    static void transformParticleToSensorFrame(Point& transformedPoint,const Point& oriPoint,const Position& position,const Rotation& rotation);
    static void rotateVectorByQuaternion(const float *ori_vector, const Eigen::Quaternionf& quaternion, float *rotated_vector);
    static float vectorMultiply(float &x1, float &y1, float &z1, float &x2, float &y2, float &z2);
    static float vectorCross2d(float& x1,float& y1,float& x2,float& y2);
    static float clusterDistance(ClusterFeature &c1, ClusterFeature &c2);
    static void velocityEstimationThread();
    static float mod(float value,float modulus){return fmod(fmod(value,modulus)+modulus,modulus);}
    static float generateRandomFloat(float min, float max);


};
}



#endif