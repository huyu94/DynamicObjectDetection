#ifndef _POS_CHECKER_
#define _POS_CHECKER_





#include "static/grid_map.h"
#include "static/raycast.h"
#include "tracker_pool.h"
#include <ros/ros.h>
#include <Eigen/Eigen>

using Eigen::Vector3d;

class PosChecker
{
private:
    GridMap::Ptr grid_map_;
    TrackerPool::Ptr tracker_pool_;
    double hrz_safe_radius_, vtc_safe_radius_; // 水平安全半径，竖直安全半径
    double copter_diag_len_; // 无人机对角长度 
    double resolution_; // 地图分辨率
    double dt_; // 遍历轨迹用的步长
    bool inflate_; // 是否膨胀

    // 获取一条线从起点到终点经历的网格
    void getlineGrids(const Vector3d &s_p, const Vector3d &e_p, std::vector<Vector3d> &grids);

    /**
     * @brief check the state of the position
     * @param pos position to check
     * @param check_time time to check
     * @param collision_type 0: grid map collision, 1: slide box collision
    */
    bool checkState(const Vector3d &pos, ros::Time check_time, int &collision_type );


    bool checkCollisionInGridMap(const Vector3d &pos);

    bool checkCollisionInSlideBox(const Vector3d &pos, const ros::Time &pos_time, int &collision_id);

public:
    PosChecker(){};

    ~PosChecker(){};

    void init(const ros::NodeHandle &nh)
    {
        nh.param("pos_checker/hrz_safe_radius", hrz_safe_radius_, 0.0);
        nh.param("pos_checker/vtc_safe_radius", vtc_safe_radius_, 0.0);
        nh.param("pos_checker/copter_diag_len", copter_diag_len_, 0.0); // 无人机的对角长度
        nh.param("pos_checker/dt", dt_, 0.0); // 
        nh.param("pos_checker/inflate", inflate_, false);
        ROS_WARN_STREAM("[pos_checker] param: hrz_safe_radius: " << hrz_safe_radius_);
        ROS_WARN_STREAM("[pos_checker] param: vtc_safe_radius: " << vtc_safe_radius_);
        ROS_WARN_STREAM("[pos_checker] param: copter_diag_len: " << copter_diag_len_);
        ROS_WARN_STREAM("[pos_checker] param: dt: " << dt_);
        ROS_WARN_STREAM("[pos_checker] param: inflate: " << inflate_);
    };


    // 设置地图类
    void setMap(const GridMap::Ptr &grid_map)
    {
        grid_map_ = grid_map;
        resolution_ = grid_map_->getResolution();
    };

    void setTrackerPool(const TrackerPool::Ptr &tracker_pool)
    {
        tracker_pool_ = tracker_pool;
    };



    ros::Time getLocalTime()
    {
        return grid_map_->getLocalTime();
    }


    void generateSlideBox(vector<TrackerOutput> &current_tracker, vector<TrackerOutput>& tracker_outputs);
    




    bool validatePosSurround(const Vector3d &pos);


    typedef share_ptr<PosChecker> Ptr;
};



#endif