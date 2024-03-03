#ifndef _POS_CHECKER_
#define _POS_CHECKER_





#include "static/grid_map.h"
#include "static/raycast.h"
#include "dynamic/tracker_pool.h"
#include <ros/ros.h>
#include <Eigen/Eigen>

using Eigen::Vector3d;
using Eigen::Quaterniond;




class PosChecker
{

// struct SlideBox
// {
//     Vector3d min_bound;
//     Vector3d max_bound;
//     Quaterniond orientation;
//     int id;
//     // typedef shared_ptr<SlideBox> Ptr;
// };





private:
    GridMap::Ptr grid_map_;
    TrackerPool::Ptr tracker_pool_;
    double hrz_safe_radius_, vtc_safe_radius_; // 水平安全半径，竖直安全半径
    double copter_diag_len_; // 无人机对角长度 
    double resolution_; // 地图分辨率
    double dt_; // 遍历轨迹用的步长
    bool inflate_; // 是否膨胀


private:
    vector<SlideBox> tracker_slide_boxs_; // 用于一次性生成本次拓扑运动规划的滑动立方体

public:
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

    bool checkCollisionInTrackerPool(const Vector3d &pos, const ros::Time &pos_time, int &collision_id);


    /**
     * @brief generate slide box for the current tracker
    */
    void generateSlideBox(double forward_time);

    /**
     * @brief check collision in slide box
    */
    bool checkCollisionInSlideBox(const Vector3d &pos, int &collision_id);

public:
    PosChecker(){};

    ~PosChecker(){};

    void init(const ros::NodeHandle &nh);






    // 设置地图类
    void setGridMap(const GridMap::Ptr &grid_map)
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

    void getResolution()
    {
        return resolution_;
    }






    bool validatePosSurround(const Vector3d &pos);


    typedef shared_ptr<PosChecker> Ptr;
};



#endif