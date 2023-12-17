#ifndef _POS_CHECKER_
#define _POS_CHECKER_

#include "plan_env/grid_map.h"
#include "plan_env/raycast.h"
#include <ros/ros.h>
#include <Eigen/Eigen>

using Eigen::Vector3d;

class PosChecker
{

    enum useMap{GridMap = 1, DspMap = 2};
private:
    GridMap::Ptr grid_map_;                    // 占据地图的单例
    double hrz_safe_radius_, vtc_safe_radius_; // 水平安全半径，竖直安全半径
    double copter_diag_len_;                   // 无人机对角长度
    double resolution_;                        // 地图分辨率
    double dt_;                                // 遍历轨迹用的步长
    bool inflate_;                             // 是否膨胀
    int use_


    void getlineGrids(const Vector3d &s_p, const Vector3d & e_p,
                      std::vector<Vector3d> &grids);;

    bool checkState(const Vector3d &pos, const Vector3d &vel, const Vector3d &acc);


public:
    PosChecker(){};
    
    ~PosChecker(){};

    void init(const ros::NodeHandle &nh);

    void setGridMap();

    void setDspMap();

    int getVoxelState(const Vector3d &pos);

    ros::Time getLocalTime();

    // 检查点周围膨胀半径内的栅格是否安全
    bool validatePosSurround(const Vector3d &pos);
  
}