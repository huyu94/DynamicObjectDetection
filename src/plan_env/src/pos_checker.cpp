#include "plan_env/pos_checker.h"
#include "pos_checker.h"

void PosChecker::getlineGrids(const Vector3d &s_p, const Vector3d &e_p, std::vector<Vector3d> &grids)
{
    RayCaster raycaster;
    Eigen::Vector3d ray_pt;
    Eigen::Vector3d start = s_p / resolution_, end = e_p / resolution_;
    bool need_ray = raycaster.setInput(start, end);
    if (need_ray)
    {
    while (raycaster.step(ray_pt))
    {
        Eigen::Vector3d tmp = (ray_pt)*resolution_;
        tmp[0] += resolution_ / 2.0;
        tmp[1] += resolution_ / 2.0;
        tmp[2] += resolution_ / 2.0;
        grids.push_back(tmp);
    }
    }

    //check end
    Eigen::Vector3d end_idx;
    end_idx[0] = std::floor(end.x());
    end_idx[1] = std::floor(end.y());
    end_idx[2] = std::floor(end.z());

    ray_pt[0] = (double)end_idx[0];
    ray_pt[1] = (double)end_idx[1];
    ray_pt[2] = (double)end_idx[2];
    Eigen::Vector3d tmp = (ray_pt)*resolution_;
    tmp[0] += resolution_ / 2.0;
    tmp[1] += resolution_ / 2.0;
    tmp[2] += resolution_ / 2.0;
    grids.push_back(tmp);
}


bool PosChecker::checkState(const Vector3d &pos, ros::Time check_time, int &collision_type)
{
    if(checkCollisionInGridMap(pos))
    {
        collision_type = 0;
        return false;
    }

    int collision_id; 
    if(checkCollisionInSlideBox(pos, ros::Time::now(),collision_id))
    {
        collision_type = 1;
        return false;
    }
}



bool PosChecker::checkCollisionInGridMap(const Vector3d &pos)
{
    // 如果不选择膨胀地图，就要在规划时考虑机身半径
    if (!inflate_)
    {
        ROS_WARN("not inflate collision check, not support now");
        return false;   
    }
    else // 如果选择，就直接直接返回对应体素的占据状态
    {
        if (grid_map_->getInflateOccupancy(pos) != 0) // 0 is free, 1 is collision, -1 not in map unknown 
        {
            // cout << "collision: "<< pos.transpose() << endl;
            return false;
        }
        return true;
    }
}


bool PosChecker::checkCollisionInSlideBox(const Vector3d &pos, const ros::Time &pos_time, int &collision_id)
{
    vector<SlideBox> slide_boxes;
    tracker_pool_->forwardSlideBox(slide_boxes, pos_time);
    for (auto &slide_box : slide_boxes)
    {
        if(slide_box.isInBox(pos))
        {
            collision_id = slide_box.getId();
            return true;
        }
    }
    return false;
    
}
