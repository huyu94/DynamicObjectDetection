#include <plan_env/pos_checker.h>

void PosChecker::init(const ros::NodeHandle &nh)
{
    nh_ = nh;
    // nh.param("pos_checker/hrz_safe_radius", hrz_safe_radius_, 0.0);
    // nh.param("pos_checker/vtc_safe_radius", vtc_safe_radius_, 0.0);
    // nh.param("pos_checker/copter_diag_len", copter_diag_len_, 0.0); // 无人机的对角长度
    // nh.param("pos_checker/dt", dt_, 0.0); // 
    // nh.param("pos_checker/inflate", inflate_, false);
    nh.param("pos_checker/virtual_wall",virtual_wall_,false);
    nh.param("pos_checker/virtual_ground",virtual_ground_,0.0);
    nh.param("pos_checker/virtual_ceil", virtual_ceil_,3.0);
    // ROS_WARN_STREAM("[pos_checker] param: hrz_safe_radius: " << hrz_safe_radius_);
    // ROS_WARN_STREAM("[pos_checker] param: vtc_safe_radius: " << vtc_safe_radius_);
    // ROS_WARN_STREAM("[pos_checker] param: copter_diag_len: " << copter_diag_len_);
    // ROS_WARN_STREAM("[pos_checker] param: dt: " << dt_);
    // ROS_WARN_STREAM("[pos_checker] param: inflate: " << inflate_);
};






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


bool PosChecker::checkCollision(const Vector3d &pos, ros::Time check_time, int &collision_type, int &collision_id)
{
    if(checkCollisionInGridMap(pos))
    {
        collision_type = 0;
        return true;
    }

    if(checkCollisionInTrackerPool(pos, check_time,collision_id))
    {
        collision_type = 1;
        return true;
    }
    return false;
}



bool PosChecker::checkCollisionInGridMap(const Vector3d &pos)
{
    if (grid_map_->getInflateOccupancy(pos) != 0) // 0 is free, 1 is collision, -1 not in map unknown 
    {
        // cout << "collision: "<< pos.transpose() << endl;
        return true;
    }
    if(virtual_wall_)
    {
        if(pos(2) > virtual_ceil_ || pos(2) < virtual_ground_)
        {
            return true;
        }
    }
    return false;
    // // 如果不选择膨胀地图，就要在规划时考虑机身半径
    // if (!inflate_)
    // {
    //     ROS_WARN("not inflate collision check, not support now");
    //     return false;   
    // }
    // else // 如果选择，就直接直接返回对应体素的占据状态
    // {
    //     if (grid_map_->getInflateOccupancy(pos) != 0) // 0 is free, 1 is collision, -1 not in map unknown 
    //     {
    //         // cout << "collision: "<< pos.transpose() << endl;
    //         return false;
    //     }
    //     return true;
    // }
}


bool PosChecker::checkCollisionInTrackerPool(const Vector3d &pos , const ros::Time &pos_time, int &collision_id)
{
    collision_id = -1;
    vector<TrackerOutput> target_tracker_outputs;
    tracker_pool_->forwardPool(target_tracker_outputs, pos_time);
    for(auto &obj : target_tracker_outputs)
    {
        Vector3d obj_pos = obj.state.head(3);
        Vector3d obj_axis = obj.length/2;

        if(abs(pos.x() - obj_pos.x()) < obj_axis.x()  / 2 &&
           abs(pos.y() - obj_pos.y()) < obj_axis.y()  / 2 &&
           abs(pos.z() - obj_pos.z()) < obj_axis.z()  / 2)
        {
            collision_id = obj.id;
            return true;
        }
    }
    return false;
}


void PosChecker::generateSlideBox(double forward_time)
{
    tracker_slide_boxs_.clear();
    tracker_pool_->forwardSlideBox(tracker_slide_boxs_,ros::Time::now() + ros::Duration(forward_time));
    vector<VisualizeSlideBox> visual_slide_boxes;
    for(auto &t : tracker_slide_boxs_)
    {
        visual_slide_boxes.emplace_back(t.getCenter(),t.getLength(),t.getRotation(),t.getId());
    }
    map_visualizer_ptr_->visualizeSlideBox(visual_slide_boxes);
}

bool PosChecker::checkCollisionInSlideBox(const Vector3d &pos, int &object_id, Vector3d &object_pos)
{
    for(auto & slide_box : tracker_slide_boxs_)
    {
        if(slide_box.isInBox(pos))
        {
            object_pos = slide_box.getCenter();
            object_id = slide_box.getId();
            return true;
        }
    }
    object_id = -1;
    if(virtual_wall_)
    {
        if(pos(2) > virtual_ceil_ || pos(2) < virtual_ground_)
        {
            return true;
        }
    }
    return false;
}