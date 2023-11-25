#ifndef TRACKING_POOL_H
#define TRACKING_POOL_H

#include <iostream>
#include <vector>
#include <queue>
#include <memory>
#include <dynamic_tracker/typedef.hpp>
#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>




namespace dynamic_tracker
{

    
    class TrackingPool 
    {
    private:

        struct Tracking{
            bool is_alive_;
            int id_;
            Vector6d state_;
            Vector3d axis_;
            Matrix6d P_; // 6x6

            int disappear_time_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
            std::queue<Eigen::Vector3d> history_traj_;
            /* TODO : add a constructer */
            Tracking(int id,const Vector6d& state,const Vector3d& axis,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
            :state_(state),axis_(axis),cloud_(cloud),id_(id),is_alive_(true),disappear_time_(0){
                history_traj_.push(state_.head<3>());
                P_ = Matrix6d::Zero();
            }

            void reset(int id,const Vector6d& state,const Vector3d& axis,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
                id_ = id;
                state_ = state;
                axis_ = axis;
                cloud_ = cloud;
                is_alive_ = true;
                disappear_time_ = 0;
                history_traj_ = std::queue<Vector3d>();
                history_traj_.push(state_.head<3>());
            }

            typedef std::shared_ptr<Tracking> Ptr;
        };
    private:
        int size_;
        std::vector<Tracking::Ptr> pool_; // 存储所有的对象
        std::queue<int> free_ids; // 存储可用的ID

        /* kalman filter update related */
        Matrix6d A_; // 6x6
        Matrix6d Rc_; // 6x6
        Matrix6d Qc_; // 6x6
        Matrix6d H_; // 6x6
        double ts_;
        int missing_time_threshold_;
        ros::NodeHandle node_;



    private:
        /* tracking update */
        void updateTracking(int id,const Vector6d &obs,const Vector3d& axis,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        /* tracking forward propagation */
        Vector6d predictTracking(const Tracking::Ptr track);
        Vector6d predictTracking(const Tracking::Ptr track, int timestep);
        /* 在池子里新添加一个tracking */
        void createTracking(int new_id,const Vector6d& obs,const Vector3d& axis,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        /* 复活池子里的tracking */
        void resetTracking(int exist_id,const Vector6d& obs,const Vector3d& axis,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        /* add a new tracking */
        int addTracking(const Vector6d& obs,const Vector3d& axis,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        /* delete tracking by given id */
        void removeTracking(int id);


        /* get tracking by given id */
        Tracking::Ptr getTracking(int id){
            return pool_[id];
        }


    public:
        TrackingPool(){};
        ~TrackingPool(){};

        void init(ros::NodeHandle& nh);


        inline int size(){
            return size_;
        }

        void getPool(std::vector<std::pair<int,std::pair<Vector6d,Vector3d>>> &alive_trackings);

        void updatePool(std::vector<int> &obs_match_ids, std::vector<Vector6d> &obs, std::vector<Vector3d> &obs_axis,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& obs_cloud);

        void predictPool(std::vector<std::pair<int,Vector6d>> &alive_trackings);

        void predictPool(std::vector<std::pair<int,Vector6d>> &alive_trackings, int timestep);







    
        typedef std::shared_ptr<TrackingPool> Ptr;
    
    };


}
#endif