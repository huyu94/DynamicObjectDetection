#include <plan_manager/plan_manager.h>
#include <plan_manager/uniform_bspline/uniform_bspline.h>
#include <plan_manager/plan_container.hpp>
#include <thread>


namespace fast_planner
{

    PlanManager::PlanManager(){}

    PlanManager::~PlanManager() {std::cout << "destroy plan-manager" << std::endl; }

    void PlanManager::initPlanModules(ros::NodeHandle& nh)
    {
        
        nh.param("manager/max_vel", pp_.max_vel_, -1.0);
        nh.param("manager/max_acc", pp_.max_vel_, -1.0);
        nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);

        nh.param("manager/dynamic_environment", pp_.dynamic_, -1);
        nh.param("manager/clearance_threshold", pp_.clearance_, -1.0); 
        nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
        nh.param("manager/control_points_distance", pp_.ctrl_pt_dist_, -1.0);

        bool use_kinodynamic_path, use_topo_path, use_optimzation;
        bool use_geometric_path, use_active_perception;

        local_data_.traj_id_ = 0;
        env_manger_.reset(new EnvManager);
        env_manger_->init(nh);

        if(use_kinodynamic_path)
        {
            kino_path_finder_.reset(new KinodynamicAstar);
            kino_path_finder_->setParam(nh);
            kino_path_finder_->setEnvironment();
            kino_path_finder_->init();
        }

        if(use_optimzation)
        {
            bspline_optimizers_.resize(10);
            for(int i=0; i<10; i++)
            {
                bspline_optimizers_[i].reset(new BsplineOptimizer);
                bspline_optimizers_[i]->setParam(nh);
                bspline_optimizers_[i]->setEnvironment();
            }
        }

        if(use_topo_path)
        {
            topo_prm_.reset(new TopoPRM);
            topo_prm_->setEnvironment();
            topo_prm_->init();
        }
        
        /* env */
        env_manger_.reset(new EnvManager);
        env_manger_->init(nh);
    }

    bool PlanManager::checkTrajCollision()
    {
        double t_now = (ros::Time::now() - local_data_.start_time_).toSec(); // 轨迹增量时间

        double tm, tmp;
        local_data_.position_traj_.getTimeSpan(tm,tmp); //tm:轨迹当前时间
        Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now); 
    
        double      radius = 0;
        Vector3d    fut_pt;
        double      fut_t = 0.02;

        /* static obstalces */
        while(radius < 6.0 && t_now + fut_t < local_data_.duration_)
        {
            fut_pt = local_data_.position_traj_.evaluateDeBoor(tm + t_now + fut_t);
            bool collision = env_manger_->checkCollisionInGridMap(fut_pt);
            if(collision)
            {
                return false;
            }

            radius = (fut_pt - cur_pt).norm();
            fut_t += 0.02;
        }

        /* dynamic obstalces */



        return true;
    }



    bool PlanManager::kinodynamicReplan(Vector3d start_pos, Vector3d start_vel, 
                                        Vector3d start_acc, Vector3d end_pos, 
                                        Vector3d end_vel)
    {
        ROS_INFO_STREAM("[kino replan]: ---------------------");
        ROS_INFO_STREAM("start: " << start_pos.transpose() << ", " << start_vel.transpose() << ", " 
                        << start_acc.transpose() << "\ngoal:" << end_pos.transpose() << ", " 
                        << end_vel.transpose());

        if((start_pos - end_pos).norm() < 0.2)
        {
            ROS_INFO_STREAM("Close goal");
            return false;
        }

        ros::Time t1,t2;
        local_data_.start_time_ = ros::Time::now();
        double t_search = 0.0, t_opt = 0.0, t_adjust = 0.0;

        Vector3d init_pos = start_pos;
        Vector3d init_vel = start_vel;
        Vector3d init_acc = start_acc;

        // kinodynamic path searching
        t1 = ros::Time::now();
        kino_path_finder_->reset();

        int status = kino_path_finder_->search(start_pos, start_vel, start_acc, end_pos, end_vel, true);

        if(status == KinodynamicAstar::NO_PATH)
        {
            ROS_WARN_STREAM("[kino replan]: kinodynamic search fail!" );
            kino_path_finder_->reset();
            status = kino_path_finder_->search(start_pos,start_vel, start_acc, end_pos, end_vel, false);

            if(status == KinodynamicAstar::NO_PATH)
            {
                ROS_WARN_STREAM("[kino replan]: kinodynamic second search fail!" );
                return false;
            }
            else
            {
                ROS_WARN_STREAM("[kino replan]: kinodynamic second search success!" );
            }
        }
        else{
            ROS_WARN_STREAM("[kino replan]: kinodynamic search success!" );
        }

        plan_data_.kino_path_ = kino_path_finder_->getKinoTraj(0.01);

        t_search = (ros::Time::now() - t1).toSec();

        double ts = pp_.ctrl_pt_dist_ / pp_.max_vel_;
        vector<Vector3d> point_set, start_end_derivatives;
        kino_path_finder_->getSamples(ts,point_set,start_end_derivatives);

        MatrixXd ctrl_pts;
        UniformBspline::parameterizeToBspline(ts,point_set,start_end_derivatives,ctrl_pts);
        UniformBspline init(ctrl_pts,3,ts);

        /* TODO  */

    }




}