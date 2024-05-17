#include <plan_manager/plan_manager.h>
#include <plan_manager/plan_container.hpp>
#include <thread>


namespace fast_planner
{

    PlanManager::PlanManager(){}

    PlanManager::~PlanManager() {std::cout << "destroy plan-manager" << std::endl; }

    void PlanManager::initPlanModules(ros::NodeHandle& nh)
    {
        
        nh.param("manager/max_vel", pp_.max_vel_, -1.0);
        nh.param("manager/max_acc", pp_.max_acc_, -1.0);
        nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);

        nh.param("manager/dynamic_environment", pp_.dynamic_environment_, -1);
        nh.param("manager/local_segment_length", pp_.local_traj_len_, -1.0);
        nh.param("manager/control_points_distance", pp_.ctrl_pt_dist_, -1.0);

        bool use_kinodynamic_path, use_topo_path, use_optimzation;
        nh.param("manager/use_kinodynamic_path", use_kinodynamic_path,true);
        nh.param("manager/use_optimization", use_optimzation,false);
        nh.param("manager/use_topological_path", use_topo_path,false);



        // bool use_geometric_path, use_active_perception;

        local_data_.traj_id_ = 0;
        
        /* EnvManager */
        env_manager_ptr_.reset(new EnvManager);
        env_manager_ptr_->init(nh);
        ROS_INFO_STREAM("EnvManager init successfully !!!");

        /* VisualRviz */
        traj_visual_ptr_.reset(new TrajVisualizer);
        traj_visual_ptr_->init(nh);
        ROS_INFO_STREAM("VisualRviz init successfully !!!");

        /* Kinodynamic Astar */
        if(use_kinodynamic_path)
        {
            ROS_INFO_STREAM("use kinodynamic path");
            kino_path_finder_ptr_.reset(new KinodynamicAstar);
            kino_path_finder_ptr_->init(nh,env_manager_ptr_,pp_.max_vel_,pp_.max_acc_);
            ROS_INFO_STREAM("kinodynamic search init successfully !!!");
        }
        


        /* Optimization threads */
        if(use_optimzation)
        {
            ROS_INFO_STREAM("use optimization");

            bspline_optimizer_ptrs_.resize(10);
            for(int i=0; i<10; i++)
            {
                bspline_optimizer_ptrs_[i].reset(new BsplineOptimizer);
                bspline_optimizer_ptrs_[i]->init(nh,env_manager_ptr_,pp_.max_vel_,pp_.max_acc_);
            }
            ROS_INFO_STREAM("optimization init successfully !!!");
        }

        /* Topological Path Searching */
        if(use_topo_path)
        {
            ROS_INFO_STREAM("use topological path");
            topo_prm_ptr_.reset(new TopoPRM);
            topo_prm_ptr_->init(nh,env_manager_ptr_);
            ROS_INFO_STREAM("topological path init successfully !!!");
        }
        ROS_INFO_STREAM("plan manager init successfully !!!");
    }


    void PlanManager::setGlobalWaypoints(vector<Vector3d>& waypoints)
    {
        plan_data_.global_waypoints_ = waypoints;
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
            bool collision = env_manager_ptr_->checkCollisionInGridMap(fut_pt);
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

        /* 1. kinodynamic path searhcing */
        t1 = ros::Time::now();
        kino_path_finder_ptr_->reset();
        ROS_INFO_STREAM("START SEARCH");
        int status = kino_path_finder_ptr_->search(start_pos, start_vel, start_acc, end_pos, end_vel, true);
        ROS_INFO_STREAM("FINISH START SEARCH");
        ROS_INFO_STREAM("search status : " << status);
        if(status == KinodynamicAstar::NO_PATH)
        {
            ROS_WARN_STREAM("[kino replan]: kinodynamic search fail!" );
            kino_path_finder_ptr_->reset();
            status = kino_path_finder_ptr_->search(start_pos,start_vel, start_acc, end_pos, end_vel, false);

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
    
        ROS_INFO_STREAM("FINISH SEARCH");
        plan_data_.kino_path_ = kino_path_finder_ptr_->getKinoTraj(0.01);
        ROS_INFO_STREAM("path size : " << plan_data_.kino_path_.size());
        if(plan_data_.kino_path_.size() == 0)
        {
            return false;
        }

        t_search =  (ros::Time::now() - t1).toSec();
        traj_visual_ptr_->visualizeKinodynamicTraj(plan_data_.kino_path_, false,false);

        /* 2. parameterize the path to bspline */
        ROS_INFO_STREAM("ctrl_pt_dist : " << pp_.ctrl_pt_dist_ << "max_vel : " << pp_.max_vel_);
        double ts = pp_.ctrl_pt_dist_ / pp_.max_vel_;
        vector<Vector3d> point_set, start_end_derivatives;
        kino_path_finder_ptr_->getSamples(ts,point_set,start_end_derivatives);

        MatrixXd ctrl_pts;
        UniformBspline::parameterizeToBspline(ts,point_set,start_end_derivatives,ctrl_pts);
        UniformBspline init(ctrl_pts,3,ts);
        traj_visual_ptr_->visualizeBsplineTraj(point_set,ctrl_pts,true);

        /* 3.1 bound optimization */
        t1 = ros::Time::now();
        
        // int cost_function = BsplineOptimizer

        // t_search = (ros::Time::now() - t1).toSec();

        return true;


        /* TODO  */

    }

    bool PlanManager::planGlobalTraj(Vector3d start_pos)
    {

        vector<Vector3d> points = plan_data_.global_waypoints_;
        if(points.size() == 0)
        {
            ROS_INFO_STREAM("no global waypooints");
        }        

        points.insert(points.begin(), start_pos);

        vector<Vector3d> inter_points;
        const double     dist_thresh = 4.0;

        for(int i = 0; i < points.size() - 1; i++)
        {
            inter_points.push_back(points.at(i));

            double dist = (points.at(i + 1) - points.at(i)).norm();

            if(dist > dist_thresh)
            {
                int id_num = floor(dist /dist_thresh) + 1;
                for(int j = 1; j < id_num; j++)
                {
                    Vector3d inter_pt = points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                    inter_points.push_back(inter_pt);
                }
            }
        }

        inter_points.push_back(points.back());

        if(inter_points.size() == 2) // minimum snap 函数无法处理两端点轨迹
        {
            Vector3d mid = (inter_points[0] + inter_points[1]) * 0.5;
            inter_points.insert(inter_points.begin() + 1, mid);
        }


        // write position matrix 
        int         pt_num = inter_points.size();
        MatrixXd    pos(3,pt_num);
        for(int i = 0; i < pt_num; i++)
        {
            pos.col(i) = inter_points[i];
        }

        Vector3d zero(0,0,0);
        VectorXd time(pt_num - 1);
        for(int i = 0; i < pt_num - 1; i++)
        {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
        }

        time(0) *= 2.0;
        time(0) = max(1.0, time(0));
        time(time.cols() - 1) *= 2.0;
        time(time.cols() - 1) = max(1.0, time(time.cols() - 1));

        PolynomialTraj gl_traj = PolynomialTraj::minSnapTraj(pos,zero,zero,zero,zero,time);

        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj,time_now);
        ROS_INFO_STREAM("global trajectory generated");

          // truncate a local trajectory
        double          dt, duration;
        MatrixXd        ctrl_pts = reparamLocalTraj(0.0, dt, duration);
        UniformBspline  bspline(ctrl_pts,3,dt);
        global_data_.setLocalTraj(bspline, 0.0, duration, 0.0);
        updateTrajInfo(bspline, time_now);
        ROS_INFO_STREAM("finish update local trajectory");
        return true;
    }

    bool PlanManager::emergencyStop(Vector3d &pos)
    {
        MatrixXd control_points(3,6);
        for(int i=0;i<6;i++)
        {
            control_points.col(i) = pos;
        }

        updateTrajInfo(UniformBspline(control_points,3,1.0),ros::Time::now());
        return true;
    }

    void PlanManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
    {
        local_data_.start_time_             = time_now;
        local_data_.position_traj_          = position_traj;
        local_data_.velocity_traj_          = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_      = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_              = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_               = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;

    }

    MatrixXd PlanManager::reparamLocalTraj(double start_t, double &dt, double &duration)
    {
         /* get the sample points local traj within radius */
        vector<Vector3d> point_set;
        vector<Vector3d> start_end_derivative;

        global_data_.getTrajByRadius(start_t, pp_.local_traj_len_, pp_.ctrl_pt_dist_, point_set, start_end_derivative, dt, duration);


        /* parameterization of B-spline */
        MatrixXd ctrl_pts;
        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);

        return ctrl_pts;
    }

    MatrixXd PlanManager::reparamLocalTraj(double start_t, double duration, int seg_num, double &dt)
    {
        vector<Vector3d> point_set;
        vector<Vector3d> start_end_derivative;

        global_data_.getTrajByDuration(start_t, duration, seg_num, point_set, start_end_derivative, dt);
        plan_data_.local_start_end_derivatives_ = start_end_derivative;

        /* parameterization of B-spline */
        Eigen::MatrixXd ctrl_pts;
        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);

        return ctrl_pts;
    }

    void PlanManager::reparamBspline(UniformBspline &bspline, vector<Vector3d> &start_end_derivative, double ratio, MatrixXd &ctrl_pts, double &dt, double &time_inc)
    {
        int prev_num        = bspline.getControlPoint().cols();
        double time_origin  = bspline.getTimeSum();
        int seg_num         = bspline.getControlPoint().cols() - 3;

        ratio = min(1.01, ratio);
        bspline.lengthenTime(ratio);
        double duration     = bspline.getTimeSum();
        dt                  = duration / double(seg_num);
        time_inc            = duration - time_origin; // 时间增量
        
        vector<Vector3d> point_set;
        for(double time=0.0; time <= duration + 1e-4; time += dt)
        {
            point_set.push_back(bspline.evaluateDeBoor(time));
        }

        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    }

} // namespace fast_planner




