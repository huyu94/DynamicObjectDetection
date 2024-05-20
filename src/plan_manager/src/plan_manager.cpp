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


    // void PlanManager::setGlobalWaypoints(vector<Vector3d>& waypoints)
    // {
    //     plan_data_.global_waypoints_ = waypoints;
    // }

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




    bool PlanManager::kinodynamicReplan(const Vector3d& start_pos, const Vector3d& start_vel, const Vector3d& start_acc,
                                        const Vector3d& end_pos, const Vector3d& end_vel)
    {
        // ROS_INFO_STREAM("[kino replan]: ---------------------");
        // ROS_INFO_STREAM("start: " << start_pos.transpose() << ", " << start_vel.transpose() << ", " 
        //                 << start_acc.transpose() << "\ngoal:" << end_pos.transpose() << ", " 
        //                 << end_vel.transpose());

        if((start_pos - end_pos).norm() < 0.2)
        {
            ROS_INFO_STREAM("Close goal");
            return false;
        }

        ros::Time t_start;
        local_data_.start_time_ = ros::Time::now();
        ros::Duration t_search, t_opt, t_refine;

        Vector3d init_pos = start_pos;
        Vector3d init_vel = start_vel;
        Vector3d init_acc = start_acc;

        /* 1. kinodynamic path searhcing */
        t_start = ros::Time::now();
        kino_path_finder_ptr_->reset();
        int status = kino_path_finder_ptr_->search(start_pos, start_vel, start_acc, end_pos, end_vel, true, ros::Time::now());
        if(status == KinodynamicAstar::NO_PATH)
        {
            ROS_WARN_STREAM("[kino replan]: kinodynamic search fail!" );
            kino_path_finder_ptr_->reset();
            status = kino_path_finder_ptr_->search(start_pos,start_vel, start_acc, end_pos, end_vel, false, ros::Time::now());

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
    
        plan_data_.kino_path_ = kino_path_finder_ptr_->getKinoTraj(0.01);
        // ROS_INFO_STREAM("path size : " << plan_data_.kino_path_.size());
        if(plan_data_.kino_path_.size() == 0)
        {
            return false;
        }
        double ts = pp_.ctrl_pt_dist_ / pp_.max_vel_;
        vector<Vector3d> point_set, start_end_derivatives;
        kino_path_finder_ptr_->getSamples(ts,point_set,start_end_derivatives);

        t_search =  (ros::Time::now() - t_start);
        traj_visual_ptr_->visualizeKinodynamicTraj(plan_data_.kino_path_, false,false);

        /* 2. parameterize the path to bspline */
        // ROS_INFO_STREAM("ctrl_pt_dist : " << pp_.ctrl_pt_dist_ << "max_vel : " << pp_.max_vel_);




        t_start = ros::Time::now();
        MatrixXd ctrl_pts;
        UniformBspline::parameterizeToBspline(ts,point_set,start_end_derivatives,ctrl_pts);
        UniformBspline init(ctrl_pts,3,ts);
        // traj_visual_ptr_->visualizeBsplineTraj(point_set,ctrl_pts,true);

        /* 3.1 bound optimization */
        // ROS_INFO_STREAM("start a* search");
        vector<vector<Vector3d>> astar_pathes;
        astar_pathes = bspline_optimizer_ptrs_[0]->initControlPoints(ctrl_pts,true);
        // ROS_INFO_STREAM("end a* search");
        // ROS_INFO_STREAM("astar_pathes size : " << astar_pathes.size());
        // traj_visual_ptr_->visualizeAstarPath(astar_pathes,false);

        bool flag_step_1_success = bspline_optimizer_ptrs_[0]->BsplineOptimizeTrajRebound(ctrl_pts, ts);

        // ROS_INFO_STREAM("flag_step_1_success : " << flag_step_1_success);
        if(!flag_step_1_success)
        {
            continuous_failures_count_++;
            return false;
        }
        // ROS_INFO_STREAM("ctrl_pts size : " << ctrl_pts.cols());
        vector<Vector3d> vis_point_set;
        // UniformBspline::parameterizeToBspline(ts,vis_point_set,start_end_derivatives,ctrl_pts);
        UniformBspline rebound_traj(ctrl_pts,3,ts);
        t_opt = (ros::Time::now() - t_start);

        // double rebound_t_sum = rebound_traj.getTimeSum();
        // for(double i = 0; i < rebound_t_sum; i+= 0.1)
        // {
        //     vis_point_set.push_back(rebound_traj.evaluateDeBoorT(i));
        
        // }
        // traj_visual_ptr_->visualizeOptimalTraj(vis_point_set,false,true);
        /* 3.2 refine */
        t_start = ros::Time::now();
        rebound_traj.setPhysicalLimits(pp_.max_vel_,pp_.max_acc_,pp_.feasibility_tolerance_);
        double ratio;
        bool flag_step_2_success = true;
        Eigen::MatrixXd optimal_control_points;
        if(!rebound_traj.checkFeasibility(ratio,false))
        {
            // ROS_INFO_STREAM("Need to reallocate time");
            
            flag_step_2_success = refineTraj(rebound_traj, start_end_derivatives, ratio, ts, optimal_control_points);
            if(flag_step_2_success)
            {
                rebound_traj = UniformBspline(optimal_control_points,3,ts);
            }
        }
        t_refine = ros::Time::now() - t_start;

        if (!flag_step_2_success)
        {
            printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
            continuous_failures_count_++;
            return false;
        }
        vector<Vector3d> opti_point_set = sampleTraj(optimal_control_points,ts);
        traj_visual_ptr_->visualizeOptimalTraj(opti_point_set,false,true);


        updateTrajInfo(rebound_traj,ros::Time::now());



        static double sum_time = 0;
        static int count_success = 0;
        sum_time += (t_search + t_opt + t_refine).toSec();
        count_success++;
        ROS_INFO_STREAM("total_time : " << (t_search + t_opt + t_refine).toSec() 
                        << ", search_time : " << t_search.toSec()
                        << ", opt_time : " << t_opt.toSec() 
                        << ", refine_time : " << t_refine.toSec()
                        << ", avg time : " << sum_time / count_success);
        // cout << "total time:\033[42m" << (t_search + t_opt + t_refine).toSec() << "\033[0m,optimize:" << t_opt.toSec() << ",refine:" << t_refine.toSec() << endl;

        


        
        // int cost_function = BsplineOptimizer

        // t_search = (ros::Time::now() - t1).toSec();
        

        return true;


        /* TODO  */

    }

    bool PlanManager::planGlobalTraj(const Vector3d& start_pos, const Vector3d& start_vel, const Vector3d& start_acc,
                                     const Vector3d& end_pos, const Vector3d& end_vel, const Vector3d& end_acc)
    {

        vector<Vector3d> points;
        points.push_back(start_pos);
        points.push_back(end_pos);

        // insert intermediate points if too far
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



        // write position matrix
        int             pt_num          = inter_points.size();
        Eigen::MatrixXd pos(3, pt_num);
        for (int i = 0; i < pt_num; ++i)
        {
            pos.col(i) = inter_points[i];
        }

        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd time(pt_num - 1);
        for (int i = 0; i < pt_num - 1; ++i)
        {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
        }

        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;

        PolynomialTraj gl_traj;
        if (pos.cols() >= 3)
        {
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
        }
        else if (pos.cols() == 2)
        {
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
        }
        else
        {
            return false;
        }



        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj,time_now);
        ROS_INFO_STREAM("global trajectory generated");


        return true;
    }

    bool PlanManager::planGlobalTrajWaypoints(const Vector3d& start_pos, const Vector3d& start_vel, const Vector3d& start_acc,
                                                std::vector<Vector3d> &waypoints, const Vector3d& end_vel, const Vector3d& end_acc)
    {
        // generate global reference trajectory;

        vector<Vector3d> points;
        points.push_back(start_pos);

        for(size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
        {
            points.push_back(waypoints[wp_i]);
        }

        double total_len = 0;
        total_len += (start_pos - waypoints[0]).norm();
        for(size_t i = 0; i < waypoints.size() - 1; i++)
        {
            total_len += (waypoints[i] - waypoints[i + 1]).norm();
        }

        // insert intermediate points if too far
        vector<Vector3d> inter_points;
        double dist_thresh = max(total_len / 8, 4.0);

        for(size_t i = 0; i < points.size() -1; i++)
        {
            inter_points.push_back(points.at(i));
            double dist = (points.at(i + 1) - points.at(i)).norm();

            if (dist > dist_thresh)
            {
                int id_num = floor(dist / dist_thresh) + 1;

                for (int j = 1; j < id_num; ++j)
                {
                Eigen::Vector3d inter_pt =
                    points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                inter_points.push_back(inter_pt);
                }
            }
        }
        inter_points.push_back(points.back());

                // write position matrix
        int pt_num = inter_points.size();
        Eigen::MatrixXd pos(3, pt_num);
        for (int i = 0; i < pt_num; ++i)
        {
            pos.col(i) = inter_points[i];
        }

        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd time(pt_num - 1);
        for (int i = 0; i < pt_num - 1; ++i)
        {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
        }

        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;

        PolynomialTraj gl_traj;
        if (pos.cols() >= 3)
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
        else if (pos.cols() == 2)
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
        else
            return false;

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

    bool PlanManager::refineTraj(UniformBspline &traj, vector<Vector3d> &start_end_derivatives, double ratio, double &ts, MatrixXd &optimal_control_points)
    {
        double t_inc;
        Eigen::MatrixXd ctrl_pts;

        reparamBspline(traj, start_end_derivatives, ratio, ctrl_pts, ts, t_inc);

        traj = UniformBspline(ctrl_pts, 3, ts);

        double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
        bspline_optimizer_ptrs_[0]->ref_pts_.clear();
        for(double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
        {
            bspline_optimizer_ptrs_[0]->ref_pts_.push_back(traj.evaluateDeBoor(t));
        }
        bool success = bspline_optimizer_ptrs_[0]->BsplineOptimizeTrajRefine(ctrl_pts,ts,optimal_control_points);

        return success;
    }

    vector<Vector3d> PlanManager::sampleTraj(Eigen::MatrixXd& ctrl_pts, double ts)
    {
        UniformBspline traj(ctrl_pts,3,ts);
        vector<Vector3d> point_set;
        double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);

        for(double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
        {
            point_set.push_back(traj.evaluateDeBoor(t));
        }
        return point_set;

    }

} // namespace fast_planner




