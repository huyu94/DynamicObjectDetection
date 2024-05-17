#include "plan_manager/uniform_bspline_opt/bspline_optimizer.h"
#include "plan_manager/uniform_bspline_opt/gradient_descent_optimizer.h"
#include <plan_manager/uniform_bspline_opt/lbfgs.hpp>
#include <plan_manager/path_searching/astar.h>
// using namespace std;

namespace fast_planner
{

    void BsplineOptimizer::init(const ros::NodeHandle &nh, const EnvManager::Ptr &env_manager, double max_vel, double max_acc)
    {
        this->setParam(nh);
        this->setEnvironment(env_manager);
        max_vel_ = max_vel;
        max_acc_ = max_acc;
        this->a_star_.reset(new AStar);
        this->a_star_->initGridMap(env_manager_->getGridMap(), Eigen::Vector3i(100, 100, 100));
    }

    void BsplineOptimizer::setParam(const ros::NodeHandle &nh)
    {
        nh.param("optimization/lambda_smooth", lambda1_, -1.0);
        nh.param("optimization/lambda_collision", lambda2_, -1.0);
        nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
        nh.param("optimization/lambda_fitness", lambda4_, -1.0);

        nh.param("optimization/dist0", dist0_, -1.0);
        nh.param("optimization/order", order_, 3);
        // ROS_INFO_STREAM("lambda_smooth: " << lambda1_);
        // ROS_INFO_STREAM("lambda_collision: " << lambda2_);
        // ROS_INFO_STREAM("lambda_feasibility: " << lambda3_);
        // ROS_INFO_STREAM("lambda_fitness: " << lambda4_);
        // ROS_INFO_STREAM("dist0: " << dist0_);
        // ROS_INFO_STREAM("order: " << order_);
    }

    void BsplineOptimizer::setEnvironment(const EnvManager::Ptr &env_manager)
    {
        this->env_manager_ = env_manager;
    }

    void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
    {
        cps_.points = points;
    }

    void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

    /* This function is very similar to check_collision_and_rebound().
     * It was written separately, just because I did it once and it has been running stably since March 2020.
     * But I will merge then someday.*/
    std::vector<std::vector<Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
    {

        if (flag_first_init)
        {
            cps_.clearance = dist0_;
            cps_.resize(init_points.cols());
            cps_.points = init_points;
        }
        // ROS_INFO_STREAM("after flag first init");
        // ROS_INFO_STREAM("init_points cols = " << init_points.cols());
        /*** Segment the initial trajectory according to obstacles ***/
        constexpr int ENOUGH_INTERVAL = 2;
        double step_size = env_manager_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 2;
        // ROS_INFO_STREAM("step size : " << step_size);
        int in_id, out_id;
        vector<std::pair<int, int>> segment_ids;
        int same_occ_state_times = ENOUGH_INTERVAL + 1;
        bool occ, last_occ = false;
        bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
        int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; // only check closed 2/3 points.

        // ROS_INFO_STREAM("i end = " << i_end);
        // ROS_INFO_STREAM("order_ = " << order_);
        for (int i = order_; i <= i_end; ++i)
        {
            // ROS_INFO_STREAM(" *" << i-1 << "*");
            // cout <<  ;
            for (double a = 1.0; a > 0.0; a -= step_size)
            {
                // ROS_INFO_STREAM(" *" << a << "*");

                occ = env_manager_->checkCollisionInGridMap(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
                // cout << " " << occ;
                // cout << setprecision(5);
                // cout << (a * init_points.col(i - 1) + (1 - a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

                if (occ && !last_occ)
                {
                    if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
                    {
                        in_id = i - 1;
                        flag_got_start = true;
                    }
                    same_occ_state_times = 0;
                    flag_got_end_maybe = false; // terminate in advance
                }
                else if (!occ && last_occ)
                {
                    out_id = i;
                    flag_got_end_maybe = true;
                    same_occ_state_times = 0;
                }
                else
                {
                    ++same_occ_state_times;
                }

                if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
                {
                    flag_got_end_maybe = false;
                    flag_got_end = true;
                }

                last_occ = occ;

                if (flag_got_start && flag_got_end)
                {
                    flag_got_start = false;
                    flag_got_end = false;
                    segment_ids.push_back(std::pair<int, int>(in_id, out_id));
                }
            }
        }
        // ROS_INFO_STREAM("in [init control point] : astar search 1");
        // ROS_INFO_STREAM("segment_ids size = " << segment_ids.size());
        /*** a star search ***/
        vector<vector<Eigen::Vector3d>> a_star_pathes;
        for (size_t i = 0; i < segment_ids.size(); ++i)
        {
            // cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
            Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
            ROS_INFO_STREAM("in : " << segment_ids[i].first << ";" << "out : " << segment_ids[i].second);

            if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
            {
                // ROS_INFO_STREAM("in [astar searhcing]");
                a_star_pathes.push_back(a_star_->getPath());
            }
            else
            {
                ROS_ERROR("a star error, force return!");
                return a_star_pathes;
            }
        }

        // ROS_INFO_STREAM("in [init control point] : astar search 2");
        /**
         * segments_ids[i]: 一段进入障碍物的轨迹控制点
         * segments_ids[i].first: 进入障碍物的轨迹前一个控制点的id
         * segments_ids[i].second: 离开障碍物的轨迹第一个控制点的id
         */
        /*** calculate bounds ***/
        int id_low_bound, id_up_bound;
        vector<std::pair<int, int>> bounds(segment_ids.size());
        for (size_t i = 0; i < segment_ids.size(); i++)
        {

            if (i == 0) // first segment
            {
                id_low_bound = order_;
                if (segment_ids.size() > 1)
                {
                    id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
                }
                else
                {
                    id_up_bound = init_points.cols() - order_ - 1;
                }
            }
            else if (i == segment_ids.size() - 1) // last segment, i != 0 here
            {
                id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
                id_up_bound = init_points.cols() - order_ - 1;
            }
            else
            {
                id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
                id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
            }

            bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
        }

        /*** Adjust segment length ***/
        vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
        constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient force
        int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
        // MINIMUM_PERCENT = 0.0, 那么minimum_points不是怎么都是0吗
        for (size_t i = 0; i < segment_ids.size(); i++)
        {
            /*** Adjust segment length ***/
            num_points = segment_ids[i].second - segment_ids[i].first + 1; // 碰撞片段中控制点的数量
            // cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
            if (num_points < minimum_points)
            {
                double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

                final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

                final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
            }
            else
            {
                final_segment_ids[i].first = segment_ids[i].first;
                final_segment_ids[i].second = segment_ids[i].second;
            }

            // cout << "final:" << "i = " << i << " first = " << adjusted_segment_ids[i].first << " second = " << adjusted_segment_ids[i].second << endl;
        }

        /*** Assign data to each segment ***/
        for (size_t i = 0; i < segment_ids.size(); i++)
        {
            // step 1
            for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
                cps_.flag_temp[j] = false;

            // step 2
            int got_intersection_id = -1;
            for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
            {
                // j: 第一个进障碍物的点
                // j + 1 : 进障碍物的第二个点
                // j - 1 : 进障碍物的前一个点
                Eigen::Vector3d ctrl_pts_law(init_points.col(j + 1) - init_points.col(j - 1));
                Vector3d intersection_point;
                // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
                int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id;
                double val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law), last_val = val;
                while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
                {
                    last_Astar_id = Astar_id;

                    if (val >= 0)
                        --Astar_id;
                    else
                        ++Astar_id;

                    val = (a_star_pathes[i][Astar_id] - init_points.col(j)).dot(ctrl_pts_law);

                    if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
                    {
                        intersection_point =
                            a_star_pathes[i][Astar_id] +
                            ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                             (ctrl_pts_law.dot(init_points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                            );

                        // cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

                        got_intersection_id = j;
                        break;
                    }
                }

                if (got_intersection_id >= 0)
                {
                    cps_.flag_temp[j] = true;
                    double length = (intersection_point - init_points.col(j)).norm();
                    if (length > 1e-5)
                    {
                        for (double a = length; a >= 0.0; a -= env_manager_->getResolution())
                        {
                            occ = env_manager_->checkCollisionInGridMap((a / length) * intersection_point + (1 - a / length) * init_points.col(j));

                            if (occ || a < env_manager_->getResolution())
                            {
                                if (occ)
                                {
                                    a += env_manager_->getResolution();
                                }
                                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * init_points.col(j));
                                cps_.direction[j].push_back((intersection_point - init_points.col(j)).normalized());
                                // cout << "A " << j << endl;
                                break;
                            }
                        }
                    }
                }
            }

            /* Corner case: the segment length is too short.
            Here the control points may outside the A* path, leading to opposite gradient direction.
            So I have to take special care of it */
            if (segment_ids[i].second - segment_ids[i].first == 1)
            {
                Eigen::Vector3d ctrl_pts_law(init_points.col(segment_ids[i].second) - init_points.col(segment_ids[i].first)), intersection_point;
                Eigen::Vector3d middle_point = (init_points.col(segment_ids[i].second) + init_points.col(segment_ids[i].first)) / 2;
                int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
                double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
                while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
                {
                    last_Astar_id = Astar_id;

                    if (val >= 0)
                        --Astar_id;
                    else
                        ++Astar_id;

                    val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

                    if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
                    {
                        intersection_point =
                            a_star_pathes[i][Astar_id] +
                            ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                             (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                            );

                        if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
                        {
                            cps_.flag_temp[segment_ids[i].first] = true;
                            cps_.base_point[segment_ids[i].first].push_back(init_points.col(segment_ids[i].first));
                            cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

                            got_intersection_id = segment_ids[i].first;
                        }
                        break;
                    }
                }
            }

            // step 3
            if (got_intersection_id >= 0)
            {
                for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
                {
                    if (!cps_.flag_temp[j])
                    {
                        cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
                        cps_.direction[j].push_back(cps_.direction[j - 1].back());
                        // cout << "AAA " << j << endl;
                    }
                }

                for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
                {
                    if (!cps_.flag_temp[j])
                    {
                        cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
                        cps_.direction[j].push_back(cps_.direction[j + 1].back());
                        // cout << "AAAA " << j << endl;
                    }
                }

                // final_segment_ids.push_back(final_segment_ids[i]);
            }
            else
            {
                // Just ignore, it does not matter ^_^.
                // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
            }
        }

        return a_star_pathes;
    }

    int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
    {
        BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
        // cout << "k=" << k << endl;
        // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
        return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
    }

    double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
    {
        BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

        double cost;
        opt->combineCostRebound(x, grad, cost, n);

        opt->iter_num_ += 1;
        return cost;
    }

    double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
    {
        BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

        double cost;
        opt->combineCostRefine(x, grad, cost, n);

        opt->iter_num_ += 1;
        return cost;
    }

    void BsplineOptimizer::calcMovingObjCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
    {
        cost = 0.0;
        int end_idx = q.cols() - order_;
        // int end_idx = q.cols() - order_ - (double)(q.cols() - 2 * order_) * 1.0 / 3.0; // Only check the first 2/3 points

        constexpr double CLEARANCE = 2.0;
        ros::Time t_now = ros::Time::now();

        vector<Tracker::Ptr> alive_trackers;
        env_manager_->getTrackerPool()->getAliveTracker(alive_trackers);
        for (int i = order_; i < end_idx; i++)
        {
            double time = (double)(i - order_ + 1) * bspline_interval_;
            // double time = ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;
            ros::Duration inc_time = ros::Duration(time);
            for (int id = 0; id < alive_trackers.size(); id++)
            {
                Eigen::VectorXd obj_prid_state = alive_trackers[id]->forward(t_now + inc_time);
                Eigen::Vector3d obj_prid_pos = obj_prid_state.head(3);
                double dist = (cps_.points.col(i) - obj_prid_pos).norm();
                double dist_err = CLEARANCE - dist;

                Eigen::Vector3d dist_grad = (cps_.points.col(i) - obj_prid_pos).normalized();

                if (dist_err < 0)
                {
                    /* do nothing */
                }
                else
                {
                    cost += pow(dist_err, 2);
                    gradient.col(i) += -2.0 * dist_err * dist_grad;
                }
            }
        }
    }

    // void BsplineOptimizer::calcMovingObjCost2(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
    // {
    //     cost = 0.0;
    //     int end_idx = q.cols() - order_;
    //     constexpr double CLEARANCE = 1.5;
    //     double t_now = ros::Time::now().toSec();

    //     for (int i = order_; i < end_idx; i++)
    //     {
    //         double time = ((double)(order_ - 1) / 2 + (i - order_ + 1)) * bspline_interval_;

    //         for (int id = 0; id < moving_objs_->getObjNums(); id++)
    //         {
    //             Eigen::Vector3d obj_prid = moving_objs_->evaluateConstVel(id, t_now + time);
    //             double dist = (cps_.points.col(i) - obj_prid).norm();
    //             // cout /*<< "cps_.points.col(i)=" << cps_.points.col(i).transpose()*/ << " moving_objs_=" << obj_prid.transpose() << " dist=" << dist << endl;
    //             double dist_err = CLEARANCE - dist;
    //             Eigen::Vector3d dist_grad = (cps_.points.col(i) - obj_prid).normalized();

    //             if (dist_err < 0)
    //             {
    //                 /* do nothing */
    //             }
    //             else
    //             {
    //                 cost += pow(dist_err, 2);
    //                 gradient.col(i) += -2.0 * dist_err * dist_grad;
    //             }
    //         }
    //         // cout << "time=" << time << " i=" << i << " order_=" << order_ << " end_idx=" << end_idx << endl;
    //         // cout << "--" << endl;
    //     }
    //     // cout << "---------------" << endl;
    // }

    void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                   Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
    {
        cost = 0.0;
        int end_idx = q.cols() - order_;
        double demarcation = cps_.clearance;
        double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

        force_stop_type_ = DONT_STOP;
        if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
        {
            check_collision_and_rebound();
        }

        /*** calculate distance cost and gradient ***/
        for (auto i = order_; i < end_idx; ++i)
        {
            for (size_t j = 0; j < cps_.direction[i].size(); ++j)
            {
                double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
                double dist_err = cps_.clearance - dist;
                Eigen::Vector3d dist_grad = cps_.direction[i][j];

                if (dist_err < 0)
                {
                    /* do nothing */
                }
                else if (dist_err < demarcation)
                {
                    cost += pow(dist_err, 3);
                    gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
                }
                else
                {
                    cost += a * dist_err * dist_err + b * dist_err + c;
                    gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
                }
            }
        }
    }

    void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
    {

        cost = 0.0;

        int end_idx = q.cols() - order_;

        // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
        double a2 = 25, b2 = 1;
        for (auto i = order_ - 1; i < end_idx + 1; ++i)
        {
            Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
            Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

            double xdotv = x.dot(v);
            Eigen::Vector3d xcrossv = x.cross(v);

            double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
            cost += f;

            Eigen::Matrix3d m;
            m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
            Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

            gradient.col(i - 1) += df_dx / 6;
            gradient.col(i) += 4 * df_dx / 6;
            gradient.col(i + 1) += df_dx / 6;
        }
    }

    void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                              Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
    {

        cost = 0.0;

        if (falg_use_jerk)
        {
            Eigen::Vector3d jerk, temp_j;

            for (int i = 0; i < q.cols() - 3; i++)
            {
                /* evaluate jerk */
                jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
                cost += jerk.squaredNorm();
                temp_j = 2.0 * jerk;
                /* jerk gradient */
                gradient.col(i + 0) += -temp_j;
                gradient.col(i + 1) += 3.0 * temp_j;
                gradient.col(i + 2) += -3.0 * temp_j;
                gradient.col(i + 3) += temp_j;
            }
        }
        else
        {
            Eigen::Vector3d acc, temp_acc;

            for (int i = 0; i < q.cols() - 2; i++)
            {
                /* evaluate acc */
                acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
                cost += acc.squaredNorm();
                temp_acc = 2.0 * acc;
                /* acc gradient */
                gradient.col(i + 0) += temp_acc;
                gradient.col(i + 1) += -2.0 * temp_acc;
                gradient.col(i + 2) += temp_acc;
            }
        }
    }

    void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                               Eigen::MatrixXd &gradient)
    {

        // #define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

        cost = 0.0;
        double demarcation = 1.0; // 1m/s, 1m/s/s
        double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
        double al = ar, bl = -br, cl = cr;

        /* abbreviation */
        double ts, ts_inv2, ts_inv3;
        ts = bspline_interval_;
        ts_inv2 = 1 / ts / ts;
        ts_inv3 = 1 / ts / ts / ts;

        /* velocity feasibility */
        for (int i = 0; i < q.cols() - 1; i++)
        {
            Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

            for (int j = 0; j < 3; j++)
            {
                if (vi(j) > max_vel_ + demarcation)
                {
                    double diff = vi(j) - max_vel_;
                    cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

                    double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
                    gradient(j, i + 0) += -grad;
                    gradient(j, i + 1) += grad;
                }
                else if (vi(j) > max_vel_)
                {
                    double diff = vi(j) - max_vel_;
                    cost += pow(diff, 3) * ts_inv3;
                    ;

                    double grad = 3 * diff * diff / ts * ts_inv3;
                    ;
                    gradient(j, i + 0) += -grad;
                    gradient(j, i + 1) += grad;
                }
                else if (vi(j) < -(max_vel_ + demarcation))
                {
                    double diff = vi(j) + max_vel_;
                    cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

                    double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
                    gradient(j, i + 0) += -grad;
                    gradient(j, i + 1) += grad;
                }
                else if (vi(j) < -max_vel_)
                {
                    double diff = vi(j) + max_vel_;
                    cost += -pow(diff, 3) * ts_inv3;

                    double grad = -3 * diff * diff / ts * ts_inv3;
                    gradient(j, i + 0) += -grad;
                    gradient(j, i + 1) += grad;
                }
                else
                {
                    /* nothing happened */
                }
            }
        }

        /* acceleration feasibility */
        for (int i = 0; i < q.cols() - 2; i++)
        {
            Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

            for (int j = 0; j < 3; j++)
            {
                if (ai(j) > max_acc_ + demarcation)
                {
                    double diff = ai(j) - max_acc_;
                    cost += ar * diff * diff + br * diff + cr;

                    double grad = (2.0 * ar * diff + br) * ts_inv2;
                    gradient(j, i + 0) += grad;
                    gradient(j, i + 1) += -2 * grad;
                    gradient(j, i + 2) += grad;
                }
                else if (ai(j) > max_acc_)
                {
                    double diff = ai(j) - max_acc_;
                    cost += pow(diff, 3);

                    double grad = 3 * diff * diff * ts_inv2;
                    gradient(j, i + 0) += grad;
                    gradient(j, i + 1) += -2 * grad;
                    gradient(j, i + 2) += grad;
                }
                else if (ai(j) < -(max_acc_ + demarcation))
                {
                    double diff = ai(j) + max_acc_;
                    cost += al * diff * diff + bl * diff + cl;

                    double grad = (2.0 * al * diff + bl) * ts_inv2;
                    gradient(j, i + 0) += grad;
                    gradient(j, i + 1) += -2 * grad;
                    gradient(j, i + 2) += grad;
                }
                else if (ai(j) < -max_acc_)
                {
                    double diff = ai(j) + max_acc_;
                    cost += -pow(diff, 3);

                    double grad = -3 * diff * diff * ts_inv2;
                    gradient(j, i + 0) += grad;
                    gradient(j, i + 1) += -2 * grad;
                    gradient(j, i + 2) += grad;
                }
                else
                {
                    /* nothing happened */
                }
            }
        }

#else

        cost = 0.0;
        /* abbreviation */
        double ts, /*vm2, am2, */ ts_inv2;
        // vm2 = max_vel_ * max_vel_;
        // am2 = max_acc_ * max_acc_;

        ts = bspline_interval_;
        ts_inv2 = 1 / ts / ts;

        /* velocity feasibility */
        for (int i = 0; i < q.cols() - 1; i++)
        {
            Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

            // cout << "temp_v * vi=" ;
            for (int j = 0; j < 3; j++)
            {
                if (vi(j) > max_vel_)
                {
                    // cout << "zx-todo VEL" << endl;
                    // cout << vi(j) << endl;
                    cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

                    gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
                    gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
                }
                else if (vi(j) < -max_vel_)
                {
                    cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

                    gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
                    gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
                }
                else
                {
                    /* code */
                }
            }
        }

        /* acceleration feasibility */
        for (int i = 0; i < q.cols() - 2; i++)
        {
            Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

            // cout << "temp_a * ai=" ;
            for (int j = 0; j < 3; j++)
            {
                if (ai(j) > max_acc_)
                {
                    // cout << "zx-todo ACC" << endl;
                    // cout << ai(j) << endl;
                    cost += pow(ai(j) - max_acc_, 2);

                    gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
                    gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
                    gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
                }
                else if (ai(j) < -max_acc_)
                {
                    cost += pow(ai(j) + max_acc_, 2);

                    gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
                    gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
                    gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
                }
                else
                {
                    /* code */
                }
            }
            // cout << endl;
        }

#endif
    }

    bool BsplineOptimizer::check_collision_and_rebound(void)
    {

        int end_idx = cps_.size - order_;

        /*** Check and segment the initial trajectory according to obstacles ***/
        int in_id, out_id;
        vector<std::pair<int, int>> segment_ids;
        bool flag_new_obs_valid = false;
        int i_end = end_idx - (end_idx - order_) / 3;
        for (int i = order_ - 1; i <= i_end; ++i)
        {

            bool occ = env_manager_->checkCollisionInGridMap(cps_.points.col(i));

            /*** check if the new collision will be valid ***/
            if (occ)
            {
                for (size_t k = 0; k < cps_.direction[i].size(); ++k)
                {
                    cout.precision(2);
                    if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * env_manager_->getResolution()) // current point is outside all the collision_points.
                    {
                        occ = false; // Not really takes effect, just for better hunman understanding.
                        break;
                    }
                }
            }

            if (occ)
            {
                flag_new_obs_valid = true;

                int j;
                for (j = i - 1; j >= 0; --j)
                {
                    occ = env_manager_->checkCollisionInGridMap(cps_.points.col(j));
                    if (!occ)
                    {
                        in_id = j;
                        break;
                    }
                }
                if (j < 0) // fail to get the obs free point
                {
                    ROS_ERROR("ERROR! the drone is in obstacle. This should not happen.");
                    in_id = 0;
                }

                for (j = i + 1; j < cps_.size; ++j)
                {
                    occ = env_manager_->checkCollisionInGridMap(cps_.points.col(j));

                    if (!occ)
                    {
                        out_id = j;
                        break;
                    }
                }
                if (j >= cps_.size) // fail to get the obs free point
                {
                    ROS_WARN("WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

                    force_stop_type_ = STOP_FOR_ERROR;
                    return false;
                }

                i = j + 1;

                segment_ids.push_back(std::pair<int, int>(in_id, out_id));
            }
        }

        if (flag_new_obs_valid)
        {
            vector<vector<Eigen::Vector3d>> a_star_pathes;
            for (size_t i = 0; i < segment_ids.size(); ++i)
            {
                /*** a star search ***/
                Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
                if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
                {
                    a_star_pathes.push_back(a_star_->getPath());
                }
                else
                {
                    ROS_ERROR("a star error");
                    segment_ids.erase(segment_ids.begin() + i);
                    i--;
                }
            }

            /*** Assign parameters to each segment ***/
            for (size_t i = 0; i < segment_ids.size(); ++i)
            {
                // step 1
                for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
                    cps_.flag_temp[j] = false;

                // step 2
                int got_intersection_id = -1;
                for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
                {
                    Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
                    int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
                    double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
                    while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
                    {
                        last_Astar_id = Astar_id;

                        if (val >= 0)
                            --Astar_id;
                        else
                            ++Astar_id;

                        val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

                        // cout << val << endl;

                        if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
                        {
                            intersection_point =
                                a_star_pathes[i][Astar_id] +
                                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                                 (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                                );

                            got_intersection_id = j;
                            break;
                        }
                    }

                    if (got_intersection_id >= 0)
                    {
                        cps_.flag_temp[j] = true;
                        double length = (intersection_point - cps_.points.col(j)).norm();
                        if (length > 1e-5)
                        {
                            for (double a = length; a >= 0.0; a -= env_manager_->getResolution())
                            {
                                bool occ = env_manager_->checkCollisionInGridMap((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                                if (occ || a < env_manager_->getResolution())
                                {
                                    if (occ)
                                        a += env_manager_->getResolution();
                                    cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                                    cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                                    break;
                                }
                            }
                        }
                        else
                        {
                            got_intersection_id = -1;
                        }
                    }
                }

                // step 3
                if (got_intersection_id >= 0)
                {
                    for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
                        if (!cps_.flag_temp[j])
                        {
                            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
                            cps_.direction[j].push_back(cps_.direction[j - 1].back());
                        }

                    for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
                        if (!cps_.flag_temp[j])
                        {
                            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
                            cps_.direction[j].push_back(cps_.direction[j + 1].back());
                        }
                }
                else
                    ROS_WARN("Failed to generate direction. It doesn't matter.");
            }

            force_stop_type_ = STOP_FOR_REBOUND;
            return true;
        }

        return false;
    }

    bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
    {
        setBsplineInterval(ts);

        double final_cost;
        bool flag_success = rebound_optimize(final_cost);

        optimal_points = cps_.points;

        return flag_success;
    }

    // bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double &final_cost, const ControlPoints &control_points, double ts)
    // {
    //     setBsplineInterval(ts);

    //     cps_ = control_points;

    //     bool flag_success = rebound_optimize(final_cost);

    //     optimal_points = cps_.points;

    //     return flag_success;
    // }

    bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
    {

        setControlPoints(init_points);
        setBsplineInterval(ts);

        bool flag_success = refine_optimize();

        optimal_points = cps_.points;

        return flag_success;
    }

    bool BsplineOptimizer::rebound_optimize(double &final_cost)
    {
        iter_num_ = 0;
        int start_id = order_;
        int end_id = this->cps_.size - order_; // Fixed end
        // int end_id = this->cps_.size; // Free end
        variable_num_ = 3 * (end_id - start_id);
        // double final_cost;

        ros::Time t0 = ros::Time::now(), t1, t2;
        int restart_nums = 0, rebound_times = 0;
        ;
        bool flag_force_return, flag_occ, success;
        new_lambda2_ = lambda2_;
        constexpr int MAX_RESART_NUMS_SET = 3;
        do
        {
            /* ---------- prepare ---------- */
            min_cost_ = std::numeric_limits<double>::max();
            // min_ellip_dist_ = INIT_min_ellip_dist_;
            iter_num_ = 0;
            flag_force_return = false;
            flag_occ = false;
            success = false;

            double q[variable_num_];
            memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

            lbfgs::lbfgs_parameter_t lbfgs_params;
            lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
            lbfgs_params.mem_size = 16;
            lbfgs_params.max_iterations = 200;
            lbfgs_params.g_epsilon = 0.01;

            /* ---------- optimize ---------- */
            t1 = ros::Time::now();
            int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
            t2 = ros::Time::now();
            double time_ms = (t2 - t1).toSec() * 1000;
            double total_time_ms = (t2 - t0).toSec() * 1000;

            /* ---------- success temporary, check collision again ---------- */
            if (result == lbfgs::LBFGS_CONVERGENCE ||
                result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
                result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
                result == lbfgs::LBFGS_STOP)
            {
                // ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
                flag_force_return = false;

                /*** collision check, phase 2 ***/
                UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
                double tm, tmp;
                traj.getTimeSpan(tm, tmp);
                ros::Time t_now = ros::Time::now();

                double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / env_manager_->getResolution());
                for (double t = tm; t < tmp * 2 / 3; t += t_step) // Only check the closest 2/3 partition of the whole trajectory.
                {
                    // 1. grid check
                    Eigen::Vector3d traj_pt = traj.evaluateDeBoorT(t);
                    flag_occ = env_manager_->checkCollisionInGridMap(traj_pt);

                    if (flag_occ)
                    {
                        // cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

                        if (t <= bspline_interval_) // First 3 control points in obstacles!
                        {
                            ROS_INFO_STREAM(cps_.points.col(1).transpose() << "\n"
                                                                           << cps_.points.col(2).transpose() << "\n"
                                                                           << cps_.points.col(3).transpose() << "\n"
                                                                           << cps_.points.col(4).transpose());
                            // cout << cps_.points.col(1).transpose() << "\n"
                            //      << cps_.points.col(2).transpose() << "\n"
                            //      << cps_.points.col(3).transpose() << "\n"
                            //      << cps_.points.col(4).transpose() << endl;
                            ROS_WARN("First 3 control points in obstacles! return false, t=%f", t);
                            return false;
                        }

                        break;
                    }
                }

                if (!flag_occ)
                {
                    printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
                    success = true;
                }
                else // restart
                {
                    restart_nums++;
                    initControlPoints(cps_.points, false);
                    new_lambda2_ *= 2;

                    printf("\033[32miter(+1)=%d,time(ms)=%5.3f, collided, keep optimizing\n\033[0m", iter_num_, time_ms);
                }
            }
            else if (result == lbfgs::LBFGSERR_CANCELED)
            {
                flag_force_return = true;
                rebound_times++;
                cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
            }
            else
            {
                ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
                // while (ros::ok());
            }

        } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
                 (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));
        //  while (
        //     ((flag_occ || ((min_ellip_dist_ != INIT_min_ellip_dist_) && (min_ellip_dist_ > swarm_clearance_))) && restart_nums < MAX_RESART_NUMS_SET) ||
        //     (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

        return success;
    }

    bool BsplineOptimizer::refine_optimize()
    {
        iter_num_ = 0;
        int start_id = order_;
        int end_id = this->cps_.points.cols() - order_;
        variable_num_ = 3 * (end_id - start_id);

        double q[variable_num_];
        double final_cost;

        memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

        double origin_lambda4 = lambda4_;
        bool flag_safe = true;
        int iter_count = 0;
        do
        {
            lbfgs::lbfgs_parameter_t lbfgs_params;
            lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
            lbfgs_params.mem_size = 16;
            lbfgs_params.max_iterations = 200;
            lbfgs_params.g_epsilon = 0.001;

            int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
            if (result == lbfgs::LBFGS_CONVERGENCE ||
                result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
                result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
                result == lbfgs::LBFGS_STOP)
            {
                // pass
            }
            else
            {
                ROS_ERROR("Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
            }

            UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
            double tm, tmp;
            traj.getTimeSpan(tm, tmp);
            double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / env_manager_->getResolution()); // Step size is defined as the maximum size that can passes throgth every gird.
            for (double t = tm; t < tmp * 2 / 3; t += t_step)
            {
                if (env_manager_->checkCollisionInGridMap(traj.evaluateDeBoorT(t)))
                {
                    // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

                    Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
                    for (size_t i = 0; i < ref_pts_.size(); i++)
                    {
                        ref_pts.row(i) = ref_pts_[i].transpose();
                    }

                    flag_safe = false;
                    break;
                }
            }

            if (!flag_safe)
                lambda4_ *= 2;

            iter_count++;
        } while (!flag_safe && iter_count <= 0);

        lambda4_ = origin_lambda4;

        // cout << "iter_num_=" << iter_num_ << endl;

        return flag_safe;
    }

    void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
    {
        // cout << "drone_id_=" << drone_id_ << endl;
        // cout << "cps_.points.size()=" << cps_.points.size() << endl;
        // cout << "n=" << n << endl;
        // cout << "sizeof(x[0])=" << sizeof(x[0]) << endl;

        memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

        /* ---------- evaluate cost and gradient ---------- */
        double f_smoothness, f_distance, f_feasibility, f_mov_objs;
        ;

        Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
        Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
        Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);
        Eigen::MatrixXd g_mov_objs = Eigen::MatrixXd::Zero(3, cps_.size);

        calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
        calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
        calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);
        calcMovingObjCost(cps_.points, f_mov_objs, g_mov_objs);

        f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility + new_lambda2_ * f_mov_objs;

        Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility + new_lambda2_ * g_mov_objs;
        memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
    }

    void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
    {

        memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

        /* ---------- evaluate cost and gradient ---------- */
        double f_smoothness, f_fitness, f_feasibility;

        Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
        Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
        Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

        // time_satrt = ros::Time::now();

        calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
        calcFitnessCost(cps_.points, f_fitness, g_fitness);
        calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

        /* ---------- convert to solver format...---------- */
        f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
        // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

        Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
        memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
    }

} // namespace ego_planner