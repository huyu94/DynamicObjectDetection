#include "plan_env/env_manager.h"
#include "plan_env/dbscan/dbscan_kdtree.h"
#include "plan_env/tools.h"

// double updateMean(double x_mean, double x_new,int n)
// {
//     return x_mean + (x_new - x_mean) / n;
// }

EnvManager::EnvManager()
{

}

EnvManager::~EnvManager()
{
    if(update_time_record_.is_open())
    {
        update_time_record_.close();
    }
}   

/* 初始化 */
void EnvManager::init(const ros::NodeHandle &nh)
{
    ROS_INFO("env manager start init !!!!");
    node_ = nh;


/* set tracker pool*/
    setTrackerPool();

/* set grid map */
    setGridMap();


/* record */
    node_.param<bool>("env_manager/record",record_,false);
    if(record_)
    {
        std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
        std::time_t now_c = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_c), "%Y%m%d%H%M%S");
        std::string filename = ss.str() + ".txt";
        string log_dir = ros::package::getPath("plan_env") + "/logs/";
        update_time_record_.open(log_dir + filename, std::ios::out);
        if(!update_time_record_.is_open())
        {
            ROS_ERROR("cannot open update_time_record.txt");
        }
        else
        {
            ROS_INFO("open update_time_record.txt");
            update_time_record_ << std::left << std::setw(15) << "update_count" 
                                << std::setw(10) << "point_num" 
                                << std::setw(15) << "cluster_time" 
                                << std::setw(20) << "segmentation_time" 
                                << std::setw(20) << "match_time"
                                << "total_time\n";
        }
    }

/* data */
    cloud_odom_window_ready_ = false;
    pcl_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    node_.param<int>("env_manager/slide_window_size",slide_window_size_,5);
    node_.param<double>("env_manager/tracking_update_timeout",tracking_update_timeout_,1.0);

/* visualizer */
    map_vis_ptr_.reset(new MapVisualizer(node_));



/* dbscan cluster */
    node_.param<double>("env_manager/dbscan_eps",dbscan_eps_,0.5);
    node_.param<int>("env_manager/dbscan_min_ptn",dbscan_min_ptn_,5);
    node_.param<int>("env_manager/dbscan_min_cluster_size",dbscan_min_cluster_size_,10);
    node_.param<int>("env_manager/dbscan_max_cluster_size",dbscan_max_cluster_size_,300);
    // ROS_INFO("dbscan_eps : %lf",dbscan_eps_);
    // ROS_INFO("dbscan_min_ptn : %d",dbscan_min_ptn_);
    ROS_INFO("dbscan_min_cluster_size : ", dbscan_max_cluster_size_);
    ROS_INFO("dbscan_max_cluster_size : ", dbscan_max_cluster_size_);
    // cluster_ikdtree_ptr_.reset(new KD_TREE<PointType>(0.3,0.6,0.2));

/* segmentation */
    node_.param<double>("env_manager/gamma1_threshold",gamma1_threshold_,0.5);
    node_.param<double>("env_manager/gamma2_threshold",gamma2_threshold_,0.5);
    segmentation_ikdtree_ptr_.reset(new KD_TREE<PointType>(0.3,0.6,0.2));
    
/* match */
    node_.param<double>("env_manager/distance_gate",distance_gate_,0.5);
    node_.param<double>("env_manager/cluster_max_height",cluster_max_height_,1.5);
    

/* sync subscriber */
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "cloud", 1));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_,"odom", 1));
    sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(SyncPolicyCloudOdom(10), *cloud_sub_, *odom_sub_));
    sync_cloud_odom_->registerCallback(boost::bind(&EnvManager::cloudOdomCallback, this, _1, _2));

/* publisher */


/* timer */
    update_timer_ = node_.createTimer(ros::Duration(0.1), &EnvManager::updateCallback, this);
    vis_timer_ = node_.createTimer(ros::Duration(0.05), &EnvManager::visCallback, this);

}


/* 设置跟踪池 */
void EnvManager::setTrackerPool()
{
    tracker_pool_ptr_.reset(new TrackerPool());
    tracker_pool_ptr_->init(node_);
}

/* 设置栅格地图 */
void EnvManager::setGridMap()
{
    grid_map_ptr_.reset(new GridMap());
    grid_map_ptr_->initMap(node_);
}

/* 聚类 */
void EnvManager::cluster()
{

    /* dbscan cluster */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(pcl_cloud_ptr_);
    vector<pcl::PointIndices> cluster_indices;
    DBSCANKdtreeCluster<pcl::PointXYZ> ec;
    ec.setCorePointMinPts(dbscan_min_ptn_);
    ec.setClusterTolerance(dbscan_eps_);

    ec.setMinClusterSize(dbscan_min_cluster_size_);
    ec.setMaxClusterSize(dbscan_max_cluster_size_);

    ec.setSearchMethod(tree);
    ec.setInputCloud(pcl_cloud_ptr_);
    ec.extract(cluster_indices);

    // 其余的点设置为静态点
    static_points_.clear();
    pcl::Indices static_indices;
    ec.getOtherPoints(static_indices);
    PointVectorPtr cloud = cloud_odom_slide_window_.back().first;
    PointVector &cloud_ref = (*cloud);
    for(auto &t : static_indices)
    {
        Vector3d p;
        p << cloud_ref[t].x,cloud_ref[t].y,cloud_ref[t].z;
        static_points_.push_back(p);
    }

    /* 添加索引 ，并创建对应聚类 */
    cluster_features_.clear(); 
    cluster_features_.resize(cluster_indices.size());
    for(size_t i=0; i < cluster_indices.size();i++)
    {
        if(cluster_features_[i] == nullptr)
        {
            cluster_features_[i] = make_shared<ClusterFeature>();
        }
        cluster_features_[i]->cluster_indices = cluster_indices[i];
        // std::cout << "cluster size : " << cluster_indices[i].indices.size() << std::endl;
    }


    /**
     * 1. 给ClsuterFeature 添加一些属性
     * 2. 输出给map_visualizer  */
    vector<VisualCluster> vis_clusters;
    for(auto &y : cluster_features_)
    {
        if(y == nullptr)
        {
            // ROS_WARN("in [cluster] : nullptr");
            continue;
        }
        // std::cout << "start calculate properties" << std::endl;

        calClusterFeatureProperty(y);
        // std::cout << "can calculate properties" << std::endl;
        vis_clusters.push_back(VisualCluster(y->state.head(3),y->length,y->min_bound,y->max_bound));
        
    }
    // std::cout << "step 2 finished " << std::endl;
    map_vis_ptr_->visualizeClusterResult(vis_clusters);

}
/* 计算聚类属性 */
void EnvManager::calClusterFeatureProperty(ClusterFeature::Ptr cluster_ptr)
{

    Vector3d min_bound,max_bound;
    Vector3d position;
    Vector3d length;
    Vector3d pt;
    PointVectorPtr cloud = cloud_odom_slide_window_.back().first;
    PointVector &cloud_ref = (*cloud);
    int size = cluster_ptr->cluster_indices.indices.size();
    
    for(size_t i=0; i < size;i++)
    {
        int index = cluster_ptr->cluster_indices.indices[i];
        pt.x() = cloud_ref[index].x;
        pt.y() = cloud_ref[index].y;
        pt.z() = cloud_ref[index].z;

        position += pt;
        if(i == 0)
        {
            min_bound = max_bound = pt;
        }
        else
        {
            for(int j=0;j<3;j++)
            {
                if(pt[j] < min_bound[j])
                {
                    min_bound[j] = pt[j];
                }
                if(pt[j] > max_bound[j])
                {
                    max_bound[j] = pt[j];
                }
            }
        }
    }
    position /= size;
    length = max_bound - min_bound;
    cluster_ptr->state = VectorXd(6);
    cluster_ptr->state.head(3) = position;
    cluster_ptr->state.tail(3) = Vector3d::Zero();
    cluster_ptr->length = length;
    cluster_ptr->min_bound = min_bound;
    cluster_ptr->max_bound = max_bound;
    cluster_ptr->motion_type = -1;
    cluster_ptr->gamma_1 = 0;
    cluster_ptr->gamma_2 = 0;
    cluster_ptr->match_id = -1;
    // cluster_ptr->cluster_indices.indices.clear();
    // cluster_ptr->cluster_indices.indices.shrink_to_fit();

}
/* 簇分割 */
void EnvManager::segmentation()
{
    shared_ptr<PointVector> point_vector = cloud_odom_slide_window_.back().first;
    PointVector search_vector;
    vector<float> search_distance;
    double max_dist;
    vector<float> global_nearest_distance;
    double global_average_minimum_distance,normalized_average_variance_of_distance;
    for(auto &cluster : cluster_features_)
    {
        global_nearest_distance.clear();
        Vector3d min_bound,max_bound;
        Vector3d position;

        global_average_minimum_distance = 0;
        for(size_t i=0; i < cluster->cluster_indices.indices.size();i++)
        {

            int index = cluster->cluster_indices.indices[i];
            search_vector.clear();
            search_distance.clear();
            segmentation_ikdtree_ptr_->Nearest_Search((*point_vector)[index],2,search_vector,search_distance);
            // ROS_INFO("search_point: %lf, %lf, %lf", (*point_vector)[index].x,(*point_vector)[index].y,(*point_vector)[index].z);
            // ROS_INFO("search_vector : %lf, %lf, %lf", search_vector[0].x,search_vector[0].y,search_vector[0].z);
            global_nearest_distance.push_back(search_distance[1]);
        }
        double gamma_1 = 0, gamma_2 = 0;
        for(auto dis : global_nearest_distance)
        {
            gamma_1 += dis;
        }
        gamma_1 /= global_nearest_distance.size();

        for(size_t j = 0; j < global_nearest_distance.size();j++)
        {
            gamma_2 += pow(global_nearest_distance[j] - gamma_1,2) ;
        }
        gamma_2 /= (global_nearest_distance.size() * pow(gamma_1,2));
        cluster->gamma_1 = gamma_1;
        cluster->gamma_2 = gamma_2;

        // ROS_INFO("gamma1: %lf, gamma2: %lf",gamma_1,gamma_2);

        if(cluster->state(2) > cluster_max_height_) //排除过高的物体,例如墙体等
        {
            cluster->motion_type = 2;
            continue;
        }
        if(cluster->gamma_1 < gamma1_threshold_)
        {
            cluster->motion_type = 1;
        }
        else if(cluster->gamma_1 > gamma1_threshold_ && cluster->gamma_2 < gamma2_threshold_)
        {
            cluster->motion_type = 0;
        }
        else if(cluster->gamma_1 > gamma1_threshold_ && cluster->gamma_2 > gamma2_threshold_)
        {
            cluster->motion_type = 2;
        }
        else{
            ROS_WARN("gamma1 or gamma2 is nan");
            cluster->motion_type = -1;
        }
    }


    std::vector<VisualCluster> visual_clusters;
    for(auto &t : cluster_features_)
    {
        // std::cout << "cluster motion type: " << t->motion_type << std::endl;
        if(t->motion_type == 0) // 
        {
            // std::cout << " cluster size : " << t->length.transpose() << std::endl;
            visual_clusters.push_back(VisualCluster(t->state.head(3),t->length,t->min_bound,t->max_bound,t->state.tail(3)));
        }
        else if(t->motion_type == 1 || t->motion_type == 2) // static or unkown
        {
            for(int j=0;j<t->cluster_indices.indices.size();j++)
            {
                int index = t->cluster_indices.indices[j];
                Vector3d p((*pcl_cloud_ptr_)[index].x,(*pcl_cloud_ptr_)[index].y,(*pcl_cloud_ptr_)[index].z);
                static_points_.push_back(p);
            }
        }
    }
    map_vis_ptr_->visualizeSegmentationResult(visual_clusters);
}
/* 簇匹配 */
void EnvManager::match()
{

    // ROS_INFO("current_time in [env_manager] : %lf",current_time_.toSec());
    // ROS_INFO("last_update_time in [env_manager] : %lf",last_update_time_.toSec());
    vector<TrackerOutput> tracker_outputs;
    vector<TrackerOutput> tracker_last_outputs;
    tracker_pool_ptr_->forwardPool(tracker_outputs,odom_time_);
    // float dt = (current_time_ - last_update_time_).toSec();
    ;

/* visualize current kalman filter tracker */
    vector<VisualKalmanTracker> visual_trackers;
    for(auto &t : tracker_outputs)
    {
        visual_trackers.emplace_back(t.state.head(3),t.state.tail(3),t.length,t.id);
        // visual_trackers.push_back(VisualKalmanTracker(t.id,t.state.head(3),t.state.tail(3),t.length));
    }
    map_vis_ptr_->visualizeKalmanTracker(visual_trackers);


/*  get moving clutster */
    vector<ClusterFeature::Ptr> measurement_moving_clusters;
    for(auto &cluster : cluster_features_)
    {
        if(cluster->motion_type == 0)
        {
            measurement_moving_clusters.push_back(cluster);
        }
    }

    // ROS_INFO("measurment cluster size: %d, tracker size: %d",measurement_moving_clusters.size(),tracker_outputs.size());
    if(measurement_moving_clusters.size() == 0)
    {
        // std::cout << " current_moving_clusters size == 0" << std::endl;
        return ;
    }

    // for(size_t i = 0; i< tracker_outputs.size();i++)
    // {
    //     std::cout << "tracker_outputs : " << tracker_outputs[i].state.transpose() << std::endl;
    // }

/* hugorian algorithm for match */
    if(tracker_outputs.size() == 0){
        std::cout << " tracker_outputs size == 0" << std::endl;
    }
    else
    {
        Matrix<float> matrix_cost(measurement_moving_clusters.size(),tracker_outputs.size());
        Matrix<bool> matrix_gate(measurement_moving_clusters.size(),tracker_outputs.size());
        for(size_t row=0; row < measurement_moving_clusters.size(); row++)
        {
            for(size_t col=0; col < tracker_outputs.size(); col++)
            {
                /* 确保：
                1. 在距离大于gate的时候，不会被关联中
                2. 全都没有匹配时，会调一个相对较小的距离，这个时候把这个关联去除掉 */
                float feature_distance = (measurement_moving_clusters[row]->state.head(3) - tracker_outputs[col].state.head(3)).norm();
                matrix_cost(row,col) = feature_distance < distance_gate_ ? feature_distance : 5000 * feature_distance;
                matrix_gate(row,col) = matrix_cost(row,col) < distance_gate_;
            }
        }

        Munkres<float> munkres_solver;
        munkres_solver.solve(matrix_cost);

        for(size_t row=0; row < measurement_moving_clusters.size(); row++)
        {
            bool find_match = false;
            for(size_t col=0; col < tracker_outputs.size(); col++)
            {
                if(matrix_cost(row,col) == 0.0f && matrix_gate(row,col)) // find a match
                {
                    int match_id = tracker_outputs[col].id;
                    Tracker::Ptr match_tracker; 
                    /* check if tracker exist */
                    if(!tracker_pool_ptr_->getTracker(match_id,match_tracker)){
                        ROS_WARN("match tracker, but cannot find a tracker in tracker pool");
                        continue;
                    }

                    /* find_match */
                    // ROS_INFO("match : %d -> %d",row,col);
                    find_match = true;
                    measurement_moving_clusters[row]->match_id = tracker_outputs[col].id;
                    VectorXd tracker_last_frame_state = match_tracker->getState();
                    /* velocity estimation */
                    double dt = (odom_time_ - match_tracker->getUpdateTime()).toSec();
                    measurement_moving_clusters[row]->state.tail(3) = (measurement_moving_clusters[row]->state.head(3) - tracker_last_frame_state.head(3));
                    measurement_moving_clusters[row]->state.tail(3) /= dt;
                    measurement_moving_clusters[row]->state.tail(3)(2) = 0;

                    if(measurement_moving_clusters[row]->state.tail(3).norm() < 0.3)
                    {
                        measurement_moving_clusters[row]->state.tail(3) = Vector3d::Zero();
                    }

                }
            }
            if(!find_match) // if cannot find a match for a new moving clusterFeature
            {
                // maybe a new object, or a object just be occluded before
                // set vel, match_id
                measurement_moving_clusters[row]->state.tail(3) = Vector3d::Zero();
                measurement_moving_clusters[row]->match_id = -1;
            }
        }
    }

/* update pool */
    vector<TrackerInput> tracker_inputs;
    for(auto &t : measurement_moving_clusters)
    {
        tracker_inputs.emplace_back(t->match_id,t->state,t->length);
    }
    tracker_pool_ptr_->updatePool(tracker_inputs,odom_time_);

}

/* 检查环境是否需要更新 */
bool EnvManager::checkNeedUpdate()
{
    if (last_update_time_.toSec() < 1.0) // 第一次进入
    {
        last_update_time_ = ros::Time::now(); // 初始化时间
    }
    if (!cloud_odom_window_ready_) // 如果滑窗没有达到数量，或者没有新的点云里程计到达，就是false，无法更新
    {
        // if ((ros::Time::now() - last_update_time_).toSec() > tracking_update_timeout_)
        // {
        //     ROS_ERROR("odom or cloud lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
        //               ros::Time::now().toSec(), last_tracking_update_time_.toSec(), tracking_update_timeout_);
        // }
        return false; // 没有return false, 时间超出了timeout再考虑上面的if
    }
    last_update_time_ = ros::Time::now();

    return true;
}

/* 定时更新器 */
void EnvManager::updateCallback(const ros::TimerEvent&)
{

    
    if(!checkNeedUpdate())
    {
        // ROS_WARN("cloud window is not ready || no new cloud and odom !!");
        return ;
    }
    // ROS_INFO("in [updateCallback]");
    
    static int update_count = 1;
    static double cluster_time, segmentation_time, match_time,total_time;
    // lock of slide window  
    // std::lock_guard<std::mutex> guard(slide_window_mtx_);
    ros::Time t0 = ros::Time::now();
    ros::Time t1,t2;
    t1 = ros::Time::now();
    cluster();
    t2 = ros::Time::now();
    cluster_time = updateMean(cluster_time,(t2-t1).toSec(),update_count);

    
    t1 = ros::Time::now();
    segmentation();
    t2 = ros::Time::now();
    segmentation_time = updateMean(segmentation_time,(t2-t1).toSec(),update_count);
    
    map_vis_ptr_->visualizeStaticPoint(static_points_);
    grid_map_ptr_->updateOccupancy(static_points_,cloud_odom_slide_window_.back().second);

    t1 = ros::Time::now();
    match();
    t2 = ros::Time::now();
    match_time = updateMean(match_time,(t2-t1).toSec(),update_count);

    total_time = updateMean(total_time,(t2-t0).toSec(),update_count);

    record(update_count,pcl_cloud_ptr_->points.size(),cluster_time,segmentation_time,match_time,total_time);
    // update_time_record_ << update_count << "\t" << pcl_cloud_ptr_->points.size() << "\t" << cluster_time << "\t" << segmentation_time << "\t" << match_time << "\t" << total_time << std::endl;
    cloud_odom_window_ready_ = false;
    update_count ++;
}

/* 记录 */
void EnvManager::record(int update_count, int point_size, double cluster_time, double segmentation_time, double match_time, double total_time)
{
    if(record_)
    {
        update_time_record_ << std::left << std::setw(15) << update_count 
                            << std::setw(10) << pcl_cloud_ptr_->points.size() 
                            << std::setw(15) << cluster_time 
                            << std::setw(20) << segmentation_time 
                            << std::setw(20) << match_time 
                            << total_time << "\n";
    }

}
/* 添加点云到滑动窗口，并且删除窗口中第一帧点云 */
void EnvManager::addCloudOdomToSlideWindow(PointVectorPtr &cloud, OdomPtr &odom)
{


    // initialize, build ikd-tree
    if(cloud_odom_slide_window_.size() == 0)
    {
        cloud_odom_slide_window_.push(make_pair(cloud,odom));
        segmentation_ikdtree_ptr_->Build(*cloud);
    }
    else if(cloud_odom_slide_window_.size() < slide_window_size_)
    {
        cloud_odom_slide_window_.push(make_pair(cloud,odom));
        segmentation_ikdtree_ptr_->Add_Points(*cloud,false);
    }
    else
    {
        PointVectorPtr front_cloud = cloud_odom_slide_window_.front().first;
        cloud_odom_slide_window_.pop();
        cloud_odom_slide_window_.push(make_pair(cloud,odom));
        segmentation_ikdtree_ptr_->Delete_Points(*front_cloud);
        segmentation_ikdtree_ptr_->Add_Points(*cloud,false);
    }

    if(cloud_odom_slide_window_.size() == slide_window_size_)
    {
        odom_time_ = cloud_odom_slide_window_.back().second->header.stamp;
        cloud_odom_window_ready_ = true;
    }
}


/* 同步获取点云和里程计信息 */
void EnvManager::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                   const nav_msgs::OdometryConstPtr& odom)
{
    PointVectorPtr cloud_ptr;
    OdomPtr odom_ptr;

    // ROS_INFO("Receive odom & cloud");
/* odom */
    odom_ptr = make_shared<nav_msgs::Odometry>(*odom);


/* point cloud */
    // mutex, let pcl_cloud_ptr_ and slide window buffer operate in the same time
    // std::lock_guard<std::mutex> guard(slide_window_mtx_);

    pcl_cloud_ptr_->clear();
    pcl::fromROSMsg(*cloud,*pcl_cloud_ptr_);
    if(pcl_cloud_ptr_->empty())
    {
        ROS_WARN("cloud is empty");
        return ;
    }

    // convert : 0.01ms
    cloud_ptr = make_shared<PointVector>();
    for(auto t : (*pcl_cloud_ptr_))
    {
        cloud_ptr->emplace_back(t.x,t.y,t.z);
    }


    // add PointVector to slide window buffer and build ikd_tree
    addCloudOdomToSlideWindow(cloud_ptr,odom_ptr);
    
    map_vis_ptr_->visualizeReceiveCloud(pcl_cloud_ptr_);

}



/* 可视化定时器 */
void EnvManager::visCallback(const ros::TimerEvent&)
{

    vector<SlideBox> slide_boxes;
    // vector<TrackerOutput> predicted_trackers;
    tracker_pool_ptr_->forwardSlideBox(slide_boxes,ros::Time::now() + ros::Duration(2.0));
    // tracker_pool_ptr_->forwardPool(predicted_trackers,ros::Time::now() + ros::Duration(2.0));
    vector<VisualizeSlideBox> visual_slide_boxes;
    // vector<VisualKalmanTracker> visual_trackers;
    for(auto &t : slide_boxes)
    {
        visual_slide_boxes.emplace_back(t.getCenter(),t.getLength(),t.getRotation(),t.getId());
    }

    // for(auto &t : predicted_trackers)
    // {
    //     visual_trackers.emplace_back(t.state.head(3),t.state.tail(3),t.length,t.id);
    // }
    map_vis_ptr_->visualizeSlideBox(visual_slide_boxes);
    // map_vis_ptr_->visualizeKalmanTracker(visual_trackers);
}


/* poschecker 的相关功能 */

void EnvManager::getLineGrids(const Vector3d &s_p, const Vector3d &e_p, vector<Vector3d> &grids)
{
    RayCaster raycaster;
    Eigen::Vector3d ray_pt;
    double resolution = getResolution();
    Eigen::Vector3d start = s_p / resolution, end = e_p / resolution;
    bool need_ray = raycaster.setInput(start, end);
    if (need_ray)
    {
    while (raycaster.step(ray_pt))
    {
        Eigen::Vector3d tmp = (ray_pt) * resolution;
        tmp[0] += resolution / 2.0;
        tmp[1] += resolution / 2.0;
        tmp[2] += resolution / 2.0;
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
    Eigen::Vector3d tmp = (ray_pt) * resolution;
    tmp[0] += resolution / 2.0;
    tmp[1] += resolution / 2.0;
    tmp[2] += resolution / 2.0;
    grids.push_back(tmp);
}

bool EnvManager::checkCollision(const Vector3d& pos, ros::Time check_time, int& collision_type, int& collision_id)
{
// collision type: 0: free, 1: static, 2: dynamic 
// colision id : normal : -1,  collision : dynamic object id
    if(checkCollisionInGridMap(pos))
    {
        collision_type = 1;
        collision_id = -1; 
        return true;
    }

    if(checkCollisionInTrackerPool(pos,check_time,collision_id))
    {
        collision_type = 2;
        return true;
    }
}

bool EnvManager::checkCollisionInGridMap(const Vector3d& pos)
{
    return grid_map_ptr_->getInflateOccupancy(pos) != 0;
}

bool EnvManager::checkCollisionInTrackerPool(const Vector3d& pos, const ros::Time& pos_time, int& collision_id)
{
    return tracker_pool_ptr_->checkCollision(pos,pos_time,collision_id);
}

void EnvManager::generateSlideBox(double forward_time,vector<SlideBox>& slide_boxes)
{
    slide_boxes.clear();
    tracker_pool_ptr_->forwardSlideBox(slide_boxes,ros::Time::now() + ros::Duration(forward_time));
}

bool EnvManager::checkCollisionInSlideBox(const Vector3d& pos, int& collision_id)
{
    vector<SlideBox> slide_boxes;
    tracker_pool_ptr_->forwardSlideBox(slide_boxes,ros::Time::now() + ros::Duration(0.5));
    for(auto& slide_box : slide_boxes)
    { 
        if(slide_box.isInBox(pos))
        {
            collision_id = slide_box.getId();
            return true;
        }
    }
    collision_id = -1;
    return false;
}



void EnvManager::getRegion(Eigen::Vector3d& ori, Eigen::Vector3d& size)
{
    grid_map_ptr_->getRegion(ori,size);
}