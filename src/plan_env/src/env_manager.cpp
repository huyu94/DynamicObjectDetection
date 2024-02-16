#include "plan_env/env_manager.h"

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



void EnvManager::init(const ros::NodeHandle &nh)
{
    ROS_INFO("env manager start init !!!!");
    node_ = nh;


/* set tracker pool*/
    setTrackerPool();

/* set grid map */
    // setGridMap();


/* record */
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
    }

/* data */
    cloud_odom_window_ready_ = false;
    pcl_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    node_.param<int>("env_manager/slide_window_size",slide_window_size_,5);
    node_.param<double>("env_manager/tracking_update_timeout",tracking_update_timeout_,1.0);

/* visualizer */
    map_vis_ptr_.reset(new MapVisualizer(node_));



/* dbscan cluster */
    node_.param<double>("tracker/dbscan_eps",dbscan_eps_,0.5);
    node_.param<int>("tracker/dbscan_min_ptn",dbscan_min_ptn_,5);
    cluster_kdtree_ptr_.reset(new pcl::search::KdTree<pcl::PointXYZ>);
    ROS_INFO("dbscan_eps : %lf",dbscan_eps_);
    ROS_INFO("dbscan_min_ptn : %d",dbscan_min_ptn_);
    // cluster_ikdtree_ptr_.reset(new KD_TREE<PointType>(0.3,0.6,0.2));

/* segmentation */
    node_.param<double>("tracker/gamma1_threshold",gamma1_threshold_,0.5);
    node_.param<double>("tracker/gamma2_threshold",gamma2_threshold_,0.5);
    segmentation_ikdtree_ptr_.reset(new KD_TREE<PointType>(0.3,0.6,0.2));
    

/* sync subscriber */
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "cloud", 1));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_,"odom", 1));
    sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(SyncPolicyCloudOdom(10), *cloud_sub_, *odom_sub_));
    sync_cloud_odom_->registerCallback(boost::bind(&EnvManager::cloudOdomCallback, this, _1, _2));

/* publisher */


/* timer */
    update_timer_ = node_.createTimer(ros::Duration(0.05), &EnvManager::updateCallback, this);
    vis_timer_ = node_.createTimer(ros::Duration(0.05), &EnvManager::visCallback, this);

}



void EnvManager::setTrackerPool()
{
    tracker_pool_ptr_.reset(new TrackerPool());
    tracker_pool_ptr_->init(node_);
}

void EnvManager::setGridMap()
{
    grid_map_ptr_.reset(new GridMap());
    grid_map_ptr_->initMap(node_);
}



void EnvManager::cluster()
{
    int clusterIndex = 0;
    int cloud_size = pcl_cloud_ptr_->points.size();
    // 表示点云属于哪个簇，-1表示噪声，0是初始的，一开始全是噪声
    vector<int> clusterIds(cloud_size,0);
    vector<bool> visited(cloud_size,false);//在while循环里标记这个点有没有被加入到数组中过，每次while循环结束则全部置为false。
    cluster_kdtree_ptr_->setInputCloud(pcl_cloud_ptr_);
    vector<int> neighbor_indices;
    vector<float> neighbor_distances;
    for(int i=0; i<cloud_size;i++)
    {
        int count = 0;

        // !=0 说明该点已经被分配到了某个簇内，或者是噪音点，对于噪音点，我们也不将其加入到后续的访问队列中，所以visited[i]=true
        if(clusterIds[i] != 0)
        {
            // visited[i] = true; // 减少了判断,本身就是true
            continue;
        }
        // 近邻搜索
        // cluster_kdtree_ptr_->Radius_Search((*cloud)[i],dbscan_eps_,neighbors);
        int neighbor_size = cluster_kdtree_ptr_->radiusSearch(pcl_cloud_ptr_->points[i],dbscan_eps_,neighbor_indices,neighbor_distances);
        //density check 如果这个点周围的点大于等于min_pts_,被判定为核心点，这个时候需要将它的邻居加入到队列里，同时将它赋到簇；如果小于，则是边界点，我们不看它的邻居。
        if(neighbor_size < dbscan_min_ptn_) // 圆内邻居点不够,噪音点
        {
            clusterIds[i] = -1; // 标记为噪声点
            continue;
        }

        //圈内点数够，标记为核心点 并赋予簇号
        clusterIndex ++;
        clusterIds[i] = clusterIndex;
        visited[i] = true;
        count ++;

        //seed_queue用来存放我们后续需要操作的点
        std::queue<int> seed_queue;

        // 对于核心点的所有邻居，我们后续都要检查，所以先放进seed_queue中
        for(int j=0;j<neighbor_size;j++) 
        {
            if(visited[neighbor_indices[j]]) // 访问过了，包括自己
            {
                continue; 
            }
            // 如果这个点没有访问过,visited值为true，防止后续重复添加，visited[i] = true --- 注定要被划分的簇内的元素
            seed_queue.push(neighbor_indices[j]);
            visited[neighbor_indices[j]] = true;
        }
        //对周围邻居点进行核心点检查
        while(!seed_queue.empty())
        {   
            // 取出邻居之一
            int neighbor_points_index = seed_queue.front();
            seed_queue.pop();

            
            //不管怎么样，先把邻居加到这个簇内
            clusterIds[neighbor_points_index] = clusterIndex;
            visited[neighbor_points_index] = true;
            count ++;

            // 判断这个邻居是不是核心点
            neighbor_size = cluster_kdtree_ptr_->radiusSearch(pcl_cloud_ptr_->points[neighbor_points_index],dbscan_eps_,neighbor_indices,neighbor_distances);
            if(neighbor_size >= dbscan_min_ptn_) //如果是核心点
            {
                for(int j=0;j<neighbor_size;j++)
                {
                    if(visited[neighbor_indices[j]]){
                        continue; 
                    }
                    seed_queue.push(neighbor_indices[j]);
                    visited[neighbor_indices[j]] = true;                            
                }
            }
            //不然就是边界点，加到簇里就不管它了
        }
    }
    /* 添加索引 ，并创建对应聚类 */
    cluster_features_.clear();
    cluster_features_.resize(clusterIndex);
    for(size_t i=0;i<clusterIds.size();i++)
    {
        if(clusterIds[i] <= 0) // 噪声点
        {
            continue; 
        }
        if(cluster_features_[clusterIds[i]-1]==nullptr) // 没有初始化
        {
            cluster_features_[clusterIds[i]-1] = make_shared<ClusterFeature>();
        }

        cluster_features_[clusterIds[i]-1]->cluster_indices.indices.push_back(i);
    }
    ROS_INFO("cluster size : %d",cluster_features_.size());

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
    cluster_ptr->motion_type = 2;
    cluster_ptr->gamma_1 = 0;
    cluster_ptr->gamma_2 = 0;
    cluster_ptr->match_id = -1;
    // cluster_ptr->cluster_indices.indices.clear();
    // cluster_ptr->cluster_indices.indices.shrink_to_fit();

}




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

        if(cluster->gamma_1 < gamma1_threshold_)
        {
            cluster->motion_type = 1;
        }
        else if(cluster->gamma_1 > gamma1_threshold_ && cluster->gamma_2 < gamma2_threshold_)
        {
            cluster->motion_type = 0;
        }
        else
        {
            cluster->motion_type = 2;
        }
    }

    std::vector<VisualCluster> visual_clusters;
    for(auto &t : cluster_features_)
    {
        if(t->motion_type == 0)
        {
            visual_clusters.push_back(VisualCluster(t->state.head(3),t->length,t->min_bound,t->max_bound,t->state.tail(3)));
        }
    }

    map_vis_ptr_->visualizeSegmentationResult(visual_clusters);
}


void EnvManager::match()
{
    static int match_count = 1;
    std::cout << "match_count : " << match_count++ << std::endl;
    // ROS_INFO("current_time in [env_manager] : %lf",current_time_.toSec());
    // ROS_INFO("last_update_time in [env_manager] : %lf",last_update_time_.toSec());
    vector<TrackerOutput> tracker_outputs;
    vector<TrackerOutput> tracker_last_outputs;
    tracker_pool_ptr_->forwardPool(tracker_outputs,odom_time_);
    // float dt = (current_time_ - last_update_time_).toSec();
    float distance_gate = 1.5;

/*  get moving clutster */
    vector<ClusterFeature::Ptr> measurement_moving_clusters;
    for(auto &cluster : cluster_features_)
    {
        if(cluster->motion_type == 0)
        {
            measurement_moving_clusters.push_back(cluster);
        }
    }

    ROS_INFO("measurment cluster size: %d, tracker size: %d",measurement_moving_clusters.size(),tracker_outputs.size());
    if(measurement_moving_clusters.size() == 0)
    {
        std::cout << " current_moving_clusters size == 0" << std::endl;
        return ;
    }

    for(size_t i = 0; i< tracker_outputs.size();i++)
    {
        std::cout << "tracker_outputs : " << tracker_outputs[i].state.transpose() << std::endl;
    }

/* hugorian algorithm for match */
    if(tracker_outputs.size() == 0){
        std::cout << " tracker_outputs size == 0" << std::endl;
    }
    else
    {
        ROS_INFO("Hungarian algorithm init");
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
                matrix_cost(row,col) = feature_distance < distance_gate ? feature_distance : 5000 * feature_distance;
                matrix_gate(row,col) = matrix_cost(row,col) < distance_gate;
            }
        }

        for(size_t row=0; row < measurement_moving_clusters.size(); row++)
        {
            for(size_t col=0; col < tracker_outputs.size(); col++)
            {
                std::cout << matrix_cost(row,col) << " ";
            }
            std::cout << std::endl;
        }


        ROS_INFO("Hungarian algorithm solving");
        Munkres<float> munkres_solver;
        munkres_solver.solve(matrix_cost);
        ROS_INFO("Hungarian algorithm solved");

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
                    ROS_INFO("match : %d -> %d",row,col);
                    find_match = true;
                    measurement_moving_clusters[row]->match_id = tracker_outputs[col].id;
                    VectorXd tracker_last_frame_state = match_tracker->getState();
                    /* velocity estimation */
                    double dt = (odom_time_ - match_tracker->getUpdateTime()).toSec();
                    measurement_moving_clusters[row]->state.tail(3) = (measurement_moving_clusters[row]->state.head(3) - tracker_last_frame_state.head(3));
                    measurement_moving_clusters[row]->state.tail(3) /= dt;
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
    std::cout << "in [match], Hungarian solved " << std::endl;

/* update pool */
    std::cout << "start update pool" << std::endl;
    vector<TrackerInput> tracker_inputs;
    for(auto &t : measurement_moving_clusters)
    {
        tracker_inputs.emplace_back(t->match_id,t->state,t->length);
        // tracker_inputs.push_back(TrackerInput{t->match_id,t->state.head(3),t->length});
    }
    std::cout << "start update pool  -- 1" << std::endl;
    tracker_pool_ptr_->updatePool(tracker_inputs,odom_time_);
    std::cout << "end update pool" << std::endl;

}

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


void EnvManager::updateCallback(const ros::TimerEvent&)
{

    
    if(!checkNeedUpdate())
    {
        ROS_WARN("cloud window is not ready || no new cloud and odom !!");
        return ;
    }
    ROS_INFO("in [updateCallback]");
    
    // lock of slide window  
    std::lock_guard<std::mutex> guard(slide_window_mtx_);
    update_time_record_ << "point cloud num : " << pcl_cloud_ptr_->points.size() << std::endl;
    ros::Time t0 = ros::Time::now();
    ros::Time t1,t2;
    t1 = ros::Time::now();
    cluster();
    t2 = ros::Time::now();
    // std::cout << "cluster finished " << std::endl;
    update_time_record_ << "cluster time : " << (t2-t1).toSec() << std::endl;
    
    t1 = ros::Time::now();
    // segmentation();
    t2 = ros::Time::now();
    update_time_record_ << "segmentation time : " << (t2-t1).toSec() << std::endl;
    
    t1 = ros::Time::now();
    // match();
    t2 = ros::Time::now();
    update_time_record_ << "match time : " << (t2-t1).toSec() << std::endl;


    update_time_record_ << "total time : " << (t2-t0).toSec() << std::endl;


    cloud_odom_window_ready_ = false;


}


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


void EnvManager::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                   const nav_msgs::OdometryConstPtr& odom)
{
    PointVectorPtr cloud_ptr;
    OdomPtr odom_ptr;

    ROS_INFO("Receive odom & cloud");
/* odom */
    odom_ptr = make_shared<nav_msgs::Odometry>(*odom);


/* point cloud */
    // mutex, let pcl_cloud_ptr_ and slide window buffer operate in the same time
    std::lock_guard<std::mutex> guard(slide_window_mtx_);

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



void EnvManager::visCallback(const ros::TimerEvent&)
{

}


