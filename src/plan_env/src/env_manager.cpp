#include "plan_env/env_manager.h"

void EnvManager::init(const ros::NodeHandle &nh)
{
    ROS_INFO("env manager start init !!!!");
    node_ = nh;


/* set tracker pool*/
    setTrackerPool();

/* set grid map */
    // setGridMap();


/* visualizer */
    map_vis_ptr_.reset(new MapVisualizer(node_));



/* dbscan cluster */
    node_.param<double>("tracker/dbscan_eps",dbscan_eps_,0.5);
    node_.param<int>("tracker/dbscan_min_ptn",dbscan_min_ptn_,5);
    // cluster
    // cluster_kdtree_ptr_.reset(new KD_TREE<PointType>(0.3,0.6,0.2));



/* segmentation */
    node_.param<double>("tracker/gamma1_threshold",gamma1_threshold_,0.5);
    node_.param<double>("tracker/gamma2_threshold",gamma2_threshold_,0.5);
    node_.param<int>("env_manager/slide_window_size",slide_window_size_,5);
    segmentation_kdtree_ptr_.reset(new KD_TREE<PointType>(0.3,0.6,0.2));
    ikd_tree_built_ = false;
    cloud_window_ready_ = false;

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
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = make_shared<PointVector>(cloud_slide_window_.back());
    // shared_ptr<PointVector> cloud = cloud_slide_window_.back();
    
    int clusterIndex = 0;
    int cloud_size = pcl_cloud_ptr_->points.size();
    // 表示点云属于哪个簇，-1表示噪声，0是初始的，一开始全是噪声
    vector<int> clusterIds;
    cluster_kdtree_.setInputCloud(pcl_cloud_ptr_);
    vector<int> neighbor_indices;
    // PointVector neighbors;
    vector<float> neighbor_distances;
    vector<bool> visited(cloud_size,false);//在while循环里标记这个点有没有被加入到数组中过，每次while循环结束则全部置为false。
    clusterIds.resize(cloud_size);
    for(int i=0; i<cloud_size;i++)
    {
        // !=0 说明该点已经被分配到了某个簇内，或者是噪音点，对于噪音点，我们也不将其加入到后续的访问队列中，所以visited[i]=true
        if(clusterIds[i] != 0)
        {
            visited[i] = true; // 减少了判断
            continue;
        }
        // 近邻搜索
        // cluster_kdtree_ptr_->Radius_Search((*cloud)[i],dbscan_eps_,neighbors);
        int neighbor_size = cluster_kdtree_.radiusSearch(pcl_cloud_ptr_->points[i],dbscan_eps_,neighbor_indices,neighbor_distances);
        //density check 如果这个点周围的点大于等于min_pts_,被判定为核心点，这个时候需要将它的邻居加入到队列里，同时将它赋到簇；如果小于，则是边界点，我们不看它的邻居。
        if(neighbor_size < dbscan_min_ptn_) // 圆内邻居点不够,噪音点
        {
            clusterIds[i] = -1; // 标记为噪声点
            continue;
        }
        else{
            //圈内点数够，标记为核心点 并赋予簇号
            clusterIndex ++;
            clusterIds[i] = clusterIndex;

            //seed_queue用来存放我们后续需要操作的点
            std::queue<int> seed_queue;

            // 对于核心点的所有邻居，我们后续都要检查，所以先放进seed_queue中
            for(int j=0;j<neighbor_size;j++) 
            {
                if(neighbor_indices[j] == i)
                {
                    continue;
                }
                // 如果这个点没有访问过，并且不是噪声点
                if(!visited[neighbor_indices[j]] && clusterIds[neighbor_indices[j]] == 0) 
                {
                    seed_queue.push(neighbor_indices[j]);
                    visited[neighbor_indices[j]] = true;
                }
            }
            //对周围邻居点进行核心点检查
            while(!seed_queue.empty())
            {   
                // 取出邻居之一
                int neighbor_point = seed_queue.front();
                seed_queue.pop();

                //不管怎么样，先把邻居加到这个簇内
                clusterIds[neighbor_point] = clusterIndex;

                // 判断这个邻居是不是核心点
                neighbor_size = cluster_kdtree_.radiusSearch(pcl_cloud_ptr_->points[neighbor_point],dbscan_eps_,neighbor_indices,neighbor_distances);
                if(neighbor_size >= dbscan_min_ptn_) //如果是核心点
                {
                    for(int j=0;j<neighbor_size;j++)
                    {
                        if(!visited[neighbor_indices[j]])
                        {
                            seed_queue.push(neighbor_indices[j]);
                            visited[neighbor_indices[j]] = true;                            
                        }
                    }
                }
                // std::cout << seed_queue.size() << std::endl;
                //不然就是边界点，加到簇里就不管它了
            }
            std::fill(visited.begin(),visited.end(),false);
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
    int size = cluster_ptr->cluster_indices.indices.size();
    for(size_t i=0; i < size;i++)
    {
        int index = cluster_ptr->cluster_indices.indices[i];
        pt.x() = (*current_cloud_ptr_)[index].x;
        pt.y() = (*current_cloud_ptr_)[index].y;
        pt.z() = (*current_cloud_ptr_)[index].z;

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
    shared_ptr<PointVector> point_vector = cloud_slide_window_.back();
    PointVector search_vector;
    vector<float> search_distance;
    double max_dist;
    vector<float> global_nearest_distance;
    double global_average_minimum_distance,normalized_average_variance_of_distance;
    double dt = (current_time_ - last_update_time_).toSec();
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
            segmentation_kdtree_ptr_->Nearest_Search((*point_vector)[index],2,search_vector,search_distance);
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
    vector<TrackerOutput> tracker_outputs;
    vector<TrackerOutput> tracker_last_outputs;
    tracker_pool_ptr_->forwardPool(tracker_outputs,current_time_);
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
                    ROS_INFO("match : %d -> %d",row,col);
                    // find_match
                    find_match = true;
                    measurement_moving_clusters[row]->match_id = tracker_outputs[col].id;
                    VectorXd tracker_last_frame_state = tracker_pool_ptr_->getTracker(tracker_outputs[col].id)->getState();
                    /* velocity estimation */
                    double dt = (current_time_ - tracker_pool_ptr_->getTracker(tracker_outputs[col].id)->getUpdateTime()).toSec();
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
    
    vector<TrackerInput> tracker_inputs;
    for(auto &t : measurement_moving_clusters)
    {
        tracker_inputs.emplace_back(t->match_id,t->state,t->length);
        // tracker_inputs.push_back(TrackerInput{t->match_id,t->state.head(3),t->length});
    }
    tracker_pool_ptr_->updatePool(tracker_inputs,current_time_);






}

void EnvManager::updateCallback(const ros::TimerEvent&)
{
    ros::Time t1 = ros::Time::now();
    ros::Time t2;
    std::lock_guard<std::mutex> guard(slide_window_mtx_);
    if(!cloud_window_ready_)
    {
        ROS_WARN("cloud window is not ready");
        return ;
    }
    ROS_INFO("in [updateCallback]");
    
    cluster();
    std::cout << "cluster finished " << std::endl;

    segmentation();
    std::cout << "segmentation finished " << std::endl;
    

    match();
    std::cout << "km match finished " << std::endl;


    t2 = ros::Time::now();
    ROS_INFO("update time : %lf",(t2-t1).toSec());


    last_update_time_ = current_time_; 
}

void EnvManager::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                   const nav_msgs::OdometryConstPtr& odom)
{
    ROS_INFO("Receive odom & cloud");
/* odom */
    current_odom_ = *odom;
    current_time_ = odom->header.stamp;
/* point cloud */

    // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud,*pcl_cloud_ptr_);
    if(pcl_cloud_ptr_->empty())
    {
        ROS_WARN("cloud is empty");
        return ;
    }
    ROS_INFO("point cloud size : %d",pcl_cloud_ptr_->size());
    std::lock_guard<std::mutex> guard(slide_window_mtx_);
    // PointVector point_vector;
    ros::Time t1 = ros::Time::now();

    current_cloud_ptr_ = make_shared<PointVector>();
    for(auto t : (*pcl_cloud_ptr_))
    {
        PointType p;
        p.x = t.x;
        p.y = t.y;
        p.z = t.z;
        current_cloud_ptr_->push_back(p);
    }
    
    // add PointVector to slide window buffer and build ikd_tree
    if(!cloud_window_ready_)
    {
        cloud_slide_window_.push(current_cloud_ptr_);

        if(!ikd_tree_built_)
        {
            segmentation_kdtree_ptr_->Build(*current_cloud_ptr_);
            ikd_tree_built_ = true;
            ros::Time t2 = ros::Time::now();
            ROS_INFO("first build slide window, time : %lf ", (t2-t1).toSec());
            return ;
        }
        segmentation_kdtree_ptr_->Add_Points(*current_cloud_ptr_,false);
        if(cloud_slide_window_.size() == slide_window_size_){
            cloud_window_ready_ = true;
        }
    }
    else
    {
        shared_ptr<PointVector> front_cloud_ptr = cloud_slide_window_.front();
        cloud_slide_window_.pop();
        cloud_slide_window_.push(current_cloud_ptr_);
        segmentation_kdtree_ptr_->Delete_Points(*front_cloud_ptr);
        segmentation_kdtree_ptr_->Add_Points(*current_cloud_ptr_,false);
    }
    ros::Time t2 = ros::Time::now();

    ROS_INFO("build slide window, time : %lf ", (t2-t1).toSec());
    ROS_INFO("slide window size :%d ",slide_window_size_);

    map_vis_ptr_->visualizeReceiveCloud(pcl_cloud_ptr_);

}

void EnvManager::visCallback(const ros::TimerEvent&)
{

}


