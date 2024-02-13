#include "plan_env/env_manager.h"
#include "env_manager.h"

void EnvManager::init(const ros::NodeHandle &nh)
{
    ROS_INFO("start init !!!!");
    node_ = nh;

    setTrackerPool();
    setGridMap();



/* dbscan cluster */
    node_.param<double>("tracker/dbscan_eps",dbscan_eps_,0.5);
    node_.param<int>("tracker/dbscan_min_ptn",dbscan_min_ptn_,5);
    cluster_kdtree_ptr_.reset(new KD_TREE<PointType>(0.3,0.6,0.2));




/* segmentation */
    node_.param<double>("tracker/gamma_1_threshold",gamma_1_threshold_,0.5);
    node_.param<double>("tracker/gamma_2_threshold",gamma_2_threshold_,0.5);
    node_.param<int>("env_manager/slide_windows_size",slide_windows_size_,5);
    segmentation_kdtree_ptr_.reset(new KD_TREE<PointType>(0.3,0.6,0.2));
    ikd_tree_built_ = false;
    cloud_window_ready_ = false;

/* sync subscriber */
    cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "~cloud", 1));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_,"~odom", 1));
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
                    std::cout << "find search point. " << std::endl;
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

    cluster_features_.clear();
    cluster_features_.resize(clusterIndex);
    for(size_t i=0;i<clusterIds.size();i++)
    {
        if(clusterIds[i] > 0)
        {
            cluster_features_[clusterIds[i]-1].cluster_indices.indices.push_back(i);
        }
    }

}




void EnvManager::segmentation()
{
    shared_ptr<PointVector> point_vector = cloud_slide_window_.back();
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
        for(size_t i=0; i < cluster.cluster_indices.indices.size();i++)
        {

            int index = cluster.cluster_indices.indices[i];
            search_vector.clear();
            search_distance.clear();
            segmentation_kdtree_ptr_->Nearest_Search((*point_vector)[index],1,search_vector,search_distance);
            global_nearest_distance.push_back(search_distance[0]);
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
        cluster.gamma_1 = gamma_1;
        cluster.gamma_2 = gamma_2;

        if(cluster.gamma_1 < gamma_1_threshold_)
        {
            cluster.motion_type = 1;
        }
        else if(cluster.gamma_1 > gamma_1_threshold_ && cluster.gamma_2 < gamma_2_threshold_)
        {
            cluster.motion_type = 0;
        }
        else
        {
            cluster.motion_type = 2;
        }
    }
}


void EnvManager::match()
{
    vector<TrackerOutput> tracker_outputs;
    tracker_pool_ptr_->forwardPool(tracker_outputs,current_time_);

    double distance_gate = 1.5;
    if(tracker_outputs.size() == 0 || cluster_features_.size() == 0)
    {
        std::cout << " cluster or last cluster size == 0" << std::endl;
        return ;
    }
    else // 匹配
    {
        ROS_INFO("size != 0, 匈牙利算法求解");
        Matrix<float> matrix_cost(cluster_features_.size(),tracker_outputs.size());
        Matrix<bool> matrix_gate(cluster_features_.size(),tracker_outputs.size());
        for(size_t row=0; row < cluster_features_.size(); row++)
        {
            for(size_t col=0; col < tracker_outputs.size(); col++)
            {
                // Vector3d measure_pos = cluster_features_[row].state.head(3);
                // Vector3d predict_pos = tracker_outputs[col].state.head(3);
                // double distance = (measure_pos - predict_pos).norm();
                float feature_distance = (cluster_features_[row].state.head(3) - tracker_outputs[col].state.head(3)).norm();
                matrix_cost(row,col) = feature_distance < distance_gate ? feature_distance : 5000 * feature_distance;
                matrix_gate(row,col) = matrix_cost(row,col) < distance_gate;
            }
        }

        Munkres<float> munkres_solver;
        munkres_solver.solve(matrix_cost);
        for(size_t row=0; row < cluster_features_.size(); row++)
        {
            bool find_match = false;
            for(size_t col=0; col < tracker_outputs.size(); col++)
            {
                if(matrix_cost(row,col) == 0.0f && matrix_gate(row,col)) // find a match
                {
                    find_match = true;
                    // find_match
                    cluster_features_[row].match_id = tracker_outputs[col].id;
                    

                }
            }
        }
    }

}

void EnvManager::updateCallback(const ros::TimerEvent&)
{
    std::lock_guard<std::mutex> guard(slide_window_mtx_);
}

void EnvManager::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                   const nav_msgs::OdometryConstPtr& odom)
{

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
    std::lock_guard<std::mutex> guard(slide_window_mtx_);
    // PointVector point_vector;
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
            return ;
        }
        segmentation_kdtree_ptr_->Add_Points(*current_cloud_ptr_,false);
        if(cloud_slide_window_.size() == 5){
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
}


