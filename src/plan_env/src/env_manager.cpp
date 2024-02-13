#include "plan_env/env_manager.h"
#include "env_manager.h"

void EnvManager::init(const ros::NodeHandle &nh)
{
    ROS_INFO("start init !!!!");
    node_ = nh;

    setTrackerPool();
    setGridMap();

    node_.param<double>("tracker/dbscan_eps",dbscan_eps_,0.5);
    node_.param<int>("tracker/dbscan_min_ptn",dbscan_min_ptn_,5);


/* segmentation */
    ikd_tree_ptr_.reset(new KD_TREE<PointType>(0.3,0.6,0.2));
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


void EnvManager::cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    int clusterIndex = 0;
    // 表示点云属于哪个簇，-1表示噪声，0是初始的，一开始全是噪声
    vector<int> clusterIds;
    dbscan_kdtree_.setInputCloud(cloud);
    vector<int> neighbor_indices;
    vector<float> neighbor_distances;
    vector<bool> visited(cloud->size(),false);//在while循环里标记这个点有没有被加入到数组中过，每次while循环结束则全部置为false。
    clusterIds.resize(cloud->size());
    for(int i=0; i<cloud->points.size();i++)
    {
        // !=0 说明该点已经被分配到了某个簇内，或者是噪音点，对于噪音点，我们也不将其加入到后续的访问队列中，所以visited[i]=true
        if(clusterIds[i] != 0)
        {
            visited[i] = true; // 减少了判断
            continue;
        }
        // 近邻搜索
        int neighbor_size = dbscan_kdtree_.radiusSearch(cloud->points[i],dbscan_eps_,neighbor_indices,neighbor_distances);
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
                if(neighbor_indices[j] != i)
                {
                    if(!visited[neighbor_indices[j]] && clusterIds[neighbor_indices[j]] == 0) // 如果这个点
                    {
                        seed_queue.push(neighbor_indices[j]);
                        visited[neighbor_indices[j]] = true;
                    }
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
                neighbor_size = dbscan_kdtree_.radiusSearch(cloud->points[neighbor_point],dbscan_eps_,neighbor_indices,neighbor_distances);
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

}

void EnvManager::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                   const nav_msgs::OdometryConstPtr& odom)
{

/* odom */
    current_odom_ = *odom;
    
/* point cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud,*pcl_cloud);
    if(pcl_cloud->empty())
    {
        ROS_WARN("cloud is empty");
        return ;
    }
    std::lock_guard<std::mutex> guard(slide_window_mtx_);
    // PointVector point_vector;
    current_cloud_ptr_ = make_shared<PointVector>();
    for(auto t : (*pcl_cloud))
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
            ikd_tree_ptr_->Build(*current_cloud_ptr_);
            ikd_tree_built_ = true;
            return ;
        }
        ikd_tree_ptr_->Add_Points(*current_cloud_ptr_,false);
        if(cloud_slide_window_.size() == 5){
            cloud_window_ready_ = true;
        }
    }
    else
    {
        shared_ptr<PointVector> front_cloud_ptr = cloud_slide_window_.front();
        cloud_slide_window_.pop();
        cloud_slide_window_.push(current_cloud_ptr_);
        ikd_tree_ptr_->Delete_Points(*front_cloud_ptr);
        ikd_tree_ptr_->Add_Points(*current_cloud_ptr_,false);
    }
}


