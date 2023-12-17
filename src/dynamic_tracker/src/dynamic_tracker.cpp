#include "dynamic_tracker/dynamic_tracker.hpp"

namespace dynamic_tracker
{


    void DynamicTracker::init(const ros::NodeHandle &nh)
    {
        ROS_INFO("start init !!!!");
        node_ = nh;
        node_.param("dynamic_tracker/tracking_update_timeout", tracking_update_timeout_, 1.0);
        node_.param("dynamic_tracker/tracking_update_rate", tracking_update_rate_, 10.0);
        node_.param("dynamic_tracker/tracking_ts", ts_, 0.35);
        node_.param("dynamic_tracker/pose_type", pose_type_, 2);
        node_.param("dynamic_tracker/clip_height", clip_height_, 0.15);
        node_.param("dynamic_tracker/cloud_topic", cloud_topic_, std::string("/cloud_registered"));
        node_.param("dynamic_tracker/odom_topic", odom_topic_, std::string("/iris/mavros/local_position/odom"));
        node_.param("dynamic_tracker/pose_topic", pose_topic_, std::string("/pose"));

        last_tracking_update_time_.fromSec(0);
        tracking_need_update_ = false;
        ROS_INFO("start init2 !!!!");

        /* 初始化聚类器 */
        node_.param("dynamic_tracker/adaptive",cluster_adaptive_,false);
        node_.param("dynamic_tracker/cluster_pts", minPts_, 4);
        node_.param("dynamic_tracker/epsilon", eps_, 1.0);
        cluster_ptr_.reset(new Cluster(minPts_,0.5,2.0,eps_));

        ROS_INFO("eps: %f",eps_);
        ROS_INFO("adaptive: %d",cluster_adaptive_);

        // cluster_ptr_->init(minPts_, eps_);
        
        /* 初始化tracker pool */
        tracking_pool_ptr_.reset(new TrackingPool());
        tracking_pool_ptr_->init(node_);

        /* 点云收集者 */
        current_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

        /* color table */
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_real_distribution<double> distrib(0.0,1.0);
        color_table_.resize(100,std::vector<double>(3));
        for(int i=0;i<100;i++){
            color_table_[i][0] = distrib(gen);
            color_table_[i][1] = distrib(gen);
            color_table_[i][2] = distrib(gen);
        }

        /* init callback */
        // subscriber 
        cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, cloud_topic_, 1));
        if (pose_type_ == POSE_STAMPED)
        {
            pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, pose_topic_, 1));
            sync_cloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(SyncPolicyCloudPose(10), *cloud_sub_, *pose_sub_));
            sync_cloud_pose_->registerCallback(boost::bind(&DynamicTracker::cloudPoseCallback, this, _1, _2));
        }
        else if (pose_type_ == ODOMETRY)
        {
            odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, odom_topic_, 1));
            sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(SyncPolicyCloudOdom(10), *cloud_sub_, *odom_sub_));
            sync_cloud_odom_->registerCallback(boost::bind(&DynamicTracker::cloudOdomCallback, this, _1, _2));
        }
        // publisher
        temp_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("dynamic_tracking/temp_cloud", 10);
        color_cloud_pub_ = node_.advertise<sensor_msgs::PointCloud2>("dynamic_tracking/color_cloud", 10);
        ellipsoid_pub_ = node_.advertise<visualization_msgs::MarkerArray>("dynamic_tracking/ellipsoid", 1);
        aabb_pub_ = node_.advertise<visualization_msgs::MarkerArray>("dynamic_tracking/aabb", 1);
        // timer
        update_timer_ = node_.createTimer(ros::Duration(0.05),&DynamicTracker::updateDynamicTrackingCallback, this);
        vis_timer_ = node_.createTimer(ros::Duration(0.05),&DynamicTracker::visCallback, this);

        // indep_cloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>(cloud_topic_,10,&DynamicTracker::indepCloudCallback, this);
        // indep_odom_sub_ = node_.subscribe<nav_msgs::Odometry>(odom_topic_,10,&DynamicTracker::indepOdomCallback,this);
        tracking_need_update_ = false;
        
        /* 雷达相对于机身的位置 */
        // lidar2body_ << 0.0, 0.0, 1.0, 0.0,
        //     -1.0, 0.0, 0.0, 0.0,
        //     0.0, -1.0, 0.0, 0.0,
        //     0.0, 0.0, 0.0, 1.0;
        lidar2body_ << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;
    }

    void DynamicTracker::indepOdomCallback(const nav_msgs::OdometryConstPtr &odom){
        std::cout << "dad, i got odom";
        std::cout << "time : " << odom->header.stamp << std::endl;
    }

    void DynamicTracker::indepCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
        std::cout << "dad, i got cloud";
        std::cout << "time : " << cloud->header.stamp << std::endl;

    }

    void DynamicTracker::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                                           const nav_msgs::OdometryConstPtr &odom)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*cloud, *latest_cloud);

        if (latest_cloud->size() == 0)
            return;

        // 获取位姿
        Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                       odom->pose.pose.orientation.x,
                                                       odom->pose.pose.orientation.y,
                                                       odom->pose.pose.orientation.z);
        Eigen::Matrix3d body_R = body_q.toRotationMatrix();
        Eigen::Matrix4d body2world;
        body2world.block<3, 3>(0, 0) = body_R;
        body2world(0, 3) = odom->pose.pose.position.x;
        body2world(1, 3) = odom->pose.pose.position.y;
        body2world(2, 3) = odom->pose.pose.position.z;
        body2world(3, 3) = 1.0;

        Eigen::Matrix4d lidar_T = body2world * lidar2body_;
        lidar_pos_(0) = lidar_T(0, 3);
        lidar_pos_(1) = lidar_T(1, 3);
        lidar_pos_(2) = lidar_T(2, 3);
        lidar_r_m_ = lidar_T.block<3, 3>(0, 0);
        // std::cout << lidar_T << std:: endl;

        for(auto & point : *latest_cloud)
        {
            point.intensity = (Eigen::Vector3d(point.x, point.y, point.z) - lidar_pos_).norm();
        }

        current_cloud = latest_cloud;

        // current_cloud = latest_cloud;
        // 应该不需要做变换，因为fast-lio 输出的cloud_register本来就是世界坐标系下的点云
        // pcl::transformPointCloud(*latest_cloud, *current_cloud, lidar_T);

        // 去除地面点
        pcl::ExtractIndices<pcl::PointXYZI> cliper;
        cliper.setInputCloud(current_cloud);
        pcl::PointIndices indices;
        for (size_t i = 0; i < current_cloud->points.size(); i++)
        {
            if (current_cloud->points[i].z < clip_height_)
            {
                indices.indices.push_back(i);
            }
        }
        /* 若setNegative(true)，filter输出不在索引里面的点,也可以用getRemoveIndices取得索引的补集。*/
        cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
        cliper.setNegative(true);
        cliper.filter(*current_cloud);

        // 滤波
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        vg.setInputCloud(current_cloud);
        vg.setLeafSize(0.1, 0.1, 0.1);
        vg.filter(*current_cloud);
        // ROS_INFO("GET CLOUD AND ODOM ! AND HAVE DEALED WITH IT !");
        tracking_need_update_ = true;
    }

    void DynamicTracker::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                    const geometry_msgs::PoseStampedConstPtr& pose)
    {
        // do nothing   
    }


    bool DynamicTracker::checkCloudOdomNeedUpdate()
    {
        if (last_tracking_update_time_.toSec() < 1.0)
        {
            last_tracking_update_time_ = ros::Time::now();
        }
        if (!tracking_need_update_)
        {
            if ((ros::Time::now() - last_tracking_update_time_).toSec() > tracking_update_timeout_)
            {
                // ROS_ERROR("odom or cloud lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
                //           ros::Time::now().toSec(), last_tracking_update_time_.toSec(), tracking_update_timeout_);
            }
            return false; // 没有return false, 时间超出了timeout再考虑上面的if
        }
        last_tracking_update_time_ = ros::Time::now();

        return true;
    }

    void DynamicTracker::updateDynamicTrackingCallback(const ros::TimerEvent & /*event*/)
    {
        if (!checkCloudOdomNeedUpdate())
            return;

        ros::Time t0, t1, t2, t3, t4, t5;
        t0 = ros::Time::now();

        // 1. cluster.run() && kalman forward predict
        pcl::Indices cloud_ids;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        cluster(current_cloud,cloud_ids,clusters);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        transferToColorCloud(current_cloud,colorCloud,cloud_ids);
        publishColorCloud(colorCloud);
        t1 = ros::Time::now();
        // ROS_INFO("cluster time: %f",(t1-t0).toSec());

        // forward predict, 但是这里应该还要考虑把一些
        // 2. foward propagation && hogorian match
        update(clusters);

        t2 = ros::Time::now();
        ROS_INFO("match and update time : %f", (t2 - t1).toSec());

        // 3. kalman filter update
        t3 = ros::Time::now();

        // 4. vote for velocity estimation
        t4 = ros::Time::now();

        // 5.
        tracking_need_update_ = false;
    }

    void DynamicTracker::publishColorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*cloud, output);
        output.header.frame_id = "map";
        color_cloud_pub_.publish(output);

    }
    void DynamicTracker::publishCloud()
    {
        sensor_msgs::PointCloud2 output;
        // current_cloud->header.frame_id = "map";
        // current_cloud->width = current_cloud->points.size();
        // current_cloud->height = 1;
        // current_cloud->is_dense = true;
        pcl::toROSMsg(*current_cloud, output);
        output.header.frame_id = "map";
        // output.header.stamp = ros::Time::now();

        // std::cout << "output.width = " << output.width << std::endl;


        temp_cloud_pub_.publish(output);
    }
    void DynamicTracker::publishDynamicEllipsoid()
    {
        visualization_msgs::MarkerArray marker_array;
        std::vector<std::pair<int,std::pair<Vector6d,Vector3d>>> alive_trackings;
        tracking_pool_ptr_->getPool(alive_trackings);
        // ROS_INFO("alive_trackings size: %d ", alive_trackings.size());
        for (size_t i = 0; i < alive_trackings.size(); i++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "dynamic_ellipsoid";
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = alive_trackings[i].first;
            if(alive_trackings[i].first > 100){
                ROS_ERROR("cluster number is large than predefined size !!!");
                return ;
            }

            marker.color.r = color_table_[alive_trackings[i].first][0];
            marker.color.g = color_table_[alive_trackings[i].first][1];
            marker.color.b = color_table_[alive_trackings[i].first][2];
            marker.color.a = 0.5;

            marker.pose.position.x = alive_trackings[i].second.first[0];
            marker.pose.position.y = alive_trackings[i].second.first[1];
            marker.pose.position.z = alive_trackings[i].second.first[2];

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;


            marker.scale.x = alive_trackings[i].second.second[0];
            marker.scale.y = alive_trackings[i].second.second[1];
            marker.scale.z = alive_trackings[i].second.second[2];
            
            marker_array.markers.push_back(marker);        
        }
        ellipsoid_pub_.publish(marker_array);

    }

    void DynamicTracker::publishAABB(std::vector<Vector6d>& pos,std::vector<std::pair<Vector3d,Vector3d>>& minmax)
    {
        visualization_msgs::MarkerArray marker_array;
        // ROS_INFO("alive_trackings size: %d ", alive_trackings.size());
        for (size_t i = 0; i < pos.size(); i++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "observed_AABB";
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.id = i;
            if(i > 100){
                ROS_ERROR("cluster number is large than predefined size !!!");
                return ;
            }

            marker.color.r = color_table_[i][0];
            marker.color.g = color_table_[i][1];
            marker.color.b = color_table_[i][2];
            marker.color.a = 0.5;

            marker.pose.position.x = pos[i][0];
            marker.pose.position.y = pos[i][1];
            marker.pose.position.z = pos[i][2];

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            // ROS_INFO("[publishAABB] : min: %f\t%f\t%f,max: %f\t%f\t%f", minmax[i].second[0], minmax[i].second[1], minmax[i].second[2],minmax[i].first[0], minmax[i].first[1], minmax[i].first[2]);


            marker.scale.x = minmax[i].second[0] - minmax[i].first[0];
            marker.scale.y = minmax[i].second[1] - minmax[i].first[1];
            marker.scale.z = minmax[i].second[2] - minmax[i].first[2];
            marker_array.markers.push_back(marker);        
        }
        aabb_pub_.publish(marker_array);
    }


    void DynamicTracker::visCallback(const ros::TimerEvent &)
    {
        publishCloud();
        publishDynamicEllipsoid();
        // std::cout << "has published" << std::endl;
    }

    /********************************core**********************************************/



    void DynamicTracker::cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::vector<pcl::PointIndices> &cluster_indices, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &observed_clusters)
    {
        cluster_ptr_->run(cloud, cluster_indices,cluster_adaptive_);

        /* 把每一个簇的点云放在一个单独的PointCloud中，*/
        observed_clusters.clear();
        observed_clusters.reserve(cluster_indices.size());
        for(size_t i=0; i < cluster_indices.size();i++)
        {
            
        }
        for (size_t i = 0; i < cloud_ids.size(); i++)
        {
            if (cloud_ids[i] == 0 || cloud_ids[i] == -1) // 噪音点和无定义的点
            {
                continue;
            }
            else if (cloud_ids[i] > 0)
            {
                if(observed_clusters[cloud_ids[i]-1] == nullptr){
                    observed_clusters[cloud_ids[i]-1].reset(new pcl::PointCloud<pcl::PointXYZ>());
                }
                else{
                    pcl::PointXYZ point;
                    point.x = cloud->points[i].x;
                    point.y = cloud->points[i].y;
                    point.z = cloud->points[i].z;
                    observed_clusters[cloud_ids[i]-1]->push_back(point);
                }
            }
            else
            {
                ROS_ERROR("there is a negative cluster id in [Cluster]");
            }
        }
        ROS_INFO("cluster finished! ");
        // for(size_t i = 0 ; i< observed_clusters.size();i++){
        //     ROS_INFO("cluster:%d,size:%d",i,observed_clusters[i]->size());
        // }
    }

    

    void DynamicTracker::update(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &observed_clusters)
    {
        static int update_times = 0;
        update_times++;
        /* 根据点云生成椭圆，中心，半长轴*/
        std::vector<Vector6d> obs_state;
        std::vector<Vector3d> obs_axis;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> obs_cloud;
        std::vector<int> match_ids;
        std::vector<std::pair<Vector3d,Vector3d>> minmax;
        // ROS_INFO("before update : ");
        // for(size_t i = 0;i<observed_clusters.size();i++)
        // {
        //     ROS_INFO("cluster:%d,size:%d",i,observed_clusters[i]->size());
        // }
        for (size_t i = 0; i < observed_clusters.size(); i++)
        {
            Vector6d obs;
            Vector3d axis,vmin,vmax;
            generateAABB(observed_clusters[i],obs,vmin,vmax);
            bool big_enough = generateEllipsoid(observed_clusters[i],obs,axis);
            // ROS_INFO("observed_clusters size : %d ",observed_clusters[i]->size());
            if(!big_enough){
                // ROS_INFO("not big_enough");
                continue;
            }

            obs_state.emplace_back(obs);
            obs_axis.emplace_back(axis);
            obs_cloud.emplace_back(observed_clusters[i]);
            // ROS_INFO("cluster size : %d ",obs_cloud.back()->size());
            // ROS_INFO("first obs_axis : %f\t %f \t %f",obs_axis.back()[0],obs_axis.back()[1],obs_axis.back()[2]);
            match_ids.emplace_back(-1);
            minmax.emplace_back(std::make_pair(vmax,vmin));
        }
        // for(auto &t : obs_state){
        //     std::cout << t.head(3) << std::endl;
        // }
        publishAABB(obs_state,minmax);
        
        int valid_number = obs_state.size();
        if(obs_state.size() != obs_axis.size() || obs_state.size() != obs_cloud.size() || obs_state.size() != match_ids.size()){
            ROS_ERROR("match size are not same");
            return;
        }

        std::vector<std::pair<int,Vector6d>> alive_trackings;
        tracking_pool_ptr_->predictPool(alive_trackings);

        // ROS_INFO("alive tracking");
        // for(auto &t : alive_trackings){
        //     std::cout << "id : " << t.first << " pos " << t.second<< std::endl;
        // }

        /* 匈牙利算法求解 */
        double distance_gate = 1.5;
        // ROS_INFO("valid_number : %d",valid_number);
        std::vector<std::pair<int,std::pair<Vector6d,Vector3d>>> last_trackings;
        tracking_pool_ptr_->getPool(last_trackings);
        ROS_INFO("alive_trackings.size : %d",last_trackings.size());
        for(size_t i=0;i<alive_trackings.size();i++){
            ROS_INFO("last_trackings[%d] : pos : %f %f %f,axis: %f %f %f",i,
            last_trackings[i].second.first[0],last_trackings[i].second.first[1],last_trackings[i].second.first[2],
            last_trackings[i].second.second[0],last_trackings[i].second.second[1],last_trackings[i].second.second[2]);
        }
        if(observed_clusters.size() == 0 || alive_trackings.size() == 0){
            // update straightly
            ROS_INFO("size == 0, 直接更新pool");
            tracking_pool_ptr_->updatePool(match_ids,obs_state,obs_axis,obs_cloud);

        }
        else{
            ROS_INFO("size != 0, 匈牙利算法求解");

            Matrix<double> matrix_cost(valid_number, alive_trackings.size());
            Matrix<double> matrix_gate(valid_number, alive_trackings.size());
            for (size_t row = 0; row < valid_number; row++)
            {
                for (size_t col = 0; col < alive_trackings.size(); col++)
                {
                    Vector3d cur_pos = obs_state[row].head<3>();
                    Vector3d last_pos = alive_trackings[col].second.head<3>();
                    double distance = (cur_pos - last_pos).norm();
                    // ROS_INFO("distance : %f",distance);
                    // ROS_INFO("get here");
                    // double distance = (obs_state[row].head<3>() - alive_trackings[col].second.head<3>(0)).norm();
                    if(distance > distance_gate){ 
                        // 如果距离超出了阈值，我们就认为不可能匹配上，因此代价矩阵为无穷
                        // 同时，如果实在没有能匹配的tracking了，但是MK算法还是给我们匹配了一个最可能的，那我们通过matrix_gate给消除掉，仍然部匹配。
                        matrix_cost(row, col) = distance_gate * 5000; 
                        matrix_gate(row, col) = 0.f;
                    }
                    else{
                        matrix_cost(row, col) = distance;
                        matrix_gate(row, col) = 1.f;
                    }
                }
            }
            // ROS_INFO("START ");
            // for(size_t row=0;row<valid_number;row++){
            //     for(size_t col=0;col<alive_trackings.size();col++){
            //         std::cout << matrix_cost(row,col) << "\t";
            //     }
            //     std::cout << std::endl;
            // }
            Munkres<double> munkres_solver;
            munkres_solver.solve(matrix_cost);
            // ROS_INFO("END ");

            for(size_t row=0; row < valid_number; row++)
            {

                for(size_t col=0; col < alive_trackings.size(); col++)
                {
                    if(matrix_cost(row, col) == 0.0f && matrix_gate(row,col) > 0.01f){ // found a match
                        // ROS_INFO("obs id :%d  ----> tracking : %d, obs_axis:%f\t%f\t%f\t",row,alive_trackings[col].first,obs_axis[row][0],obs_axis[row][1],obs_axis[row][2]);
                        // ROS_INFO("obs_cloud size : %d ",obs_cloud.size());
                        match_ids[row] = alive_trackings[col].first;
                        // ROS_INFO("HERE?");
                        
                        /* TODO : veloctiy estimation and kamlan update */                    
                        // 先简单速度估计，两个质心点之间
                        Vector3d cur_pos = obs_state[row].head<3>();
                        Vector3d last_pos = alive_trackings[col].second.head<3>();
                        obs_state[row].tail<3>() = (cur_pos - last_pos) / ts_;
                        // ROS_INFO("can finish velocity estimation");
                        // obs_state[row].tail<3>()[0] = (obs_state[row][0] - alive_trackings[col].second[0]) / ts_;
                        // obs_state[row].tail<3>()[1] = (obs_state[row][1] - alive_trackings[col].second[1]) / ts_;
                        // obs_state[row].tail<3>()[2] = 0;
                        break;
                    }
                }

            
            }
            ROS_INFO("START %dth UPDATE",update_times);
            for(int row = 0; row < obs_cloud.size(); row++){
                ROS_INFO("id : %d->%d, pos : %f\t%f\t%f, axis : %f\t%f\t%f",row,match_ids[row],obs_state[row][0],obs_state[row][1],obs_state[row][2],obs_axis[row][0],obs_axis[row][1],obs_axis[row][2]);
            }
            tracking_pool_ptr_->updatePool(match_ids,obs_state,obs_axis,obs_cloud);
        }
        // ROS_INFO("mk finished !!");




    }



    bool DynamicTracker::generateAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vector6d &obs_state, Vector3d& vmin,Vector3d &vmax)
    {
        obs_state << 0,0,0,0,0,0;
        int n = cloud->points.size();
        if(n < 4){
            // points number is so small,不足以成为一个簇

            return false;
        }
        pcl::PointXYZ pmin(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max());
        pcl::PointXYZ pmax(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(), -std::numeric_limits<double>::max());
        
        Vector3d centroid(0,0,0);
        // 遍历点云计算AABB框和累计质心坐标

        for (const auto& point : *cloud) {
            if(!pcl::isFinite(point)){
                continue;
            }
            // 更新AABB框的最小点和最大点
            pmin.x = std::min(pmin.x, point.x);
            pmin.y = std::min(pmin.y, point.y);
            pmin.z = std::min(pmin.z, point.z);
            pmax.x = std::max(pmax.x, point.x);
            pmax.y = std::max(pmax.y, point.y);
            pmax.z = std::max(pmax.z, point.z);

            // 累计质心坐标
            centroid[0] += point.x;
            centroid[1] += point.y;
            centroid[2] += point.z;
        }

        // 计算平均质心坐标
        centroid[0] /= cloud->size();
        centroid[1] /= cloud->size();
        centroid[2] /= cloud->size();
        /* calculate the center of AABB, used for the ellipsoid center instead of centroid of mass*/
        // centroid
        // obs_state.head<3>() = centroid;

        vmin[0] = pmin.x;
        vmin[1] = pmin.y;
        vmin[2] = pmin.z;
        vmax[0] = pmax.x;
        vmax[1] = pmax.y;
        vmax[2] = pmax.z;
        // centerc
        obs_state.head<3>() = (vmax + vmin) / 2;
        // ROS_INFO("[generateAABB] : min: %f\t%f\t%f, max: %f\t%f\t%f",
        //         vmin[0],vmin[1],vmin[2],vmax[0],vmax[1],vmax[2]);

        return true;
    }


    bool DynamicTracker::generateEllipsoid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vector6d &obs_state, Vector3d &obs_axis)
    {
        /* get AABB*/
        Vector3d vmin,vmax;
        bool flag = generateAABB(cloud,obs_state,vmin,vmax);    

        /* enlarge the AABBB semi-axis*/
        // ROS_INFO("max : %f\t%f\t%f ; min : %f\t%f\t%f",max[0],max[1],max[2],min[0],min[1],min[2]);
        obs_axis =  (vmax - vmin);
        return flag;
    }



    void DynamicTracker::transferToColorCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &input,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &output,const std::vector<int> clusterIds)
    {

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distrib(0,255);

        int maxa = std::numeric_limits<int>::min();
        for(int i=0;i<clusterIds.size();i++)
        {
            if(maxa < clusterIds[i]) maxa = clusterIds[i];
        }
        // std::cout << "clusters' size" << maxa << std::endl;

        std::vector<std::vector<int> >colortable(2000,std::vector<int>(3));
        // std::cout << " 1 " << std::endl;
        for(int i=0;i<maxa;i++)
        {
            colortable[i][0] = floor(distrib(gen));
            colortable[i][1] = floor(distrib(gen));
            colortable[i][2] = floor(distrib(gen));
        }  

        output->points.clear();
        pcl::PointXYZRGB point;
        // std::cout << " 2 " << std::endl;
        int noise_point_number = 0;
        for(int i=0;i<input->points.size();i++)
        {
            point.x = input->points[i].x;
            point.y = input->points[i].y;
            point.z = input->points[i].z;
            if(clusterIds[i] <= 0){
                noise_point_number++;
                continue;
            }
            point.r = colortable[clusterIds[i]-1][0];
            point.g = colortable[clusterIds[i]-1][1];
            point.b = colortable[clusterIds[i]-1][2];

            output->points.push_back(point);
        }
        // ROS_INFO("noise_point_number: %d",noise_point_number);
    }
}