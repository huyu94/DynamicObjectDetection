#include "dynamic_tracker/cluster_tester.hpp"


namespace cluster_tester
{

    void ClusterTester::init(const ros::NodeHandle &nh)
    {
        node_ = nh;
        node_.param("cluster_tester/cloud_topic", cloud_topic_, std::string("/cloud_registered"));
        node_.param("cluster_tester/odom_topic", odom_topic_, std::string("/iris/mavros/local_position/odom"));
        node_.param("cluster_tester/pose_topic", pose_topic_, std::string("/pose"));
        node_.param("cluster_tester/pose_type", pose_type_, 2);
        node_.param("cluster_tester/cluster_type", cluster_type_, 1);
        node_.param("cluster_tester/clip_height", clip_height_, 0.15);

        /* 初始化聚类器 */
        node_.param("cluster_tester/adaptive",cluster_adaptive_,false);
        node_.param("cluster_tester/cluster_pts", minPts_, 4);
        node_.param("cluster_tester/epsilon", eps_, 1.0);
        cluster_ptr_.reset(new Cluster(minPts_,0.5,2.0,eps_));

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

        lidar2body_ << 1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                    0.0, 0.0, 0.0, 1.0;

        /* init callback */
        // subscriber 
        cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, cloud_topic_, 1));
        if (pose_type_ == 1)
        {
            pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, pose_topic_, 1));
            sync_cloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(SyncPolicyCloudPose(10), *cloud_sub_, *pose_sub_));
            sync_cloud_pose_->registerCallback(boost::bind(&ClusterTester::cloudPoseCallback, this, _1, _2));
        }
        else if (pose_type_ == 2)
        {
            odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, odom_topic_, 1));
            sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(SyncPolicyCloudOdom(10), *cloud_sub_, *odom_sub_));
            sync_cloud_odom_->registerCallback(boost::bind(&ClusterTester::cloudOdomCallback, this, _1, _2));
        }

        update_timer_ = node_.createTimer(ros::Duration(0.05),&ClusterTester::updateCallback, this);
        aabb_pub_ = node_.advertise<visualization_msgs::MarkerArray>("dynamic_tracking/aabb", 1);

        tracking_need_update_ = false;

    }


    void ClusterTester::cloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &cloud, const geometry_msgs::PoseStampedConstPtr &pose)
    {
    }

    void ClusterTester::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud, const nav_msgs::OdometryConstPtr &odom)
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

    void ClusterTester::cluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, pcl::Indices &cloud_ids, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &observed_clusters)
    {
        if(current_cloud->size() == 0)
            return ;
        if(cluster_type_ == 1)
        {
            ROS_WARN("USE DBSCAN");
            cluster_ptr_->run(cloud, cloud_ids,cluster_adaptive_);

            /* 把每一个簇的点云放在一个单独的PointCloud中，*/
            observed_clusters.clear();

            int number_of_clusters = 0;
            for(size_t i = 0; i < cloud_ids.size(); i++){
                if(cloud_ids[i] > number_of_clusters){
                    number_of_clusters = cloud_ids[i];
                }
            }
            observed_clusters.clear();
            observed_clusters.resize(number_of_clusters);
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
        }
        else if(cluster_type_ == 2)
        {
            ROS_INFO("use EuclideanCluster");
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
            tree->setInputCloud(cloud);

            // kd-tree 欧式聚类
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            ec.setClusterTolerance(2 * 0.5);
            ec.setMinClusterSize(5);
            ec.setMaxClusterSize(10000);

            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud);
            ec.extract(cluster_indices);

            observed_clusters.clear(); 
            // observed_clusters.resize(cluster_indices.size(),pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>));
            // for(size_t i=0;i<observed_clusters.size();i++)
            // {
            //     std::cout << "seq : " << i << "size : " << observed_clusters[i]->size() << std::endl;
            // }
            for(size_t i = 0;i < cluster_indices.size();i++)
            {
                ROS_INFO("i : %d" , i);
                std::vector<int> cluster_indice = cluster_indices[i].indices;
                // std::cout << "cluster i : " << i << std::endl;
                // std::cout << "cluster Indice size : " << cluster_indices[i].indices.size() << std::endl;
                int ocount = 0;
                // for(int indice : cluster_indice.indices)

                pcl::PointCloud<pcl::PointXYZ>::Ptr temp_ptr(new pcl::PointCloud<pcl::PointXYZ>());
                for(size_t j=0;j<cluster_indice.size();j++)
                {   
                    pcl::PointXYZ point;
                    // std::cout << "indice : " << indice << std::endl;
                    point.x = cloud->points[cluster_indice[j]].x;
                    point.y = cloud->points[cluster_indice[j]].y;
                    point.z = cloud->points[cluster_indice[j]].z;
                    // std::cout << "point : " << point.x << " " << point.y << " " << point.z << std::endl;
                    temp_ptr->push_back(point);
                }
                observed_clusters.push_back(temp_ptr);
                // std::cout << "=================================" << std::endl;
            }
            for(size_t i=0;i<observed_clusters.size();i++)
            {
                std::cout << "seq : " << i << "size : " << observed_clusters[i]->size() << std::endl;
            }

            ROS_INFO("cluster finished !");
        }
    }

    bool ClusterTester::checkCloudOdomNeedUpdate()
    {
        if (last_update_time_.toSec() < 1.0)
        {
            last_update_time_ = ros::Time::now();
        }
        if (!tracking_need_update_)
        {
            return false; // 没有return false, 时间超出了timeout再考虑上面的if
        }
        last_update_time_ = ros::Time::now();

        return true;
    }

    void ClusterTester::updateCallback(const ros::TimerEvent & /*event*/)
    {
        if (!checkCloudOdomNeedUpdate())
            return;

        ros::Time t0, t1, t2, t3, t4, t5;
        t0 = ros::Time::now();

        std::vector<Vector6d> obs_state;
        std::vector<std::pair<Vector3d,Vector3d>> minmax;
        // 1. cluster.run() && kalman forward predict
        pcl::Indices cloud_ids;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
        cluster(current_cloud,cloud_ids,clusters);
        for(size_t i=0;i<clusters.size();i++)
        {
            Vector6d obs;
            Vector3d axis,vmin,vmax;
            generateAABB(clusters[i],obs,vmin,vmax);
            // std::cout << "seq : " << i << "=============================" << std::endl;
            // for(size_t j=0;j<clusters[i]->size();j++)
            // {
            //     std::cout << "point : " << clusters[i]->points[j].x << " " << clusters[i]->points[j].y << " " << clusters[i]->points[j].z << std::endl;
            // }
            // std::cout << "vmin : " << vmin[0] <<" " << vmin[1] << " " << vmin[2] << std::endl;
            // std::cout << "vmax: " << vmax[0] <<" " << vmax[1] << " " << vmax[2] << std::endl;
            obs_state.emplace_back(obs);
            minmax.emplace_back(std::make_pair(vmax,vmin));
        }
        publishAABB(obs_state,minmax);

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        // transferToColorCloud(current_cloud,colorCloud,cloud_ids);
        // publishColorCloud(colorCloud);
        t1 = ros::Time::now();
        // ROS_INFO("cluster time: %f",(t1-t0).toSec());

        // forward predict, 但是这里应该还要考虑把一些
        // 2. foward propagation && hogorian match
        // update(clusters);

        t2 = ros::Time::now();
        // ROS_INFO("match and update time : %f", (t2 - t1).toSec());

        // 3. kalman filter update
        t3 = ros::Time::now();

        // 4. vote for velocity estimation
        t4 = ros::Time::now();

        // 5.
        tracking_need_update_ = false;
    }

    bool ClusterTester::generateAABB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Vector6d &obs_state, Vector3d &vmin, Vector3d &vmax)
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
    void ClusterTester::publishAABB(std::vector<Vector6d> &pos, std::vector<std::pair<Vector3d, Vector3d>> &minmax)
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

};