#include "plan_env/particle_map.h"



namespace particle_map
{

void ParticleMap::initMap(ros::NodeHandle &nh)
{
    node_ = nh;    
    node_.param("/particle_map/pose_type",mp_.pose_type_,2);
    node_.param("/particle_map/frame_id",mp_.frame_id_,string("world"));
    node_.param("/particle_map/odom_lidar_timeout",mp_.odom_lidar_timeout_,0.1);
    node_.param("/particle_map/local_update_range_x",mp_.local_update_range3d_(0),-1.0);
    node_.param("/particle_map/local_update_range_y",mp_.local_update_range3d_(1),-1.0);
    node_.param("/particle_map/local_update_range_z",mp_.local_update_range3d_(2),-1.0);
    node_.param("/particle_map/half_fov_horizontal",mp_.half_fov_horizontal_,180.0);
    node_.param("/particle_map/half_fov_vertical",mp_.half_fov_vertical_,27.0);
    node_.param("/particle_map/enable_virtual_wall",mp_.enable_virtual_wall_,false);
    node_.param("/particle_map/virtual_ceil",mp_.virtual_ceil_,-1.0);
    node_.param("/article_map/virtual_ground",mp_.virtual_ground_,-1.0);    
    ROS_INFO("local_update_range3d_x : %f",mp_.local_update_range3d_(0));
    ROS_INFO("local_update_range3d_y : %f",mp_.local_update_range3d_(1));
    ROS_INFO("local_update_range3d_z : %f",mp_.local_update_range3d_(2));

    /* particle map update paramters */
    md_.position_guassian_random_seq_ = 0;
    md_.velocity_gaussian_random_seq_ = 0;
    mp_.position_prediction_stddev = 0.2f;
    mp_.velocity_prediction_stddev = 0.1f;
    mp_.standard_gaussian_pdf_num_ = 20000;
    mp_.sigma_ob = 0.2f;
    mp_.kappa = 0.01f;
    mp_.P_detection = 0.95f;
    md_.update_time_ = 0.f;
    md_.update_counter_ = 0;
    md_.expected_new_born_objects_ = 0.f;
    md_.new_born_particle_weight_ = 0.04f;
    md_.new_born_particle_number_each_point_ = 20;
    mp_.if_record_particle_csv = false;
    // mp_.record_time = 0.0;
    // update_times = 0;
    int init_particle_num = 0;
    float init_weight = 0.01f;

    
    /* particle map parameters */
    mp_.voxel_resolution_ = 0.2f; //
    mp_.voxel_resolution_inv_ = 1 / mp_.voxel_resolution_;
    ROS_INFO("voxel_resolution_inv_ : %f",mp_.voxel_resolution_inv_);
    mp_.angle_resolution_ = 1;
    mp_.angle_resolution_inv_ = 1 / mp_.angle_resolution_;
    mp_.pyramid_neighbor_ = 2;
    mp_.pyramid_neighbor_num_ = (2 * mp_.pyramid_neighbor_ + 1 ) * (2 * mp_.pyramid_neighbor_ + 1);
    mp_.max_particle_num_in_voxel_ = 30;
    mp_.prediction_time_ = 6;
    mp_.prediction_future_time_ = {0.05f,0.2f,0.5f,1.0f,1.5f,2.f};
    mp_.voxel_objects_number_dimension = 4 + mp_.prediction_time_ + 1;
    mp_.observation_pyramid_num_horizontal_ = (int)mp_.half_fov_horizontal_ * 2 / mp_.angle_resolution_;
    mp_.observation_pyramid_num_vertical_ = (int)mp_.half_fov_vertical_ * 2 / mp_.angle_resolution_;
    mp_.observation_pyramid_num_ = mp_.observation_pyramid_num_horizontal_ * mp_.observation_pyramid_num_vertical_;
    /* initialize data buffers */
    mp_.local_update_range3i_ = (mp_.local_update_range3d_ * mp_.voxel_resolution_inv_).array().ceil().cast<int>();
    mp_.local_update_range3d_ = mp_.local_update_range3i_.array().cast<double>() * mp_.voxel_resolution_;
    md_.ringbuffer_size3i_ = 2 * mp_.local_update_range3i_;

    md_.ringbuffer_inf_size3i_ = md_.ringbuffer_size3i_ + Vector3i(2 * mp_.inf_grid_,2*mp_.inf_grid_,2*mp_.inf_grid_);
    Vector3i map_voxel_num3i = 2 * mp_.local_update_range3i_;
    mp_.voxel_num_ =  map_voxel_num3i(0) * map_voxel_num3i(1) * map_voxel_num3i(2);
    mp_.pyramid_num_ = 360 * 180 / mp_.angle_resolution_ / mp_.angle_resolution_;
    mp_.safe_particle_num_in_voxel_ = mp_.max_particle_num_in_voxel_ * 2 ;
    mp_.safe_particle_num_in_pyramid_ = mp_.safe_particle_num_ / mp_.pyramid_num_ * 2;

    int buffer_inf_size = (map_voxel_num3i(0) + 2 * mp_.inf_grid_) * (map_voxel_num3i(1) + 2 * mp_.inf_grid_) * (map_voxel_num3i(2) + 2 * mp_.inf_grid_);
    md_.ringbuffer_origin3i_ = Vector3i(0,0,0);
    md_.ringbuffer_inf_origin3i_ = Vector3i(0,0,0);

    /* data vector initialization */
    md_.cloud_in_current_view_rotated.reset(new pcl::PointCloud<pcl::PointXYZ>());
    md_.input_cloud_with_velocity.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    mp_.p_gaussian_randoms = vector<float>(mp_.guassian_random_num_);
    mp_.v_gaussian_randoms = vector<float>(mp_.guassian_random_num_);
    mp_.standard_gaussian_pdf = vector<float>(mp_.standard_gaussian_pdf_num_);
    md_.voxels_with_particles = vector<vector<vector<float>>>(mp_.voxel_num_,vector<vector<float>>(mp_.safe_particle_num_in_voxel_,vector<float>(9,0.0)));
    md_.voxels_objects_number = vector<vector<float>>(mp_.voxel_num_,vector<float>(mp_.voxel_objects_number_dimension));
    md_.pyramids_in_fov = vector<vector<vector<float >>>(mp_.observation_pyramid_num_,vector<vector<float>>(mp_.safe_particle_num_in_pyramid_,vector<float>(3,0.0)));
    md_.observation_pyramid_neighbours = vector<vector<int>>(mp_.observation_pyramid_num_,vector<int>(mp_.pyramid_neighbor_num_+1));
    md_.future_status = vector<vector<float>>(mp_.voxel_num_,vector<float>(mp_.prediction_time_));
    md_.input_points_ = vector<float>(mp_.max_point_num_*3);
    ROS_INFO("input_points_ size: %zu", md_.input_points_.size());
    ROS_INFO("cloud_in_current_view_rotated size: %zu", md_.cloud_in_current_view_rotated->size());
    ROS_INFO("input_cloud_with_velocity size: %zu", md_.input_cloud_with_velocity->size());
    ROS_INFO("p_gaussian_randoms size: %zu", mp_.p_gaussian_randoms.size());
    ROS_INFO("v_gaussian_randoms size: %zu", mp_.v_gaussian_randoms.size());
    ROS_INFO("future_status size: %zu", md_.future_status.size());
    ROS_INFO("observation_pyramid_neighbours size: %zu", md_.observation_pyramid_neighbours.size());
    ROS_INFO("pyramids_in_fov size: %zu", md_.pyramids_in_fov.size());
    ROS_INFO("voxels_objects_number size: %zu", md_.voxels_objects_number.size());
    ROS_INFO("voxels_with_particles size: %zu", md_.voxels_with_particles.size());

    /* velocity estimation */
    mp_.dynamic_cluster_max_point_num_ = 200;
    mp_.dynamic_cluster_max_center_height_ = 1.5;


    md_.camera2body_ << 1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 1.0;

    lidar_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_,"particle_map/depth",1));
    if(mp_.pose_type_ == POSE_STAMPED)
    {
        pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_,"particle_map/pose",1));
        sync_lidar_pose_.reset(
            new message_filters::Synchronizer<SyncPolicyLidarPose>(
                SyncPolicyLidarPose(100),*lidar_sub_,*pose_sub_
            )
        );
        sync_lidar_pose_->registerCallback(boost::bind(&ParticleMap::lidarPoseCallback,this,_1,_2));
    }
    else if(mp_.pose_type_ == ODOMETRY)
    {
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_,"particle_map/odom",1));
        sync_lidar_odom_.reset(
            new message_filters::Synchronizer<SyncPolicyLidarOdom>(
                SyncPolicyLidarOdom(100),*lidar_sub_,*odom_sub_
            )
        );
        sync_lidar_odom_->registerCallback(boost::bind(&ParticleMap::lidarOdomCallback,this,_1,_2));
    }
    else
    {
        ROS_ERROR("Unknown pose type!");
    }
    
}



void ParticleMap::lidarOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                    const nav_msgs::OdometryConstPtr &odom_msg)
{

}

void ParticleMap::lidarPoseCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                    const geometry_msgs::PoseStampedConstPtr &pose_msg)
{

}

void ParticleMap::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    ROS_INFO("DAD, i have got lidar");
}

void ParticleMap::odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    ROS_INFO("DAD, i have got odom");
}

void ParticleMap::setOriginVoxelFilterResolution(float res)
{
    mp_.voxel_filter_resolution_ = res;
}
void ParticleMap::findPyramidNeighborIndexInFOV(const int index, vector<vector<float>>& neighobrs_list)
{
    int horizontal_index = index / mp_.observation_pyramid_num_vertical_;
    int vertical_index = index % mp_.observation_pyramid_num_vertical_;
    neighobrs_list[index][0] = 0;
    for(int i = -mp_.pyramid_neighbor_;i<=mp_.pyramid_neighbor_;i++)
    {
        for(int j = -mp_.pyramid_neighbor_;j<=mp_.pyramid_neighbor_;j++)
        {
            int h = horizontal_index + i;
            int v = vertical_index + j;
            if(h >= 0 && h < mp_.observation_pyramid_num_horizontal_ && v >= 0 && v < mp_.observation_pyramid_num_vertical_)
            {
                neighobrs_list[index][neighobrs_list[index][0]] = h * mp_.observation_pyramid_num_vertical_ + v;
                neighobrs_list[index][0] += 1;
            }
        }
    }
}
void ParticleMap::removeParticle(float * ori_particle_flag_ptr)
{
    *ori_particle_flag_ptr = 0.f;
}


float ParticleMap::standardNormalPDF(float value)
{
    float fx = (1.f / (sqrtf(2.f * M_PI_2f32))) * expf(-powf(value,2)/(2));
    return fx;
}

void ParticleMap::calculateNormalPDFBuffer()
{
    for(int i=0;i<mp_.standard_gaussian_pdf_num_;i++)
    {
        mp_.standard_gaussian_pdf[i] = standardNormalPDF((float)(i-10000) * 0.001f); // range[-10, 10]; 10 sigma
    }
}

float ParticleMap::queryNormalPDF(float &x, float &mu, float &sigma)
{
    float corrected_x = (x - mu) / sigma;
    if(corrected_x > 9.9f) corrected_x = 9.9f;
    else if(corrected_x < - 9.9f) corrected_x = -9.9f;
    return mp_.standard_gaussian_pdf[(int)(corrected_x * 1000 + 10000)];
}

void ParticleMap::transformParticleToSensorFrame(Vector3d& transformPoint,const Vector3d &oriPoint, const Vector3d &position,const Quaterniond &rotation)
{
    Quaterniond tempQuaternion, targetQuaternion;
    tempQuaternion.w() = 0;
    tempQuaternion.x() = oriPoint.x();
    tempQuaternion.y() = oriPoint.y();
    tempQuaternion.z() = oriPoint.z();
    targetQuaternion = rotation * tempQuaternion * rotation.inverse();
    transformPoint.x() = targetQuaternion.x() + position.x();
    transformPoint.y() = targetQuaternion.y() + position.y();
    transformPoint.z() = targetQuaternion.z() + position.z();
}
float ParticleMap::clusterDistance(ClusterFeature & c1, ClusterFeature & c2)
{
    float distance = (c1.center - c2.center).norm();
    return distance;
}

float ParticleMap::generateRandomFloat(float min,float max)
{
    return min + static_cast<float>(rand()) / (static_cast<float> (RAND_MAX / (max - min)));
}


void ParticleMap::velocityEstimationThread()
{
    if(md_.cloud_in_current_view_rotated->points.empty()) return ;

    md_.input_cloud_with_velocity->clear();

    // remove ground and transform data, 除去地面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_grond_points(new pcl::PointCloud<pcl::PointXYZ>());

    for(auto &p : md_.cloud_in_current_view_rotated->points)
    {
        if(p.z > mp_.voxel_filter_resolution_)// 滤掉地面点。
        {
            non_grond_points->points.push_back(p); // 非地面点：
        }else{ // 地面静态点
            static_points->points.push_back(p);  // 静态点：一定包含地面点，
        }
    }

    //cluster 欧式聚类
    static std::vector<ClusterFeature> cluster_feature_vector_dynamic_last; // 上一帧的分类的cluster
    std::vector<ClusterFeature> clusters_feature_vector_dynamic; // 当前帧分类的cluster
    std::vector<pcl::PointIndices> cluster_indices; // 簇中点云到全局点云的索引
    vector<bool> cluster_possibly_dynamic;  // 簇是否可能是动态的

    if(!non_grond_points->empty()) // 非地面点： 非地面静态点 + 非地面动态点
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(non_grond_points);
        // kd-tree 欧式聚类
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(2 * mp_.voxel_filtered_resolution_);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(10000);

        ec.setSearchMethod(tree);
        ec.setInputCloud(non_grond_points);
        ec.extract(cluster_indices);

        for(const auto & cluster_indice : cluster_indices) // 欧式聚类后的簇集合
        {
            ClusterFeature cluster_this;
            cluster_this.intensity = generateRandomFloat(0.1f,0.1f); //For visualization
            
            for(int indice : cluster_indice.indices)
            {
                cluster_this.center.x() += (*non_grond_points)[indice].x;
                cluster_this.center.y() += (*non_grond_points)[indice].y;
                cluster_this.center.z() += (*non_grond_points)[indice].z;
                ++ cluster_this.point_num;
            }
            
            // average
            cluster_this.center.x() /= (float) cluster_this.point_num;
            cluster_this.center.y() /= (float) cluster_this.point_num;
            cluster_this.center.z() /= (float) cluster_this.point_num;

            if(cluster_indice.indices.size() > mp_.dynmaic_cluster_max_point_num_ || cluster_this.center.z() > mp_.dynamic_cluster_max_center_height_)
            { // 如果簇中点数大于阈值，或者簇中心高度大于阈值，则认为是静态点
                // Static 
                for(int indice : cluster_indice.indices)
                {
                    static_points->push_back((*non_grond_points)[indice]);
                }
            }else{
                // Possibly dynamic
                cluster_feature_vector_dynamic.push_back(cluster_this);
                cluster_possibly_dynamic.push_back(true);
            }
        }

        // km algorithm 
        static float distance_gate = 1.5f;
        static float point_num_gate = 100;
        static float maximum_velocity = 5.f;

        /// Move last feature vector d and match by KM algorithm
        if(!cluster_feature_vector_dynamic_last.empty() && !cluster_possibly_dynamic.empty())
        {
            if(delt_t_from_last_observation > 0.00001 && delt_t_from_last_observation < 10.0)
            {
                Matrix<float> matrix_cost(cluster_feature_vector_dynamic.size(), cluster_feature_vector_dynamic_last.size());
                Matrix<float> matrix_gate(cluster_feature_vector_dynamic.size(), cluster_feature_vector_dynamic_last.size());

                for(size_t row = 0; row < cluster_feature_vector_dynamic.size(); ++row)
                {
                    for(size_t col = 0; col < cluster_feature_vector_dynamic_last.size(); ++col)
                    {
                        float distance = clusterDistance(cluster_feature_vector_dynamic_last[row], cluster_feature_vector_dynamic[col]);
                        if(abs(cluster_feature_vector_dynamic_last[row].point_num - cluster_feature_vector_dynamic[col].point_num) > point_num_gate 
                            || distance >= distance_gate)
                        {
                            matrix_gate(row,col) = 0.f;
                            matrix_cost(row,col) = distance_gate * 5000.f;
                        }
                        else{
                            matrix_gate(row,col) = 1.f;
                            matrix_cost(row,col) = distance / distance_gate * 1000.f;;
                        }
                    }
                }

                Munkres<float> munkres_solver;
                munkres_solver.solve(matrix_cost);

                for(size_t row = 0; row < cluster_feature_vector_dynamic.size(); ++row)
                {
                    for(size_t col = 0; col < cluster_feature_vector_dynamic_last.size(); ++col)
                    {
                        if(matrix_cost(row,col) == 0.f && matrix_gate(row,col) > 0.01f) // found a match
                        {
                            cluster_feature_vector_dynamic[row].match_cluster_seq = col;
                            cluster_feature_vector_dynamic[row].velocity.x() =(cluster_possibly_dynamic[row].center.x() - cluster_feature_vector_dynamic_last[col].center.x()) / delt_t_from_last_observation;
                            clusters_feature_vector_dynamic[row].velocity.y() = (clusters_feature_vector_dynamic[row].center.y() - clusters_feature_vector_dynamic_last[col].center.y()) / delt_t_from_last_observation;
                            clusters_feature_vector_dynamic[row].velocity.z() = (clusters_feature_vector_dynamic[row].center.z() - clusters_feature_vector_dynamic_last[col].center.z()) / delt_t_from_last_observation;
                            clusters_feature_vector_dynamic[row].velocityNorm = clusters_feature_vector_dynamic[row].velocity.norm();
                            clusters_feature_vector_dynamic[row].intensity = clusters_feature_vector_dynamic[row].intensity;

                            if(clusters_feature_vector_dynamic[row].velocityNorm > maximum_velocity)
                            {
                                clusters_feature_vector_dynamic[row].velocityNorm = 0.f;
                                clusters_feature_vector_dynamic[row].velocity.setZero();
                            }
                            
                            break;
                        }
                    }
                }
            }
        }

        /// Velocity Allocation to Points
        // Use normal to store velocity
        int cluster_indice_seq = 0;
        int cluster_dynamic_vector_seq = 0;
        for(const auto & cluster_indice : cluster_indices)
        {
            if(cluster_possibly_dynamic[cluster_indice_seq])
            {
                for(int indice : cluster_indice.indices)
                {
                    pcl::_PointXYZINormal p;
                    p.x = (*non_grond_points)[indice].x;
                    p.y = (*non_grond_points)[indice].y;
                    p.z = (*non_grond_points)[indice].z;
                    p.normal_x = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].velocity.x();
                    p.normal_y = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].velocity.y();
                    p.normal_z = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].velocity.z();
                    p.intensity = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].intensity;
                    md_.input_cloud_with_velocity->push_back(p);
                }
                ++ cluster_dynamic_vector_seq;
            }
        }

        for(auto &static_point : static_points->points)
        {
            pcl::PointXYZINormal p;
            p.x = static_point.x;
            p.y = static_point.y;
            p.z = static_point.z;
            p.normal_x = 0.f;
            p.normal_y = 0.f;
            p.normal_z = 0.f;
            p.intensity = 0.f;
            md_.input_cloud_with_velocity->push_back(p);
        }

        cluster_feature_vector_dynamic_last = clusters_feature_vector_dynamic;
        ROS_INFO("Velocity estimation done");
        ROS_INFO("cluster_feature_vector_dynamic_last size: %zu", cluster_feature_vector_dynamic_last.size());

    }




}
} // end namespace particle_map