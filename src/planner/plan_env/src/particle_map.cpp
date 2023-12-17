#include "plan_env/particle_map.hpp"



namespace ego_planner
{

void ParticleMap::initMap(ros::NodeHandle &nh)
{
    std::string lidar_topic,odom_topic,pose_topic;
    node_ = nh;    
    node_.param("particle_map/pose_type",mp_.pose_type_,2);
    node_.param("particle_map/frame_id",mp_.frame_id_,string("world"));
    node_.param("particle_map/odom_lidar_timeout",mp_.odom_lidar_timeout_,0.1);
    node_.param("particle_map/local_update_range_x",mp_.local_update_range3d_(0),-1.0);
    node_.param("particle_map/local_update_range_y",mp_.local_update_range3d_(1),-1.0);
    node_.param("particle_map/local_update_range_z",mp_.local_update_range3d_(2),-1.0);
    node_.param("particle_map/inf_grid",mp_.inf_grid_,1);
    node_.param("particle_map/half_fov_horizontal",mp_.half_fov_horizontal_,180);
    node_.param("particle_map/half_fov_vertical",mp_.half_fov_vertical_,50);
    node_.param("particle_map/enable_virtual_wall",mp_.enable_virtual_wall_,false);
    node_.param("particle_map/virtual_ceil",mp_.virtual_ceil_,5.0);
    node_.param("particle_map/virtual_ground",mp_.virtual_ground_,0.0);    
    node_.param("particle_map/lidar_topic",lidar_topic,std::string("lidar_topic"));
    node_.param("particle_map/odom_topic",odom_topic,std::string("odom_topic"));
    node_.param("particle_map/pose_topic",pose_topic,std::string("pose_topic"));
    node_.param("particle_map/occupancy_thresh",mp_.occupancy_thresh_,0.5);
    node_.param("particle_map/enable_future_prediction",mp_.enable_future_prediction_,false);
    node_.param("particle_map/prediction_z_height",mp_.prediction_z_height_,0.3);
    node_.param("particle_map/prediction_y_offset",mp_.prediciton_y_offset_,15.0);
    node_.param("particle_map/position_prediction_stddev",mp_.position_prediction_stddev,0.2);
    node_.param("particle_map/velocity_prediction_stddev",mp_.velocity_prediction_stddev,0.1);
    node_.param("particle_map/sigma_ob",mp_.sigma_ob,0.2);
    node_.param("particle_map/new_born_particle_weight",mp_.new_born_particle_weight_,0.04);
    node_.param("particle_map/new_born_particle_number_each_point",mp_.new_born_particle_number_each_point_,20);
    node_.param("particle_map/voxel_filter_resolution",mp_.voxel_filter_resolution_,0.2);
    node_.param("particle_map/risk_thresh",mp_.risk_thresh_,1.0);
    node_.param("particle_map/distance_gate",mp_.distance_gate_,5.0F);
    node_.param("particle_map/dynamic_cluster_max_center_height",mp_.dynamic_cluster_max_center_height_,5.0F);
    node_.param("particle_map/dynamic_cluster_max_point_num",mp_.dynamic_cluster_max_point_num_,200);
    ROS_INFO("risk_thresh : %f",mp_.risk_thresh_);
    ROS_INFO("local_update_range3d_x : %f",mp_.local_update_range3d_(0));
    ROS_INFO("local_update_range3d_y : %f",mp_.local_update_range3d_(1));
    ROS_INFO("local_update_range3d_z : %f",mp_.local_update_range3d_(2));
    ROS_INFO("ODOM : %s",odom_topic.c_str());
    ROS_INFO("POSE : %s",pose_topic.c_str());
    ROS_INFO("LIDAR : %s",lidar_topic.c_str());
    ROS_INFO("sigma_ob %f",mp_.sigma_ob);
    ROS_INFO("new_born_particle_weight %f",mp_.new_born_particle_weight_);
    ROS_INFO("new_born_particle_number_each_point %d",mp_.new_born_particle_number_each_point_);
    ROS_INFO("voxel_filter_resolution %f",mp_.voxel_filter_resolution_);

    
    /* map */
    mp_.voxel_resolution_ = 0.2f; //
    mp_.voxel_resolution_inv_ = 1 / mp_.voxel_resolution_;
    mp_.angle_resolution_ = 1;
    mp_.angle_resolution_inv_ = 1 / mp_.angle_resolution_;
    // mp_.voxel_filter_resolution_ = 0.15; // 下面设置了
    // mp_.angle_resolution_rad_ = (float)mp_.angle_resolution_ / 180.f * M_PIf32;
    // mp_.half_angle_resolution_rad_ = mp_.angle_resolution_rad_ / 2.f;
    // mp_.half_fov_horizontal_ = 180;
    // mp_.half_fov_vertical_ = 27;
    mp_.half_fov_horizontal_rad_ = (float)mp_.half_fov_horizontal_ / 180.f * M_PI_2f32;
    mp_.half_fov_vertical_rad_ = (float)mp_.half_fov_vertical_ / 180.f * M_PI_2f32;

    ROS_INFO("voxel_resolution_inv_ : %f",mp_.voxel_resolution_inv_);
    ROS_INFO("half_fov_horizontal : %d",mp_.half_fov_horizontal_);
    ROS_INFO("half_fov_vertical : %d",mp_.half_fov_vertical_);
    
    mp_.max_particle_num_in_voxel_ = 30;
    mp_.safe_particle_num_in_voxel_ = mp_.max_particle_num_in_voxel_ * 2;
    mp_.safe_particle_num_in_pyramid_ = 36;
    
    /* ringbuffer */
    mp_.local_update_range3i_ = (mp_.local_update_range3d_ * mp_.voxel_resolution_inv_).array().ceil().cast<int>();
    // ROS_INFO("local_update_range3i_ : %d, %d, %d",mp_.local_update_range3i_(0),mp_.local_update_range3i_(1),mp_.local_update_range3i_(2));
    mp_.local_update_range3d_ = mp_.local_update_range3i_.array().cast<double>() * mp_.voxel_resolution_;
    md_.ringbuffer_size3i_ = 2 * mp_.local_update_range3i_;
    md_.ringbuffer_inf_size3i_ = md_.ringbuffer_size3i_ + Vector3i(2 * mp_.inf_grid_,2*mp_.inf_grid_,2*mp_.inf_grid_);
    // Vector3i map_voxel_num3i = 2 * mp_.local_update_range3i_;
    md_.ringbuffer_origin3i_ = Vector3i(0,0,0);
    md_.ringbuffer_inf_origin3i_ = Vector3i(0,0,0);
    int buffer_inf_size = (md_.ringbuffer_size3i_(0) + 2 * mp_.inf_grid_) * (md_.ringbuffer_size3i_(1) + 2 * mp_.inf_grid_) * (md_.ringbuffer_size3i_(2) + 2 * mp_.inf_grid_);
    // ROS_INFO("buffer_inf_size  : %d ",buffer_inf_size);
    md_.occupancy_buffer_inflate_ = vector<uint16_t>(buffer_inf_size,0);

    /* voxel num && pyramid num */
    ROS_INFO("ringbuffer 3i : %d, %d, %d",md_.ringbuffer_size3i_(0),md_.ringbuffer_size3i_(1),md_.ringbuffer_size3i_(2));
    mp_.voxel_num_ =  md_.ringbuffer_size3i_(0) * md_.ringbuffer_size3i_(1) * md_.ringbuffer_size3i_(2);
    ROS_INFO("vovxels buffer size : %d, %d, %d", md_.ringbuffer_size3i_(0),md_.ringbuffer_size3i_(1),md_.ringbuffer_size3i_(2));
    mp_.observation_pyramid_num_horizontal_ = (int)mp_.half_fov_horizontal_ * 2 / mp_.angle_resolution_;
    mp_.observation_pyramid_num_vertical_ = (int)mp_.half_fov_vertical_ * 2 / mp_.angle_resolution_;
    mp_.observation_pyramid_num_ = mp_.observation_pyramid_num_horizontal_ * mp_.observation_pyramid_num_vertical_;
    mp_.observation_max_points_num_one_pyramid_ = 100;


    /* pyramid neighbor */
    mp_.pyramid_neighbor_one_dimension_ = 2;
    mp_.pyramid_neighbor_num_ = (2 * mp_.pyramid_neighbor_one_dimension_ + 1 ) * (2 * mp_.pyramid_neighbor_one_dimension_ + 1);

    /* prediction */
    mp_.prediction_time_ = 6;
    mp_.prediction_future_time_ = {0.05f,0.2f,0.5f,1.0f,1.5f,2.f};
    mp_.voxel_objects_number_dimension = 4 + mp_.prediction_time_;
    

    /* velocity estimation */
    // mp_.dynamic_cluster_max_point_num_ = 200;
    // mp_.dynamic_cluster_max_center_height_ = 5.0f;
    // mp_.distance_gate_ = 1.5f;
    mp_.point_num_gate_ = 100;
    mp_.maximum_velocity_ = 5.f;

    /* particle pararmeters */
    /* particle map update paramters */
    mp_.position_guassian_random_seq_ = 0;
    mp_.velocity_gaussian_random_seq_ = 0;
    // 下面三个参数在后面设置了
    // mp_.position_prediction_stddev = 0.2f;
    // mp_.velocity_prediction_stddev = 0.1f;
    // mp_.sigma_ob = 0.2f;
    // mp_.new_born_particle_weight_ = 0.04f;
    // mp_.new_born_particle_number_each_point_ = 20;
    mp_.kappa = 0.01f;
    mp_.P_detection = 0.95f;
    // md_.update_time_ = 0.f;
    // md_.update_counter_ = 0;
    mp_.expected_new_born_objects_ = 0.f;
    mp_.if_record_particle_csv = false;
    // mp_.record_time = 0.0;
    // update_times = 0;
    int init_particle_num = 0;
    float init_weight = 0.01f;


    /* data vector initialization */
    md_.current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    md_.input_cloud_with_velocity_.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    ROS_INFO("mp_.guassian_random_num_ : %d",mp_.guassian_random_num_); 
    ROS_INFO("mp_.standard_gaussian_pdf_num_ : %d",mp_.standard_gaussian_pdf_num_); 

    mp_.p_gaussian_randoms = vector<float>(mp_.guassian_random_num_);
    mp_.v_gaussian_randoms = vector<float>(mp_.guassian_random_num_);
    mp_.standard_gaussian_pdf = vector<float>(mp_.standard_gaussian_pdf_num_);
    ROS_INFO("mp_.voxel_num_ : %d",mp_.voxel_num_); 
    md_.voxels_with_particles = vector<vector<vector<float>>>(mp_.voxel_num_,vector<vector<float>>(mp_.safe_particle_num_in_voxel_,vector<float>(9,0.0)));
    md_.voxels_objects_number = vector<vector<float>>(mp_.voxel_num_,vector<float>(mp_.voxel_objects_number_dimension));
    md_.pyramids_in_fov = vector<vector<vector<int >>>(mp_.observation_pyramid_num_,vector<vector<int>>(mp_.safe_particle_num_in_pyramid_,vector<int>(3)));
    md_.point_cloud_in_pyramid = vector<vector<vector<float >>>(mp_.observation_pyramid_num_,vector<vector<float>>(mp_.observation_max_points_num_one_pyramid_,vector<float>(5)));
    md_.observation_pyramid_neighbours = vector<vector<int>>(mp_.observation_pyramid_num_,vector<int>(mp_.pyramid_neighbor_num_+1));
    md_.point_num_in_pyramid = vector<int>(mp_.observation_pyramid_num_);
    md_.max_depth_in_pyramid = vector<float>(mp_.observation_pyramid_num_);
    md_.future_status = vector<vector<float>>(mp_.voxel_num_,vector<float>(mp_.prediction_time_));
    md_.input_points_ = vector<float>(mp_.max_point_num_*3);
    ROS_INFO("input_points_ size: %zu", md_.input_points_.size());
    ROS_INFO("cloud_in_current_view_rotated size: %zu", md_.current_cloud_->size());
    ROS_INFO("input_cloud_with_velocity size: %zu", md_.input_cloud_with_velocity_->size());
    ROS_INFO("p_gaussian_randoms size: %zu", mp_.p_gaussian_randoms.size());
    ROS_INFO("v_gaussian_randoms size: %zu", mp_.v_gaussian_randoms.size());
    ROS_INFO("future_status size: %zu", md_.future_status.size());
    ROS_INFO("observation_pyramid_neighbours size: %zu", md_.observation_pyramid_neighbours.size());
    ROS_INFO("pyramids_in_fov size: %zu", md_.pyramids_in_fov.size());
    ROS_INFO("voxels_objects_number size: %zu", md_.voxels_objects_number.size());
    ROS_INFO("voxels_with_particles size: %zu", md_.voxels_with_particles.size());

    /* time && odom lidar received */
    md_.occ_need_update_ = false;
    md_.has_first_lidar_ = false;
    md_.has_odom_ = false;
    md_.last_update_time_.fromSec(0);
    md_.current_update_time_.fromSec(0);
    md_.update_times_ = 0;

    md_.flag_have_ever_received_lidar_ = false;
    md_.flag_lidar_odom_timeout_ = false;

    /* position and rotation */
    md_.lidar_position_.setZero();
    md_.last_lidar_position_.setZero();
    md_.lidar_rotation_.setZero();
    md_.last_lidar_rotation_.setZero();
    md_.lidar2body_ << 1.0, 0.0, 0.0, 0.0,
                        0.0, 1.0, 0.0, 0.0,
                        0.0, 0.0, 1.0, 0.0,
                        0.0, 0.0, 0.0, 1.0;


    /* pyramid neighbor */
    for(int i=0;i<mp_.observation_pyramid_num_;i++)
    {
        findPyramidNeighborIndexInFOV(i);
    }

    srand(static_cast <unsigned> (time(0)));
    calculateNormalPDFBuffer(); //计算高斯随机分布函数的缓存，存储到数组

    // for(int i=0;i<mp_.standard_gaussian_pdf_num_;i++)
    // {
    //     cout << mp_.standard_gaussian_pdf[i] << endl;
    // }


    generateGaussianRandomsVectorZeroCenter();

    addRandomParticles(init_particle_num,init_weight);

    if(mp_.pose_type_ == 1)
    {
        ROS_INFO("We choose POSE_STAMPED as pose type!");
    }
    else
    {
        ROS_WARN("We choose ODOMETRY as pose type!");
    }



    lidar_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_,lidar_topic,1));
    if(mp_.pose_type_ == POSE_STAMPED)
    {
        pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_,pose_topic,1));
        sync_lidar_pose_.reset(
            new message_filters::Synchronizer<SyncPolicyLidarPose>(
                SyncPolicyLidarPose(100),*lidar_sub_,*pose_sub_
            )
        );
        sync_lidar_pose_->registerCallback(boost::bind(&ParticleMap::lidarPoseCallback,this,_1,_2));
    }
    else if(mp_.pose_type_ == ODOMETRY)
    {
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_,odom_topic,1));
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

    occ_update_timer_ = node_.createTimer(ros::Duration(0.1),&ParticleMap::updateOccupancyCallback,this);
    vis_timer_ = node_.createTimer(ros::Duration(0.1),&ParticleMap::visCallback,this);
    map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/particle_map/particle_map",1);
    map_inflate_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/particle_map/inflate_map",1);
    map_future_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/particle_map/future_map",1);
    pose_pub_ = node_.advertise<geometry_msgs::PoseStamped>("particle_map/pose",1);

    ROS_INFO("Particle map initialized!");
    
}

/* ======================== like ego ringbuffeer functions */

void ParticleMap::publishPose()
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = mp_.frame_id_;
    pose_msg.pose.position.x = md_.lidar_position_(0);
    pose_msg.pose.position.y = md_.lidar_position_(1);
    pose_msg.pose.position.z = md_.lidar_position_(2);
    Quaterniond q(md_.lidar_rotation_);
    pose_msg.pose.orientation.w = q.w();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();

    pose_pub_.publish(pose_msg);
}

void ParticleMap::visCallback(const ros::TimerEvent &e)
{
    publishPose();
    publishMapWithFutureStatus();

}   

void ParticleMap::lidarOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                    const nav_msgs::OdometryConstPtr &odom_msg)
{
    md_.current_update_time_ = odom_msg->header.stamp;
    // ROS_WARN(" ihave recevied odom and lidar ");
    /*====*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg,*latest_cloud);

    Quaterniond body_q = Quaterniond(   odom_msg->pose.pose.orientation.w,
                                        odom_msg->pose.pose.orientation.x,
                                        odom_msg->pose.pose.orientation.y,
                                        odom_msg->pose.pose.orientation.z);   
    Matrix3d body_R = body_q.toRotationMatrix();
    Matrix4d body2world;
    body2world.block<3, 3>(0,0) = body_R;
    body2world(0,3) = odom_msg->pose.pose.position.x;
    body2world(1,3) = odom_msg->pose.pose.position.y;
    body2world(2,3) = odom_msg->pose.pose.position.z;
    body2world(3,3) = 1.0;

    Matrix4d lidar_T = body2world * md_.lidar2body_;
    md_.last_lidar_position_ = md_.lidar_position_;
    md_.last_lidar_rotation_ = md_.lidar_rotation_;
    md_.lidar_position_(0) = lidar_T(0,3);
    md_.lidar_position_(1) = lidar_T(1,3);
    md_.lidar_position_(2) = lidar_T(2,3);
    md_.lidar_rotation_ = lidar_T.block<3,3>(0,0);

    md_.current_cloud_ = latest_cloud;
    // ROS_WARN("Received cloud size : %d", md_.current_cloud_->size());
    // clip ground point 
    // pcl::ExtractIndices<pcl::PointXYZ> cliper;
    // cliper.setInputCloud(md_.current_cloud_);
    // pcl::PointIndices indices;
    // for(size_t i=0; i < md_.current_cloud_->points.size();i++)
    // {
    //     // 没定义ground filter height 这个参数，简单用voxel代替
    //     if(md_.current_cloud_->points[i].z < mp_.voxel_resolution_)
    //     {
    //         indices.indices.push_back(i);
    //     }
    // }
    // cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    // cliper.setNegative(true);
    // cliper.filter(*(md_.current_cloud_));

    // voxel filters
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(md_.current_cloud_);
    vg.setLeafSize(mp_.voxel_filter_resolution_,mp_.voxel_filter_resolution_,mp_.voxel_filter_resolution_);
    vg.filter(*(md_.current_cloud_));

    md_.occ_need_update_ = true;
    /*====*/
    // md_.last_update_time_ = md_.current_update_time_;
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

/*
bool GridMap::checkDepthOdomNeedupdate()
{
  if (md_.last_occ_update_time_.toSec() < 1.0) // 初始化为0 没有更新过的情况。
  {
    md_.last_occ_update_time_ = ros::Time::now();
  }
  if (!md_.occ_need_update_) // 
  {
    if (md_.flag_have_ever_received_depth_ && (ros::Time::now() - md_.last_occ_update_time_).toSec() > mp_.odom_depth_timeout_)
    {
      ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
                ros::Time::now().toSec(), md_.last_occ_update_time_.toSec(), mp_.odom_depth_timeout_);
      md_.flag_depth_odom_timeout_ = true;
    }
    return false;
  }
  md_.last_occ_update_time_ = ros::Time::now();

  return true;
}
*/

inline bool ParticleMap::checkLidarOdomNeedUpdate()
{
    if(md_.last_update_time_.toSec() < 1.0)// 初始化为0 没有更新过的情况。
    {
        md_.last_update_time_ = ros::Time::now();
    }

    if(!md_.occ_need_update_) // lidar odom sync callback function 里面设置为true， updateOccupancycallback里面设置为false
    {
        if(md_.flag_have_ever_received_lidar_ && (ros::Time::now() - md_.last_update_time_).toSec() > mp_.odom_lidar_timeout_)
        {   // flag_have_ever_received_lidar_ 表示曾经收到过雷达数据，如果没有收到过雷达数据，就不会进入这个判断。
            // 也就是说只有中途中断了，才会进这层
            ROS_ERROR("odom or depth lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
                        ros::Time::now().toSec(), md_.last_update_time_.toSec(), mp_.odom_lidar_timeout_);
            md_.flag_lidar_odom_timeout_ = true;
        }
        return false;
    }
    // md_.last_update_time_ = ros::Time::now();
    return true;
}


void ParticleMap::updateOccupancyCallback(const ros::TimerEvent &e)
{
    if(!checkLidarOdomNeedUpdate())
    {
        return;
    }
    // ROS_INFO("i am in updateOccupancyCallback");
    // ros::Time t1;
    // // map move 
    // t1 = ros::Time::now();
    moveRingBuffer();
    // ROS_INFO("moving map cost time : %f", (ros::Time::now() - t1).toSec());

    for(int i=0; i<mp_.observation_pyramid_num_;i++)
    {
        md_.point_num_in_pyramid[i] = 0;
        md_.max_depth_in_pyramid[i] = -1.f;
    }

    // 放在全局坐标系下，因此不需要旋转了
    // md_.current_cloud_->clear();

    // t1 = ros::Time::now();
    int iter_num = 0;
    int valid_points = 0;
    for(int p_seq = 0; p_seq < md_.current_cloud_->size();p_seq++)
    {
        Vector3d origin_point, rotated_point;
        origin_point.x() = md_.current_cloud_->points[p_seq].x;
        origin_point.y() = md_.current_cloud_->points[p_seq].y;
        origin_point.z() = md_.current_cloud_->points[p_seq].z; 
        transformParticleToSensorFrame(origin_point,rotated_point);
        // ROS_INFO("transformParticleToSensorFrame: %f %f %f",rotated_point.x(),rotated_point.y(),rotated_point.z());

        if(inPyramidsAreaInSensorFrame(rotated_point[0],rotated_point[1],rotated_point[2]))
        {
            // 找到对应的金字塔子空间
            int pyramid_index_horizontal = findPointPyramidHorizontalIndexInSensorFrame(rotated_point[0],rotated_point[1],rotated_point[2]);
            int pyramid_index_vertical = findPointPyramidVerticalIndexInSensorFrame(rotated_point[0],rotated_point[1],rotated_point[2]);
            int pyramid_index = pyramid_index_horizontal * mp_.observation_pyramid_num_vertical_ + pyramid_index_vertical;
            int observation_inner_seq = md_.point_num_in_pyramid[pyramid_index];
            // ROS_INFO("observation_inner_seq : %d", observation_inner_seq);
            // ROS_INFO("pyramid_index: %d,pyramid_index_horizontal: %d,pyramid_index_vertical: %d",pyramid_index,pyramid_index_horizontal,pyramid_index_vertical);

            float length = rotated_point.norm();

            // ROS_INFO("add point cloud to pyramid!");
            md_.point_cloud_in_pyramid[pyramid_index][observation_inner_seq][0] = origin_point[0];
            md_.point_cloud_in_pyramid[pyramid_index][observation_inner_seq][1] = origin_point[1];
            md_.point_cloud_in_pyramid[pyramid_index][observation_inner_seq][2] = origin_point[2];
            md_.point_cloud_in_pyramid[pyramid_index][observation_inner_seq][3] = 0.f;
            md_.point_cloud_in_pyramid[pyramid_index][observation_inner_seq][4] = length;

            // ROS_INFO("add point cloud to pyramid finished !");
            // 金字塔子空间的最大长度
            if(md_.max_depth_in_pyramid[pyramid_index] < length)
            {
                md_.max_depth_in_pyramid[pyramid_index] = length;
            }
            
            //这个金字塔子空间的观测点数+1
            // ROS_INFO("point num in pyramid ++!");
            md_.point_num_in_pyramid[pyramid_index]++;
            // ROS_INFO("point num in pyramid ++! finished ");

            // 如果超过了最大粒子数，就要限制住
            // Omit the overflowed observation points. It is suggested to used a voxel filter for the original input point clouds to avoid overflow.
            if(md_.point_num_in_pyramid[pyramid_index] >= mp_.observation_max_points_num_one_pyramid_)
            {
                md_.point_num_in_pyramid[pyramid_index] = mp_.observation_max_points_num_one_pyramid_ - 1;
            }

            ++valid_points;
            // ROS_INFO("here");
        }

        iter_num += 3 * sizeof(float);
    }
    // ROS_INFO("valid_points : %d ", valid_points);    
    // ROS_INFO("add point cloud to pyramid cost time : %f", (ros::Time::now() - t1).toSec());

    mp_.expected_new_born_objects_ = mp_.new_born_particle_weight_ * (float) valid_points * (float) mp_.new_born_particle_number_each_point_;
    // ROS_INFO("expected_new_born_objects_ : %f ", mp_.expected_new_born_objects_);
    mp_.new_born_each_object_weight_ = mp_.new_born_particle_weight_ * (float) mp_.new_born_particle_number_each_point_;

    // ROS_INFO("start velocity estimation");
    std::thread velocity_estimation(&ParticleMap::velocityEstimationThread,this);

    // ROS_INFO("start map prediction");
    // t1 = ros::Time::now();
    mapPrediction();
    // ROS_INFO("map Prediction cost time : %f", (ros::Time::now() - t1).toSec());
    // ROS_INFO("map Prediction finished");

    // t1 = ros::Time::now();
    if(md_.current_cloud_->size() >= 0)
    {
        mapUpdate();
    }
    else
    {
        ROS_WARN("no points to update");
    }
    // ROS_INFO("map Update cost time : %f", (ros::Time::now() - t1).toSec());

    // ROS_INFO("map Update finished !");

    /** Wait until initial velocity estimation is finished **/
    velocity_estimation.join();

    /** Add updated new born particles ***/
    // t1 = ros::Time::now();

    if(md_.current_cloud_->size() >= 0)
    {
        mapAddNewBornParticleByObservation();
    }
    // ROS_INFO("mapAddNewBornParticleByObservation cost time : %f", (ros::Time::now() - t1).toSec());
    // ROS_INFO("mapAddNewBornParticleByObservation finished !");
    /** Calculate object number and Resample **/
    /// NOTE in this step the flag which is set to be 7.f in prediction step will be changed to 1.f or 0.6f.
    /// Removing this step will make prediction malfunction unless the flag is reset somewhere else.
    // t1 = ros::Time::now();
    mapOccupancyCalculationAndResample();
    // ROS_INFO("map Occupancy Calculation and resample cost time : %f", (ros::Time::now() - t1).toSec());
    // ROS_INFO("map Occupancy Calculation and resample finished !");
    
    // t1 = ros::Time::now();
    if(mp_.enable_future_prediction_)
    {

        publishMapWithFutureStatus();
    }
    else{
        publishMap();
    }
    // ROS_INFO("publish map cost time : %f", (ros::Time::now() - t1).toSec());
    if(!md_.flag_have_clear_prediction_this_turn_)
    {
        clearOccupancyMapPrediction();
    }

    md_.last_update_time_ = md_.current_update_time_;
    md_.occ_need_update_ = false;

}



void ParticleMap::publishMap()
{
    if(map_pub_.getNumSubscribers() <= 0)
    {
        return ;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    double lbz = mp_.enable_virtual_wall_ ? max(md_.ringbuffer_lowbound3d_(2),mp_.virtual_ground_) : md_.ringbuffer_lowbound3d_(2);
    double ubz = mp_.enable_virtual_wall_ ? min(md_.ringbuffer_upbound3d_(2),mp_.virtual_ceil_) : md_.ringbuffer_upbound3d_(2);
    if (md_.ringbuffer_upbound3d_(0) - md_.ringbuffer_lowbound3d_(0) > mp_.voxel_resolution_ && (md_.ringbuffer_upbound3d_(1) - md_.ringbuffer_lowbound3d_(1)) > mp_.voxel_resolution_ && (ubz - lbz) > mp_.voxel_resolution_)
    {
        for(double xd = md_.ringbuffer_lowbound3d_(0) + mp_.voxel_resolution_ / 2; xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.voxel_resolution_)
        {
            for(double yd = md_.ringbuffer_lowbound3d_(1) + mp_.voxel_resolution_ / 2; yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.voxel_resolution_)
            {
                for(double zd = md_.ringbuffer_lowbound3d_(2) + mp_.voxel_resolution_ / 2; zd <= md_.ringbuffer_upbound3d_(2); zd += mp_.voxel_resolution_)
                {
                    if(md_.voxels_objects_number[globalIdx2BufIdx(pos2GlobalIdx(Vector3d(xd,yd,zd)))][0] > mp_.occupancy_thresh_)
                    {
                        cloud.push_back(pcl::PointXYZ(xd,yd,zd));
                    }
                }
            }
        }
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud,cloud_msg);
    map_pub_.publish(cloud_msg);

}


void ParticleMap::publishMapInflate()
{
    ROS_INFO("still not implemented !");
}


void ParticleMap::publishMapWithFutureStatus()
{

    if(map_future_pub_.getNumSubscribers() <= 0 || map_pub_.getNumSubscribers() <= 0)
    {
        return ;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZRGB> cloud_future;
    double lbz = mp_.enable_virtual_wall_ ? max(md_.ringbuffer_lowbound3d_(2),mp_.virtual_ground_) : md_.ringbuffer_lowbound3d_(2);
    double ubz = mp_.enable_virtual_wall_ ? min(md_.ringbuffer_upbound3d_(2),mp_.virtual_ceil_) : md_.ringbuffer_upbound3d_(2);
    double mdz = mp_.prediction_z_height_;
    double offset = 0.f;

    for(int i=0;i<md_.voxels_objects_number.size();i++)
    {
        for(int j=0;j<mp_.prediction_future_time_.size();j++){
            md_.future_status[i][j] = md_.voxels_objects_number[i][4+j];
            md_.voxels_objects_number[i][j+4] = 0.f;
        }
    }

    
    if (md_.ringbuffer_upbound3d_(0) - md_.ringbuffer_lowbound3d_(0) > mp_.voxel_resolution_ && (md_.ringbuffer_upbound3d_(1) - md_.ringbuffer_lowbound3d_(1)) > mp_.voxel_resolution_ && (ubz - lbz) > mp_.voxel_resolution_)
    {
        for(double xd = md_.ringbuffer_lowbound3d_(0) + mp_.voxel_resolution_ / 2; xd <= md_.ringbuffer_upbound3d_(0); xd += mp_.voxel_resolution_)
        {
            for(double yd = md_.ringbuffer_lowbound3d_(1) + mp_.voxel_resolution_ / 2; yd <= md_.ringbuffer_upbound3d_(1); yd += mp_.voxel_resolution_)
            {
                for(double zd = md_.ringbuffer_lowbound3d_(2) + mp_.voxel_resolution_ / 2; zd <= md_.ringbuffer_upbound3d_(2); zd += mp_.voxel_resolution_)
                {
                    Vector3i global_idx = pos2GlobalIdx(Vector3d(xd,yd,zd));
                    int buf_idx = globalIdx2BufIdx(global_idx);

                    if(md_.voxels_objects_number[buf_idx][0] > mp_.occupancy_thresh_)
                    {
                        // ROS_INFO("voxel pos : %f, %f, %f, weight : %f",xd,yd,zd,md_.voxels_objects_number[buf_idx][0]);

                        cloud.push_back(pcl::PointXYZ(xd,yd,zd));
                    }

                    float weight_this = 0;

                    for(int n=0; n < mp_.prediction_future_time_.size(); ++n)
                    {
                        weight_this += md_.future_status[buf_idx][n];
                        if(weight_this > 0.01f)
                        {
                        int r,g,b;
                        pcl::PointXYZRGB p_future;
                        colorAssign(r,g,b,weight_this,0.f,0.1f,1);

                        p_future.x = xd;
                        p_future.y = yd;
                        p_future.z = zd;
                        p_future.r = r;
                        p_future.g = g;
                        p_future.b = b;
                        cloud_future.push_back(p_future);
                        }   
                    }


                }
            }
        }
    }
    cloud.width = cloud.points.size();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud,cloud_msg);

    cloud_future.width = cloud_future.points.size();
    cloud_future.height = 1;
    cloud_future.is_dense = true;
    cloud_future.header.frame_id = mp_.frame_id_;
    sensor_msgs::PointCloud2 cloud_future_msg;
    pcl::toROSMsg(cloud_future,cloud_future_msg);


    map_future_pub_.publish(cloud_future_msg);
    map_pub_.publish(cloud_msg);

    md_.flag_have_clear_prediction_this_turn_ = true;
}

void ParticleMap::initMapBoundary()
{
    mp_.have_initialized_ = true;
    md_.center_last3i_ = pos2GlobalIdx(md_.lidar_position_);

    // l = c - i, u = c + i, u -= (1,1,1)
    md_.ringbuffer_lowbound3i_ = md_.center_last3i_ - mp_.local_update_range3i_;
    md_.ringbuffer_lowbound3d_ = md_.ringbuffer_lowbound3i_.cast<double>() * mp_.voxel_resolution_;
    md_.ringbuffer_upbound3i_ = md_.center_last3i_ + mp_.local_update_range3i_;
    md_.ringbuffer_upbound3d_ = md_.ringbuffer_upbound3i_.cast<double>() * mp_.voxel_resolution_;
    md_.ringbuffer_upbound3i_ -= Vector3i(1,1,1);

    const Vector3i inf_grid3i(mp_.inf_grid_,mp_.inf_grid_,mp_.inf_grid_);
    const Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.voxel_resolution_;
    md_.ringbuffer_inf_lowbound3i_ = md_.ringbuffer_lowbound3i_ - inf_grid3i;
    md_.ringbuffer_inf_lowbound3d_ = md_.ringbuffer_lowbound3d_ - inf_grid3d;
    md_.ringbuffer_inf_upbound3i_ = md_.ringbuffer_upbound3i_ + inf_grid3i;
    md_.ringbuffer_inf_upbound3d_ = md_.ringbuffer_upbound3d_ + inf_grid3d;

    for(int i=0; i < 3; i++)
    {
        while(md_.ringbuffer_origin3i_(i) < md_.ringbuffer_lowbound3i_(i))
        {
            md_.ringbuffer_origin3i_(i) += md_.ringbuffer_size3i_(i);
        }
        while(md_.ringbuffer_origin3i_(i) > md_.ringbuffer_upbound3i_(i))
        {
            md_.ringbuffer_origin3i_(i)-= md_.ringbuffer_size3i_(i);
        }

        // inf
        while(md_.ringbuffer_inf_origin3i_(i) < md_.ringbuffer_inf_lowbound3i_(i))
        {
            md_.ringbuffer_inf_origin3i_(i) += md_.ringbuffer_inf_size3i_(i);
        }
        while(md_.ringbuffer_inf_origin3i_(i) > md_.ringbuffer_inf_upbound3i_(i))
        {
            md_.ringbuffer_inf_origin3i_(i) -= md_.ringbuffer_inf_size3i_(i);
        }
    }


    ROS_INFO("Particle map initialized!");
    ROS_INFO("RINGBUFFER START ============================");
    ROS_INFO("ringbuffer_lowbound3i_ : %d, %d, %d",md_.ringbuffer_lowbound3i_(0),md_.ringbuffer_lowbound3i_(1),md_.ringbuffer_lowbound3i_(2));
    ROS_INFO("ringbuffer_lowbound3d_ : %f, %f, %f",md_.ringbuffer_lowbound3d_(0),md_.ringbuffer_lowbound3d_(1),md_.ringbuffer_lowbound3d_(2));
    ROS_INFO("ringbuffer_upbound3i_ : %d, %d, %d",md_.ringbuffer_upbound3i_(0),md_.ringbuffer_upbound3i_(1),md_.ringbuffer_upbound3i_(2));
    ROS_INFO("ringbuffer_upbound3d_ : %f, %f, %f",md_.ringbuffer_upbound3d_(0),md_.ringbuffer_upbound3d_(1),md_.ringbuffer_upbound3d_(2));
    ROS_INFO("ringbuffer_origin_3i : %d, %d, %d",md_.ringbuffer_origin3i_(0),md_.ringbuffer_origin3i_(1),md_.ringbuffer_origin3i_(2));
    ROS_INFO("rinfbuffer_origin_3d : %f, %f, %f", md_.lidar_position_(0),md_.lidar_position_(1),md_.lidar_position_(2));
    ROS_INFO("ringbuffer_size_3i_ : %d, %d, %d",md_.ringbuffer_size3i_(0),md_.ringbuffer_size3i_(1),md_.ringbuffer_size3i_(2));
    ROS_INFO("RINGBUFFER END ============================");

#if GRID_MAP_NEW_PLATFORM_TEST
    testIndexingCost();
#endif
}



void ParticleMap::clearBuffer(char casein, int bound)
{
    for(int x = (casein == 0 ? bound : md_.ringbuffer_lowbound3i_(0)); x <= (casein == 1 ? bound : md_.ringbuffer_upbound3i_(0)); ++ x)
    {
        for(int y = (casein == 2 ? bound : md_.ringbuffer_lowbound3i_(1)); y <= (casein == 3 ? bound : md_.ringbuffer_upbound3i_(1)); ++ y)
        {
            for(int z = (casein == 4 ? bound : md_.ringbuffer_lowbound3i_(2)); z <= (casein == 5 ? bound : md_.ringbuffer_upbound3i_(2)); ++ z)
            {
                Vector3i id_global(x,y,z);
                int id_buf = globalIdx2BufIdx(id_global);
                int id_buf_inf = globalIdx2InfBufIdx(id_global);
                Eigen::Vector3i id_global_inf_clr(  (casein == 0 ? x + mp_.inf_grid_ : (casein == 1 ? x - mp_.inf_grid_ : x)),
                                                    (casein == 2 ? y + mp_.inf_grid_ : (casein == 3 ? y - mp_.inf_grid_ : y)),
                                                    (casein == 4 ? z + mp_.inf_grid_ : (casein == 5 ? z - mp_.inf_grid_ : z)));
                /* TODO clear according voxel index 
                    清理你要清理的空间，体素空间，金字塔空间，膨胀空间
                */
                // voxel_objects_number
                // md_.voxels_objects_number[id_buf][0] = 0.f; //weight
                // md_.voxels_objects_number[id_buf][1] = 0.f; //vx
                // md_.voxels_objects_number[id_buf][2] = 0.f; //vy
                // md_.voxels_objects_number[id_buf][3] = 0.f; //vz

            }
        }
    }
}

void ParticleMap::moveRingBuffer()
{
    if(!mp_.have_initialized_)
    {
        initMapBoundary();
    }
    // 更新新的环形缓冲区边界
    Vector3i center_new = pos2GlobalIdx(md_.lidar_position_);
    Vector3i ringbuffer_lowbound3i_new = center_new - mp_.local_update_range3i_;
    Vector3d ringbuffer_lowbound3d_new = ringbuffer_lowbound3i_new.cast<double>() * mp_.voxel_resolution_;
    Vector3i ringbuffer_upbound3i_new = center_new + mp_.local_update_range3i_;
    Vector3d ringbuffer_upbound3d_new = ringbuffer_upbound3i_new.cast<double>() * mp_.voxel_resolution_;
    ringbuffer_upbound3i_new -= Vector3i(1,1,1);

    const Vector3i inf_grid3i(mp_.inf_grid_,mp_.inf_grid_,mp_.inf_grid_);
    const Vector3d inf_grid3d = inf_grid3i.array().cast<double>() * mp_.voxel_resolution_;

    Vector3i ringbuffer_inf_lowbound3i_new = ringbuffer_lowbound3i_new - inf_grid3i;
    Vector3d ringbuffer_inf_lowbound3d_new = ringbuffer_lowbound3d_new - inf_grid3d;
    Vector3i ringbuffer_inf_upbound3i_new = ringbuffer_inf_upbound3i_new + inf_grid3i;
    Vector3d ringbuffer_inf_upbound3d_new = ringbuffer_inf_upbound3d_new + inf_grid3d;

    if(center_new(0) < md_.center_last3i_(0))
    {
        clearBuffer(0,ringbuffer_upbound3i_new(0));
    }
    if(center_new(0) > md_.center_last3i_(0))
    {
        clearBuffer(1,ringbuffer_lowbound3i_new(0));
    }
    if(center_new(1) < md_.center_last3i_(1))
    {
        clearBuffer(2,ringbuffer_upbound3i_new(1));
    }
    if(center_new(1) > md_.center_last3i_(1))
    {
        clearBuffer(3,ringbuffer_lowbound3i_new(1));
    }
    if(center_new(2) < md_.center_last3i_(2))
    {
        clearBuffer(4,ringbuffer_lowbound3i_new(2));
    }
    if(center_new(2) > md_.center_last3i_(2))
    {
        clearBuffer(5,ringbuffer_upbound3i_new(2));
    }

    for(int i=0;i<3;i++)
    {
        while(md_.ringbuffer_origin3i_(i) < md_.ringbuffer_lowbound3i_(i))
        {
            md_.ringbuffer_origin3i_(i) += md_.ringbuffer_size3i_(i);
        }
        while(md_.ringbuffer_origin3i_(i) > md_.ringbuffer_upbound3i_(i))
        {
            md_.ringbuffer_origin3i_(i) -= md_.ringbuffer_size3i_(i);
        }

        while(md_.ringbuffer_inf_origin3i_(i) < md_.ringbuffer_inf_lowbound3i_(i))
        {
            md_.ringbuffer_inf_origin3i_(i) += md_.ringbuffer_inf_size3i_(i);
        }
        while(md_.ringbuffer_inf_origin3i_(i) > md_.ringbuffer_inf_upbound3d_(i))
        {
            md_.ringbuffer_inf_origin3i_(i) -= md_.ringbuffer_inf_size3i_(i);
        }
    }

    md_.center_last3i_ = center_new;
    md_.ringbuffer_lowbound3i_ = ringbuffer_lowbound3i_new;
    md_.ringbuffer_lowbound3d_ = ringbuffer_lowbound3d_new;
    md_.ringbuffer_upbound3i_ = ringbuffer_upbound3i_new;
    md_.ringbuffer_upbound3d_ = ringbuffer_upbound3d_new;
    md_.ringbuffer_inf_lowbound3i_ = ringbuffer_inf_lowbound3i_new;
    md_.ringbuffer_inf_lowbound3d_ = ringbuffer_inf_lowbound3d_new;
    md_.ringbuffer_inf_upbound3i_ = ringbuffer_inf_upbound3i_new;
    md_.ringbuffer_inf_upbound3d_ = ringbuffer_inf_upbound3d_new;

    // ROS_INFO("moving buffer finished ");
}

/* =========================== particle map core funtion =======================================*/


void ParticleMap::mapPrediction()
{
    int operation_counter = 0;
    int exist_particles = 0;
    int voxel_full_remove_counter = 0, pyramid_full_remove_counter = 0;
    int moves_out_counter = 0;
    
    float two_frame_duration = ( md_.current_update_time_ - md_.last_update_time_).toSec();
    // ROS_INFO("two frame_duration : %f ",two_frame_duration);
    // md_.update_time = md_.current_update_time_;
    // update_time_update_counter_ ++;

    /* 清空fov金字塔空间中的*/
    for(auto &j : md_.pyramids_in_fov)
    { // pyramid num 
        for(auto & i : j) 
        { // safe particle num
            i[0] = 0;
        }
    }
    /// Update Particles' state and index in both voxels and pyramids
    ros::Time t1 = ros::Time::now();
    for(int v_index = 0; v_index < mp_.voxel_num_; ++ v_index)//遍历所有体素子空间
    {
        for(int p = 0; p < mp_.safe_particle_num_in_voxel_; ++ p) // 遍历体素子空间中的所有粒子
        {
            // 如果存在，并且不是新move过来的点
            if(md_.voxels_with_particles[v_index][p][0] > 0.1f && md_.voxels_with_particles[v_index][p][0] < 6.f)// exsit, but not new moved
            {
                md_.voxels_with_particles[v_index][p][0] = 1.f;
                // ++ operation_counter;//操作计数
                if(fabs(md_.voxels_with_particles[v_index][p][1]*md_.voxels_with_particles[v_index][p][2]*md_.voxels_with_particles[v_index][p][3]) < 1e-6)
                {
                    // keep small, for static obstacles
                }
                else
                {
                    md_.voxels_with_particles[v_index][p][1] += getVelocityGaussianZeroCenter();
                    md_.voxels_with_particles[v_index][p][2] += getVelocityGaussianZeroCenter();
                    md_.voxels_with_particles[v_index][p][3] += getVelocityGaussianZeroCenter();
                }
#if(LIMIT_MOVEMENT_IN_XY_PLANE)
                md_.voxels_with_particles[v_index][p][3] = 0.f;
#endif       
                // 现在改成粒子的全局坐标,粒子的移动
                // ROS_INFO("[before move] : %f, %f, %f", md_.voxels_with_particles[v_index][p][4],md_.voxels_with_particles[v_index][p][5],md_.voxels_with_particles[v_index][p][6]);
                md_.voxels_with_particles[v_index][p][4] += two_frame_duration * md_.voxels_with_particles[v_index][p][1];
                md_.voxels_with_particles[v_index][p][5] += two_frame_duration * md_.voxels_with_particles[v_index][p][2];
                md_.voxels_with_particles[v_index][p][6] += two_frame_duration * md_.voxels_with_particles[v_index][p][3];
                // ROS_INFO("[after move] : %f, %f, %f", md_.voxels_with_particles[v_index][p][4],md_.voxels_with_particles[v_index][p][5],md_.voxels_with_particles[v_index][p][6]);
                //上面就是计算在这一帧的传感器时刻下，粒子相对于机器人的位置。
                int particle_voxel_index_new; // 粒子在某个体素空间中的新下标

                if(isInBuf( md_.voxels_with_particles[v_index][p][4],
                            md_.voxels_with_particles[v_index][p][5],
                            md_.voxels_with_particles[v_index][p][6]))
                {
                    Vector3d global_pos = { md_.voxels_with_particles[v_index][p][4],
                                            md_.voxels_with_particles[v_index][p][5],
                                            md_.voxels_with_particles[v_index][p][6]};
                    // ros::Time t1 = ros::Time::now();
                    Vector3i global_idx = pos2GlobalIdx(global_pos);
                    particle_voxel_index_new = globalIdx2BufIdx(global_idx);
                    // ROS_INFO("pos to idx cost time : %d ",(ros::Time::now() - t1).toNSec());
                    // ROS_INFO("in [moveAParticle]");
                    int move_flag = moveAParticle(particle_voxel_index_new,v_index,p);

                    // ROS_INFO("move flag : %d", move_flag);

                    if(move_flag == -2) // move pyramid space failed
                    {
                        // Move the particle, if fails, "moveParticleByVoxel" will delete the particle
                        ++ pyramid_full_remove_counter;
                        continue;
                    }
                    else if(move_flag == -1) // move voxel space failed 
                    {
                        ++ voxel_full_remove_counter;
                        continue;
                    }
                    ++ exist_particles;
                }
                else
                {
                    // ROS_INFO("in [removeParticle]");
                    // particle moves out
                    removeParticle(v_index,p);


                    ++ moves_out_counter;
                }

            }
        }
    }
    ros::Time t2 = ros::Time::now();
    // ROS::INFO("[in] map prediction cost time : %d ",(t2 - t1).toNSec());

    if(moves_out_counter > 10000)
    {
        ROS_WARN("!!!!! An error occured! moves out two many particles !!!!  delt_t = %f",two_frame_duration);
    }
}

void ParticleMap::mapUpdate()
{
    int operation_counter_update = 0;
    md_.update_times_++;
    for(int i=0;i<mp_.observation_pyramid_num_;i++)
    {
        for(int j=0; j<md_.point_num_in_pyramid[i];j++)
        {
            for(int n_seq=0; n_seq < md_.observation_pyramid_neighbours[i][0]; n_seq++)
            {
                // 找邻居中的粒子
                int pyramid_check_index = md_.observation_pyramid_neighbours[i][n_seq+1];
                // ROS_INFO("pyramid check index : %d ", pyramid_check_index);
                for(int particle_seq = 0; particle_seq < mp_.safe_particle_num_in_pyramid_; particle_seq++)
                {
                    if(md_.pyramids_in_fov[pyramid_check_index][particle_seq][0] & O_MAKE_VALID)
                    {
                        int particle_voxel_index = md_.pyramids_in_fov[pyramid_check_index][particle_seq][1];
                        int particle_voxel_inner_index = md_.pyramids_in_fov[pyramid_check_index][particle_seq][2];
                        // alg.27
                        float gk =  
                            queryNormalPDF(md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][4], md_.point_cloud_in_pyramid[i][j][0],mp_.sigma_ob) * 
                            queryNormalPDF(md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][5], md_.point_cloud_in_pyramid[i][j][1],mp_.sigma_ob) *
                            queryNormalPDF(md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][6], md_.point_cloud_in_pyramid[i][j][2],mp_.sigma_ob);

                        //alg.25 cal Ck
                        md_.point_cloud_in_pyramid[i][j][3] += mp_.P_detection * md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][7] * gk;
                        // ROS_INFO("md_.pointcloud_in_pyramid[i][j][3] : %f",md_.point_cloud_in_pyramid[i][j][3]);
                    }
                }
            }

            /// add weight for new born particles
            /// kappa should be that kk
            md_.point_cloud_in_pyramid[i][j][3] += (mp_.expected_new_born_objects_ + mp_.kappa);
            // ROS_INFO("md_.pointcloud_in_pyramid[i][j][3] : %f",md_.point_cloud_in_pyramid[i][j][3]);

        }
    }
    // ROS_INFO("map update 1 finished");

    for(int i=0; i< mp_.observation_pyramid_num_;i++)
    {
        int current_pyramid_index = i;
        for(int inner_seq = 0; inner_seq < mp_.safe_particle_num_in_pyramid_; inner_seq ++)
        {
            // Iteration of particles in pyramid
            if(md_.pyramids_in_fov[current_pyramid_index][inner_seq][0] & O_MAKE_VALID)//update only valid particle
            {
                int neighbor_num = md_.observation_pyramid_neighbours[current_pyramid_index][0];

                int particle_voxel_index = md_.pyramids_in_fov[current_pyramid_index][inner_seq][1];
                int particle_voxel_inner_index = md_.pyramids_in_fov[current_pyramid_index][inner_seq][2];

                float px_this = md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][4] - md_.lidar_position_(0);
                float py_this = md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][5] - md_.lidar_position_(1);
                float pz_this = md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][6] - md_.lidar_position_(2);

                float particle_distance = sqrtf(powf(px_this,2) + powf(py_this,2) + powf(pz_this,2));

                // Update only particles that are not occluded, use voxel_resolution as the distance metric.
                // point_cloud_max_length[i] > 0.f 表示这个金字塔空间中有粒子
                // particle_dist_length > point_cloud_max_length[i] + voxel_resolution 当前粒子在点云观测点之后，意味着被遮挡了
                if(md_.max_depth_in_pyramid[i] > 0.f && particle_distance > md_.max_depth_in_pyramid[i] + mp_.voxel_resolution_)
                {
                    // occluded
                    continue; 
                }
                
                //如果这个粒子没有被遮挡
                ///alg.35
                float sum_by_zk = 0.f;
                for(int neighbor_seq = 0; neighbor_seq < neighbor_num; ++ neighbor_seq)
                {
                    int neighbor_index = md_.observation_pyramid_neighbours[current_pyramid_index][neighbor_seq+1];
                    
                    for(int z_seq=0; z_seq < md_.point_num_in_pyramid[neighbor_index]; ++ z_seq)
                    {
                        float gk = queryNormalPDF(px_this, md_.point_cloud_in_pyramid[neighbor_index][z_seq][0],mp_.sigma_ob)
                                    * queryNormalPDF(py_this, md_.point_cloud_in_pyramid[neighbor_index][z_seq][1],mp_.sigma_ob)
                                    * queryNormalPDF(pz_this, md_.point_cloud_in_pyramid[neighbor_index][z_seq][2],mp_.sigma_ob);
                        sum_by_zk += mp_.P_detection * gk / md_.point_cloud_in_pyramid[neighbor_index][z_seq][3];
                        // ROS_INFO("sum_by_zk : %f",sum_by_zk);
                        ++ operation_counter_update;
                    }
                }
                
                // w_k^i
                // ROS_INFO("update times : %d ", md_.update_times_);
                if(md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][7] > 50)
                {
                    // ROS_INFO("[update time : %d] , particle_weight : %f",md_.update_times_,md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][7]);
                }
                md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][7] *= ((1 - mp_.P_detection) + sum_by_zk);
                md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][8] = (md_.current_update_time_ - md_.start_time_).toSec();
                // ROS_INFO("weight after updated : %f", md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][7]);

            }
        }
    }
    // ROS_INFO("map update 2 finished");

        //    cout << "operation_counter_update=" << operation_counter_update <<endl;

}


void ParticleMap::mapAddNewBornParticleByObservation()
{
    /** Calculate normalization coefficient first **/
    float normalization_coefficient = 0.f;
    for(int i=0; i<mp_.observation_pyramid_num_; i++)
    {
        for(int j=0; j < md_.point_num_in_pyramid[i]; j++)
        {
            normalization_coefficient += 1.f / md_.point_cloud_in_pyramid[i][j][3];
        }
    }
    float updated_weight_new_born = mp_.new_born_particle_weight_ * normalization_coefficient;
    // ROS_INFO("updated weight new born : %f", updated_weight_new_born);
    /* Add new born particles */
    static int min_static_new_born_particle_number_each_point = (int)((float)mp_.new_born_particle_number_each_point_ * 0.15f);
    static int static_new_born_particle_number_each_point = (int)((float)mp_.new_born_particle_number_each_point_ * 0.4f);
    static int pf_derive_new_born_particle_number_each_point = (int)((float)mp_.new_born_particle_number_each_point_ * 0.5f);
    static const int model_generated_particle_number_each_point = (int)((float)mp_.new_born_particle_number_each_point_ * 0.8f);
    // ROS_INFO("start add particle by veloctiy point ");
    int successfully_born_particles = 0;
    /// TODO: Improve efficiency in this new born procedure
    // int dyn_p_num = 0;
    for(auto & point : md_.input_cloud_with_velocity_->points)
    {
        // 这里原本是局部的，p_corrected 是局部坐标系的点
        pcl::PointXYZ p_corrected;
        p_corrected.x = point.x;
        p_corrected.y = point.y;
        p_corrected.z = point.z;

        // ROS_INFO("measure point vel : %f, %f, %f", point.normal_x,point.normal_y,point.normal_z);
        int point_voxel_index;
        float static_particle_weight_sum = 0.f;
        float dynamic_particle_weight_sum = 0.f;
        float static_or_dynamic_weight_sum = 0.f;

        if(isInBuf(p_corrected.x,p_corrected.y,p_corrected.z))
        {
            
            Vector3d global_pos = {p_corrected.x,p_corrected.y,p_corrected.z};
            Vector3i global_idx = pos2GlobalIdx(global_pos);
            point_voxel_index = globalIdx2BufIdx(global_idx);
            //This condition should always be true because the point cloud outside of the map should be omitted in the first place. Just an insurance.
            for(int kk = 0; kk < mp_.safe_particle_num_in_voxel_; ++kk)
            {
                if(md_.voxels_with_particles[point_voxel_index][kk][0] > 0.9f && md_.voxels_with_particles[point_voxel_index][kk][0] < 14.f)//not new born
                {
                    float v_abs =   fabs(md_.voxels_with_particles[point_voxel_index][kk][1]) +
                                    fabs(md_.voxels_with_particles[point_voxel_index][kk][2]) +
                                    fabs(md_.voxels_with_particles[point_voxel_index][kk][3]);
                    
                    if(v_abs < 0.1f)
                    {
                        //static
                        static_particle_weight_sum += md_.voxels_with_particles[point_voxel_index][kk][7];
                    }
                    else if(v_abs < 0.5f)
                    {
                        // static or dynamic
                        static_or_dynamic_weight_sum += md_.voxels_with_particles[point_voxel_index][kk][7];
                    }
                    else
                    {
                        // dynamic
                        dynamic_particle_weight_sum += md_.voxels_with_particles[point_voxel_index][kk][7];
                    }
                }
            }
        }
        else
        {
            continue;
        }
        // ROS_INFO("static_particle_weight_sum: %f ,dynamic_particle_weight_sum : %f, static_or_dynamic_weight_sum: %f ",static_particle_weight_sum,dynamic_particle_weight_sum,static_or_dynamic_weight_sum);
        // ROS_INFO("D-S Theory");
        // Dempster-Shafer Theory
        /* alg.43 - 46 */
        float total_weight_voxel = static_particle_weight_sum + dynamic_particle_weight_sum + static_or_dynamic_weight_sum;
        float m_static = static_particle_weight_sum / total_weight_voxel;
        float m_dynamic = dynamic_particle_weight_sum / total_weight_voxel;
        float m_static_or_dynamic = static_or_dynamic_weight_sum / total_weight_voxel;
        
        float p_static = (m_static + m_static + m_static_or_dynamic) * 0.5f;
        float p_dynamic = (m_dynamic + m_dynamic + m_static_or_dynamic) * 0.5f;
        float normalization_p = p_static + p_dynamic;
        float p_static_normalized = p_static / normalization_p;
        float p_dynamic_normalized = p_dynamic / normalization_p;

       // model_generated_particle_number_each_point : \beta
        // p_static_normalized : \lambda_1
        // p
        static_new_born_particle_number_each_point = (int)((float)model_generated_particle_number_each_point * p_static_normalized);
        pf_derive_new_born_particle_number_each_point = model_generated_particle_number_each_point - static_new_born_particle_number_each_point;
        
        // set a minimum number of static particles
        static_new_born_particle_number_each_point = max(min_static_new_born_particle_number_each_point,static_new_born_particle_number_each_point);

        for(int p=0;p<mp_.new_born_particle_number_each_point_;p++)
        {
            std::shared_ptr<Particle> particle_ptr{new Particle};

            particle_ptr->position.x() = p_corrected.x + getPositionGaussianZeroCenter();
            particle_ptr->position.y() = p_corrected.y + getPositionGaussianZeroCenter();
            particle_ptr->position.z() = p_corrected.z + getPositionGaussianZeroCenter();
        

            if(isInBuf(particle_ptr->position.x(),particle_ptr->position.y(),particle_ptr->position.z()))
            {
                particle_ptr->voxel_index = globalIdx2BufIdx(pos2GlobalIdx(particle_ptr->position));
                if(p < static_new_born_particle_number_each_point) // static particle;  p < static num
                {
                    particle_ptr->velocity.x() = 0.f;
                    particle_ptr->velocity.y() = 0.f;
                    particle_ptr->velocity.z() = 0.f;
                }
                else if(point.normal_x > -100.f && p < model_generated_particle_number_each_point) //static num <= p < dynamic num
                {
                    // use estimated velocity to generate new particles
                    if(point.intensity > 0.01f)
                    {

                        particle_ptr->velocity.x() = point.normal_x + 2 * getVelocityGaussianZeroCenter();
                        particle_ptr->velocity.y() = point.normal_y + 2 * getVelocityGaussianZeroCenter();
                        particle_ptr->velocity.z() = point.normal_z + 2 * getVelocityGaussianZeroCenter();
                        // dyn_p_num++;
                        // ROS_INFO("use point cloud vel, vel : %f, %f, %f",point.normal_x,point.normal_y,point.normal_z);
                    }
                    else // static points like ground
                    {
                        particle_ptr->velocity.x() = 0.f;
                        particle_ptr->velocity.y() = 0.f;
                        particle_ptr->velocity.z() = 0.f;    
                    }
                }
                else   /// Considering Random Noise
                {
                    if(point.intensity > 0.01f)
                    {
                        particle_ptr->velocity.x() = generateRandomFloat(-1.5f,1.5f);
                        particle_ptr->velocity.y() = generateRandomFloat(-1.5f,1.5f);
                        particle_ptr->velocity.z() = generateRandomFloat(-0.5f,0.5f);
                    }
                    else //static points like ground
                    {
                        particle_ptr->velocity.x() = 0.f;
                        particle_ptr->velocity.y() = 0.f;
                        particle_ptr->velocity.z() = 0.f;
                    }
                }
#if(LIMIT_MOVEMENT_IN_XY_PLANE)
                particle_ptr->velocity.z() = 0.f;
#endif

                particle_ptr->weight = updated_weight_new_born;
                // ROS_INFO("new add particle weight = %f", particle_ptr->weight);


                // ROS_INFO("try to add a particle, particle_voxel: %d",particle_ptr->voxel_index);
                bool test = addAParticle(particle_ptr, particle_ptr->voxel_index);
                if(test){
                    ++ successfully_born_particles;
                }
                // ROS_INFO("add a particle finished ");
            }
        }
    }
    // std::cout << "dynamic particle num : " << dyn_p_num << std::endl;
    // cout << "successfully_born_particles = "<<successfully_born_particles<<endl;

}


void ParticleMap::mapOccupancyCalculationAndResample()
{
    int removed_particle_counter = 0;
    int particle_num_after_resampling_should_be = 0;
    
    // 占据概率计算和未来状态估计
    for(int v_index=0; v_index < mp_.voxel_num_; ++ v_index)
    {
        // Calculate estimated object number in each voxel
        float weight_sum_voxel, vx_sum_voxel, vy_sum_voxel, vz_sum_voxel;
        weight_sum_voxel = 0.f;
        vx_sum_voxel = vy_sum_voxel = vz_sum_voxel = 0.f;

        int particle_num_voxel = 0; //体素中粒子数
        int old_particle_num_voxel = 0; //体素中之前的粒子数
        for(int p=0; p < mp_.safe_particle_num_in_voxel_; p++) //遍历体素中所有粒子
        {
            if(md_.voxels_with_particles[v_index][p][0] > 0.1f)  // 不是INVALID 粒子
            {
                if(md_.voxels_with_particles[v_index][p][7] < 1e-3)// 权重很小，几乎为0
                {
                    md_.voxels_with_particles[v_index][p][0] = 0.f; // Remove the particle directly if the weight is too small
                }
                else  //如果有一定权重
                {
                    if(md_.voxels_with_particles[v_index][p][0] < 10.f) //exclude new-born particles 排除新生粒子
                    {
                        ++old_particle_num_voxel;
                        vx_sum_voxel += md_.voxels_with_particles[v_index][p][1]; //体素x轴的速度
                        vy_sum_voxel += md_.voxels_with_particles[v_index][p][2]; //体素y轴的速度
                        vz_sum_voxel += md_.voxels_with_particles[v_index][p][3]; //体素z轴的速度
                        /*** Future status prediction ***/      
                        float px_future, py_future, pz_future; 
                        for(int time = 0; time < mp_.prediction_future_time_.size(); ++time)
                        {
                            float prediction_time = mp_.prediction_future_time_[time]; // 
                            // std::cout << prediction_time << std::endl;
                            px_future = md_.voxels_with_particles[v_index][p][4] + md_.voxels_with_particles[v_index][p][1] * prediction_time; //粒子未来x轴位置
                            py_future = md_.voxels_with_particles[v_index][p][5] + md_.voxels_with_particles[v_index][p][2] * prediction_time; //粒子未来y轴位置
                            pz_future = md_.voxels_with_particles[v_index][p][6] + md_.voxels_with_particles[v_index][p][3] * prediction_time; //粒子未来z轴位置

                            // ROS_INFO("particle pos: %f, %f, %f, particle vel: %f, %f, %f",md_.voxels_with_particles[v_index][p][4],
                            //             md_.voxels_with_particles[v_index][p][5],md_.voxels_with_particles[v_index][p][6],md_.voxels_with_particles[v_index][p][1],
                            //             md_.voxels_with_particles[v_index][p][2],md_.voxels_with_particles[v_index][p][3]);
                            int prediction_index;  //预测的体素下标
                            if(isInBuf(px_future,py_future,pz_future))
                            {
                                // 未来6个预测时刻的占据概率
                                // prediction index表示了未来位置对应的体素
                                // 则对应体素的未来状态加上该粒子权重
                                Vector3d global_pos = {px_future,py_future,pz_future};
                                Vector3i global_idx = pos2GlobalIdx(global_pos);
                                prediction_index = globalIdx2BufIdx(global_idx);
                                md_.voxels_objects_number[prediction_index][4+time] += md_.voxels_with_particles[v_index][p][7];   //weight
                                // ROS_INFO("this particle weight : %f",md_.voxels_with_particles[v_index][p][7]);
                                // ROS_INFO("after add this particle, voxel weight : %f",md_.voxels_objects_number[prediction_index][4+time]);
                            }
                        }
                        /**** End of prediction ****/
                    
                    }
                    // 这个粒子是新生粒子，因此将其不再标记为新生粒子
                    md_.voxels_with_particles[v_index][p][0] = 1.f; // Remove newborn flag and moved flag in prediction
                    ++particle_num_voxel; // 体素中增加一个粒子
                    // ROS_INFO("HERE");
                    weight_sum_voxel += md_.voxels_with_particles[v_index][p][7];  //体素中总的weight加上当前粒子的weight
                }
            }
        }
        // ROS_INFO("particle_num_voxel = %d",particle_num_voxel);
        // ROS_INFO("weight_sum_voxel = %f",weight_sum_voxel);
        // ROS_INFO("voxel weight: %f",md_.voxels_objects_number[v_index][0]);
        md_.voxels_objects_number[v_index][0] = weight_sum_voxel; // 将当前体素的权重和传到voxels_objects_number中对应的位置

        if(old_particle_num_voxel > 0)
        {// 1. objects number; 2-4. Avg vx, vy, vz; 5-. Future objects number
            // 如果体素中有旧的粒子，则更新体素对象数组中的对应体素速度
            md_.voxels_objects_number[v_index][1] = vx_sum_voxel / (float)old_particle_num_voxel; //速度取平均
            md_.voxels_objects_number[v_index][2] = vy_sum_voxel / (float)old_particle_num_voxel; //速度取平均
            md_.voxels_objects_number[v_index][3] = vz_sum_voxel / (float)old_particle_num_voxel; //速度取平均
        }else
        {
            /// 没有粒子，则速度是0
            md_.voxels_objects_number[v_index][1] = 0.f; 
            md_.voxels_objects_number[v_index][2] = 0.f;
            md_.voxels_objects_number[v_index][3] = 0.f;
        }

        if(particle_num_voxel < 5){  //Too few particles, no need to resample.
            particle_num_after_resampling_should_be += particle_num_voxel; //for test
            continue;
        }

        // Resampling
        // Calculate desired particle number after resampling
        //alg.40
        int particle_num_voxel_after_resample;
        if(particle_num_voxel > mp_.max_particle_num_in_voxel_){ // 如果体素内粒子数量超了，则拒绝一些采样
            particle_num_voxel_after_resample = mp_.max_particle_num_in_voxel_; 
        }else{ //反之则维持现状。
            particle_num_voxel_after_resample = particle_num_voxel;
        }

        static float weight_after_resample; //重采样后的体素权重
        weight_after_resample = weight_sum_voxel / (float) particle_num_voxel_after_resample;
        // 这个体素内，经过重采样后，的粒子数 加到整个地图所有体素的粒子计数器中   
        particle_num_after_resampling_should_be += particle_num_voxel_after_resample;

        // Resample
        float acc_ori_weight = 0.f;//原始权重
        float acc_new_weight = weight_after_resample * 0.5f;//新权重
        for(int p=0;p<mp_.safe_particle_num_in_voxel_;++p)
        {
            if(md_.voxels_with_particles[v_index][p][0] > 0.7f)
            { //exclude invalid and newly_added_by_resampling particles
                float ori_particle_weight = md_.voxels_with_particles[v_index][p][7]; //原始的粒子权重
                // ROS_INFO("ori_particle_weight : %f",ori_particle_weight);
                acc_ori_weight += ori_particle_weight; //体素原始权重 + 单个粒子的权重

                if(acc_ori_weight > acc_new_weight)
                { //如果体素原来的权重大于体素新的权重
                    md_.voxels_with_particles[v_index][p][7] = weight_after_resample; // keep the particle but change weight
                    acc_new_weight += weight_after_resample; // 

                    // 标志体素空间粒子是否存储满了
                    int if_space_is_currently_full = 0;
                    /** copy particles that have a very large weight **/
                    int p_i=0;
                    //如果加了权重，acc_ori_weight还是大，说明这个权重很大，比平均权重weight_after_resample还要大
                    while(acc_ori_weight > acc_new_weight)
                    { // copy the particle if the original weight is very large
                        int if_found_position_in_voxel = 0; // 标志在体素中找到了空闲位置
                        if(!if_space_is_currently_full){ //如果当前体素现在不是满的
                            for( ; p_i<mp_.safe_particle_num_in_voxel_; ++p_i){ // 遍历体素中空闲的粒子存放区
                                // 把这个点放在合理的体素空间上
                                if(md_.voxels_with_particles[v_index][p_i][0] < 0.1f)
                                { // find an empty position in voxel
                                    // Now copy the particle
                                    md_.voxels_with_particles[v_index][p_i][0] = 0.6f; // Flag: newly_added_by_resampling 
                                    for(int k=1; k<9; k++)
                                    {
                                        md_.voxels_with_particles[v_index][p_i][k] = md_.voxels_with_particles[v_index][p][k]; //把粒子属性放进去
                                    }
                                    if_found_position_in_voxel = 1; //标志在体素中找到了空闲区域，复制成功
                                    break;
                                }
                            }
                        }

                        if(!if_found_position_in_voxel)
                        { // 如果没有找到空闲区域
                            // 如果没有空闲区域，就把权重加到区域内最后的一个粒子上
                            // If the particle should be copied but no space left in either voxel or pyramid, add the weight of the original particle to keep the total weight unchanged.
                            md_.voxels_with_particles[v_index][p][7] += weight_after_resample;  //
                            if_space_is_currently_full = 1; // 标记满了
                        }

                        acc_new_weight += weight_after_resample;
                    }

                }else
                { //如果原来的权重小于新的权重
                    // Remove the particle
                    md_.voxels_with_particles[v_index][p][0] = 0.f; //删除这个粒子
                    removed_particle_counter ++; //计数：删除了多少个粒子
                }
            }
        }


    }
    
    
    
}

///NOTE: If you don't want to use any visualization functions like "getOccupancyMap"
///      or "getOccupancyMapWithFutureStatus", you must call this function after the update step.
void ParticleMap::clearOccupancyMapPrediction()
{
    for(int i=0;i< mp_.voxel_num_;i++)
    {
        for(int j=4; j < mp_.voxel_objects_number_dimension;j++)
        {
            md_.voxels_objects_number[i][j] = 0.f;
        }
    }
}


bool ParticleMap::addAParticle(shared_ptr<Particle> p, int voxel_index)
{
    for(int i=0;i<mp_.safe_particle_num_in_voxel_;i++)
    {
        if(md_.voxels_with_particles[voxel_index][i][0] < 0.1f) // found an empty particle position
        {
            md_.voxels_with_particles[voxel_index][i][0] = 15.f; // new born flag
            md_.voxels_with_particles[voxel_index][i][1] = p->velocity.x();
            md_.voxels_with_particles[voxel_index][i][2] = p->velocity.y();
            md_.voxels_with_particles[voxel_index][i][3] = p->velocity.z();
            md_.voxels_with_particles[voxel_index][i][4] = p->position.x();
            md_.voxels_with_particles[voxel_index][i][5] = p->position.y();
            md_.voxels_with_particles[voxel_index][i][6] = p->position.z();
            md_.voxels_with_particles[voxel_index][i][7] = p->weight;
            md_.voxels_with_particles[voxel_index][i][8] = (md_.current_update_time_ - md_.start_time_).toSec();
            return true;
        }
        // if no space, omit thsi particle in voxel 
    }

    return false;
}

void ParticleMap::addRandomParticles(int particle_num, float avg_weight)
{
    /* initializa some particles */
    int successfully_added_num = 0;
    int voxel_overflow_num = 0;
    for(int i=0;i<particle_num;i++)
    {
        std::shared_ptr<Particle> particle_ptr{new Particle};

        particle_ptr->position.x() = generateRandomFloat(md_.ringbuffer_lowbound3d_(0),md_.ringbuffer_upbound3d_(0));
        particle_ptr->position.y() = generateRandomFloat(md_.ringbuffer_lowbound3d_(1),md_.ringbuffer_upbound3d_(1));
        particle_ptr->position.z() = generateRandomFloat(md_.ringbuffer_lowbound3d_(2),md_.ringbuffer_upbound3d_(2));
        particle_ptr->velocity.x() = generateRandomFloat(-1.f,1.f);
        particle_ptr->velocity.y() = generateRandomFloat(-1.f,1.f);
        particle_ptr->velocity.z() = 0.f;
        particle_ptr->weight = avg_weight;

        if(isInBuf(particle_ptr->position))
        {
            Vector3i global_idx = pos2GlobalIdx(particle_ptr->position);
            int buf_idx = globalIdx2BufIdx(global_idx);
            bool add_successfully = addAParticle(particle_ptr,buf_idx);
            if(add_successfully)
            {
                ++ successfully_added_num;
            }
            else
            {
                ++ voxel_overflow_num;
            }
        }
    }
    ROS_INFO("successfully add num : %d ",successfully_added_num);
    ROS_INFO("voxel overflow num : %d ",voxel_overflow_num);

}

int ParticleMap::moveAParticle(int new_voxel_index, int current_v_index, int current_v_inner_index)
{
    int new_voxel_inner_index = current_v_inner_index;
    if(new_voxel_index != current_v_index) //当前体素和目标体素不是同一个
    {
        md_.voxels_with_particles[current_v_index][current_v_inner_index][0] = 0.f;// Remove from ori voxel first，因此已经nonvalid了，0.f

        /// Find a space in the new voxel and then pyramid. If no space in either voxel or pyramid. This particle would vanish.
        bool successfully_moved_by_voxel = false;

        for(int i=0;i<mp_.safe_particle_num_in_voxel_;++i)
        {
            if(md_.voxels_with_particles[new_voxel_index][i][0] < 0.1f)//empty //如果目标体素下某个存储空间是空的，就将新粒子移动到对应的位置。
            {
                new_voxel_inner_index = i;
                successfully_moved_by_voxel = true;

                md_.voxels_with_particles[new_voxel_index][i][0] = 7.f; // newly moved flag
                for(int k=1; k<9;k++)
                {
                    md_.voxels_with_particles[new_voxel_index][i][k] = md_.voxels_with_particles[current_v_index][current_v_inner_index][k];
                }
                break; // Important
            }
        }

        if(!successfully_moved_by_voxel)
        {
            return -1;
        }
    }


    Vector3d originParticle = { md_.voxels_with_particles[new_voxel_index][new_voxel_inner_index][4],
                                md_.voxels_with_particles[new_voxel_index][new_voxel_inner_index][5],
                                md_.voxels_with_particles[new_voxel_index][new_voxel_inner_index][6]};
    Vector3d transformParticle;
    transformParticleToSensorFrame(originParticle,transformParticle);
    // ROS_INFO("transformParticleToSensorFrame: %f %f %f",transformParticle.x(),transformParticle.y(),transformParticle.z());

    if(inPyramidsAreaInSensorFrame(transformParticle.x(),transformParticle.y(),transformParticle.z()))
    {
        int h_index = findPointPyramidHorizontalIndexInSensorFrame(transformParticle[0],transformParticle[1],transformParticle[2]);
        int v_index = findPointPyramidVerticalIndexInSensorFrame(transformParticle[0],transformParticle[1],transformParticle[2]);


        int particle_pyramid_index_new = h_index * mp_.observation_pyramid_num_vertical_ + v_index;
        // ROS_INFO("[moveparticle]: particle_pyramid_index_new = %d",particle_pyramid_index_new);
        // 找对应金字塔子空间的存储区域中有没有空的位置
        bool successfully_moved_by_pyramid = false;
        for(int j = 0; j < mp_.safe_particle_num_in_pyramid_;j++)
        {
            if(md_.pyramids_in_fov[particle_pyramid_index_new][j][0] == 0)
            {
                // ROS_INFO("[moveparticle]: successfully_moved_by_pyramid");
                md_.pyramids_in_fov[particle_pyramid_index_new][j][0] = 1;
                md_.pyramids_in_fov[particle_pyramid_index_new][j][1] = new_voxel_index;
                md_.pyramids_in_fov[particle_pyramid_index_new][j][2] = new_voxel_inner_index;
                successfully_moved_by_pyramid = true;
                break; //Important
            }
        }
        // ROS_INFO("[movepartilce] : finish moving in pyramid");
        //如果没有成功移动到对应的金字塔，就返回-2
        if(!successfully_moved_by_pyramid)
        {
            md_.voxels_with_particles[new_voxel_index][new_voxel_inner_index][0] = 0.f; /// vanish
            return -2;
        }

        /// Add Gaussian randoms to velocities of particles inside FOV
        if(fabs(md_.voxels_with_particles[new_voxel_index][new_voxel_inner_index][1] * 
                md_.voxels_with_particles[new_voxel_index][new_voxel_inner_index][2] *
                md_.voxels_with_particles[new_voxel_index][new_voxel_inner_index][3]) < 1e-6)
        {
            // keep small, for static obstacles
        }
        else
        {
            md_.voxels_with_particles[new_voxel_index][new_voxel_inner_index][1] += getVelocityGaussianZeroCenter();
            md_.voxels_with_particles[new_voxel_index][new_voxel_inner_index][2] += getVelocityGaussianZeroCenter();
            md_.voxels_with_particles[new_voxel_index][new_voxel_inner_index][3] = 0.f; //getVelocityGaussianZeroCenter();
        }
    }
    // ROS_INFO("end of [moveAParticle]");
    return 1;
}



void ParticleMap::setOriginVoxelFilterResolution(float res)
{
    mp_.voxel_filter_resolution_ = res;
}
void ParticleMap::findPyramidNeighborIndexInFOV(const int index)
{
    int horizontal_index = index / mp_.observation_pyramid_num_vertical_;
    int vertical_index = index % mp_.observation_pyramid_num_vertical_;
    int &neighbor_index = md_.observation_pyramid_neighbours[index][0];
    neighbor_index = 0;
    // ROS_INFO("md_.observation_pyramid_neighbors[i][0]: %d",md_.observation_pyramid_neighbours[index][0]);

    for(int i = -mp_.pyramid_neighbor_one_dimension_;i<=mp_.pyramid_neighbor_one_dimension_;i++)
    {
        for(int j = -mp_.pyramid_neighbor_one_dimension_;j<=mp_.pyramid_neighbor_one_dimension_;j++)
        {
            int h = horizontal_index + i;
            int v = vertical_index + j;
            if(h >= 0 && h < mp_.observation_pyramid_num_horizontal_ && v >= 0 && v < mp_.observation_pyramid_num_vertical_)
            {
                md_.observation_pyramid_neighbours[index][++neighbor_index] = h * mp_.observation_pyramid_num_vertical_ + v;
                // neighbor_index += 1;
            }
        }
    }
    // ROS_INFO("md_.observation_pyramid_neighbors[i][0]: %d",md_.observation_pyramid_neighbours[index][0]);
}

void ParticleMap::removeParticle(int voxel_index, int voxel_inner_index)
{
    md_.voxels_with_particles[voxel_index][voxel_inner_index][0] = 0.f;
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

float ParticleMap::queryNormalPDF(float x, float mu, double sigma)
{
    // ROS_INFO("[queryNormalPDF] : x: %f, mu : %f",x,mu);
    float corrected_x = (x - mu) / sigma;
    if(corrected_x > 9.9f) corrected_x = 9.9f;
    else if(corrected_x < - 9.9f) corrected_x = -9.9f;
    return mp_.standard_gaussian_pdf[(int)(corrected_x * 1000 + 10000)];
}

void ParticleMap::generateGaussianRandomsVectorZeroCenter()
{
    std::default_random_engine random(time(NULL));
    std::normal_distribution<double> n1(0,mp_.position_prediction_stddev);
    std::normal_distribution<double> n2(0,mp_.velocity_prediction_stddev);

    for(int i=0;i<mp_.guassian_random_num_;i++)
    {
        mp_.p_gaussian_randoms[i] = n1(random);
        mp_.v_gaussian_randoms[i] = n2(random);
    }
}

float ParticleMap::getPositionGaussianZeroCenter()
{
    float delt_p = mp_.p_gaussian_randoms[mp_.position_guassian_random_seq_];
    mp_.position_guassian_random_seq_++;
    if(mp_.position_guassian_random_seq_ >= mp_.guassian_random_num_)
    {
        mp_.position_guassian_random_seq_ = 0;
    }
    return delt_p;
}
float ParticleMap::getVelocityGaussianZeroCenter()
{
    float delt_v = mp_.v_gaussian_randoms[mp_.velocity_gaussian_random_seq_];
    mp_.velocity_gaussian_random_seq_++;
    if(mp_.velocity_gaussian_random_seq_ >= mp_.guassian_random_num_)
    {
        mp_.velocity_gaussian_random_seq_ = 0;
    }
    return delt_v;
}

bool ParticleMap::inPyramidsAreaInSensorFrame(float x, float y, float z)
{
    float sinf = z / sqrt(powf(x,2) + powf(y,2) + powf(z,2));
    float f_abs = fabs(asinf(sinf));
    if(f_abs >= 0 && f_abs <= sin(2 * mp_.half_fov_vertical_ * mp_.one_degree_rad_))
    {
        return true;
    }
    else
    {
        return false;
    }
}

int ParticleMap::findPointPyramidHorizontalIndexInSensorFrame(float x, float y, float z)
{
    float horizontal_rad = fmod(atan2(y,x) + 2 * M_PI, 2*M_PI);

    int horizontal_index = std::floor(horizontal_rad / mp_.one_degree_rad_);
    
    if(horizontal_index >= 0 && horizontal_index < 2 * mp_.observation_pyramid_num_horizontal_)
    {
        return horizontal_index;
    }
    ROS_INFO("!!!!!! Please use Function ifInPyramidsArea() to filter the points first before using findPointPyramidHorizontalIndex()");
    return -1;
}

int ParticleMap::findPointPyramidVerticalIndexInSensorFrame(float x,float y,float z)
{
    float vertical_rad = atan2(z,sqrtf(powf(x,2) + powf(y,2)));
    int vertical_index = floor(vertical_rad + mp_.half_fov_vertical_rad_ / mp_.one_degree_rad_);
    if(vertical_index >= 0 && vertical_index < 2 * mp_.observation_pyramid_num_vertical_ )
    {
        return vertical_index;
    }
    ROS_INFO("!!!!!! Please use Function ifInPyramidsAreaInSensorFrame() to filter the points first before using findPyramidVerticalIndexInSensorFrame()");
    return -1;
}


void ParticleMap::transformParticleToSensorFrame(const Vector3d &oriPoint,Vector3d& transformPoint)
{
    Quaterniond tempQuaternion, targetQuaternion;
    tempQuaternion.w() = 0;
    tempQuaternion.x() = oriPoint.x();
    tempQuaternion.y() = oriPoint.y();
    tempQuaternion.z() = oriPoint.z();
    targetQuaternion = md_.lidar_rotation_ * tempQuaternion * md_.lidar_rotation_.inverse();
    transformPoint.x() = targetQuaternion.x() + md_.lidar_position_.x();
    transformPoint.y() = targetQuaternion.y() + md_.lidar_position_.y();
    transformPoint.z() = targetQuaternion.z() + md_.lidar_position_.z();
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

void ParticleMap::colorAssign(int &r, int &g, int &b, float v, float value_min, float value_max,int reverse_color=0)
{
    v = std::max(v, value_min);
    v = std::min(v, value_max);

    float v_range = value_max - value_min;
    int value = floor((v - value_min) / v_range * 240); // Mapping 0~1.0 to 0~240
    value = std::min(value, 240);

    if(reverse_color){
        value = 240 - value;
    }

    int section = value / 60;
    float float_key = (value % 60) / (float)60 * 255;
    int key = floor(float_key);
    int nkey = 255 - key;

    switch(section) {
        case 0: // G increase
            r = 255;
            g = key;
            b = 0;
            break;
        case 1: // R decrease
            r = nkey;
            g = 255;
            b = 0;
            break;
        case 2: // B increase
            r = 0;
            g = 255;
            b = key;
            break;
        case 3: // G decrease
            r = 0;
            g = nkey;
            b = 255;
            break;
        case 4: // Sky blue
            r = 0;
            g = 255;
            b = 255;
            break;
        default: // White
            r = 255;
            g = 255;
            b = 255;
    }

}

void ParticleMap::velocityEstimationThread()
{
    if(md_.current_cloud_->points.empty()) return ;

    md_.input_cloud_with_velocity_->clear();

    // remove ground and transform data, 除去地面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_grond_points(new pcl::PointCloud<pcl::PointXYZ>());

    for(auto &p : md_.current_cloud_->points)
    {
        if(p.z > mp_.virtual_ground_)// 滤掉地面点。
        {
            non_grond_points->points.push_back(p); // 非地面点：
        }else{ // 地面静态点
            static_points->points.push_back(p);  // 静态点：一定包含地面点，
        }
    }

    //cluster 欧式聚类
    /**
     * 1. 如果是墙体，超大型物体，树等，就不是簇
     * 2. clusters include: static cluster, dynamic cluster
    */
    std::vector<pcl::PointIndices> cluster_indices; // 簇中点云到全局点云的索引
    std::vector<ClusterFeature> clusters; // 当前帧分类的cluster
    std::vector<bool> is_cluster; // 这个聚类是不是簇

    if(!non_grond_points->empty()) // 非地面点： 非地面静态点 + 非地面动态点
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        tree->setInputCloud(non_grond_points);
        // kd-tree 欧式聚类
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(2 * mp_.voxel_filter_resolution_);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(10000);

        ec.setSearchMethod(tree);
        ec.setInputCloud(non_grond_points);
        ec.extract(cluster_indices);

        /* =============================== get ClusterFeature ======================================== */
        for(size_t i = 0; i<cluster_indices.size();i++) // 欧式聚类后的簇集合
        {
            ClusterFeature cluster_this;
            cluster_this.intensity = generateRandomFloat(0.1f,0.1f); //For visualization
            
            // 中心计算
            for(int indice : cluster_indices[i].indices)
            {
                cluster_this.center.x() += non_grond_points->points[indice].x;
                cluster_this.center.y() += non_grond_points->points[indice].y;
                cluster_this.center.z() += non_grond_points->points[indice].z;
                ++ cluster_this.point_num;
            }
            cluster_this.center.x() /= (float) cluster_this.point_num;
            cluster_this.center.y() /= (float) cluster_this.point_num;
            cluster_this.center.z() /= (float) cluster_this.point_num;


            // get cluster
            if(cluster_indices[i].indices.size() > mp_.dynamic_cluster_max_point_num_ || cluster_this.center.z() > mp_.dynamic_cluster_max_center_height_)
            { // 如果簇中点数大于阈值，或者簇中心高度大于阈值，则认为是静态点, 墙，房屋，树，不纳入cluster
                ROS_INFO("not cluster, reason : size : %d, height : %f",cluster_indices[i].indices.size(),cluster_this.center.z());
                // Static 
                for(int indice : cluster_indices[i].indices)
                {
                    static_points->push_back(non_grond_points->points[indice]);
                }
                is_cluster.push_back(false);
            }else{
                clusters.push_back(cluster_this);
                is_cluster.push_back(true);
            }
        }
        
        /* ================================ velocity estimation =================================*/
        float delt_t_from_last_observation = (md_.current_update_time_ - md_.last_update_time_).toSec();
        // km algorithm 
        if(!md_.clusters_last_.empty() && !clusters.empty())
        {
            if(delt_t_from_last_observation > 0.00001 && delt_t_from_last_observation< 10.0)
            {
                Matrix<float> matrix_cost(clusters.size(), md_.clusters_last_.size());
                Matrix<bool> matrix_gate(clusters.size(), md_.clusters_last_.size());

                for(size_t i = 0; i < clusters.size(); i++)
                {
                    for(size_t j = 0; j < md_.clusters_last_.size(); j++)
                    {
                        float featureDistance = (clusters[i].center - md_.clusters_last_[j].center).norm();
                        matrix_cost(i,j) = featureDistance < mp_.distance_gate_ ? featureDistance : mp_.distance_gate_ * 5000;
                        matrix_gate(i,j) = matrix_cost(i,j) < mp_.distance_gate_;
                    }
                }

                Munkres<float> munkres_solver;
                munkres_solver.solve(matrix_cost);

                for(size_t i = 0; i < clusters.size(); i++)
                {
                    for(size_t j = 0; j < md_.clusters_last_.size(); j++)
                    {
                        if(matrix_cost(i,j) == 0.0f && matrix_gate(i,j)) // found a match
                        {
                            // ROS_INFO("find a mathc");
                            clusters[i].match_cluster_seq = j;
                            clusters[i].velocity = (clusters[i].center - md_.clusters_last_[j].center) / delt_t_from_last_observation;
                            clusters[i].velocityNorm = clusters[i].velocity.norm();
                            clusters[i].intensity = md_.clusters_last_[j].intensity;  // for visualization 
                            // ROS_INFO("cluster vel : %lf\t%lf\t%lf.",clusters[i].velocity.x(),clusters[i].velocity.y(),clusters[i].velocity.z());

                            if(clusters[i].velocity.norm() > 10.0f)
                            {
                                clusters[i].velocity.setZero();
                            }
                            break;
                        }

                    }
                    /// If no match is found. The cluster velocity is given by struct initialization (v=-1000).
                    clusters[i].match_cluster_seq = -1;
                    clusters[i].velocity.setZero();
                    clusters[i].velocityNorm = 0.f;
                }

            }
        }

        // Use normal to store velocity
        // ROS_INFO("clusters size: %zu",clusters.size());
        // ROS_INFO("clusters_maybe_dynamic size: %zu",clusters_maybe_dynamic.size());

        // if(clusters.size() != clusters_maybe_dynamic.size())
        // {
        //     ROS_WARN("clusters.size() != clusters_maybe_dynamic.size()");
        //     return ;
        // }
        /* ======================== Velocity Allocation to Points ============================== */
        int cluster_id = 0;
        for(size_t i=0;i<cluster_indices.size();i++)
        {
            if(!is_cluster[i]) continue;
            for(int indice : cluster_indices[i].indices)
            {
                pcl::PointXYZINormal p;
                p.x = non_grond_points->points[indice].x;
                p.y = non_grond_points->points[indice].y;
                p.z = non_grond_points->points[indice].z;
                p.normal_x = clusters[cluster_id].velocity.x();
                p.normal_y = clusters[cluster_id].velocity.y();
                p.normal_z = clusters[cluster_id].velocity.z();
                p.intensity = clusters[cluster_id].intensity;
                md_.input_cloud_with_velocity_->push_back(p);
            }
            cluster_id ++;
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
            md_.input_cloud_with_velocity_->push_back(p);
        }

        // ROS_INFO("dynamic_points : %d, static points : %d",non_grond_points->size(),static_points->size());

        md_.clusters_last_ = clusters;
        // ROS_INFO("Velocity estimation done");

    }






}

} // end namespace particle_map