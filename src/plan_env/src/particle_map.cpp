#include "plan_env/particle_map.h"
#include "particle_map.h"



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
    // md_.update_time_ = 0.f;
    // md_.update_counter_ = 0;
    md_.expected_new_born_objects_ = 0.f;
    md_.new_born_particle_weight_ = 0.04f;
    md_.new_born_particle_number_each_point_ = 20;
    mp_.if_record_particle_csv = false;
    // mp_.record_time = 0.0;
    // update_times = 0;
    int init_particle_num = 0;
    float init_weight = 0.01f;

    /* map */
    mp_.voxel_resolution_ = 0.2f; //
    mp_.voxel_resolution_inv_ = 1 / mp_.voxel_resolution_;
    mp_.angle_resolution_ = 1;
    mp_.angle_resolution_inv_ = 1 / mp_.angle_resolution_;
    ROS_INFO("voxel_resolution_inv_ : %f",mp_.voxel_resolution_inv_);
    ROS_INFO("half_fov_horizontal : %f",mp_.half_fov_horizontal_);
    ROS_INFO("half_fov_vertical : %f",mp_.half_fov_vertical_);
    
    
    mp_.angle_resolution_rad_ = (float)mp_.angle_resolution_ / 180.f * M_PI_2f32;
    mp_.half_angle_resolution_rad_ = (float)mp_.angle_resolution_rad_ / 2.f;
    mp_.half_fov_horizontal_rad_ = (float)mp_.half_fov_horizontal_ / 180.f * M_PI_2f32;
    mp_.half_fov_vertical_rad_ = (float)mp_.half_fov_vertical_ / 180.f * M_PI_2f32;


    /* particle map parameters */
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
    md_.cloud_in_map_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    md_.input_cloud_with_velocity_.reset(new pcl::PointCloud<pcl::PointXYZINormal>());
    mp_.p_gaussian_randoms = vector<float>(mp_.guassian_random_num_);
    mp_.v_gaussian_randoms = vector<float>(mp_.guassian_random_num_);
    mp_.standard_gaussian_pdf = vector<float>(mp_.standard_gaussian_pdf_num_);
    md_.voxels_with_particles = vector<vector<vector<float>>>(mp_.voxel_num_,vector<vector<float>>(mp_.safe_particle_num_in_voxel_,vector<float>(9,0.0)));
    md_.voxels_objects_number = vector<vector<float>>(mp_.voxel_num_,vector<float>(mp_.voxel_objects_number_dimension));
    md_.pyramids_in_fov = vector<vector<vector<float >>>(mp_.observation_pyramid_num_,vector<vector<float>>(mp_.safe_particle_num_in_pyramid_,vector<float>(3,0.0)));
    md_.point_cloud_in_pyramid = vector<vector<vector<float >>>(mp_.observation_pyramid_num_,vector<vector<float>>(mp_.observation_max_points_num_one_pyramid_,vector<float>(5)));
    md_.observation_pyramid_neighbours = vector<vector<int>>(mp_.observation_pyramid_num_,vector<int>(mp_.pyramid_neighbor_num_+1));
    md_.point_num_in_pyramid = vector<int>(mp_.observation_pyramid_num_);
    md_.max_depth_in_pyramid = vector<float>(mp_.observation_pyramid_num_);
    md_.future_status = vector<vector<float>>(mp_.voxel_num_,vector<float>(mp_.prediction_time_));
    md_.input_points_ = vector<float>(mp_.max_point_num_*3);
    ROS_INFO("input_points_ size: %zu", md_.input_points_.size());
    ROS_INFO("cloud_in_current_view_rotated size: %zu", md_.cloud_in_map_->size());
    ROS_INFO("input_cloud_with_velocity size: %zu", md_.input_cloud_with_velocity_->size());
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
    mp_.distance_gate_ = 1.5f;
    mp_.point_num_gate_ = 100;
    mp_.maximum_velocity_ = 5.f;


    /* time && odom lidar received */
    md_.occ_need_update_ = false;
    md_.has_first_lidar_ = false;
    md_.has_odom_ = false;
    md_.last_update_time_.fromSec(0);

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

/* ======================== like ego ringbuffeer functions */


void ParticleMap::lidarOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                    const nav_msgs::OdometryConstPtr &odom_msg)
{
    md_.current_update_time_ = odom_msg->header.stamp;

    /*====*/




    /*====*/


    md_.last_update_time_ = md_.current_update_time_;
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
    md_.last_update_time_ = ros::Time::now();
    return true;
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
            }
        }
    }
}


/* =========================== particle map core funtion =======================================*/


void ParticleMap::mapPrediction()
{
    int operation_counter = 0;
    int exist_particles = 0;
    int voxel_full_remove_counter = 0, pyramid_full_remove_counter = 0;
    int moves_out_counter = 0;
    
    float two_frame_duration = ( md_.current_update_time_ - md_.last_update_time_).toSec();
    // md_.update_time = md_.current_update_time_;
    // update_time_update_counter_ ++;

    // for(auto &j : md_.pyramids_in_fov){
    //     for(auto & i : j)
    //     {
    //         i[0] &= O_MAKE_INVALID;
    //     }
    // }
    /// Update Particles' state and index in both voxels and pyramids
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
                md_.voxels_with_particles[v_index][p][4] += two_frame_duration * md_.voxels_with_particles[v_index][p][1];
                md_.voxels_with_particles[v_index][p][5] += two_frame_duration * md_.voxels_with_particles[v_index][p][2];
                md_.voxels_with_particles[v_index][p][6] += two_frame_duration * md_.voxels_with_particles[v_index][p][3];

                //上面就是计算在这一帧的传感器时刻下，粒子相对于机器人的位置。
                int particle_voxel_index_new; // 粒子在某个体素空间中的新下标

                if(isInBuf( md_.voxels_with_particles[v_index][p][4],
                            md_.voxels_with_particles[v_index][p][5],
                            md_.voxels_with_particles[v_index][p][6]))
                {
                    Vector3d global_pos = { md_.voxels_with_particles[v_index][p][4],
                                            md_.voxels_with_particles[v_index][p][5],
                                            md_.voxels_with_particles[v_index][p][6]};
                    Vector3i global_idx = pos2GlobalIdx(global_pos);
                    particle_voxel_index_new = globalIdx2BufIdx(global_idx);
                    int move_flag = moveAParticle(particle_voxel_index_new,v_index,p);
                    if(move_flag == -2)
                    {
                        // Move the particle, if fails, "moveParticleByVoxel" will delete the particle
                        ++ pyramid_full_remove_counter;
                        continue;
                    }
                    else if(move_flag == -1)
                    {
                        ++ voxel_full_remove_counter;
                        continue;
                    }
                    ++ exist_particles;
                }
                else
                {
                    // particle moves out
                    removeParticle(v_index,p);
                    ++ moves_out_counter;
                }

            }
        }
    }

    if(moves_out_counter > 10000)
    {
        ROS_WARN("!!!!! An error occured! moves out two many particles !!!!  delt_t = %f",two_frame_duration);
    }
}

void ParticleMap::mapUpdate()
{
    int operation_counter_update = 0;
    for(int i=0;i<mp_.observation_pyramid_num_;i++)
    {
        for(int j=0; j<md_.point_num_in_pyramid[i];j++)
        {
            for(int n_seq=0; n_seq < md_.observation_pyramid_neighbours[i][0]; n_seq++)
            {
                // 找邻居中的粒子
                int pyramid_check_index = md_.observation_pyramid_neighbours[i][n_seq+1];
                for(int particle_seq = 0; particle_seq < mp_.safe_particle_num_in_pyramid_; particle_seq++)
                {
                    if(md_.pyramids_in_fov[pyramid_check_index][particle_seq][0] & O_MAKE_VALID)
                    // if(true)
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
                    }
                }
            }
            /// add weight for new born particles
            /// kappa should be that kk
            md_.point_cloud_in_pyramid[i][j][3] += (md_.expected_new_born_objects_ + mp_.kappa);
        }
    }

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
                        ++ operation_counter_update;
                    }
                }

                md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][7] *= ((1 - mp_.P_detection) + sum_by_zk);
                md_.voxels_with_particles[particle_voxel_index][particle_voxel_inner_index][8] = (md_.current_update_time_ - md_.start_time_).toSec();

            }
        }
    }

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

    /* Add new born particles */
    static int min_static_new_born_particle_number_each_point = (int)((float)mp_.new_born_particle_number_each_point_ * 0.15f);
    static int static_new_born_particle_number_each_point = (int)((float)mp_.new_born_particle_number_each_point_ * 0.4f);
    static int pf_derive_new_born_particle_number_each_point = (int)((float)mp_.new_born_particle_number_each_point_ * 0.5f);
    static const int model_generated_particle_number_each_point = (int)((float)mp_.new_born_particle_number_each_point_ * 0.8f);

    int successfully_born_particles = 0;
    /// TODO: Improve efficiency in this new born procedure
    for(auto & point : md_.input_cloud_with_velocity_)
    {
        pcl::PointXYZ p_corrected;
        p_corrected.x = point.x;
        p_corrected.y = point.y;
        p_corrected.z = point.z;

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
                if(p < static_new_born_particle_number_each_point)
                {
                    particle_ptr->velocity.x() = 0.f;
                    particle_ptr->velocity.y() = 0.f;
                    particle_ptr->velocity.z() = 0.f;
                }
                else if(point.normal_x > -100.f && p < model_generated_particle_number_each_point)
                {
                    // use estimated velocity to generate new particles
                    if(point.intensity > 0.01f)
                    {
                        particle_ptr->velocity.x() = point.normal_x + 4 * getVelocityGaussianZeroCenter();
                        particle_ptr->velocity.y() = point.normal_y + 4 * getVelocityGaussianZeroCenter();
                        particle_ptr->velocity.z() = point.normal_z + 4 * getVelocityGaussianZeroCenter();
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


                bool test = addAParticle(*particle_ptr, particle_ptr->voxel_index);
                if(test){
                    ++ successfully_born_particles;
                }
            }
        }
    }
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
                        for(int times = 0; times < mp_.prediction_time_; ++times)
                        {
                            float prediction_time = mp_.prediction_future_time_[times]; // 
                            px_future = md_.voxels_with_particles[v_index][p][4] + md_.voxels_with_particles[v_index][p][1] * prediction_time; //粒子未来x轴位置
                            py_future = md_.voxels_with_particles[v_index][p][5] + md_.voxels_with_particles[v_index][p][2] * prediction_time; //粒子未来y轴位置
                            pz_future = md_.voxels_with_particles[v_index][p][6] + md_.voxels_with_particles[v_index][p][3] * prediction_time; //粒子未来z轴位置

                            int prediction_index;  //预测的体素下标
                            if(isInBuf(px_future,py_future,pz_future))
                            {
                                // 未来6个预测时刻的占据概率
                                // prediction index表示了未来位置对应的体素
                                // 则对应体素的未来状态加上该粒子权重
                                Vector3d global_pos = {px_future,py_future,pz_future};
                                Vector3i global_idx = pos2GlobalIdx(global_pos);
                                prediction_index = globalIdx2BufIdx(global_idx);
                                md_.voxels_objects_number[prediction_index][4+times] += md_.voxels_with_particles[v_index][p][7];   //weight
                            }
                        }
                        /**** End of prediction ****/
                    
                    }
                    // 这个粒子是新生粒子，因此将其不再标记为新生粒子
                    md_.voxels_with_particles[v_index][p][0] = 1.f; // Remove newborn flag and moved flag in prediction
                    ++particle_num_voxel; // 体素中增加一个粒子
                    weight_sum_voxel += md_.voxels_with_particles[v_index][p][7];  //体素中总的weight加上当前粒子的weight
                }
            }
        }
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
            if(md_.voxels_with_particles[v_index][p][0] > 0.7f){ //exclude invalid and newly_added_by_resampling particles
                float ori_particle_weight = md_.voxels_with_particles[v_index][p][7]; //原始的粒子权重
                acc_ori_weight += ori_particle_weight; //体素原始权重 + 单个粒子的权重
                if(acc_ori_weight > acc_new_weight){ //如果体素原来的权重大于体素新的权重
                    md_.voxels_with_particles[v_index][p][7] = weight_after_resample; // keep the particle but change weight
                    acc_new_weight += weight_after_resample; // 

                    // 标志体素空间粒子是否存储满了
                    int if_space_is_currently_full = 0;
                    /** copy particles that have a very large weight **/
                    int p_i=0;
                    //如果加了权重，acc_ori_weight还是大，说明这个权重很大，比平均权重weight_after_resample还要大
                    while(acc_ori_weight > acc_new_weight){ // copy the particle if the original weight is very large
                        int if_found_position_in_voxel = 0; // 标志在体素中找到了空闲位置
                        if(!if_space_is_currently_full){ //如果当前体素现在不是满的
                            for( ; p_i<mp_.safe_particle_num_in_voxel_; ++p_i){ // 遍历体素中空闲的粒子存放区
                                // 把这个点放在合理的体素空间上
                                if(md_.voxels_with_particles[v_index][p_i][0] < 0.1f){ // find an empty position in voxel
                                    // Now copy the particle
                                    md_.voxels_with_particles[v_index][p_i][0] = 0.6f; // Flag: newly_added_by_resampling 
                                    for(int k=1; k<9; k++){
                                        md_.voxels_with_particles[v_index][p_i][k] = md_.voxels_with_particles[v_index][p][k]; //把粒子属性放进去
                                    }
                                    if_found_position_in_voxel = 1; //标志在体素中找到了空闲区域，复制成功
                                    break;
                                }
                            }
                        }

                        if(!if_found_position_in_voxel){ // 如果没有找到空闲区域
                            // 如果没有空闲区域，就把权重加到区域内最后的一个粒子上
                            // If the particle should be copied but no space left in either voxel or pyramid, add the weight of the original particle to keep the total weight unchanged.
                            md_.voxels_with_particles[v_index][p][7] += weight_after_resample;  //
                            if_space_is_currently_full = 1; // 标记满了
                        }

                        acc_new_weight += weight_after_resample;
                    }

                }else{ //如果原来的权重小于新的权重
                    // Remove the particle
                    md_.voxels_with_particles[v_index][p][0] = 0.f; //删除这个粒子
                    removed_particle_counter ++; //计数：删除了多少个粒子
                }
            }
        }


    }
    
    
    
}


bool ParticleMap::addAParticle(const Particle &p, int voxel_index)
{
    for(int i=0;i<mp_.safe_particle_num_in_voxel_;i++)
    {
        if(md_.voxels_with_particles[voxel_index][i][0] < 0.1f) // found an empty particle position
        {
            md_.voxels_with_particles[voxel_index][i][0] = 15.f; // new born flag
            md_.voxels_with_particles[voxel_index][i][1] = p.velocity.x();
            md_.voxels_with_particles[voxel_index][i][2] = p.velocity.y();
            md_.voxels_with_particles[voxel_index][i][3] = p.velocity.z();
            md_.voxels_with_particles[voxel_index][i][4] = p.position.x();
            md_.voxels_with_particles[voxel_index][i][5] = p.position.y();
            md_.voxels_with_particles[voxel_index][i][6] = p.position.z();
            md_.voxels_with_particles[voxel_index][i][7] = p.weight;
            md_.voxels_with_particles[voxel_index][i][8] = (md_.current_update_time_ - md_.start_time_).toSec();
            return true;
        }
        // if no space, omit thsi particle in voxel 
    }

    return false;
}


int ParticleMap::moveAParticle(int new_voxel_index, int current_v_index, int current_v_inner_index)
{
    int new_voxel_inner_index = current_v_inner_index;
    if(new_voxel_index != current_v_index) //当前体素和目标体素不是同一个
    {
        md_.voxels_with_particles[current_v_index][current_v_inner_index][0] = 0.f;// Remove from ori voxel first，因此已经nonvalid了，0.f

        /// Find a space in the new voxel and then pyramid. If no space in either voxel or pyramid. This particle would vanish.
        int successfully_moved_by_voxel = 0;
        for(int i=0;i<mp_.safe_particle_num_in_voxel_;++i)
        {
            if(md_.voxels_with_particles[new_voxel_index][i][0] < 0.1f)//empty //如果目标体素下某个存储空间是空的，就将新粒子移动到对应的位置。
            {
                new_voxel_inner_index = i;
                successfully_moved_by_voxel = 1;

                md_.voxels_with_particles[new_voxel_index][i][0] = 7.f; // newly moved flag
                for(int k=1; k<9;k++)
                {
                    md_.voxels_with_particles[new_voxel_index][i][k] = md_.voxels_with_particles[current_v_index][current_v_inner_index][k];
                }
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

    if(inPyramidsAreaInSensorFrame(transformParticle.x(),transformParticle.y(),transformParticle.z()))
    {
        int h_index = findPointPyramidHorizontalIndexInSensorFrame(transformParticle[0],transformParticle[1],transformParticle[2]);
        
        int v_index = findPointPyramidVerticalIndexInSensorFrame(transformParticle[0],transformParticle[1],transformParticle[2]);

        int particle_pyramid_index_new = h_index * mp_.observation_pyramid_num_horizontal_ + v_index;

        // 找对应金字塔子空间的存储区域中有没有空的位置
        int successfully_moved_by_pyramid = 0;
        for(int j = 0; j < mp_.safe_particle_num_in_pyramid_;j++)
        {
            if(md_.pyramids_in_fov[particle_pyramid_index_new][j][0] == 0)
            {
                md_.pyramids_in_fov[particle_pyramid_index_new][j][0] |= O_MAKE_VALID;
                md_.pyramids_in_fov[particle_pyramid_index_new][j][1] = new_voxel_index;
                md_.pyramids_in_fov[particle_pyramid_index_new][j][2] = new_voxel_inner_index;
                successfully_moved_by_pyramid = 1;
                break;
            }
        }
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
            md_.voxels_with_particles[new_voxel_index][new_voxel_inner_index][3] += //getVelocityGaussianZeroCenter();
        }
    }
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

float ParticleMap::queryNormalPDF(float &x, float &mu, float &sigma)
{
    float corrected_x = (x - mu) / sigma;
    if(corrected_x > 9.9f) corrected_x = 9.9f;
    else if(corrected_x < - 9.9f) corrected_x = -9.9f;
    return mp_.standard_gaussian_pdf[(int)(corrected_x * 1000 + 10000)];
}

float ParticleMap::getPositionGaussianZeroCenter()
{
    float delt_p = mp_.p_gaussian_randoms[md_.position_guassian_random_seq_];
    md_.position_guassian_random_seq_++;
    if(md_.position_guassian_random_seq_ >= mp_.guassian_random_num_)
    {
        md_.position_guassian_random_seq_ = 0;
    }
    return delt_p;
}
float ParticleMap::getVelocityGaussianZeroCenter()
{
    float delt_v = mp_.v_gaussian_randoms[md_.velocity_gaussian_random_seq_];
    md_.velocity_gaussian_random_seq_++;
    if(md_.velocity_gaussian_random_seq_ >= mp_.guassian_random_num_)
    {
        md_.velocity_gaussian_random_seq_ = 0;
    }
    return delt_v;
}

bool ParticleMap::inPyramidsAreaInSensorFrame(float x, float y, float z)
{
    float sinf = z / sqrt(powf(x,2) + powf(y,2) + powf(z,2));
    float f = asinf(sinf);
    if(f >= 0 && f<= sin(2 * mp_.half_fov_vertical_ * M_PI_2f32 / 180.f))
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
    float horizontal_rad = fmod(atan2(y,x) + 2 * M_1_PI, 2*M_1_PI);

    int horizontal_index = std::floor(horizontal_rad / mp_.angle_resolution_rad_);
    
    if(horizontal_index >= 0 && horizontal_index < 360)
    {
        return horizontal_index;
    }
    ROS_INFO("!!!!!! Please use Function ifInPyramidsArea() to filter the points first before using findPointPyramidHorizontalIndex()");
    return -1;
}

int ParticleMap::findPointPyramidVerticalIndexInSensorFrame(float x,float y,float z)
{
    float vertical_rad = atan2(z,sqrtf(powf(x,2) + powf(y,2)));
    float sinf = z / sqrtf( powf(x,2) + powf(y,2) + powf(z,2));
    float f = asinf(sinf);
    int vertialIndex = std::floor(f / mp_.angle_resolution_rad_);
    if(vertialIndex <= 2 * mp_.half_fov_vertical_ / mp_.angle_resolution_)
    {
        return vertialIndex;
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


void ParticleMap::velocityEstimationThread()
{
    if(md_.cloud_in_map_->points.empty()) return ;

    md_.input_cloud_with_velocity_->clear();

    // remove ground and transform data, 除去地面点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_grond_points(new pcl::PointCloud<pcl::PointXYZ>());

    for(auto &p : md_.cloud_in_map_->points)
    {
        if(p.z > mp_.voxel_filter_resolution_)// 滤掉地面点。
        {
            non_grond_points->points.push_back(p); // 非地面点：
        }else{ // 地面静态点
            static_points->points.push_back(p);  // 静态点：一定包含地面点，
        }
    }

    //cluster 欧式聚类
    // static std::vector<ClusterFeature> cluster_feature_vector_dynamic_last; // 上一帧的分类的cluster
    std::vector<ClusterFeature> clusters_feature_vector_dynamic; // 当前帧分类的cluster
    std::vector<pcl::PointIndices> cluster_indices; // 簇中点云到全局点云的索引
    vector<bool> cluster_possibly_dynamic;  // 簇是否可能是动态的

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

            if(cluster_indice.indices.size() > mp_.dynamic_cluster_max_point_num_ || cluster_this.center.z() > mp_.dynamic_cluster_max_center_height_)
            { // 如果簇中点数大于阈值，或者簇中心高度大于阈值，则认为是静态点
                // Static 
                for(int indice : cluster_indice.indices)
                {
                    static_points->push_back(non_grond_points->points[indice]);
                }
            }else{
                // Possibly dynamic
                clusters_feature_vector_dynamic.push_back(cluster_this);
                cluster_possibly_dynamic.push_back(true);
            }
        }

        // km algorithm 
        // static float distance_gate = 1.5f;
        // static float point_num_gate = 100;
        // static float maximum_velocity = 5.f;
        float delt_t_from_last_observation = (md_.current_update_time_ - md_.last_update_time_).toSec();

        /// Move last feature vector d and match by KM algorithm
        if(!md_.cluster_features_dynamic_last_.empty() && !cluster_possibly_dynamic.empty())
        {
            if(delt_t_from_last_observation > 0.00001 && delt_t_from_last_observation< 10.0)
            {
                Matrix<float> matrix_cost(clusters_feature_vector_dynamic.size(), md_.cluster_features_dynamic_last_.size());
                Matrix<float> matrix_gate(clusters_feature_vector_dynamic.size(), md_.cluster_features_dynamic_last_.size());

                for(size_t row = 0; row < clusters_feature_vector_dynamic.size(); ++row)
                {
                    for(size_t col = 0; col < md_.cluster_features_dynamic_last_.size(); ++col)
                    {
                        float distance = clusterDistance(clusters_feature_vector_dynamic[row], md_.cluster_features_dynamic_last_[col]);
                        if(abs(clusters_feature_vector_dynamic[row].point_num - md_.cluster_features_dynamic_last_[col].point_num) > mp_.point_num_gate_ 
                            || distance >= mp_.distance_gate_)
                        {
                            matrix_gate(row,col) = 0.f;
                            matrix_cost(row,col) = mp_.distance_gate_ * 5000.f;
                        }
                        else{
                            matrix_gate(row,col) = 1.f;
                            matrix_cost(row,col) = distance / mp_.distance_gate_ * 1000.f;;
                        }
                    }
                }

                Munkres<float> munkres_solver;
                munkres_solver.solve(matrix_cost);

                for(size_t row = 0; row < clusters_feature_vector_dynamic.size(); ++row)
                {
                    for(size_t col = 0; col < md_.cluster_features_dynamic_last_.size(); ++col)
                    {
                        if(matrix_cost(row,col) == 0.f && matrix_gate(row,col) > 0.01f) // found a match
                        {
                            clusters_feature_vector_dynamic[row].match_cluster_seq = col;
                            clusters_feature_vector_dynamic[row].velocity.x() =(clusters_feature_vector_dynamic[row].center.x() - md_.cluster_features_dynamic_last_[col].center.x()) / delt_t_from_last_observation;
                            clusters_feature_vector_dynamic[row].velocity.y() = (clusters_feature_vector_dynamic[row].center.y() - md_.cluster_features_dynamic_last_[col].center.y()) / delt_t_from_last_observation;
                            clusters_feature_vector_dynamic[row].velocity.z() = (clusters_feature_vector_dynamic[row].center.z() - md_.cluster_features_dynamic_last_[col].center.z()) / delt_t_from_last_observation;
                            clusters_feature_vector_dynamic[row].velocityNorm = clusters_feature_vector_dynamic[row].velocity.norm();
                            clusters_feature_vector_dynamic[row].intensity = clusters_feature_vector_dynamic[row].intensity;

                            if(clusters_feature_vector_dynamic[row].velocityNorm > mp_.maximum_velocity_)
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
                    md_.input_cloud_with_velocity_->push_back(p);
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
            md_.input_cloud_with_velocity_->push_back(p);
        }

        md_.cluster_features_dynamic_last_ = clusters_feature_vector_dynamic;
        ROS_INFO("Velocity estimation done");
        ROS_INFO("cluster_feature_vector_dynamic_last size: %zu", md_.cluster_features_dynamic_last_.size());

    }




}

} // end namespace particle_map