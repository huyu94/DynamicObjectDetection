#include "plan_env/dsp_map.h"
#include <string>

using namespace std;

// particle_save_folder = ".";

namespace dsp_map
{


DspMap::DspMap()
{
    cout << "Map is created!!!!" << endl;
}

void DspMap::initMap(ros::NodeHandle& nh)
{
    /********************set hype-parameters******************************/
    node_ = nh;
    voxelResolution_ = VOXEL_RESOLUTION;
    angleResolution_ = ANGLE_RESOLUTION;
    max_particle_num_voxel = MAX_PARTICLE_NUM_VOXEL;
    max_input_points = 5000;
    velocity_gaussian_random_seq = 0;
    position_gaussian_random_seq = 0;
    position_prediction_stddev = 0.2f;
    velocity_prediction_stddev = 0.1f;
    sigma_ob = 0.2f;
    kappa = 0.01f;
    P_detection = 0.95f;
    update_time = 0.f;
    update_counter = 0;
    expected_new_born_objects = 0.f;
    new_born_particle_weight = 0.04f;
    new_born_particle_number_each_point = 20;
    if_record_particle_csv = 1;
    record_time = 10.f;
    new_born_particle_weight = 0.f;
    total_time = 0.0;
    update_times = 0;
    int init_particle_num = 0;
    float init_weight=0.01f;
    int pose_type = 1;
    
    cloud_in_current_view_rotated.reset(new pcl::PointCloud<pcl::PointXYZ>()) ;
    input_cloud_with_velocity.reset(new pcl::PointCloud<pcl::PointXYZINormal>);

   /*** Set map parameters **/
    currentSensorPosition_.setZero();
    lastSensorPosition_.setZero();
    currentSensorRotation_.setIdentity();
    mapPosition_.setZero();
    mapSize_ = {MAP_LENGTH_VOXEL_NUM,MAP_WIDTH_VOXEL_NUM,MAP_HEIGHT_VOXEL_NUM};
    mapLength_ = voxelResolution_ * mapSize_.cast<float>();
    mapStartIndex_.setZero();
    mapMinBoundary_ = mapPosition_ - mapLength_;
    mapMaxBoundary_ = mapPosition_ + mapLength_;
    last_timestamp_ = timestamp_ = 0;
    first_call_ = true;
    



    voxels_total_num = VOXEL_NUM; // 总体素数量

    pyramid_num_h = 360 / angleResolution_; //金字塔空间存储行
    pyramid_num_v = 180 / angleResolution_; //金字塔空间存储列
    pyramid_total_num = PYRAMID_NUM; // 金字塔空间数量

    half_fov_h_rad = (float)half_fov_h / 180.f * M_PIf32; // 金字塔空间水平角的一半-弧度制
    half_fov_v_rad = (float)half_fov_v / 180.f * M_PIf32; // 金字塔空间垂直角的一半-弧度制

    angle_resolution_half = (float)angleResolution_ /  2.f; // 角度分辨率的一半

    angle_resolution_rad = (float)angleResolution_ / 180.f * M_PIf32; // 角度分辨率-弧度制
    angle_resolution_rad_half = angle_resolution_rad/2.f; //角度分辨率的一半-弧度制

    // Initialize voxel，体素中的所有存储粒子的空间初始化
    for(auto & i : voxels_with_particle){
        for(auto & j : i){
            for(float & k : j){
                k = 0.f;
            }
        }
    }

    // Initialize pyramids，金字塔子空间的存储空间初始化
    for(auto & pyramid : pyramids_in_fov){
        for(auto & p : pyramid){
            p[0] = 0;
            p[1] = 0;
        }
    }

    /// New: set pyramid plane initial parameters
    // int h_start_seq = 0; // 金字塔空间序列水平的遍历下标起点
    // int h_end_seq = 2 * half_fov_h / angle_resolution_; // 金字塔空间序列水平的遍历下标终点
    // 给金字塔子空间赋上水平方向的法向量
    // for(int i=h_start_seq; i<h_end_seq; i++){ // 0-359
    //     pyramid_BPnorm_params_ori_h[i][0] = cos((float)i*angle_resolution_rad); // x
    //     pyramid_BPnorm_params_ori_h[i][1] = sin((float)i*angle_resolution_rad); // y
    //     pyramid_BPnorm_params_ori_h[i][2] = 0.f; // z
    // }

    // int v_start_seq = 0;// 金字塔空间序列垂直的遍历下标起点
    // int v_end_seq = 2 * half_fov_v / angle_resolution_;// 金字塔空间序列垂直的遍历下标终点
    // 给金字塔子空间赋上垂直方向的法向量
    // for(int i=v_start_seq; i<v_end_seq; i++){ // 0-53
    //     pyramid_BPnorm_params_ori_v[i][0] = cos((float)i*angle_resolution_rad);  // x
    //     pyramid_BPnorm_params_ori_v[i][1] = 0.f; // y
    //     pyramid_BPnorm_params_ori_v[i][2] = sin((float)i*angle_resolution_rad); // z
    // }

    // Find neighborhood pyramids' indexes for observation pyramids
    for(int i=0; i< observation_pyramid_num; i++){  //Initialize point num in the storage
        findPyramidNeighborIndexInFOV(i, observation_pyramid_neighbors[i][0], &observation_pyramid_neighbors[i][1]);
    }

    // Generate Gaussian randoms.
    srand (static_cast <unsigned> (time(0))); //TEST
    generateGaussianRandomsVectorZeroCenter(); //生成速度和位置的零均值高斯噪音
    calculateNormalPDFBuffer(); //计算高斯随机分布函数的缓存，存储到数组

    setPredictionVariance(0.05,0.05);
    setObservationStdDev(0.1);
    setNewBornParticleNumberofEachPoint(20);
    setNewBornParticleWeight(0.0001);
    setOriginalVoxelFilterResolution(voxelResolution_);
    setParticleRecordFlag(0,19.0);

    /**********************************************************************/
    addRandomParticles(init_particle_num, init_weight);

    cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    

    /*init callback*/
    pointcloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_,"/dsp_map/pointcloud",50));
    if(pose_type == POSE_STAMPED)
    {
        // ROS_INFO("POSE_TYPE:%d",pose_type);
        pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_,"/dsp_map/pose",25));
        sync_pointcloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyPointCloudPose>(
            SyncPolicyPointCloudPose(100),*pointcloud_sub_,*pose_sub_));
        sync_pointcloud_pose_->registerCallback(boost::bind(&DspMap::pointcloudPoseCallback,this,_1,_2));
    }
    else if(pose_type == ODOMETRY)
    {
        odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_,"/dsp_map/odom",25));
        sync_pointcloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyPointCloudOdom>(
            SyncPolicyPointCloudOdom(100),*pointcloud_sub_,*odom_sub_));
        sync_pointcloud_odom_->registerCallback(boost::bind(&DspMap::pointcloudOdomCallback,this,_1,_2));
    }

    occ_timer_ = node_.createTimer(ros::Duration(0.05),&DspMap::updateMapCallback,this);
    // vis_timer_ = node_.createTimer(ros::Duration(0.05),&DspMap::visCallback,this);

    map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/dsp_map/occupancy",1);
    inflate_map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/dsp_map/inflate_map",1);
    future_map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("/dsp_map/future_Map",1);
    cube_marker_pub_ = node_.advertise<visualization_msgs::Marker>("/dsp_map/map_cube",1);

    // indep_pointcloud_sub_ = node_.subscribe<sensor_msgs::PointCloud2>("/dsp_map/pointcloud",25,&DspMap::indepPointCloudCallback,this);
    // indep_pose_sub_ = node_.subscribe<geometry_msgs::PoseStamped>("/dsp_map/pose",25,&DspMap::indepPoseCallback,this);
    cout << "Map is ready to update!" << endl;




}






// void DspMap::indepPointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointcloud)
// {
//     // ROS_INFO("getPointcloud!!!!");
//     inflate_map_pub_.publish(*pointcloud);
// }
// void DspMap::indepPoseCallback(const geometry_msgs::PoseStampedConstPtr& pose)
// {
//     position_.x() = (float)pose->pose.position.x;
//     position_.y() = (float)pose->pose.position.y;
//     position_.z() = (float)pose->pose.position.z;
// }


void DspMap::pointcloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                                const nav_msgs::OdometryConstPtr &odom)
{
    cout << "DspMap is pointcloudOdomCallback" << endl;
    Position sensor_position;
    Rotation sensor_att;
    float data_time_stamp =  cloud->header.stamp.toSec();
    sensor_position.x() = (float)odom->pose.pose.position.x;
    sensor_position.y() = (float)odom->pose.pose.position.y;
    sensor_position.z() = (float)odom->pose.pose.position.z;
    sensor_att.x() = odom->pose.pose.orientation.x;
    sensor_att.y() = odom->pose.pose.orientation.y;
    sensor_att.z() = odom->pose.pose.orientation.z;
    sensor_att.w() = odom->pose.pose.orientation.w;
    // 降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *cloud_in);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(voxelResolution_,voxelResolution_,voxelResolution_);
    sor.filter(*cloud_filtered);

    // 检测地图内的点云，并放入数组。
    int useful_point_num = 0; 
    for(size_t i=0;i<cloud_filtered->points.size();i++)
    {
        
        float x = cloud_filtered->points.at(i).x;
        float y = cloud_filtered->points.at(i).y;
        float z = cloud_filtered->points.at(i).z;
        // ROS_INFO("x:%f,y:%f,z:%f",x,y,z);
        if(ifPositionInMap(x,y,z))
        {
            input_points[useful_point_num*3] = x;
            input_points[useful_point_num*3+1] = y;
            input_points[useful_point_num*3+2] = z;
            ++ useful_point_num;

            if(useful_point_num >= MAX_POINT_NUM)
            {
                break;
            }
        }
    }
    // ROS_INFO("useful_point_num:%d",useful_point_num);


}

void DspMap::pointcloudPoseCallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                                    const geometry_msgs::PoseStampedConstPtr &pose)
{
    // 传感器位置、姿态读入
    Position sensor_position;
    Rotation sensor_att;
    double data_time_stamp = cloud->header.stamp.toSec();
    sensor_position.x() = (float)pose->pose.position.x;
    sensor_position.y() = (float)pose->pose.position.y;
    sensor_position.z() = (float)pose->pose.position.z;
    sensor_att.x() = pose->pose.orientation.x;
    sensor_att.y() = pose->pose.orientation.y;
    sensor_att.z() = pose->pose.orientation.z;
    sensor_att.w() = pose->pose.orientation.w;
    if(first_call_){
        first_call_ = false;
        lastSensorPosition_ = currentSensorPosition_ = sensor_position;
        currentSensorRotation_ = sensor_att;
        last_timestamp_ = timestamp_ = data_time_stamp;
    }
    // // updatePosAndBoundary(sensor_position);
    // position_ = sensor_position;
    // ROS_INFO("map_length_half:%f\t%f\t%f",map_length_half(0),map_length_half(1),map_length_half(2));
    // ROS_INFO("max_boundary:%f\t%f\t%f",map_max_boundary(0),map_max_boundary(1),map_max_boundary(2));
    // ROS_INFO("min_boundary:%f\t%f\t%f",map_min_boundary(0),map_min_boundary(1),map_min_boundary(2));
    


    // 点云下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud, *cloud_in);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(voxelResolution_,voxelResolution_,voxelResolution_);
    sor.filter(*cloud_filtered);

    /*如果接收到的点云是世界坐标系的，就要用下面的转换矩阵转到传感器坐标系*/
    // Eigen::Matrix4f sensor2world = Eigen::Matrix4f::Identity();
    // sensor2world.block<3,3>(0,0) = sensor_att.toRotationMatrix();
    // sensor2world(0,3) = sensor_position.x();
    // sensor2world(1,3) = sensor_position.y();
    // sensor2world(2,3) = sensor_position.z();
    // Eigen::Matrix4f world2sensor = sensor2world.inverse();
    // pcl::transformPointCloud(*cloud_filtered,*cloud_filtered,world2sensor);
    
    // 点云输入
    int useful_point_num = 0; 
    for(size_t i=0;i<cloud_filtered->points.size();i++)
    {
        
        float x = cloud_filtered->points[i].x;
        float y = cloud_filtered->points[i].y;
        float z = cloud_filtered->points[i].z;
        // ROS_INFO("x:%f,y:%f,z:%f",x,y,z);
        if(ifPositionInMap(x,y,z))
        {
            input_points[useful_point_num*3] = x;
            input_points[useful_point_num*3+1] = y;
            input_points[useful_point_num*3+2] = z;
            ++ useful_point_num;

            if(useful_point_num >= MAX_POINT_NUM)
            {
                break;
            }
        }
    }
    // ROS_INFO("useful_point_num:%d",useful_point_num);
    /* 测试 */
    // pcl::PointCloud<pcl::PointXYZ> temp_cloud;
    // pcl::PointXYZ temp_point;
    // for(int i=0;i<useful_point_num;i++)
    // {
    //     temp_point.x = input_points[i*3];
    //     temp_point.y = input_points[i*3+1];
    //     temp_point.z = input_points[i*3+2];
    //     temp_cloud.push_back(temp_point);
    // }
    // // ROS_INFO("useful_point_num:%d",useful_point_num);
    // // 发布test cloud
    // sensor_msgs::PointCloud2 output_cloud;
    // pcl::toROSMsg(temp_cloud,output_cloud);
    // output_cloud.header.frame_id = "world";
    // output_cloud.header.stamp = cloud->header.stamp;
    // map_pub_.publish(output_cloud);

    /*暂时先不更新,只看是否可以接受到点云*/
    /// 地图更新
    clock_t start1, finish1;
    start1 = clock();

    // std::cout << "uav_position="<<sensor_position.x() <<", "<<sensor_position.y()<<", "<<sensor_position.z()<<endl;

    // This is the core function we use
    if(!update(useful_point_num, 3, input_points,
                sensor_position, data_time_stamp,
                sensor_att)){
        return;
    }

    /// Display update time
    finish1 = clock();
    double duration1 = (double)(finish1 - start1) / CLOCKS_PER_SEC;
    // printf( "****** Map update time %f seconds\n", duration1);

    static double total_time = 0.0;
    static int update_times = 0;

    total_time += duration1;
    update_times ++;
    // printf( "****** Map avg time %f seconds\n \n", total_time / update_times);

        /// Get occupancy status, including future status.
    // clock_t start2, finish2;
    // start2 = clock();

    int occupied_num=0;
    pcl::PointCloud<pcl::PointXYZ> cloud_to_publish;
    sensor_msgs::PointCloud2 cloud_to_pub_transformed;
    /** Note: The future status is stored with voxel structure.
     * The voxels are indexed with one dimension.
     * You can use Function getVoxelPositionFromIndexPublic() to convert index to real position.
     * future_status[*][0] is current status considering delay compensation.
    **/
    getOccupancyMapWithFutureStatus(occupied_num, cloud_to_publish, &future_status[0][0], 0.3);
    // ROS_INFO("occupied_num:%d",occupied_num);
    // ROS_INFO("cloud size:%d",cloud_to_publish.points.size());
    pcl::toROSMsg(cloud_to_publish,cloud_to_pub_transformed);
    cloud_to_pub_transformed.header.frame_id = "world";
    cloud_to_pub_transformed.header.stamp = cloud->header.stamp;
    map_pub_.publish(cloud_to_pub_transformed);
 
}    


void DspMap::updateMapCallback(const ros::TimerEvent& /*event*/)
{
    pcl::PointXYZI pt;
    pcl::PointCloud<pcl::PointXYZI> cloud_to_publish;
    for(int i=0;i<voxels_total_num;i++){
        for(int j=0;j<SAFE_PARTICLE_NUM_VOXEL;j++){
            pt.x = voxels_with_particle[i][j][4];
            pt.y = voxels_with_particle[i][j][5];
            pt.z = voxels_with_particle[i][j][6];
            pt.intensity = voxels_with_particle[i][j][7];
            cloud_to_publish.push_back(pt);
        }
    }
    sensor_msgs::PointCloud2 cloud_to_pub_transformed;
    pcl::toROSMsg(cloud_to_publish,cloud_to_pub_transformed);
    cloud_to_pub_transformed.header.frame_id = "world";
    // cloud_to_pub_transformed.header.stamp = (float/;
    inflate_map_pub_.publish(cloud_to_pub_transformed);
}

void DspMap::visCallback(const ros::TimerEvent& /*event*/)
{
    // visualization_msgs::Marker cube_marker;
    // cube_marker.header.frame_id = "world";
    // cube_marker.header.stamp = ros::Time::now();
    // cube_marker.type = visualization_msgs::Marker::CUBE;
    // cube_marker.action = visualization_msgs::Marker::ADD;
    // cube_marker.id = 0;
    // cube_marker.color.r = 1;
    // cube_marker.color.g = 1;
    // cube_marker.color.b = 0;
    // cube_marker.color.a = 0.4;
    
    // cube_marker.pose.position.x = position_(0);
    // cube_marker.pose.position.y = position_(1);
    // cube_marker.pose.position.z = position_(2);
    // cube_marker.pose.orientation.w = 1.f;
    // cube_marker.pose.orientation.x = 0.f;
    // cube_marker.pose.orientation.y = 0.f;
    // cube_marker.pose.orientation.z = 0.f;
    
    // cube_marker.scale.x = length_(0);
    // cube_marker.scale.y = length_(1);
    // cube_marker.scale.z = length_(2);

    // cube_marker_pub_.publish(cube_marker);
    
    
}



    /// @brief 更新地图
    /// @param point_cloud_num 降采样后的有效观测点云数量
    /// @param size_of_one_point 一个点的大小
    /// @param point_cloud_ptr 传入的观测点云指针
    /// @param sensor_px 传感器位置
    /// @param sensor_py 传感器位置
    /// @param sensor_pz 传感器位置
    /// @param time_stamp_second 传感器时间戳
    /// @param sensor_quaternion_w 传感器姿态四元数
    /// @param sensor_quaternion_x 传感器姿态四元数
    /// @param sensor_quaternion_y 传感器姿态四元数
    /// @param sensor_quaternion_z 传感器姿态四元数
    /// @return 
int DspMap::update(int point_cloud_num, int size_of_one_point, float *point_cloud_ptr,
            const Position& sensorPosition, double time_stamp_second,
            const Rotation& sensorQuaternion)  /// also requires point cloud and velocity
{
    /** Get delt p **/
    // static float sensor_px_last = sensor_px;
    // static float sensor_py_last = sensor_py;
    // static float sensor_pz_last = sensor_pz;
    // static double time_stamp_second_last = time_stamp_second;

    // Check if the odometry data is invalid
    if(fabs(sensorQuaternion.w()) > 1.001f || fabs(sensorQuaternion.x()) > 1.001f || fabs(sensorQuaternion.y()) > 1.001f || fabs(sensorQuaternion.z()) > 1.001f){
        cout << "Invalid quaternion." <<endl;
        return 0;
    }
    // 传感器位置差
    float odom_delt_px = sensorPosition.x() - lastSensorPosition_.x(); 
    float odom_delt_py = sensorPosition.y() - lastSensorPosition_.y(); 
    float odom_delt_pz = sensorPosition.z() - lastSensorPosition_.z();
    // 传感器时间差
    auto delt_t = (float)(time_stamp_second - last_timestamp_);

    // 偏差过大
    if(fabs(odom_delt_px) > 10.f || fabs(odom_delt_py) > 10.f || fabs(odom_delt_pz) > 10.f || delt_t < 0.f || delt_t > 10.f){
        cout << "!!! delt_t = "<< delt_t <<endl;
        cout << "!!! sensor_px_last = " << lastSensorPosition_.x() << "sensor_px = " << sensorPosition.x() << " odom_delt_px="<<odom_delt_px<<endl;
        cout << "!!! sensor_py_last = " << lastSensorPosition_.y() << "sensor_py = " << sensorPosition.y() << " odom_delt_py="<<odom_delt_py<<endl;
        cout << "!!! sensor_pz_last = " << lastSensorPosition_.z() << "sensor_py = " << sensorPosition.z() << " odom_delt_py="<<odom_delt_pz<<endl;
        return 0;
    }

    // clock_t start11, finish11;
    // start11 = clock();

    // 
    // 当前位置 与 姿态
    currentSensorPosition_ = sensorPosition;
    lastSensorPosition_ = currentSensorPosition_;
    currentSensorRotation_ = sensorQuaternion;
    last_timestamp_ = time_stamp_second;
    delt_t_from_last_observation = delt_t;
    mapMove(currentSensorPosition_);

    /** Update pyramid boundary planes' normal vectors' parameters **/
    // 传感器姿态四元数



    // 旋转金字塔空间的法线向量
    //将sensor坐标系下的金字塔空间法向量转换到map坐标系下，map坐标系和world坐标系相差无人机的距离，姿态不差
    // for(int i=0; i<observation_pyramid_num_h+1; i++){
    //     rotateVectorByQuaternion(&pyramid_BPnorm_params_ori_h[i][0], sensor_rotation_quaternion, &pyramid_BPnorm_params_h[i][0]);
    // }
    // for(int j=0; j<observation_pyramid_num_v+1; j++){
    //     rotateVectorByQuaternion(&pyramid_BPnorm_params_ori_v[j][0], sensor_rotation_quaternion, &pyramid_BPnorm_params_v[j][0]);
    // }

    /** Insert point cloud to observation storage **/
    for(int i=0; i< observation_pyramid_num; i++){  //Initialize point num in the storage
        observation_num_each_pyramid[i] = 0; //Set number of observations in each pyramid as zero in the beginning
        point_cloud_max_length[i] = -1.f;
    }

    cloud_in_current_view_rotated->clear();  // Clear first

    int iter_num = 0; // This is defined because one point has more than one float values. like px, py, pz
    int valid_points = 0; //NOTE: the number of valid points might be different from input points number is the real FOV is larger than the defined FOV. However, if the point is outside of the map but is still in FOV, it will be counted.
    for(int p_seq=0; p_seq<point_cloud_num; ++p_seq)
    {
        Point origin_point,rotated_point;
        for(int i=0;i<3;i++)  origin_point[i] = point_cloud_ptr[iter_num+i];
        //将传感器坐标系下的点云旋转到map坐标系下，map坐标系和world坐标系相差无人机的距离，姿态不差
        // rotateVectorByQuaternion(origin_point, rotate_quaternion, rotated_point); 
        // 将世界坐标系下的点云旋转到传感器坐标系下，
        transformParticleToSensorFrame(rotated_point,origin_point,currentSensorPosition_,currentSensorRotation_);

        // Store in pyramids for update
        if(ifInPyramidsAreaInSensorFrame(rotated_point[0],rotated_point[1],rotated_point[2]))  // 如果在金字塔空间内
        {
            
            // Store in pcl point cloud for velocity estimation of new born particles
            pcl::PointXYZ p_origin;
            p_origin.x = origin_point[0];
            p_origin.y = origin_point[1];
            p_origin.z = origin_point[2];
            cloud_in_current_view_rotated->push_back(p_origin);
            // ROS_INFO("ori_point_this:%f\t%f\t%f",origin_point[0],origin_point[1],origin_point[2]);
            // ROS_INFO("rotated_point_this:%f\t%f\t%f",rotated_point_this[0], rotated_point_this[1], rotated_point_this[2]);
            // ROS_INFO("current_position:%f\t%f\t%f",current_position[0], current_position[1], current_position[2]);

            // 找到对应的金字塔子空间
            int pyramid_index_h, pyramid_index_v;
            pyramid_index_h = findPointPyramidHorizontalIndexInSensorFrame(rotated_point[0], rotated_point[1], rotated_point[2]); 
            // ROS_INFO("pyramid_index_h:%d\t",pyramid_index_h);
            pyramid_index_v = findPointPyramidsVerticalIndexInSensorFrame(rotated_point[0], rotated_point[1], rotated_point[2]);
            // ROS_INFO("pyramid_index_v:%d\t",pyramid_index_v);
            // 找到对应的金字塔子空间下标
            int pyramid_index = pyramid_index_h * observation_pyramid_num_v + pyramid_index_v;
            int  observation_inner_seq = observation_num_each_pyramid[pyramid_index];
            // ROS_INFO("pyramid_index:%d\t",pyramid_index);
            // ROS_INFO("observation_inner_seq:%d\t",observation_inner_seq);

            // 计算出观测点到达坐标系原点的距离
            float length = sqrtf( rotated_point[0]*rotated_point[0] + rotated_point[1]*rotated_point[1] + rotated_point[2]*rotated_point[2]);

            // get point cloud position in global coordinate
            // TODO 
            point_cloud[pyramid_index][observation_inner_seq][0] = rotated_point[0];
            point_cloud[pyramid_index][observation_inner_seq][1] = rotated_point[1];
            point_cloud[pyramid_index][observation_inner_seq][2] = rotated_point[2];
            point_cloud[pyramid_index][observation_inner_seq][3] = 0.f;
            point_cloud[pyramid_index][observation_inner_seq][4] = length;

            // 金字塔子空间的最大长度
            if(point_cloud_max_length[pyramid_index] < length){  // to be used to judge if a particle is occluded
                point_cloud_max_length[pyramid_index] = length;
            }

            observation_num_each_pyramid[pyramid_index] += 1; //这个金字塔子空间的观测点数+1

            // 如果超过了最大粒子数，就要限制住
            // Omit the overflowed observation points. It is suggested to used a voxel filter for the original input point clouds to avoid overflow.
            if(observation_num_each_pyramid[pyramid_index] >= observation_max_points_num_one_pyramid){
                observation_num_each_pyramid[pyramid_index] = observation_max_points_num_one_pyramid - 1;
            }

            ++ valid_points; //有效点+1
        }

        iter_num += size_of_one_point;
    }
    // ROS_INFO("VALID POINT:%d", valid_points);

    expected_new_born_objects = new_born_particle_weight * (float)valid_points * (float)new_born_particle_number_each_point;
    new_born_each_object_weight = new_born_particle_weight * (float)new_born_particle_number_each_point;


    /// Start a new thread for velocity estimation
    std::thread velocity_estimation(velocityEstimationThread);

    /*** Prediction ***/
    // Particles move in the opposite of the robot moving direction
    // clock_t start,finish;
    // start = clock();
    mapPrediction(-odom_delt_px, -odom_delt_py, -odom_delt_pz, delt_t);  
    // ROS_INFO("mapPrediction done!!!!");
    // finish = clock();
    // double duration = (double)(finish - start) / CLOCKS_PER_SEC;
    // ROS_INFO("map Prediction Time:%lf",duration);
    
    /*** Update ***/
    // start = clock();
    if(point_cloud_num >= 0){
        mapUpdate();
    }else{
        cout << "No points to update." <<endl;
    }
    // ROS_INFO("mapUpdate done!!!!");
    // finish = clock();
    // duration = (double)(finish - start) / CLOCKS_PER_SEC;
    // ROS_INFO("map update Time:%lf",duration);

    /** Wait until initial velocity estimation is finished **/
    velocity_estimation.join();

    /** Add updated new born particles ***/
    // start = clock();
    if(point_cloud_num >= 0){
        mapAddNewBornParticlesByObservation();
    }
    // finish = clock();
    // ROS_INFO("mapAddNewBornParticlesByObservation done!!!!");
    // duration = (double)(finish - start) / CLOCKS_PER_SEC;
    // ROS_INFO("mapAddNewBornParticlesByObservation Time:%lf",duration);

    /** Calculate object number and Resample **/

    /// NOTE in this step the flag which is set to be 7.f in prediction step will be changed to 1.f or 0.6f.
    /// Removing this step will make prediction malfunction unless the flag is reset somewhere else.
    // start = clock();
    mapOccupancyCalculationAndResample();
    // finish = clock();
    // ROS_INFO("mapOccupancyCalculationAndResample done!!!!");
    // duration = (double)(finish - start) / CLOCKS_PER_SEC;
    // ROS_INFO("mapOccupancyCalculationAndResample Time:%lf",duration);


    /*** Record particles for analysis  ***/
    static int recorded_once_flag = 0;

    if(if_record_particle_csv){
        if(if_record_particle_csv < 0 || (update_time > record_time && !recorded_once_flag)){
            recorded_once_flag = 1;

            ofstream particle_log_writer;
            string file_name = particle_save_folder + "particles_update_t_" + to_string(update_counter) + "_"+ to_string((int)(update_time*1000)) + ".csv";
            particle_log_writer.open(file_name, ios::out | ios::trunc);

            for(int i=0; i<voxels_total_num; i++){
                for(int j=0; j<SAFE_PARTICLE_NUM_VOXEL; j++){
                    if(voxels_with_particle[i][j][0] > 0.1f){
                        for(int k=0; k<8; k++){
                            //  1.flag 2.vx 3.vy 4.vz 5.px 6.py 7.pz
                            //  8.weight 9.update time
                            particle_log_writer << voxels_with_particle[i][j][k] <<",";
                        }
                        particle_log_writer << i <<"\n";
                    }
                }
            }
            particle_log_writer.close();
        }
    }

    return 1;
}

/// @brief 设置位置预测和速度预测的方差
/// @param p_stddev 传入的位置方差
/// @param v_stddev 传入的速度方差
void DspMap::setPredictionVariance(float p_stddev, float v_stddev){
    position_prediction_stddev = p_stddev;
    velocity_prediction_stddev = v_stddev;
    // regenerate randoms
    generateGaussianRandomsVectorZeroCenter();
}

/// @brief 设置观测方差
/// @param ob_stddev 
void DspMap::setObservationStdDev(float ob_stddev){
    sigma_ob = ob_stddev;
}

/// @brief 设置新产生粒子的权重
void DspMap::setNewBornParticleWeight(float weight){
    new_born_particle_weight = weight;
}


void DspMap::setNewBornParticleNumberofEachPoint(int num){
    new_born_particle_number_each_point = num;
}

/// @brief 设置是否记录粒子
/// record_particle_flag O: don't record; -1 or other negative value: record all; positive value: record a time
void DspMap::setParticleRecordFlag(int record_particle_flag, float record_csv_time = 1.f){
    if_record_particle_csv = record_particle_flag;
    record_time =  record_csv_time;
}

void DspMap::getOccupancyMap(int &obstacles_num, pcl::PointCloud<pcl::PointXYZ> &cloud, const float threshold=0.7){
    obstacles_num = 0;
    int address;
    Index localIndex;
    Position position;
    for(int i=0;i<getMapSize()(0);i++){
        for(int j=0;j<getMapSize()(1);j++){
            for(int k=0;k<getMapSize()(2);k++){
                localIndex = {i,j,k};
                address = getAddress(localIndex);
                if(voxels_objects_number[address][0] > threshold){
                    pcl::PointXYZ pcl_point;
                    getPosition(position,localIndex);
                    pcl_point.x = position.x();
                    pcl_point.y = position.y();
                    pcl_point.z = position.z();
                    
                    cloud.push_back(pcl_point);
                    
                    ++ obstacles_num;             

                }
                /// Clear weights for next prediction
                for(int j=4; j<voxels_objects_number_dimension; ++j)
                {
                    voxels_objects_number[address][j] = 0.f;
                }   
            }
        }
    }
}



/// @brief 获取占据地图并且获得未来占据状态
/// @param obstacles_num 
/// @param cloud 
/// @param future_status 
/// @param threshold 
void DspMap::getOccupancyMapWithFutureStatus(int &obstacles_num, pcl::PointCloud<pcl::PointXYZ> &cloud, float *future_status, const float threshold){
    obstacles_num = 0;
    int address;
    Index localIndex;
    Position position;
    for(int i=0;i<getMapSize()(0);i++){
        for(int j=0;j<getMapSize()(1);j++){
            for(int k=0;k<getMapSize()(2);k++){
                localIndex = {i,j,k};
                address = getAddress(localIndex);
                if(voxels_objects_number[address][0] > threshold){
                    pcl::PointXYZ pcl_point;
                    getPosition(position,localIndex);
                    pcl_point.x = position.x();
                    pcl_point.y = position.y();
                    pcl_point.z = position.z();
                    cloud.push_back(pcl_point);
                    
                    ++ obstacles_num;
                }

                //预测
                for(int n=0; n < PREDICTION_TIMES; ++n){ // Set future weights
                    *(future_status + address * PREDICTION_TIMES + n) = voxels_objects_number[address][n + 4];
                }
                
                // 清除预测
                for(int j=4; j<voxels_objects_number_dimension; ++j)
                {
                    voxels_objects_number[address][j] = 0.f;
                }
            }
        }
    }
}


///NOTE: If you don't want to use any visualization functions like "getOccupancyMap"
///      or "getOccupancyMapWithFutureStatus", you must call this function after the update step.
void DspMap::clearOccupancyMapPrediction(){
    for(int i=0; i<voxels_total_num; i++){
        for(int j=4; j<voxels_objects_number_dimension; ++j)
        {
            voxels_objects_number[i][j] = 0.f;
        }
    }
}

int DspMap::getInflateOccupancy(const Eigen::Vector3d& pos) const
{
    Position posf = pos.cast<float>();
    Index index;
    getIndex(index,posf);
    if(!ifPositionInMap(posf)) return -1;
    int address = getAddress(index);
    if(voxels_objects_number[address][0] > 0.3) return 1;
    else return 0;
    // int res = voxels_objects_number[address][0] > 0.0 ? 1 : 0;
    // ROS_INFO("%f\t%f\t%f\t%f",posf[0],posf[1],posf[2],res);
}
float DspMap::getResolution() const 
{
    return getVoxelResolution();
}


/// Get clustered result for visualization
void DspMap::getKMClusterResult(pcl::PointCloud<pcl::PointXYZINormal> &cluster_cloud){
    for(auto &point : *input_cloud_with_velocity){
        cluster_cloud.push_back(point);
    }
}

///@brief 设置体素滤波器的分辨率
void DspMap::setOriginalVoxelFilterResolution(float res){
    voxel_filtered_resolution = res;
}

void DspMap::setInitParameters(){

}

/// @brief 添加随机粒子
/// @param particle_num 粒子数
/// @param avg_weight 平均权重
void DspMap::addRandomParticles(int particle_num, float avg_weight)
{
    /*** Initialize some particles ***/
    int successfully_added_num = 0; //是否成功添加的标志位
    int voxel_overflow_num = 0; //

    for(int i=0; i<particle_num; i++) {
        std::shared_ptr<Particle> particle_ptr{new Particle};

        // 在地图内生成随机粒子
        particle_ptr->position.x() = generateRandomFloat(mapMinBoundary_(0), mapMaxBoundary_(0));
        particle_ptr->position.y() = generateRandomFloat(mapMinBoundary_(1), mapMaxBoundary_(1));
        particle_ptr->position.z() = generateRandomFloat(mapMinBoundary_(2), mapMaxBoundary_(2));
        particle_ptr->velocity.x() = generateRandomFloat(-1.f, 1.f);
        particle_ptr->velocity.y() = generateRandomFloat(-1.f, 1.f);
        particle_ptr->velocity.z() = generateRandomFloat(-1.f, 1.f);
        particle_ptr->weight = avg_weight;
        // ROS_INFO("RANDOM PARTICLE:%f,%f,%f",particle_ptr->px,particle_ptr->py,particle_ptr->pz);
        if (getParticleAddress(*particle_ptr, particle_ptr->voxel_index)) { //如果粒子不在地图外，则返回粒子所在体素的下标
            bool test = addAParticle(*particle_ptr, particle_ptr->voxel_index); //粒子加入到内存中
            if(test){
                successfully_added_num ++;
            }else{
                voxel_overflow_num ++;
            }
        }
    }

}

/// @brief 
/// @param odom_delt_px 
/// @param odom_delt_py 
/// @param odom_delt_pz 
/// @param delt_t 
void DspMap::mapPrediction(float odom_delt_px, float odom_delt_py, float odom_delt_pz, float delt_t)
{
    int operation_counter = 0; //操作次数计数
    int exist_particles = 0; //
    int voxel_full_remove_counter = 0, pyramid_full_remove_counter = 0; // 
    int moves_out_counter = 0; // 

    update_time += delt_t; //
    update_counter += 1; // 

    /// Clear pyramids first
    for(auto & j : pyramids_in_fov){ // 遍历每一个金字塔空间，将其观测点清零
        for(auto & i : j){
            i[0] &= O_MAKE_INVALID; 
        }
    }
    /// Update Particles' state and index in both voxels and pyramids
    for(int v_index=0; v_index<VOXEL_NUM; ++v_index) //遍历所有体素子空间
    {
        for(int p=0; p<SAFE_PARTICLE_NUM_VOXEL; p++) // 遍历体素子空间中的所有粒子
        {
            // 如果存在，并且不是新move过来的点
            // ROS_INFO("is valid: %f",voxels_with_particle[v_index][p][0]);
            if(voxels_with_particle[v_index][p][0] >0.1f && voxels_with_particle[v_index][p][0] <6.f){  /// exsit, but not new moved
                // ROS_INFO("here");
                voxels_with_particle[v_index][p][0] = 1.f; // If valid, remove resample flag.重采样的是0.6f，这里一视同仁，当作存在点
                ++ operation_counter; //操作计数

                if(fabs(voxels_with_particle[v_index][p][1]*voxels_with_particle[v_index][p][2]*voxels_with_particle[v_index][p][3]) < 1e-6){
                    // keep small, for static obstacles
                }else{
                    voxels_with_particle[v_index][p][1] += getVelocityGaussianZeroCenter();  //vx，加上高斯随机噪声
                    voxels_with_particle[v_index][p][2] += getVelocityGaussianZeroCenter();  //vy，加上高斯随机噪声
                    voxels_with_particle[v_index][p][3] += getVelocityGaussianZeroCenter();  //vz，加上高斯随机噪声
                }

#if(LIMIT_MOVEMENT_IN_XY_PLANE)
                voxels_with_particle[v_index][p][3] = 0.f;
#endif
                ///alg.15
                // 传入的odom位移是负的，也就是粒子始终在以机器人为中心的local map中移动，出去了就没了
                // 粒子相对于机器人的局部坐标
                // voxels_with_particle[v_index][p][4] += delt_t*voxels_with_particle[v_index][p][1] + odom_delt_px;  //px
                // voxels_with_particle[v_index][p][5] += delt_t*voxels_with_particle[v_index][p][2] + odom_delt_py;  //py
                // voxels_with_particle[v_index][p][6] += delt_t*voxels_with_particle[v_index][p][3] + odom_delt_pz;  //pz
                // 现在改成粒子的全局坐标
                voxels_with_particle[v_index][p][4] += delt_t*voxels_with_particle[v_index][p][1];  //px
                voxels_with_particle[v_index][p][5] += delt_t*voxels_with_particle[v_index][p][2];  //py
                voxels_with_particle[v_index][p][6] += delt_t*voxels_with_particle[v_index][p][3];  //pz

                // cout << "pos-x: " << voxels_with_particle[v_index][p][4]  << "pos-y: " << voxels_with_particle[v_index][p][5] << "pos-z: " << voxels_with_particle[v_index][p][5]  << endl;

                // ROS_INFO("delt_t: %lf",delt_t);
                // ROS_INFO("originParticle x:%f\ty:%f\tz:%f",voxels_with_particle[v_index][p][4],voxels_with_particle[v_index][p][5],voxels_with_particle[v_index][p][6]);

                ///上面就是计算在这一帧的传感器时刻下，粒子相对于机器人的位置。

                int particle_voxel_index_new; // 粒子在某个体素空间中的新下标

                if(getParticleAddress(voxels_with_particle[v_index][p][4], voxels_with_particle[v_index][p][5],
                                            voxels_with_particle[v_index][p][6], particle_voxel_index_new)) //如果移动的粒子在边界内
                {
                    // cout << "particle_voxel_index_new: " << particle_voxel_index_new << endl;
                    // cout << "pos-x: " << voxels_with_particle[v_index][p][4]  << "pos-y: " << voxels_with_particle[v_index][p][5] << "pos-z: " << voxels_with_particle[v_index][p][5]  << endl;

                    // move particle. If moved, the flag turns to 7.f. If should move but failed because target voxel is full, delete the voxel.
                    int move_flag = moveParticle(particle_voxel_index_new, v_index, p, &voxels_with_particle[v_index][p][0]);
                    if(move_flag == -2){ 
                        // Move the particle, if fails, "moveParticleByVoxel" will delete the particle
                        ++ pyramid_full_remove_counter; //因为金字塔子空间的内存满所删除的粒子数
                        continue;
                    }else if(move_flag == -1){ //因为体素子空间的内存满所删除的粒子数
                        ++ voxel_full_remove_counter;
                        continue;
                    }
                    ++ exist_particles; // 移动成功，存在粒子数++

                }
                else{
                    /// Particle moves out
                    removeParticle(&voxels_with_particle[v_index][p][0]);
                    ++ moves_out_counter;
                }

            }
        }
    }
    // cout << "exist_particles: " << exist_particles << endl; 

    if(moves_out_counter > 10000){ //应该是指粒子更新时间间隔太长了，
        cout <<"!!!!! An error occured! delt_t = " << delt_t <<endl;
        cout << "odom_delt_px = " << odom_delt_px <<" odom_delt_py = " << odom_delt_py << "odom_delt_pz=" << odom_delt_pz<< endl;
    }

}


void DspMap::mapUpdate()
{
    int operation_counter_update = 0;

    /// Calculate Ck + kappa firstobservation_pyramid_num
    for(int i=0; i<observation_pyramid_num; ++i){ // 金字塔空间数量
        for(int j=0; j<observation_num_each_pyramid[i]; ++j){ //每个金字塔空间中容纳的观测点数量
            // Iteration of z
            for(int n_seq=0; n_seq<observation_pyramid_neighbors[i][0]; ++n_seq){ //n_seq小于邻居数
                int pyramid_check_index = observation_pyramid_neighbors[i][n_seq+1]; //0.neighbors num 1-9:neighbor indexes
                for(int particle_seq=0; particle_seq<SAFE_PARTICLE_NUM_PYRAMID; ++particle_seq){ // 遍历这个金字塔子空间的所有粒子
                    // ROS_INFO("ASDFASDFASDFDSA");
                    if(pyramids_in_fov[pyramid_check_index][particle_seq][0] & O_MAKE_VALID){  //Check only valid particles
                        int particle_voxel_index = pyramids_in_fov[pyramid_check_index][particle_seq][1];
                        int particle_voxel_inner_index = pyramids_in_fov[pyramid_check_index][particle_seq][2];
//                            cout<<"pyramid_check_index="<<pyramid_check_index<<", particle_v_inner_index="<<particle_v_inner_index<<", point_cloud[i][j][0]="<<point_cloud[i][j][0]<<endl;
                        // alg.27
                        // ROS_INFO("HERE");
                        float gk = queryNormalPDF(
                                voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][4],
                                point_cloud[i][j][0], sigma_ob)
                                    * queryNormalPDF(
                                voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][5],
                                point_cloud[i][j][1], sigma_ob)
                                    * queryNormalPDF(
                                voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][6],
                                point_cloud[i][j][2], sigma_ob);
                        //alg.25 计算Ck
                        point_cloud[i][j][3] += P_detection * voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][7] * gk;
                    }
                }
            }
            /// add weight for new born particles
            /// kappa 应该是那个kk
            point_cloud[i][j][3] += (expected_new_born_objects + kappa);
        }
    }


    /// Update weight for each particle in view
    for(int i=0; i < observation_pyramid_num; i++) // 遍历所有金字塔子空间
    {
        int current_pyramid_index = i;
        for(int inner_seq=0; inner_seq<SAFE_PARTICLE_NUM_PYRAMID; inner_seq++){ // 遍历子空间中的粒子
            // Iteration of particles
            if(pyramids_in_fov[current_pyramid_index][inner_seq][0] & O_MAKE_VALID){ //update only valid particle

                int neighbor_num = observation_pyramid_neighbors[current_pyramid_index][0];

                int particle_voxel_index = pyramids_in_fov[current_pyramid_index][inner_seq][1];
                int particle_voxel_inner_index = pyramids_in_fov[current_pyramid_index][inner_seq][2];

                float px_this = voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][4] - currentSensorPosition_.x();
                float py_this = voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][5] - currentSensorPosition_.y();
                float pz_this = voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][6] - currentSensorPosition_.z();
                float particle_dist_length = sqrtf(px_this*px_this + py_this*py_this + pz_this*pz_this);

                // Update only particles that are not occluded, use voxel_resolution as the distance metric.
                // point_cloud_max_length[i] > 0.f 表示这个金字塔空间中有粒子
                // particle_dist_length > point_cloud_max_length[i] + voxel_resolution 当前粒子在点云观测点之后，意味着被遮挡了
                if(point_cloud_max_length[i] > 0.f && particle_dist_length > point_cloud_max_length[i] + voxelResolution_) 
                {
                    // ROS_INFO("HERE2 ");
                    // occluded
                    continue;
                }

                //如果这个粒子没有被遮挡
                ///alg.35
                float sum_by_zk = 0.f;
                for(int neighbor_seq=0; neighbor_seq<neighbor_num; ++neighbor_seq) //遍历邻居
                {
                    int neighbor_index = observation_pyramid_neighbors[current_pyramid_index][neighbor_seq+1]; // 
                    
                    for(int z_seq=0; z_seq<observation_num_each_pyramid[neighbor_index]; ++z_seq) //for all observation points in a neighbor pyramid
                    {
                        float gk = queryNormalPDF(px_this, point_cloud[neighbor_index][z_seq][0], sigma_ob)
                                    * queryNormalPDF(py_this, point_cloud[neighbor_index][z_seq][1], sigma_ob)
                                    * queryNormalPDF(pz_this, point_cloud[neighbor_index][z_seq][2], sigma_ob);

                        sum_by_zk += P_detection * gk / point_cloud[neighbor_index][z_seq][3];
                        ++operation_counter_update;
                    }


                }

                voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][7] *= ((1 - P_detection) + sum_by_zk);
                voxels_with_particle[particle_voxel_index][particle_voxel_inner_index][8] = update_time;
            }
        }
    }
    //    cout << "operation_counter_update=" << operation_counter_update <<endl;

}


/// @brief 基于观测去添加新粒子
void DspMap::mapAddNewBornParticlesByObservation()
{
    /** Calculate normalization coefficient first **/
    float normalization_coefficient = 0.f;
    for(int i=0; i< observation_pyramid_num; i++){
        for(int j=0; j< observation_num_each_pyramid[i]; j++){
            normalization_coefficient += 1.f / point_cloud[i][j][3];
        }
    }
    float updated_weight_new_born = new_born_particle_weight * normalization_coefficient;

    /** Add new born particles **/
    static int min_static_new_born_particle_number_each_point = (int)((float)new_born_particle_number_each_point * 0.15f);
    static int static_new_born_particle_number_each_point = (int)((float)new_born_particle_number_each_point * 0.4f);  // static points takes 3 in 10
    static int pf_derive_new_born_particle_number_each_point = (int)((float)new_born_particle_number_each_point * 0.5f); // Derived takes half
    static const int model_generated_particle_number_each_point = (int)((float)new_born_particle_number_each_point * 0.8f);

    int successfully_born_particles = 0;
    /// TODO: Improve efficiency in this new born procedure
    for(auto & point : *input_cloud_with_velocity) // 全局位置的观测点，我们要改变
    {
        // 全局位置变成了局部位置
        pcl::PointXYZ p_corrected;
        // p_corrected.x = point.x - current_position[0];
        // p_corrected.y = point.y - current_position[1];
        // p_corrected.z = point.z - current_position[2];
        p_corrected.x = point.x;
        p_corrected.y = point.y;
        p_corrected.z = point.z;

        int point_voxel_index;
        float static_particle_weight_sum = 0.f;
        float dynamic_particle_weight_sum = 0.f;
        float static_or_dynamic_weight_sum = 0.f;

        if(getParticleAddress(p_corrected.x, p_corrected.y, p_corrected.z, point_voxel_index)){
            // ROS_INFO("HERE");
            //This condition should always be true because the point cloud outside of the map should be omitted in the first place. Just an insurance.
            for(int kk=0; kk<SAFE_PARTICLE_NUM_VOXEL; ++kk){
                if(voxels_with_particle[point_voxel_index][kk][0] > 0.9f && voxels_with_particle[point_voxel_index][kk][0] < 14.f){ //not new born
                    // ROS_INFO("HRERE");
                    float v_abs = fabs(voxels_with_particle[point_voxel_index][kk][1]) + fabs(voxels_with_particle[point_voxel_index][kk][2]) + fabs(voxels_with_particle[point_voxel_index][kk][3]);
                    if(v_abs < 0.1f){
                        // Static
                        static_particle_weight_sum += voxels_with_particle[point_voxel_index][kk][7];
                    }else if(v_abs < 0.5f){
                        // Static or dynamic
                        static_or_dynamic_weight_sum += voxels_with_particle[point_voxel_index][kk][7];
                    }
                    else{
                        //Dynamic
                        dynamic_particle_weight_sum += voxels_with_particle[point_voxel_index][kk][7];
                    }
                }
            }
        }
        else{
            continue;
        }

        // Dempster-Shafer Theory
        /* alg.43-46*/
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

        // Set a minimum number of static particles
        static_new_born_particle_number_each_point = max(min_static_new_born_particle_number_each_point, static_new_born_particle_number_each_point);

        for(int p=0; p<new_born_particle_number_each_point; p++){
            std::shared_ptr<Particle> particle_ptr{new Particle};

            // particle_ptr->position.x() = p_corrected.x + getPositionGaussianZeroCenter();
            // particle_ptr->position.y() = p_corrected.y + getPositionGaussianZeroCenter();
            // particle_ptr->position.z() = p_corrected.z + getPositionGaussianZeroCenter();
            particle_ptr->position.x() = p_corrected.x + getPositionGaussianZeroCenter();
            particle_ptr->position.y() = p_corrected.y + getPositionGaussianZeroCenter();
            particle_ptr->position.z() = p_corrected.z + getPositionGaussianZeroCenter();

            if (getParticleAddress(*particle_ptr, particle_ptr->voxel_index)) {
                // Particle index might be different from the point index because a random Gaussian is added.
                if(p < static_new_born_particle_number_each_point){  // add static points
                    particle_ptr->velocity.x() = 0.f;
                    particle_ptr->velocity.y() = 0.f;
                    particle_ptr->velocity.z() = 0.f;
                }else if(point.normal_x > -100.f && p < model_generated_particle_number_each_point){ //p < pf_derive_new_born_particle_number_each_point + static_new_born_particle_number_each_point){
                    /// Use estimated velocity to generate new particles
                    if(point.intensity > 0.01f){
                        particle_ptr->velocity.x() = point.normal_x + 4*getVelocityGaussianZeroCenter();
                        particle_ptr->velocity.y() = point.normal_y + 4*getVelocityGaussianZeroCenter();
                        particle_ptr->velocity.z() = point.normal_z + 4*getVelocityGaussianZeroCenter();
                    }else{ //static points like ground
                        particle_ptr->velocity.x() = 0.f;
                        particle_ptr->velocity.y() = 0.f;
                        particle_ptr->velocity.z() = 0.f;
                    }
                }
                else{ /// Considering Random Noise
                    if(point.intensity > 0.01f){
                        particle_ptr->velocity.x() = generateRandomFloat(-1.5f, 1.5f);
                        particle_ptr->velocity.y() = generateRandomFloat(-1.5f, 1.5f);
                        particle_ptr->velocity.z() = generateRandomFloat(-0.5f, 0.5f);
                    }else{ //static points like ground
                        particle_ptr->velocity.x() = 0.f;
                        particle_ptr->velocity.y() = 0.f;
                        particle_ptr->velocity.z() = 0.f;
                    }
                }

#if(LIMIT_MOVEMENT_IN_XY_PLANE)
                particle_ptr->velocity.z() = 0.f;
#endif
                // cout << "point x: " << point.normal_x << "point vy: " << point.normal_y << "point vz: " << point.normal_z << endl;
                // cout << "particle vx: " << particle_ptr->vx <<  "particle vy: " <<particle_ptr->vy <<"particle vz: " <<particle_ptr->vz << endl;

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



void DspMap::mapOccupancyCalculationAndResample()
{
    int removed_particle_counter = 0;
    int particle_num_after_resampling_should_be = 0;

    // 占据概率计算和未来状态估计
    for(int v_index=0; v_index<VOXEL_NUM; ++v_index)
    {
        // Calculate estimated object number in each voxel
        static float weight_sum_voxel, vx_sum_voxel, vy_sum_voxel, vz_sum_voxel; // 1.体素中权重和；2.体素中各轴速度
        weight_sum_voxel = 0.f; // 体素中权重和
        vx_sum_voxel = vy_sum_voxel = vz_sum_voxel = 0.f; // 体素中各轴速度

        int particle_num_voxel = 0; //体素中粒子数
        int old_particle_num_voxel = 0; //体素中之前的粒子数
        for(int p=0; p<SAFE_PARTICLE_NUM_VOXEL; p++) //遍历体素中所有粒子
        {
            if(voxels_with_particle[v_index][p][0] > 0.1f){ // 不是INVALID 粒子
                if(voxels_with_particle[v_index][p][7] < 1e-3){ // 权重很小，几乎为0
                    voxels_with_particle[v_index][p][0] = 0.f;  // Remove the particle directly if the weight is too small
                }else{ //如果有一定权重
                    // ROS_INFO("HEWRWERWE");

                    if(voxels_with_particle[v_index][p][0] < 10.f){  //exclude new-born particles 排除新生粒子
                        ++old_particle_num_voxel;
                        vx_sum_voxel += voxels_with_particle[v_index][p][1]; //体素x轴的速度
                        vy_sum_voxel += voxels_with_particle[v_index][p][2]; //体素y轴的速度
                        vz_sum_voxel += voxels_with_particle[v_index][p][3]; //体素z轴的速度

                        /*** Future status prediction ***/
                        float px_future, py_future, pz_future; 
                        for(int times = 0; times < PREDICTION_TIMES; ++times)
                        {
                            float prediction_time = prediction_future_time[times]; // 
                            px_future = voxels_with_particle[v_index][p][4] + voxels_with_particle[v_index][p][1] * prediction_time; //粒子未来x轴位置
                            py_future = voxels_with_particle[v_index][p][5] + voxels_with_particle[v_index][p][2] * prediction_time; //粒子未来y轴位置
                            pz_future = voxels_with_particle[v_index][p][6] + voxels_with_particle[v_index][p][3] * prediction_time; //粒子未来z轴位置

                            int prediction_index;  //预测的体素下标
                            if(getParticleAddress(px_future, py_future, pz_future, prediction_index)) { //在地图内
                                // 未来6个预测时刻的占据概率
                                // prediction index表示了未来位置对应的体素
                                // 则对应体素的未来状态加上该粒子权重
                                voxels_objects_number[prediction_index][4+times] += voxels_with_particle[v_index][p][7]; //weight
                            }
                        }
                        /**** End of prediction ****/

                    }
                    // 这个粒子是新生粒子，因此将其不再标记为新生粒子
                    voxels_with_particle[v_index][p][0] = 1.f; // Remove newborn flag and moved flag in prediction
                    ++particle_num_voxel; // 体素中增加一个粒子
                    weight_sum_voxel += voxels_with_particle[v_index][p][7]; //体素中总的weight加上当前粒子的weight
                }
            }
        }
        voxels_objects_number[v_index][0] = weight_sum_voxel; // 将当前体素的权重和传到voxels_objects_number中对应的位置

        if(old_particle_num_voxel > 0){// 1. objects number; 2-4. Avg vx, vy, vz; 5-. Future objects number
            // 如果体素中有旧的粒子，则更新体素对象数组中的对应体素速度
            voxels_objects_number[v_index][1] = vx_sum_voxel / (float)old_particle_num_voxel; //速度取平均
            voxels_objects_number[v_index][2] = vy_sum_voxel / (float)old_particle_num_voxel; //速度取平均
            voxels_objects_number[v_index][3] = vz_sum_voxel / (float)old_particle_num_voxel; //速度取平均
        }else{
            /// 没有粒子，则速度是0
            voxels_objects_number[v_index][1] = 0.f; 
            voxels_objects_number[v_index][2] = 0.f;
            voxels_objects_number[v_index][3] = 0.f;
        }

        if(particle_num_voxel < 5){  //Too few particles, no need to resample.
            particle_num_after_resampling_should_be += particle_num_voxel; //for test
            continue;
        }

        // Resampling
        // Calculate desired particle number after resampling
        //alg.40
        int particle_num_voxel_after_resample;
        if(particle_num_voxel > MAX_PARTICLE_NUM_VOXEL){ // 如果体素内粒子数量超了，则拒绝一些采样
            particle_num_voxel_after_resample = MAX_PARTICLE_NUM_VOXEL; 
        }else{ //反之则维持现状。
            particle_num_voxel_after_resample = particle_num_voxel;
        }

        static float weight_after_resample; //重采样后的体素权重
        weight_after_resample = weight_sum_voxel / (float)particle_num_voxel_after_resample; 

        // 这个体素内，经过重采样后，的粒子数 加到整个地图所有体素的粒子计数器中
        particle_num_after_resampling_should_be += particle_num_voxel_after_resample;

        // Resample
        float acc_ori_weight = 0.f; //原始权重
        float acc_new_weight = weight_after_resample * 0.5f; //新权重
        for(int p=0; p<SAFE_PARTICLE_NUM_VOXEL; ++p) //体素中所有粒子
        {
            if(voxels_with_particle[v_index][p][0] > 0.7f){ //exclude invalid and newly_added_by_resampling particles
                float ori_particle_weight = voxels_with_particle[v_index][p][7]; //原始的粒子权重
                acc_ori_weight += ori_particle_weight; //体素原始权重 + 单个粒子的权重
                if(acc_ori_weight > acc_new_weight){ //如果体素原来的权重大于体素新的权重
                    voxels_with_particle[v_index][p][7] = weight_after_resample; // keep the particle but change weight
                    acc_new_weight += weight_after_resample; // 

                    // 标志体素空间粒子是否存储满了
                    int if_space_is_currently_full = 0;
                    /** copy particles that have a very large weight **/
                    int p_i=0;
                    //如果加了权重，acc_ori_weight还是大，说明这个权重很大，比平均权重weight_after_resample还要大
                    while(acc_ori_weight > acc_new_weight){ // copy the particle if the original weight is very large
                        int if_found_position_in_voxel = 0; // 标志在体素中找到了空闲位置
                        if(!if_space_is_currently_full){ //如果当前体素现在不是满的
                            for( ; p_i<SAFE_PARTICLE_NUM_VOXEL; ++p_i){ // 遍历体素中空闲的粒子存放区
                                // 把这个点放在合理的体素空间上
                                if(voxels_with_particle[v_index][p_i][0] < 0.1f){ // find an empty position in voxel
                                    // Now copy the particle
                                    voxels_with_particle[v_index][p_i][0] = 0.6f; // Flag: newly_added_by_resampling 
                                    for(int k=1; k<9; k++){
                                        voxels_with_particle[v_index][p_i][k] = voxels_with_particle[v_index][p][k]; //把粒子属性放进去
                                    }
                                    if_found_position_in_voxel = 1; //标志在体素中找到了空闲区域，复制成功
                                    break;
                                }
                            }
                        }

                        if(!if_found_position_in_voxel){ // 如果没有找到空闲区域
                            // 如果没有空闲区域，就把权重加到区域内最后的一个粒子上
                            // If the particle should be copied but no space left in either voxel or pyramid, add the weight of the original particle to keep the total weight unchanged.
                            voxels_with_particle[v_index][p][7] += weight_after_resample;  //
                            if_space_is_currently_full = 1; // 标记满了
                        }

                        acc_new_weight += weight_after_resample;
                    }

                }else{ //如果原来的权重小于新的权重
                    // Remove the particle
                    voxels_with_particle[v_index][p][0] = 0.f; //删除这个粒子
                    removed_particle_counter ++; //计数：删除了多少个粒子
                }
            }

        }

    }

}

bool DspMap::mapMove(const Position& position)
{
    bool moved = false;
    int distanceThreshold = 2.5;
    Index indexShift = {0,0,0};
    Position positionShift = position - mapPosition_;
    // ROS_INFO("position:\t%f\t%f\t%f",position[0],position[1],position[2]);
    // ROS_INFO("position_:\t%f\t%f\t%f",position_[0],position_[1],position_[2]);
    // ROS_INFO("positionShift:\t%f\t%f\t%f",positionShift[0],positionShift[1],positionShift[2]);
    // ROS_INFO("positionShiftNorm:%f",positionShift.norm());

    if(positionShift.norm() > distanceThreshold){
        

        getIndexShiftFromPositionShift(indexShift,positionShift,voxelResolution_);
        Position alignedPositionShift;
        getPositionShiftFromIndexShift(alignedPositionShift,indexShift,voxelResolution_);
        for(int i=0;i<indexShift.size();i++){
            if(abs(indexShift(i) >= getMapSize()(i))){
                clearAll();
            }else{
                int startIndex1d = mapStartIndex_(i);
                int indexShift1d = indexShift(i);
                if(indexShift1d != 0){
                    if(i == 0) clearXAxis(startIndex1d,indexShift1d);
                    else if(i == 1) clearYAxis(startIndex1d,indexShift1d);
                    else if(i == 2) clearZAxis(startIndex1d,indexShift1d);
                }

            }
        }
        mapStartIndex_ += indexShift;
        wrapIndexToRange(mapStartIndex_,getMapSize());
        // ROS_INFO("last_position_: \t%f\t%f\t%f",last_position_);
        mapPosition_ += alignedPositionShift;
        mapMinBoundary_ = mapPosition_ - mapLength_;
        mapMaxBoundary_ = mapPosition_ + mapLength_;
        // moved = indexShift.any();
        moved = true;
    }

    return moved;
    
}

bool DspMap::getParticleAddress(const Particle& particle, int& address)
{
    if(!ifPositionInMap(particle.position)) {return false;}
    
    address = getAddress(particle.position);
    // ROS_INFO("partilce address: %d",address);
    if(address < 0 || address >= VOXEL_NUM){
        return false;
    }
    return true;
}

bool DspMap::getParticleAddress(const float& x, const float& y, const float& z,int& address)
{
    Position pos = {x,y,z};
    // cout << "pos-x: " << pos[0] << "pos-y: " << pos[1] << "pos-z: " << pos[2] << endl;
    if(!ifPositionInMap(pos)){
        return false;
    }
    address = getAddress(pos);
    // ROS_INFO("partilce address: %d",address);

    if(address < 0 || address >= VOXEL_NUM){
        return false;
    }
    return true; 
}

bool DspMap::ifPositionInMap(const Position& point) const
{
    return checkIfPositionWithinMap(point,getMapLength(),getMapPosition());
}
bool DspMap::ifPositionInMap(const float& x,const float& y,const float& z) const
{
    Position point = {x,y,z};
    bool checkin = checkIfPositionWithinMap(point,getMapLength(),getMapPosition());
    // cout << "checkin" << checkin << endl;
    return checkin;
}


bool DspMap::ifParticleInMap(const Particle &p) const
{
    return checkIfPositionWithinMap(p.position,getMapLength(),getMapPosition());
}

const Size& DspMap::getMapSize() const
{
    return mapSize_;
}

const Length& DspMap::getMapLength() const
{
    return mapLength_;
}

const Position& DspMap::getMapPosition() const
{
    return mapPosition_;
}
const float DspMap::getVoxelResolution() const
{
    return voxelResolution_;
}

const int DspMap::getAngleResolution() const
{
    return angleResolution_;
}
void DspMap::setMapStartIndex(const Index& startIndex)
{
    mapStartIndex_ = startIndex;
}
Index DspMap::getMapStartIndex() const
{
    return mapStartIndex_;
}

bool DspMap::getIndex(Index& index, const Position& position) const
{
    return getIndexFromPosition(index,position,getMapLength(),getMapPosition(),voxelResolution_,getMapSize(),getMapStartIndex());
}

bool DspMap::getPosition(Position& position,const Index& index) const
{
    return getPositionFromIndex(position,index,getMapLength(),getMapPosition(),voxelResolution_,getMapSize(),getMapStartIndex());
}



int DspMap::getAddress(const Position& position) const
{
    Index bufferIndex;
    getIndex(bufferIndex,position);
    return getAddressFromBufferIndex(bufferIndex,getMapSize());

}

int DspMap::getAddress(const Index& index) const
{
    return getAddressFromBufferIndex(index,mapSize_);
}


void DspMap::clearAll()
{
    for(int i=0;i<voxels_total_num;i++){
        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
            voxels_with_particle[i][h][0] = 0.f;
        }
        // voxels_objects_number[i][10] = 0.f;
    }
}



void DspMap::clearXAxis(const int startIndex,const int indexShift)
{
    int endIndex = startIndex + indexShift;
    Index addressIndex;
    int address;
    if(indexShift > 0){ // 正向偏移
        if(endIndex >= getMapSize()(0)){
            for(int i=startIndex;i<getMapSize()(0);i++){
                for(int j=0;j<getMapSize()(1);j++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
            wrapIndexToRange(endIndex,getMapSize()(0));
            for(int i=0;i<endIndex;i++){
                for(int j=0;j<getMapSize()(1);j++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }     
            }
        }
        else if(endIndex < getMapSize()(0)){
            for(int i=startIndex;i<endIndex;i++){
                for(int j=0;j<getMapSize()(1);j++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
        }
    }
    else if(indexShift < 0){
        if(endIndex < 0){
            for(int i=0;i<=startIndex;i++){
                for(int j=0;j<getMapSize()(1);j++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
            wrapIndexToRange(endIndex,getMapSize()(0));
            for(int i=endIndex+1;i<getMapSize()(0);i++){
                for(int j=0;j<getMapSize()(1);j++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }     
            }   
        }
        else if(endIndex >= 0){
            for(int i=endIndex+1;i<=startIndex;i++){
                for(int j=0;j<getMapSize()(1);j++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
        }
    }
}
void DspMap::clearYAxis(const int startIndex,const int indexShift)
{
    int endIndex = startIndex + indexShift;
    Index addressIndex;
    int address;
    if(indexShift > 0){ // 正向偏移
        if(endIndex >= getMapSize()(1)){
            for(int j=startIndex;j<getMapSize()(1);j++){
                for(int i=0;i<getMapSize()(0);i++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
            wrapIndexToRange(endIndex,getMapSize()(1));
            for(int j=0;j<endIndex;j++){
                for(int i=0;i<getMapSize()(0);i++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }     
            }
        }
        else if(endIndex < getMapSize()(1)){
            for(int j=startIndex;j<endIndex;j++){
                for(int i=0;i<getMapSize()(0);i++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
        }
    }
    else if(indexShift < 0){
        if(endIndex < 0){
            for(int j=0;j<=startIndex;j++){
                for(int i=0;i<getMapSize()(0);i++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
            wrapIndexToRange(endIndex,getMapSize()(1));
            for(int j=endIndex+1;j<getMapSize()(0);j++){
                for(int i=0;i<getMapSize()(0);i++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }     
            }   
        }
        else if(endIndex >= 0){
            for(int j=endIndex+1;j<=startIndex;j++){
                for(int i=0;i<getMapSize()(0);i++){
                    for(int k=0;k<getMapSize()(2);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
        }
    }
}
void DspMap::clearZAxis(const int startIndex,const int indexShift)
{
    int endIndex = startIndex + indexShift;
    Index addressIndex;
    int address;
    if(indexShift > 0){ // 正向偏移
        if(endIndex >= getMapSize()(2)){
            for(int k=startIndex;k<getMapSize()(2);k++){
                for(int i=0;i<getMapSize()(0);i++){
                    for(int j=0;j<getMapSize()(1);j++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
            wrapIndexToRange(endIndex,getMapSize()(2));
            for(int k=0;k<endIndex;k++){
                for(int i=0;i<getMapSize()(0);i++){
                    for(int j=0;j<getMapSize()(1);j++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }     
            }
        }
        else if(endIndex < getMapSize()(2)){
            for(int k=startIndex;k<endIndex;k++){
                for(int i=0;i<getMapSize()(0);i++){
                    for(int j=0;j<getMapSize()(1);j++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
        }
    }
    else if(indexShift < 0){
        if(endIndex < 0){
            for(int k=0;k<=startIndex;k++){
                for(int i=0;i<getMapSize()(0);i++){
                    for(int j=0;j<getMapSize()(1);j++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
            wrapIndexToRange(endIndex,getMapSize()(2));
            for(int k=endIndex+1;k<getMapSize()(0);k++){
                for(int i=0;i<getMapSize()(0);i++){
                    for(int j=0;j<getMapSize()(1);j++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }     
            }   
        }
        else if(endIndex >= 0){
            for(int i=endIndex+1;i<=startIndex;i++){
                for(int j=0;j<getMapSize()(0);j++){
                    for(int k=0;k<getMapSize()(1);k++){
                        addressIndex = {i,j,k};
                        address = getAddressFromBufferIndex(addressIndex,getMapSize());
                        for(int h=0;h<SAFE_PARTICLE_NUM_VOXEL;h++){
                            voxels_with_particle[address][h][0] = 0.f;
                        }
                    }
                }                
            }
        }
    }
}



/// @brief 生成0均值的高斯随机向量，生成了位置预测方差和速度预测方差
void DspMap::generateGaussianRandomsVectorZeroCenter() const{
    std::default_random_engine random(time(NULL));
    std::normal_distribution<double> n1(0, position_prediction_stddev);
    std::normal_distribution<double> n2(0, velocity_prediction_stddev);

    for(int i=0; i<GAUSSIAN_RANDOMS_NUM; i++){
        *(p_gaussian_randoms+i) = n1(random);
        *(v_gaussian_randoms+i) = n2(random);
    }

}


/// @brief 获取0均值的高斯随机向量:位置
/// @return 
float DspMap::getPositionGaussianZeroCenter(){
    float delt_p = p_gaussian_randoms[position_gaussian_random_seq];
    position_gaussian_random_seq += 1;
    if(position_gaussian_random_seq >= GAUSSIAN_RANDOMS_NUM){
        position_gaussian_random_seq = 0;
    }
    return delt_p;
}

/// @brief 获取0均值的高斯随机向量:速度
float DspMap::getVelocityGaussianZeroCenter(){
    float delt_v = v_gaussian_randoms[velocity_gaussian_random_seq];
    velocity_gaussian_random_seq += 1;
    if(velocity_gaussian_random_seq >= GAUSSIAN_RANDOMS_NUM){
        velocity_gaussian_random_seq = 0;
    }
    return delt_v;
}


/// @brief 添加到对应的体素下标当中
/// @param p 传入粒子
/// @param voxel_index 传出体素下标
/// @return Return 0 if move operation fails, otherwise return 1.
bool DspMap::addAParticle(const Particle &p, const int &voxel_index) const{
    for(int i=0;i<SAFE_PARTICLE_NUM_VOXEL; i++){
        if(voxels_with_particle[voxel_index][i][0] < 0.1f){ // found an empty particle position 
            voxels_with_particle[voxel_index][i][0] = 15.f;  //New born flag
            voxels_with_particle[voxel_index][i][1] = p.velocity.x();
            voxels_with_particle[voxel_index][i][2] = p.velocity.y();
            voxels_with_particle[voxel_index][i][3] = p.velocity.z();
            voxels_with_particle[voxel_index][i][4] = p.position.x();
            voxels_with_particle[voxel_index][i][5] = p.position.y();
            voxels_with_particle[voxel_index][i][6] = p.position.z();
            voxels_with_particle[voxel_index][i][7] = p.weight;
            voxels_with_particle[voxel_index][i][8] = update_time;

            return true;
        }
    } /// If no space. Omit this particle in voxel

    return false;
}

/// @brief 将世界坐标系下的粒子移动到目标体素子空间下，
/// @param new_voxel_index 目标体素index
/// @param current_v_index 当前体素index
/// @param current_v_inner_index 当前体素中粒子的index
/// @param ori_particle_flag_ptr 粒子的一些属性
/// @return 
int DspMap::moveParticle(const int& new_voxel_index, const int& current_v_index, const int& current_v_inner_index, float *ori_particle_flag_ptr)
{

    int new_voxel_inner_index = current_v_inner_index;
    if(new_voxel_index != current_v_index){ //当前体素和目标体素不是同一个
        *ori_particle_flag_ptr = 0.f; // Remove from ori voxel first，因此已经nonvalid了，0.f

        /// Find a space in the new voxel and then pyramid. If no space in either voxel or pyramid. This particle would vanish.
        int successfully_moved_by_voxel = 0;
        for(int i=0; i<SAFE_PARTICLE_NUM_VOXEL; ++i){
            if(voxels_with_particle[new_voxel_index][i][0] < 0.1f){ //empty //如果目标体素下某个存储空间是空的，就将新粒子移动到对应的位置。
                new_voxel_inner_index = i; //
                successfully_moved_by_voxel = 1;

                voxels_with_particle[new_voxel_index][i][0] = 7.f;  //newly moved flag
                for(int k=1; k<9; ++k){  // set v, p, weight, update time,数组指针赋值
                    voxels_with_particle[new_voxel_index][i][k] = *(ori_particle_flag_ptr+k);
                }
                break; ///Important
            }
        }

        //如果没有成功移动
        if(!successfully_moved_by_voxel){  ///let the particle vanish
            return -1;
        }
    }

    // Now check pyramid, pyramids are cleared first so the particle in FOV must be added unless full.
    // 检测新移动的粒子是否在观测金字塔内
    // float rotated_particles[3]; // 这时粒子在世界坐标系下，尽管我们还没有改变，但是是假定。
    Point originParticle = {voxels_with_particle[new_voxel_index][new_voxel_inner_index][4],voxels_with_particle[new_voxel_index][new_voxel_inner_index][5],voxels_with_particle[new_voxel_index][new_voxel_inner_index][6]};
    Point transformedParticle;
    //TODO： 还没有实现
    transformParticleToSensorFrame(transformedParticle,originParticle,currentSensorPosition_,currentSensorRotation_); // 世界坐标系下的粒子移动到体素子空间下。
    // "[error]"rotateVectorByQuaternion(&voxels_with_particle[new_voxel_index][new_voxel_inner_index][4],rotate_quaternion.inverse(),rotated_particles);
    // if(ifInPyramidsArea(voxels_with_particle[new_voxel_index][new_voxel_inner_index][4], voxels_with_particle[new_voxel_index][new_voxel_inner_index][5],
    // //                     voxels_with_particle[new_voxel_index][new_voxel_inner_index][6]))
    // ROS_INFO("particle x:%f\ty:%f\tz:%f",voxels_with_particle[new_voxel_index][new_voxel_inner_index][4],
    //                                         voxels_with_particle[new_voxel_index][new_voxel_inner_index][5],
    //                                         voxels_with_particle[new_voxel_index][new_voxel_inner_index][6]); 
    // ROS_INFO("originParticle x:%f\ty:%f\tz:%f",originParticle[0],originParticle[1],originParticle[2]);
    // ROS_INFO("transformedParticle x:%f\ty:%f\tz:%f",transformedParticle[0],transformedParticle[1],transformedParticle[2]);
    if(ifInPyramidsAreaInSensorFrame(transformedParticle(0),transformedParticle(1),transformedParticle(2)))
    {
        // 如果在，找到对应h,v
        // int h_index = findPointPyramidHorizontalIndex(voxels_with_particle[new_voxel_index][new_voxel_inner_index][4], voxels_with_particle[new_voxel_index][new_voxel_inner_index][5],
        //                                               voxels_with_particle[new_voxel_index][new_voxel_inner_index][6]);

        // int v_index = findPointPyramidVerticalIndex(voxels_with_particle[new_voxel_index][new_voxel_inner_index][4], voxels_with_particle[new_voxel_index][new_voxel_inner_index][5],
        //                                             voxels_with_particle[new_voxel_index][new_voxel_inner_index][6]);

        int h_index = findPointPyramidHorizontalIndexInSensorFrame(transformedParticle(0),transformedParticle(1),transformedParticle(2));
        // ROS_INFO("pyramid_index_h:%d\t",h_index);
        int v_index = findPointPyramidsVerticalIndexInSensorFrame(transformedParticle(0),transformedParticle(1),transformedParticle(2));
        // ROS_INFO("pyramid_index_v:%d\t",v_index);

        int particle_pyramid_index_new = h_index * observation_pyramid_num_v + v_index;

        // 找对应金字塔子空间的存储区域中有没有空的位置
        int successfully_moved_by_pyramid = 0;
        for(int j=0; j<SAFE_PARTICLE_NUM_PYRAMID; j++){
            // ROS_INFO("SUCCESS");
            if(pyramids_in_fov[particle_pyramid_index_new][j][0]==0){
                pyramids_in_fov[particle_pyramid_index_new][j][0] |= O_MAKE_VALID;
                pyramids_in_fov[particle_pyramid_index_new][j][1] = new_voxel_index;
                pyramids_in_fov[particle_pyramid_index_new][j][2] = new_voxel_inner_index;
                successfully_moved_by_pyramid = 1;
                break;
            }
        }
        //如果没有成功移动到对应的金字塔，就返回-2
        if(!successfully_moved_by_pyramid){  /// let the particle vanish
            voxels_with_particle[new_voxel_index][new_voxel_inner_index][0] = 0.f; ///vanish
            return -2;
        }

        /// Add Gaussian randoms to velocities of particles inside FOV
        // 如果速度很小就算了
        if(fabs(voxels_with_particle[new_voxel_index][new_voxel_inner_index][1]*voxels_with_particle[new_voxel_index][new_voxel_inner_index][2]*voxels_with_particle[new_voxel_index][new_voxel_inner_index][3]) < 1e-6){
            // keep small, for static obstacles
//                cout << "keeped"<<endl;
        }else{ // 如果有一定速度，就给它的速度加上0均值高斯噪声
            voxels_with_particle[new_voxel_index][new_voxel_inner_index][1] += getVelocityGaussianZeroCenter();  //vx
            voxels_with_particle[new_voxel_index][new_voxel_inner_index][2] += getVelocityGaussianZeroCenter();  //vy
            voxels_with_particle[new_voxel_index][new_voxel_inner_index][3] = 0.f; //+= getVelocityGaussianZeroCenter();  //vz
        }

    } 
    // 如果在voxel子空间中成功移动，但是在金字塔子空间中没有成功移动，我们就不管了。
    //else we don't need to consider pyramids

    return 1;
}

int DspMap::ifInPyramidsAreaInSensorFrame(float& x, float& y, float& z)
{
    float sinf = z / sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    float f = asin(sinf);
    // if( sinf >= 0 && sinf <= (2 * half_fov_v * angle_resolution_rad))
    if(f >= 0 && f <= sin(2*half_fov_v*angle_resolution_rad))
    {
        return 1;
    }
    else
    {
        return 0;
    }
    
}


int DspMap::findPointPyramidHorizontalIndexInSensorFrame(float& x,float& y,float& z)
{
    float cosf = x / sqrt(pow(x,2)+pow(y,2));
    float f = 0.f;
    if(y >= 0)
    {
        f  = acos(cosf);
    }
    else if(y < 0)
    {
        f = 2 * M_PIf32 - acos(cosf);
    }
    int horizontalIndex = std::floor(f / angle_resolution_rad);
    if(horizontalIndex >= 0 && horizontalIndex <360)
    {
        return horizontalIndex;
    }
    cout << "!!!!!! Please use Function ifInPyramidsArea() to filter the points first before using findPointPyramidHorizontalIndex()" <<endl;
    return -1; // This should not happen if the function is used properly
}

int DspMap::findPointPyramidsVerticalIndexInSensorFrame(float& x,float& y,float& z)
{
    float sinf = z / sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    float f = asin(sinf);
    int verticalIndex = std::floor(f / angle_resolution_rad);
    // ROS_INFO("verticalIndex:%d",verticalIndex);
    if(verticalIndex <= 2 * half_fov_v / angleResolution_)
    {
        return verticalIndex;
    }
    cout << "!!!!!! Please use Function ifInPyramidsAreaInSensorFrame() to filter the points first before using findPyramidVerticalIndexInSensorFrame()" <<endl;
    return -1; // This should not happen if the function is used properly
}

/// @brief 给定一个fov子空间的index，返回其n邻居的下标
/// @param index_ori 要查询的fov子空间的index
/// @param neighbor_spaces_num 可能在边界上，因此返回邻居空间数量 
/// @param neighbor_spaces_index 传出的邻居空间index的数组指针
void DspMap::findPyramidNeighborIndexInFOV(const int &index_ori, int &neighbor_spaces_num, int *neighbor_spaces_index)
{
    int h_index_ori = index_ori / observation_pyramid_num_v;
    int v_index_ori = index_ori % observation_pyramid_num_v;

    neighbor_spaces_num = 0;

    for(int i=-PYRAMID_NEIGHBOR_N ; i <= PYRAMID_NEIGHBOR_N; ++i){
        for(int j=-PYRAMID_NEIGHBOR_N; j <= PYRAMID_NEIGHBOR_N; ++j){
            int h = h_index_ori + i;
            int v = v_index_ori + j;
            if(h>=0 && h<observation_pyramid_num_h && v>=0 && v<observation_pyramid_num_v)
            {
                *(neighbor_spaces_index + neighbor_spaces_num) = h*observation_pyramid_num_v + v;
                ++ neighbor_spaces_num;
            }
        }
    }

}


/// @brief 删除原空间的粒子
/// @param ori_particle_flag_ptr 
void DspMap::removeParticle(float *ori_particle_flag_ptr){
    *ori_particle_flag_ptr = 0.f;
}

/// @brief 高斯密度分布函数
/// @param value 均值
float DspMap::standardNormalPDF(float value)
{
    float fx = (1.f/(sqrtf(2.f*M_PI_2f32)))*expf(-powf(value,2)/(2));
    return fx;
}

/// @brief 计算高斯密度分布函数的缓存,[-10,10]之间的随机变量
void DspMap::calculateNormalPDFBuffer(){
    for(int i=0; i<20000;++i){
        standard_gaussian_pdf[i] = standardNormalPDF((float) (i - 10000) * 0.001f); // range[-10, 10]; 10 sigma
    }
}

/// @brief 正则化概率分布密度，
/// @param x 输入值
/// @param mu 均值
/// @param sigma 方差
/// @return 
float DspMap::queryNormalPDF(float &x, float &mu, float &sigma)
{
    float corrected_x = (x-mu)/sigma;
    if(corrected_x>9.9f) corrected_x=9.9f;
    else if(corrected_x<-9.9f) corrected_x=-9.9f;

    return standard_gaussian_pdf[(int)(corrected_x*1000+10000)];
}

/// @brief 
/// @param transformedPoint 
/// @param oriPoint 
/// @param position 当前无人机的位置
/// @param rotation 当前无人机旋转四元数
void DspMap::transformParticleToSensorFrame(Point& transformedPoint,const Point& oriPoint,const Position& position,const Rotation& rotation)
{
    Rotation tempQuaternion,targetQuaternion;
    tempQuaternion.w() = 0;
    tempQuaternion.x() = oriPoint.x();
    tempQuaternion.y() = oriPoint.y();
    tempQuaternion.z() = oriPoint.z();
    targetQuaternion = rotation * tempQuaternion * rotation.inverse();
    transformedPoint.x() = targetQuaternion.x() + position.x();
    transformedPoint.y() = targetQuaternion.y() + position.y();
    transformedPoint.z() = targetQuaternion.z() + position.z();
}



/// @brief 对金字塔空间的法线向量进行旋转
/// @param ori_vector 输入的单个金字塔体素的法线向量
/// @param quaternion 传感器的旋转矩阵
/// @param rotated_vector 传出的旋转过后的单个金字塔体素的法线向量
void DspMap::rotateVectorByQuaternion(const float *ori_vector, const Eigen::Quaternionf& quaternion, float *rotated_vector)
{
    //Lazy. Use Eigen directly
    Eigen::Quaternionf ori_vector_quaternion, vector_quaternion;
    ori_vector_quaternion.w() = 0;
    ori_vector_quaternion.x() = *ori_vector;
    ori_vector_quaternion.y() = *(ori_vector+1);
    ori_vector_quaternion.z() = *(ori_vector+2);

    vector_quaternion = quaternion * ori_vector_quaternion * quaternion.inverse();
    *rotated_vector = vector_quaternion.x();
    *(rotated_vector+1) = vector_quaternion.y();
    *(rotated_vector+2) = vector_quaternion.z();
}

/// @brief 向量乘法 
float DspMap::vectorMultiply(float &x1, float &y1, float &z1, float &x2, float &y2, float &z2){
    return x1*x2 + y1*y2 + z1*z2;
}

float DspMap::vectorCross2d(float& x1,float& y1,float& x2,float& y2)
{
    return x1 * y2 - x2 * y1;
}

/// @brief 计算两个聚类的距离
/// @return 
float DspMap::clusterDistance(ClusterFeature &c1, ClusterFeature &c2){
    float square_distance = (c1.centerPosition.x() - c2.centerPosition.x())*(c1.centerPosition.x() - c2.centerPosition.x()) +
                            (c1.centerPosition.y() - c2.centerPosition.y())*(c1.centerPosition.y() - c2.centerPosition.y()) +
                            (c1.centerPosition.z() - c2.centerPosition.z())*(c1.centerPosition.z() - c2.centerPosition.z());
    return sqrtf(square_distance);
}

/// @brief 用来估计粒子速度，观测点云原来是局部坐标系的，因为改动后我们直接输入世界坐标系下的点云，因此我们不需要对其进行转换。
void DspMap::velocityEstimationThread()
{
    if( cloud_in_current_view_rotated->points.empty()) return;

    input_cloud_with_velocity->clear();

    /// Remove ground and transform data
    // 除去地面点
    pcl::PointCloud<pcl::PointXYZ>::Ptr static_points(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_points(new pcl::PointCloud<pcl::PointXYZ>());

    for(auto &p : cloud_in_current_view_rotated->points){
        if(p.z > voxel_filtered_resolution){ // 滤掉地面点。
            non_ground_points->points.push_back(p); // 非地面点：
        }else{ // 地面静态点
            static_points->points.push_back(p);  // 静态点：一定包含地面点，
        }
    }

    /// Cluster
    // 欧式聚类
    static std::vector<ClusterFeature> clusters_feature_vector_dynamic_last; //
    std::vector<ClusterFeature> clusters_feature_vector_dynamic;
    std::vector<pcl::PointIndices> cluster_indices;
    vector<bool> cluster_possibly_dynamic; // 标记这个簇可不可能是动态的簇

    if(!non_ground_points->empty()) // 非地面点：非地面静态点+非地面动态点
    {
        
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (non_ground_points);
        //kd-tree 欧式聚类
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (2*voxel_filtered_resolution);
        ec.setMinClusterSize (5);
        ec.setMaxClusterSize (10000);

        ec.setSearchMethod (tree);
        ec.setInputCloud (non_ground_points);
        ec.extract (cluster_indices);

        for(const auto & cluster_indice : cluster_indices) // 欧式聚类后的簇集合
        {
            ClusterFeature cluster_this; // 
            cluster_this.intensity = generateRandomFloat(0.1f, 1.f); //For visualization

            for (int indice : cluster_indice.indices){
                cluster_this.centerPosition.x() += (*non_ground_points)[indice].x; //sum
                cluster_this.centerPosition.y() += (*non_ground_points)[indice].y;
                cluster_this.centerPosition.z() += (*non_ground_points)[indice].z;
                ++ cluster_this.point_num;
            }

            // average
            cluster_this.centerPosition.x() /= (float)cluster_this.point_num;
            cluster_this.centerPosition.y() /= (float)cluster_this.point_num;
            cluster_this.centerPosition.z() /= (float)cluster_this.point_num;

            // 感觉这个不合理，如果点太多了，或者点的高度超出了上限，就假定它为动态点。
            if(cluster_indice.indices.size() > DYNAMIC_CLUSTER_MAX_POINT_NUM || cluster_this.centerPosition.z() > DYNAMIC_CLUSTER_MAX_CENTER_HEIGHT){ //filter static points  //400, 1.5
                // Static 把地面点标记为静态点
                for (int indice : cluster_indice.indices){
                    static_points->push_back((*non_ground_points)[indice]);
                }
                cluster_possibly_dynamic.push_back(false);
            }else{
                // Possibly dynamic
                // 把非地面点标记为可能动态点
                clusters_feature_vector_dynamic.push_back(cluster_this);
                cluster_possibly_dynamic.push_back(true);
            }
        }

        // KM 算法
        static float distance_gate = 1.5f;
        static int point_num_gate = 100;
        static float maximum_velocity = 5.f;

        /// Move last feature vector d and match by KM algorithm
        if(!clusters_feature_vector_dynamic_last.empty() && !clusters_feature_vector_dynamic.empty()){
            if(delt_t_from_last_observation > 0.00001 && delt_t_from_last_observation < 10.0){
                Matrix<float> matrix_cost(clusters_feature_vector_dynamic.size(), clusters_feature_vector_dynamic_last.size()); //This is a Matrix defined in munkres.h
                Matrix<float> matrix_gate(clusters_feature_vector_dynamic.size(), clusters_feature_vector_dynamic_last.size());

                for(size_t row=0; row < clusters_feature_vector_dynamic.size(); ++row)
                {
                    for(size_t col=0; col < clusters_feature_vector_dynamic_last.size(); ++col){
                        float cluster_distance_this = clusterDistance(clusters_feature_vector_dynamic[row], clusters_feature_vector_dynamic_last[col]);
                        if(abs(clusters_feature_vector_dynamic[row].point_num - clusters_feature_vector_dynamic_last[col].point_num) > point_num_gate
                            || cluster_distance_this >= distance_gate){
                            matrix_gate(row, col) = 0.f;
                            matrix_cost(row, col) = distance_gate * 5000.f;
                        }else{
                            matrix_gate(row, col) = 1.f;
                            matrix_cost(row, col) = cluster_distance_this / distance_gate * 1000.f;
                        }
                    }
                }

                Munkres<float> munkres_solver;
                munkres_solver.solve(matrix_cost);

                for(size_t row=0; row < clusters_feature_vector_dynamic.size(); ++row)
                {
                    for(size_t col=0; col < clusters_feature_vector_dynamic_last.size(); ++col)
                    {
                        if(matrix_cost(row, col) == 0.f && matrix_gate(row, col) > 0.01f){ // Found a match
                            clusters_feature_vector_dynamic[row].match_cluster_seq = col;
                            clusters_feature_vector_dynamic[row].velocity.x() = (clusters_feature_vector_dynamic[row].centerPosition.x() - clusters_feature_vector_dynamic_last[col].centerPosition.x()) / delt_t_from_last_observation;
                            clusters_feature_vector_dynamic[row].velocity.y() = (clusters_feature_vector_dynamic[row].centerPosition.y() - clusters_feature_vector_dynamic_last[col].centerPosition.y()) / delt_t_from_last_observation;
                            clusters_feature_vector_dynamic[row].velocity.z() = (clusters_feature_vector_dynamic[row].centerPosition.z() - clusters_feature_vector_dynamic_last[col].centerPosition.z()) / delt_t_from_last_observation;
//                        cout << "v=("<<clusters_feature_vector_dynamic[row].vx<<", " << clusters_feature_vector_dynamic[row].vy <<", "<<clusters_feature_vector_dynamic[row].vz << ")" << endl;
                            clusters_feature_vector_dynamic[row].velocityNorm = sqrtf(clusters_feature_vector_dynamic[row].velocity.x() * clusters_feature_vector_dynamic[row].velocity.x() + clusters_feature_vector_dynamic[row].velocity.y() * clusters_feature_vector_dynamic[row].velocity.y() + clusters_feature_vector_dynamic[row].velocity.z() * clusters_feature_vector_dynamic[row].velocity.z());
                            clusters_feature_vector_dynamic[row].intensity = clusters_feature_vector_dynamic_last[col].intensity; //for visualization

                            if(clusters_feature_vector_dynamic[row].velocityNorm > maximum_velocity){
                                clusters_feature_vector_dynamic[row].velocityNorm = 0.f;
                                clusters_feature_vector_dynamic[row].velocity.x() = clusters_feature_vector_dynamic[row].velocity.y() = clusters_feature_vector_dynamic[row].velocity.z() = 0.f;
                            }

                            break;
                        }
                        /// If no match is found. The cluster velocity is given by struct initialization (v=-1000).
                    }
                }
            }
        }

        /// Velocity Allocation to Points
        // Use normal to store velocity
        int cluster_indice_seq = 0;
        int cluster_dynamic_vector_seq = 0;
        for(const auto & cluster_indice : cluster_indices) {
            if(cluster_possibly_dynamic[cluster_indice_seq]){
                for (int indice : cluster_indice.indices) {
                    pcl::PointXYZINormal p;
                    p.x = (*non_ground_points)[indice].x;
                    p.y = (*non_ground_points)[indice].y;
                    p.z = (*non_ground_points)[indice].z;
                    p.normal_x = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].velocity.x();  // Use color to store velocity
                    p.normal_y = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].velocity.y();
                    p.normal_z = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].velocity.z();
                    p.intensity = clusters_feature_vector_dynamic[cluster_dynamic_vector_seq].intensity; // For visualization. // clusters_feature_vector_dynamic[cluster_indice_seq].v / maximum_velocity;
                    input_cloud_with_velocity->push_back(p);
                }
                ++ cluster_dynamic_vector_seq;
            }

            ++ cluster_indice_seq;
        }

    }


    //  静态点：地面点
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
        input_cloud_with_velocity->push_back(p);
    }

    clusters_feature_vector_dynamic_last = clusters_feature_vector_dynamic;
    // cout << "Velocity estimation done" << endl;
}



float DspMap::generateRandomFloat(float min, float max){
    return min + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(max-min)));
}
}