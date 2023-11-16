#include "dynamic_tracker/dynamic_tracker.h"



namespace dynamic_tracker
{

    KalmanFilter::KalmanFilter(Vector4d& init_state,double ts): state_last_(init_state),ts_(ts){
        A_ = Matrix4d::Identity();
        A_(0,2) = ts;
        A_(1,3) = ts;
        P_last_ = Matrix4d::Identity();
        Rc_ = Matrix4d::Identity();
        Qc_ = Matrix4d::Identity();
        H_ = Matrix4d::Zero();
        H_(0,0) = 1;
        H_(1,1) = 1;
    }

    Vector4d KalmanFilter::update(const Vector4d& obs){
        Vector4d state_now = predict();
        Matrix4d P_now_ = A_ * P_last_ * A_.transpose() + Qc_;
        Vector4d state_obs = H_ * state_now;
        Vector4d residual = obs - state_obs;
        Matrix4d S = H_ * P_now_ * H_.transpose() + Rc_;
        Matrix4d K = P_now_ * H_.transpose() * S.inverse();
        state_last_ = state_now + K * residual;
        P_last_ = (Matrix4d::Identity() - K * H_) * P_now_;
        return state_last_;
    };

    Vector4d KalmanFilter::predict(){
        return A_ * state_last_;
    };

    Vector4d KalmanFilter::predict(int timestep){
        Eigen::MatrixPower<Matrix4d> Apow(A_);
        return Apow(timestep) * state_last_;
    }

    void DynamicTracker::init(const ros::NodeHandle &nh)
    {
        node_ = nh;
        node_.param("dynamic_tracking/tracking_update_timeout_", tracking_update_timeout_, 1.0);
        node_.param("dynamic_tracking/tracking_update_rate", tracking_update_rate_, 10.0);
        node_.param("dynamic_tracking/tracking_ts", ts_, 0.35);
        node_.param("dynamic_tracking/pose_type",pose_type_,1);
        last_tracking_update_time_.fromSec(0);
        tracking_need_update_ = false;

        /* 初始化聚类器 */
        node_.param("dynamic_tracking/cluster_pts", minPts_, 4);
        node_.param("dynamic_tracking/epsilon", eps_, 0.5);
        cluster_ptr_->init(minPts_, eps_);

        /* 初始化随机器 */
        // static std::default_random_engine eng_;
        // static std::normal_distribution<double> q_(0,1.0);
        // static std::normal_distribution<double> r_(0,1.0);

        /* system init */
        
        // q_noise_ = Vector4d::Zero().unaryExpr([](){return q_(eng_);}); // 
        // r_noise_ = Vector4d::Zero().unaryExpr([](){return r_(eng_);}); //
        
        // xi[k+1] = A * xi[k] + N(0,Q);
        // zi[k] = H * xi[k] + N(0,R); 

        /* init callback */
        cloud_sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(node_, "dynamoc_trakcing/cloud", 1));
        if(pose_type_ == POSE_STAMPED)
        {
            pose_sub_.reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(node_, "dynamic_tracking/pose", 1));
            sync_cloud_pose_.reset(new message_filters::Synchronizer<SyncPolicyCloudPose>(SyncPolicyCloudPose(10), *cloud_sub_,*pose_sub_));
            sync_cloud_pose_->registerCallback(boost::bind(&DynamicTracker::cloudPoseCallback, this, _1,_2));
        }
        else if(pose_type_ == ODOMETRY)
        {
            odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "dynamic_tracking/odom", 1));
            sync_cloud_odom_.reset(new message_filters::Synchronizer<SyncPolicyCloudOdom>(SyncPolicyCloudOdom(10), *cloud_sub_,*odom_sub_));
            sync_cloud_odom_->registerCallback(boost::bind(&DynamicTracker::cloudOdomCallback, this, _1,_2));
        }


        /* 雷达相对于机身的位置 */
        lidar2body_ << 0.0, 0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0, 0.0,
            0.0, -1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    }

    void DynamicTracker::cloudOdomCallback(const sensor_msgs::PointCloud2ConstPtr &cloud,
                                           const nav_msgs::OdometryConstPtr &odom)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud;
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
        body2world(3, 3) = odom->pose.pose.position.z;
        body2world(3, 3) = 1.0;

        Eigen::Matrix4d lidar_T = body2world * lidar2body_;
        lidar_pos_(0) = lidar_T(0, 3);
        lidar_pos_(1) = lidar_T(1, 3);
        lidar_pos_(2) = lidar_T(2, 3);
        lidar_r_m_ = lidar_T.block<3, 3>(0, 0);

        pcl::transformPointCloud(*latest_cloud, *current_cloud, lidar_T);

        tracking_need_update_ = true;

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
                ROS_ERROR("odom or cloud lost! ros::Time::now()=%f, md_.last_occ_update_time_=%f, mp_.odom_depth_timeout_=%f",
                          ros::Time::now().toSec(), last_tracking_update_time_.toSec(), tracking_update_timeout_);
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
        t1 = ros::Time::now();

        // 2. match()
        t2 = ros::Time::now();

        // 3. kalman filter update 
        t3 = ros::Time::now();

        // 4. vote for velocity estimation
        t4 = ros::Time::now();

        // 5. 

    }



    /********************************core**********************************************/

    void DynamicTracker::forwardPropagation(std::vector<Vector4d>& predicted_clusters)
    {
        for(int i=0;i<tracking_clusters.size();i++)
        {
            predicted_clusters.emplace_back(tracking_clusters[i].predict());
        }

    }

    void DynamicTracker::cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::Indices& cloud_ids,std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& observed_clusters)
    {
        pcl::Indices cloud_ids;
        cluster_ptr_->cluster(cloud,cloud_ids);

        /* 把每一个簇的点云放在一个单独的PointCloud中，*/
        observed_clusters.clear();
        
        for(int i=0;i<cloud_ids.size();i++){
            if(cloud_ids[i] == 0 || cloud_ids[i] == 1){
                continue;
            }
            else if(cloud_ids[i] > observed_clusters.size()){
                observed_clusters.emplace_back(new pcl::PointCloud<pcl::PointXYZ>);
                int cluster_id = cloud_ids[i]-1;
                observed_clusters[cluster_id]->push_back(cloud->points[i]);
            }
            else {
                ROS_ERROR("there is a negative id!");
            }
        }
    }

    void DynamicTracker::match(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const pcl::Indices& cloud_ids)
    {

        /* 根据簇中点云的位置，计算出一个最大的包裹椭圆和椭圆中心 */
        std::vector<Vector6d> observerd_ellipsoids;
        generateEllipsoids(observed_clusters,observerd_ellipsoids);


    }
}