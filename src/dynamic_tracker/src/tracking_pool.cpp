#include "dynamic_tracker/tracking_pool.hpp"


namespace dynamic_tracker
{

    Vector6d TrackingPool::predictTracking(const Tracking::Ptr track)
    {
        /* check if it's alive , need to be ensured in up layer */
        // if(!track->is_alive_){
        //     return ;
        // }
        return A_ * track->state_;
    }

    Vector6d TrackingPool::predictTracking(const Tracking::Ptr track, int timestep)
    {
        Eigen::MatrixPower<Matrix6d> A_power(A_);
        return A_power(timestep) * track->state_;
    }

    void TrackingPool::createTracking(int new_id,const Vector6d &obs, const Vector3d &axis, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pool_.emplace_back(new Tracking(new_id, obs, axis, cloud));
    }
    
    void TrackingPool::resetTracking(int exist_id,const Vector6d &obs,const  Vector3d &axis, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pool_.at(exist_id)->reset(exist_id, obs, axis, cloud);
    }

    int TrackingPool::addTracking(const Vector6d &obs,const Vector3d &axis, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        int new_id;
        if(free_ids.empty()){ // 如果没有free的id，重新生成一个id，并且构建这个新的tracking
            new_id = size_++;
            createTracking(new_id, obs, axis,cloud);
        }
        else{ // 如果有free的id，就直接调用这个id
            new_id = free_ids.front();
            free_ids.pop();
            resetTracking(new_id,obs,axis,cloud);
            size_++;
        }
        return new_id;
    }

    void TrackingPool::removeTracking(int id)
    {
        pool_[id]->is_alive_ = false;
        size_--;
        free_ids.push(id);
    }

    void TrackingPool::updateTracking(int id,const Vector6d &obs,const Vector3d& axis,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        /* history traj */
        // ROS_INFO("UPDATING ID = %d.....",id);
        Tracking::Ptr cur_track = pool_.at(id);
        Vector3d pos = cur_track->state_.head<3>();
        cur_track->history_traj_.push(pos);
        if(cur_track->history_traj_.size() > 20){
            cur_track->history_traj_.pop();
        }
        Vector6d state_now = predictTracking(cur_track);
        cur_track->P_ = A_ * cur_track->P_ * A_.transpose() + Qc_;
        Vector6d state_priori = H_ * state_now;
        Vector6d residual = obs - state_priori;
        Matrix6d S = H_ * cur_track->P_ * H_.transpose() + Rc_;
        /* regularization && lu 分解 */
        Eigen::FullPivLU<Matrix6d> lu(S);
        Matrix6d S_inv;
        if(!lu.isInvertible()){
            // ROS_INFO("S is not invertible! excute regularization !");
            double regularizationTerm = 1e-8;
            // Eigen::MatrixXd regularizedMatrix = matrix.transpose() * matrix + regularizationTerm * Eigen::MatrixXd::Identity(6, 6);
            Matrix6d regularizedMatrix = S.transpose() * S + regularizationTerm * Eigen::MatrixXd::Identity(6, 6);
            Eigen::FullPivLU<Matrix6d> nlu(regularizedMatrix);
            if(nlu.isInvertible()){
                S_inv = nlu.inverse();
            }
            else{
                ROS_ERROR("still be not inversed ");
                return;
            }
        }
        else{
            S_inv = S.inverse();
        }
        
        /* axis update */
        cur_track->axis_ = axis;
        ROS_INFO("tracking id : %d, axis : %f \t %f \t %f",cur_track->id_,axis(0),axis(1),axis(2));

 
        Matrix6d K = cur_track->P_ * H_.transpose() * S_inv;
        cur_track->state_ = state_now + K * residual;

        cur_track->P_ = (Matrix6d::Identity() - K * H_) * cur_track->P_;
        /* reborn */
        if(!cur_track->is_alive_){
            cur_track->is_alive_ = true;
        }


        /* update disappear_time */
        cur_track->disappear_time_ = 0;
        /* cloud update */
        cur_track->cloud_ = cloud;
    
    }

    void TrackingPool::getPool(std::vector<std::pair<int,std::pair<Vector6d,Vector3d>>> &alive_trackings)
    {
        alive_trackings.clear();
        for(auto &track:pool_){
            if(track->is_alive_){
                alive_trackings.emplace_back(std::make_pair(track->id_,std::make_pair(track->state_,track->axis_)));
            }
        }
    }

    void TrackingPool::predictPool(std::vector<std::pair<int,Vector6d>> &alive_trackings)
    {
        alive_trackings.clear();    
        for(auto &track:pool_){
            if(track->is_alive_){
                alive_trackings.emplace_back(track->id_,predictTracking(track));
            }
        }
    }

    void TrackingPool::predictPool(std::vector<std::pair<int, Vector6d>> &alive_trackings, int timestep)
    {
        alive_trackings.clear();
        for(auto &track:pool_){
            if(track->is_alive_){
                alive_trackings.emplace_back(track->id_,predictTracking(track,timestep));
            }
        }
    }

    void TrackingPool::init(ros::NodeHandle& nh)
    {
        node_ = nh; 
        size_ = 0;
        ts_ = node_.param<double>("dynamic_tracker/ts",0.35);
        missing_time_threshold_ = node_.param<int>("dynamic_tracker/missing_time_threshold",20);
        A_ = Matrix6d::Identity();
        A_(0, 3) = ts_;
        A_(1, 4) = ts_;
        A_(2, 5) = ts_;
        H_ = Matrix6d::Identity();
        Qc_ = Matrix6d::Identity() * 0.05;
        Qc_(2, 2) = 0;
        Qc_(5, 5) = 0;
        Rc_ = Matrix6d::Identity() * 1;
        Rc_(2, 2) = 0;
        Rc_(5, 5) = 0;
        // std::cout << "A : " << std::endl << A_ << std::endl;
        // std::cout << "H : " << std::endl << H_ << std::endl;
        // std::cout << "Q : " << std::endl << Qc_ << std::endl;
        // std::cout << "R : " << std::endl << Rc_ << std::endl;

    }

    void TrackingPool::updatePool(std::vector<int> &obs_match_ids, std::vector<Vector6d> &obs, std::vector<Vector3d> &obs_axis, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &obs_cloud)
    {
        /* obs_match_ids : 1. obs_id, 2. tracking_id*/
        /* check */
        if(obs_match_ids.size() != obs.size()){
            // ROS_ERROR("ERROR IN MATCH PROCESS !!!!");
            return; 
        }


        // disappear time ++
        for(auto& t : pool_){
            if(t->is_alive_){
                t->disappear_time_ ++;
            }
        }


        /* update */
        // ROS_INFO("UPDATE START");
        for(size_t i = 0; i < obs_match_ids.size(); i++){
            int obs_id = i;
            int track_id = obs_match_ids[i];
            if(track_id == -1){ // 没有合适的匹配
                // ROS_INFO("NO PROPER MATCH");
                track_id = addTracking(obs[obs_id],obs_axis[obs_id],obs_cloud[obs_id]);
            }
            else{ // 有合适的匹配
                // ROS_INFO("HAS PROPER MATCH");
                updateTracking(track_id,obs[obs_id],obs_axis[obs_id],obs_cloud[obs_id]);               
            }
        }
        // ROS_INFO("UPDATE FINISH");


        // 更新完以后再确定有没有超时的，更新过程中会将disappear_time置零
        for(auto& t : pool_){
            if(t->disappear_time_ > missing_time_threshold_){
                removeTracking(t->id_);
            }
        }        
    }
}