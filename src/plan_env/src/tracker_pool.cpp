#include "plan_env/dynamic/tracker_pool.h"




void TrackerPool::forwardTracker(int id, VectorXd &state, Vector3d &length, ros::Time target_time)
{
    Tracker::Ptr tracker;
    if(!getTracker(id,tracker))
    {
        return ;
    }
    double dt = (target_time - tracker->getUpdateTime()).toSec();
    state = pool_[id]->forward(dt);
    length = pool_[id]->getLength();
}

void TrackerPool::updateTracker(int id, const VectorXd &state, const Vector3d &length, ros::Time current_time)
{

    Tracker::Ptr tracker;
    if(!getTracker(id,tracker))
    {
        return ;
    }
    tracker->update(state,current_time,length);
}

int TrackerPool::addNewTracker(const VectorXd &state, const Vector3d &length, ros::Time current_time)
{
    int new_id;
    if(free_ids_.empty())
    {
        new_id = pool_.size();
        pool_.emplace_back(std::make_shared<Tracker>(state,length,new_id, current_time));
        pool_size_ ++; // pool size increase, only no id & add new tracker
    }
    else
    {
        new_id = free_ids_.front();
        free_ids_.pop();
        pool_[new_id] = std::make_shared<Tracker>(state,length,new_id, current_time);
    }
    std::cout << " success add new tracker, id :" << new_id << std::endl;
    return new_id;
}


void TrackerPool::removeTracker(int id)
{
    if(id < pool_.size())
    {
        pool_[id] = nullptr;
        free_ids_.push(id);
    }
    else{
        std::cerr<<"In [removeTracker]: id out of range"<<std::endl;
    }
    
}

void TrackerPool::checkTracker(int id, ros::Time current_time)
{
    Tracker::Ptr tracker;
    if(!getTracker(id,tracker))
    {
        return ;
    }
    if(tracker == nullptr)
    {
        std::cerr << "In [checkTracker]: tracker is not alive" << std::endl;
        return ;
    }
    double mis_match_time = (current_time - tracker->getUpdateTime()).toSec();
    std::cout << "1 " << std::endl;
    std::cout << "missing_tracking_threshold_:" <<  missing_tracking_threshold_<< std::endl;
    std::cout << "mis_match_time:" <<mis_match_time << std::endl;
    if(mis_match_time > missing_tracking_threshold_)
    {
        std::cout << "Tracker " << id << " mis-match time is larger than the threshold, remove it from the pool" << std::endl;
        removeTracker(id);
    }

}

bool TrackerPool::getTracker(int id, Tracker::Ptr &tracker_ptr)
{
    // check if id is valid 
    if(id < 0 || id > pool_.size())
    {
        std::cerr << "In [getTracker]: id out of range" << std::endl;
        return false;
    }
    //check if tracker is alive 
    if(pool_[id] == nullptr)
    {
        std::cerr << "In [getTracker]: tracker is not alive" << std::endl;
        return false;
    }
    tracker_ptr = pool_[id];
    return true;
}


void TrackerPool::init(ros::NodeHandle& nh)
{
    node_ = nh;
    pool_size_ = 0;
    missing_tracking_threshold_ = node_.param<double>("dynamic_tracker/missing_tracking_threshold",2.0);
}



void TrackerPool::forwardPool(vector<TrackerOutput> &output, ros::Time target_time)
{
    output.clear();
    double dt;
    for(auto &track : pool_)
    {
        if(track != nullptr)
        {
            VectorXd state;
            Vector3d length;
            forwardTracker(track->getId(),state,length,target_time);
            output.emplace_back(TrackerOutput{track->getId(),state,length});
        }
    }
}

void TrackerPool::getPool(vector<TrackerOutput> &outputs)
{
    outputs.clear();
    for(auto &track : pool_)
    {
        if(track != nullptr)
        {
            VectorXd state;
            Vector3d length;
            outputs.emplace_back(TrackerOutput{track->getId(),track->getState(),track->getLength()});
        }
    }
}

void TrackerPool::updatePool(const vector<TrackerInput> &input, ros::Time current_time)
{
    for(auto &in : input)
    {

        std::cout << "update tracker id : " << in.id << std::endl;
        if(in.id == -1)
        {
            addNewTracker(in.measurment,in.length,current_time);
        }
        else if (in.id < pool_.size())
        {
            updateTracker(in.id,in.measurment,in.length,current_time);
        }
        else
        {
            std::cerr << "In [updatePool]: id out of range or tracker" << std::endl;
        }
    }
    std::cout << "finish update, start check " << std::endl;

    for(int i = 0; i < pool_.size(); i++)
    {
        if(pool_[i] != nullptr)
        {
            std::cout << "check id : " << i << std::endl;
            checkTracker(i,current_time);
        }
    }

    std::cout << "end check " << std::endl;
}