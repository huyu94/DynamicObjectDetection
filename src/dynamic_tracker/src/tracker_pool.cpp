#include "dynamic_tracker/tracker_pool.h"




void TrackerPool::forwardTracker(int id, VectorXd &state, Vector3d length, double dt)
{
    state = pool_[id]->forward(dt);
    length = pool_[id]->getLength();
}

void TrackerPool::updateTracker(int id, const VectorXd &state, const Vector3d &length)
{
    pool_[id]->update(state,current_time_,length);
}

int TrackerPool::addNewTracker(const VectorXd &state, const Vector3d &length)
{
    int new_id;
    if(free_ids_.empty())
    {
        new_id = pool_.size();
        pool_.emplace_back(std::make_shared<Tracker>(state,length,new_id, current_time_));
        pool_size_ ++; // pool size increase, only no id & add new tracker
    }
    else
    {
        new_id = free_ids_.front();
        free_ids_.pop();
        pool_[new_id] = std::make_shared<Tracker>(state,length,new_id, current_time_);
    }
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

void TrackerPool::checkTracker(int id)
{
    double mis_match_time = (current_time_ - pool_[id]->last_match_time_).toSec();
    if(mis_match_time > missing_tracking_threshold_)
    {
        std::cout << "Tracker " << id << " mis-match time is larger than the threshold, remove it from the pool" << std::endl;
        removeTracker(id);
    }

}

Tracker::Ptr TrackerPool::getTracker(int id)
{
    // check if id is valid 
    if(id < 0 || id > pool_.size())
    {
        std::cerr << "In [getTracker]: id out of range" << std::endl;
        return nullptr;
    }
    //check if tracker is alive 
    if(pool_[id] == nullptr)
    {
        std::cerr << "In [getTracker]: tracker is not alive" << std::endl;
        return nullptr;
    }
    return pool_[id];
}


void TrackerPool::init(ros::NodeHandle& nh)
{
    node_ = nh;
    pool_size_ = 0;
    missing_tracking_threshold_ = node_.param<double>("dynamic_tracker/missing_tracking_threshold",2.0);
    current_time_ = ros::Time::now();
}



void TrackerPool::forwardPool(vector<TrackerOutput> &output, double dt)
{
    output.clear();
    for(auto &track : pool_)
    {
        if(track != nullptr)
        {
            VectorXd state;
            Vector3d length;
            forwardTracker(track->getId(),state,length,dt);
            output.emplace_back(TrackerOutput{track->getId(),state,length});
        }
    }
}

void TrackerPool::updatePool(const vector<TrackerInput> &input, ros::Time current_time)
{
    current_time_ = current_time;
    for(auto &in : input)
    {
        if(in.id < pool_.size() && pool_[in.id] != nullptr)
        {
            updateTracker(in.id,in.measurment,in.length);
        }
        else
        {
            std::cerr << "In [updatePool]: id out of range or tracker is not alive" << std::endl;
        }
    }
    for(int i = 0; i < pool_.size(); i++)
    {
        if(pool_[i] != nullptr)
        {
            checkTracker(i);
        }
    }
}