#ifndef TRACKER_POOL_H_
#define TRACKER_POOL_H_

#include <vector>
#include <queue>

#include "plan_env/dynamic/tracker.hpp"



using std::vector;
using std::pair;
using std::shared_ptr;


struct TrackerOutput
{
    int id;
    VectorXd state;
    Vector3d length;
};

struct TrackerInput
{
    int id;
    VectorXd measurment;
    Vector3d length;
};


class TrackerPool
{
private:
    ros::NodeHandle node_;
    
    
    int pool_size_;
    std::vector<Tracker::Ptr> pool_; // 存储所有的跟踪对象
    std::queue<int> free_ids_; // 存储可用的ID
    ros::Time current_time_;
    double dt_;
    double missing_tracking_threshold_;




private:
    /**
     * @brief single tracker predict, just predict, no update, no covariance matrix calculation
     * @param id id of the tracker in the pool
     * @param tracker tracker object
    */
    void forwardTracker(int id, VectorXd &state, Vector3d length, double dt);

    /**
     * @brief single tracker update
     * @param id id of the tracker in the pool
     * @param length length of the object
     * @param state state of the object
    */
    void updateTracker(int id, const VectorXd &state, const Vector3d &length);


    /**
     * @brief add new tracker to the pool
     * @return id of the new tracker
     * 
    */
    int addNewTracker(const VectorXd &state, const Vector3d &length);

    void removeTracker(int id);
    /**
     * @brief check if the tracker mis-match time is larger than the threshold
     * @param id id of the tracker
     * @warning should update current_time first
    */
    void checkTracker(int id );
    
    Tracker::Ptr getTracker(int id);
    


public:
    TrackerPool(){};
    ~TrackerPool(){};

    /**
     * @brief Initialize
    */
    void init(ros::NodeHandle &nh);


    inline int size()
    {
        return pool_size_;
    }

    /**
     * @brief predcit all object in the pool in dt
     * @param target_time target time
     * @param output TrackerOutput 
    */
    void forwardPool(vector<TrackerOutput> &output, ros::Time target_time);
    // void forwardPool(vector<TrackerOutput> &output, double dt);

    /**
     * @brief update all object in the pool
     * @param dt time interval
     * @param measurment measurment data:
     * @param input TrackerInput
    */
    void updatePool(const vector<TrackerInput> &input, ros::Time current_time);

    /**
     * @brief get all object states in the pool
     * @param object_states states of all objects, x,y,z,vx,vy,vz
    */
    void getPool(vector<VectorXd> &object_states);


    typedef shared_ptr<TrackerPool> Ptr;
};

#endif