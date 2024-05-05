#ifndef TRACKER_POOL_H_
#define TRACKER_POOL_H_

#include <vector>
#include <queue>

#include "plan_env/dynamic/tracker.hpp"



using std::vector;
using std::pair;
using std::shared_ptr;
using Eigen::Quaterniond;
using Eigen::Matrix3d;
using std::max, std::min;


struct TrackerOutput
{
    int id;
    VectorXd state;
    Vector3d length;
    TrackerOutput(int id, VectorXd state, Vector3d length):id(id),state(state),length(length){}
};

struct TrackerInput
{
    int id;
    VectorXd measurment;
    Vector3d length;
    TrackerInput(int id, VectorXd measurment, Vector3d length):id(id),measurment(measurment),length(length){}
};

class SlideBox
{
private:
    int id_;
    Vector3d center_;
    Vector3d length_; // 1/2 edge length
    Matrix3d rotation_;
public:
    SlideBox(int id,Vector3d center, Vector3d length, Matrix3d rotation):id_(id),center_(center),length_(length),rotation_(rotation){}
    SlideBox(const Tracker::Ptr a,ros::Time targetTime)
    {
        double dt = (targetTime - a->getUpdateTime()).toSec();
        id_ = a->getId();
        Vector3d p0 = a->getState().head(3);
        Vector3d v = a->getState().tail(3);
        v(2) = 0; //去除z轴方向的移动
        Vector3d pt = a->getState().head(3) + v * dt;
        center_ = (p0 + pt) / 2;


        Vector3d xtf,ytf,ztf, downward(0,0,-1);
        xtf = (pt - p0).normalized();
        // cornor case : xtf is parallel to downward
        if(xtf.cross(downward).norm() < 1e-6)
        {
            xtf(0) += 0.01;
        }
        else
        {
            xtf(1) += 0.01;
        }
        ytf = xtf.cross(downward).normalized();
        ztf = xtf.cross(ytf).normalized();
        rotation_.col(0) = xtf;
        rotation_.col(1) = ytf;
        rotation_.col(2) = ztf;

        Vector3d el_len(0,0,0);
        Vector3d ax(a->getLength()(0),0,0);
        Vector3d ay(0,a->getLength()(1),0);
        Vector3d az(0,0,a->getLength()(2));
        Vector3d al[3] = {ax/2,ay/2 ,az/2 };

        for(int i=0; i<3;i++)
        {   
            el_len(0) = max(abs(xtf.dot(al[i])),el_len(0));
            el_len(1) = max(abs(ytf.dot(al[i])),el_len(1));
            el_len(2) = max(abs(ztf.dot(al[i])),el_len(2));
        }
        length_(0) = (pt - p0).norm() / 2 + el_len(0);
        length_(1) = el_len(1);
        length_(2) = el_len(2);
    }

    inline int getId()
    {
        return id_;
    }
    inline Vector3d getCenter()
    {
        return center_;
    }
    inline Vector3d getLength()
    {
        return length_;
    }
    inline Matrix3d getRotation()
    {
        return rotation_;
    }

    bool isInBox(const Vector3d &pos)
    {
        Vector3d diff = pos - center_;
        Vector3d local_diff = rotation_.transpose() * diff;
        for(size_t i = 0; i < 3; i++)
        {   
            if(abs(local_diff(i)) > length_(i)) // out of box
            {
                return false;
            }
        }
        return true; // The point is inside the box only if it passes all checks
    }
};


class TrackerPool
{
private:
    ros::NodeHandle node_;
    
    
    int pool_size_;
    std::vector<Tracker::Ptr> pool_; // 存储所有的跟踪对象
    std::queue<int> free_ids_; // 存储可用的ID
    // ros::Time current_time_;
    double dt_;
    double missing_tracking_threshold_;




private:



    /**
     * @brief single tracker update
     * @param id id of the tracker in the pool
     * @param length length of the object
     * @param state state of the object
     * @param current_time update time
    */
    void updateTracker(int id, const VectorXd &state, const Vector3d &length, ros::Time current_time);

    /**
     * @brief add new tracker to the pool
     * @return id of the new tracker
     * 
    */
    int addNewTracker(const VectorXd &state, const Vector3d &length, ros::Time current_time);

    void removeTracker(int id);
    /**
     * @brief check if the tracker mis-match time is larger than the threshold
     * @param id id of the tracker
     * @warning should update current_time first
     */
    void checkTracker(int id, ros::Time current_time);


public:
    /**
     * @brief get single tracker
     * @param id id of the tracker
     * @return whether find the tracker successfully
    */
    bool getTracker(int id, Tracker::Ptr &tracker_ptr);
    
    /**
     * @brief single tracker predict, just predict, no update, no covariance matrix calculation
     * @param id id of the tracker in the pool
     * @param tracker tracker object
    */
    void forwardTracker(int id, VectorXd &state, Vector3d &length, ros::Time current_time);

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
     * @brief get all alive tracker
    */
    void getAliveTracker(vector<Tracker::Ptr> &alive_tracker);


    /**
     * @brief predcit all object in the pool in dt
     * @param target_time target time
     * @param current_objects TrackerOutput 
    */
    void forwardPool(vector<TrackerOutput> &predict_objects, ros::Time target_time);
    // void forwardPool(vector<TrackerOutput> &output, double dt);



    /**
     * @brief generate slide box, associate the current tracker with the predicted tracker, and generate slide box
     * @param slide_boxes output box
     * @param target_time target time
    */
    void forwardSlideBox(vector<SlideBox> &slide_boxes, ros::Time target_time);


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
    void getPool(vector<TrackerOutput> &object);


    bool checkCollision(const Vector3d& pos, const ros::Time& pos_time, int& collision_id);
    


    typedef shared_ptr<TrackerPool> Ptr;
};

#endif