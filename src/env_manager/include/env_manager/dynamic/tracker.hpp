#ifndef TRACKER_H_
#define TRACKER_H_

#include <memory>
#include <iostream>
#include <ros/ros.h>

#include "env_manager/dynamic/kalman_filter.h"

using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

using std::shared_ptr;


class Tracker
{
private:
    KalmanFilter kf_;
    bool is_alive_;
    int id_;
    Vector3d length_;
    ros::Time udpate_time_;

public:
    /**
     * @brief constructor
     * @param measurment initial measurment
     * @param length length of the object
     * @param id id of the object
    */
    Tracker(VectorXd measurment,Vector3d length,int id, ros::Time current_time):
            is_alive_(true),kf_(),length_(length),id_(id),udpate_time_(current_time)
    {
        Eigen::MatrixXd P(6,6),F(6,6),Q(6,6),H(6,6),R(6,6);

        /* TODO : need to modify parameters */
        // process matrix 
        F << 1,     0,      0,      1,      0,      0, 
             0,     1,      0,      0,      1,      0, 
             0,     0,      1,      0,      0,      1, 
             0,     0,      0,      1,      0,      0,
             0,     0,      0,      0,      1,      0, 
             0,     0,      0,      0,      0,      1;
        
        // mesurement matrix 
        H << 1,     0,      0,      0,      0,      0, 
             0,     1,      0,      0,      0,      0, 
             0,     0,      1,      0,      0,      0, 
             0,     0,      0,      1,      0,      0,
             0,     0,      0,      0,      1,      0, 
             0,     0,      0,      0,      0,      1;

        // process covariance matrix
        P << 1,     0,      0,      0,      0,      0, 
             0,     1,      0,      0,      0,      0, 
             0,     0,      1,      0,      0,      0, 
             0,     0,      0,      100,    0,      0,
             0,     0,      0,      0,      100,    0, 
             0,     0,      0,      0,      0,      100;
        
        // Process covariance matrix
        Q << 0.05,  0,      0,      0,      0,      0, 
             0,     0.05,   0,      0,      0,      0, 
             0,     0,      0.05,   0,      0,      0, 
             0,     0,      0,      0.05,   0,      0,
             0,     0,      0,      0,      0.05,   0, 
             0,     0,      0,      0,      0,      0.05;


        // Measurement covariance matrix
        R << 1,     0,      0,      0,      0,      0, 
             0,     1,      0,      0,      0,      0, 
             0,     0,      1,      0,      0,      0, 
             0,     0,      0,      1,      0,      0,
             0,     0,      0,      0,      1,      0, 
             0,     0,      0,      0,      0,      1;
             
        kf_.Init(measurment,P,F,H,R,Q);

    };
    ~Tracker(){};

    /**
     * @brief just used to predict future status, no update and covariance matrix calculation 
     * @param dt prediction time shift  
    */
    VectorXd forward(const ros::Time &target_time)
    {
        double dt = (target_time - udpate_time_).toSec();
        return kf_.forward(dt);
    }

    /**
     * @brief include predict step and update step of ekf
     * @param measurment measurment data
     * @param current_time current time
    */
    void update(const VectorXd &measurment,const ros::Time &current_time, const Vector3d &length)
    {
        double dt = (current_time - udpate_time_).toSec();
        // std::cout << "dt : " << dt << std::endl; 
        // ROS_INFO("dt : %lf ", dt);
        kf_.UpdateEKF(measurment,dt);
        udpate_time_ = current_time;
        length_ = length;
    }

    Vector3d getLength()
    {
        return length_;
    }

    bool isAlive()
    {
        return is_alive_;
    }

    int getId()
    {
        return id_;
    }

    VectorXd getState()
    {
        return kf_.getState();
    }

    Vector3d getPos()
    {
        VectorXd state = kf_.getState();
        return Vector3d(state(0),state(1),state(2));
    }

    Vector3d getVel()
    {
        VectorXd state = kf_.getState();
        return Vector3d(state(3),state(4),state(5));
    }


    Vector3d getAxis()
    {
        return this->length_;
    }

    ros::Time getUpdateTime()
    {
        return udpate_time_;
    }

    typedef shared_ptr<Tracker> Ptr;

};

#endif