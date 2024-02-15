#include <cmath>
#include "plan_env/dynamic/kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter(){}
KalmanFilter::~KalmanFilter(){}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, 
                       MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in)
{
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
    I_ = MatrixXd::Identity(x_.size(), x_.size());
}


Eigen::VectorXd KalmanFilter::forward(double dt)
{
    // Use A matrix to predict the state, which integrated time delta_T,
    // and it doest not influence the process matrix F_;
    Eigen::MatrixXd A = F_;
    A(0, 3) = dt;
    A(1, 4) = dt;
    A(2, 5) = dt;
    return A * x_;
}

Eigen::VectorXd KalmanFilter::getState()
{
    return x_;
}


void KalmanFilter::UpdateEKF(const Eigen::VectorXd &z, double dt)
{
    Predict(dt);
    Update(z);
}

/*-----------------------private------------------------*/

void KalmanFilter::Predict(double dt)
{
    F_(0,3) = dt;
    F_(1,4) = dt;
    F_(2,5) = dt;

    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const Eigen::VectorXd &z)
{
    VectorXd z_pred = H_ * x_;

    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;

    // new state
    x_ = x_ + (K * y);
    P_ = (I_ - K * H_) * P_;
}



