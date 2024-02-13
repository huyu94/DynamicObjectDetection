#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"


class KalmanFilter
{
private:
    // state vector
    Eigen::VectorXd x_;

    // state covariance matrix
    Eigen::MatrixXd P_;

    // state transition matrix
    Eigen::MatrixXd F_;

    // process covariance matrix
    Eigen::MatrixXd Q_;

    // measurement matrix
    Eigen::MatrixXd H_;

    // measurement covariance matrix
    Eigen::MatrixXd R_;

    // identity matrix
    Eigen::MatrixXd I_;

    /**
    * Predict Predicts the state and the state covariance
    * using the process model
    * @param dt Time between k and k+1 in s
    */
    void Predict(double dt);

    /**
     * Updates the state by using standard Kalman Filter equations
     * @param z The measurement at k+1
    */
    void Update(const Eigen::VectorXd &z);



public:
    /**
     * Constructor
     */
    KalmanFilter();

    /**
     * Destructor
     */
    virtual ~KalmanFilter();

    /**
     * Init Initializes Kalman filter
     * @param x_in Initial state
     * @param P_in Initial state covariance
     * @param F_in Transition matrix
     * @param H_in Measurement matrix
     * @param R_in Measurement covariance matrix
     * @param Q_in Process covariance matrix    
    */
    void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in, 
              Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in, Eigen::MatrixXd &Q_in);


    /**
     * @brief just forward the state, no update and covariance matrix calculation
     * @param dt time interval between k and k+1
    */
    VectorXd forward(double dt);


    /**
     * Updates the state by using Extended Kalman Filter equations
     * @param z The measurement at k+1
     * @param dt time interval 
    */
    void UpdateEKF(const Eigen::VectorXd &z,double dt);



};



#endif