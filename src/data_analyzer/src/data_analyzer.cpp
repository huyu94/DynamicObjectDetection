#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <string>
#include <numeric>
#include <vector>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>
using namespace std;

class Logger
{
private:
    std::vector<Eigen::Vector3d> path_vec;
    vector<double> vel_vec;
    ros::Time start_time; 

public:
    Logger(){
    }

    void init()
    {
        start_time = ros::Time::now();
    }
    void setPath(vector<Eigen::Vector3d> data)
    {
        this->path_vec = data;
    };
    void addVel(double data)
    {
        this->vel_vec.push_back(data);
    }
    ~Logger()
    {
        
        std::string filename = "data.txt";
        string log_dir = ros::package::getPath("data_analyzer") + "/logs/";
        std::ofstream file;
        file.open(log_dir + filename, std::ios::out);
        if (file.is_open())
        {
            std::cout << "Writing to file" << std::endl;
            // path len
            double len = 0.0;
            for(int i = 1; i< path_vec.size(); i++)
            {
                len += (path_vec[i] - path_vec[i-1]).norm();
            }
            std::cout << "Path length: " << len << std::endl;
            // average velocity
            double average_vel = std::accumulate(vel_vec.begin(), vel_vec.end(), 0.0);
            // double vel_sum = 0.0;
            // // for(auto t : vel_vec)
            // // {
            // //     file << t << std::endl;
            // // }
            average_vel /= vel_vec.size();
            std::cout << "Average velocity: " << average_vel << std::endl;
            file << len << std::endl;
            file << average_vel << std::endl; 

            double planning_time = (ros::Time::now() - start_time).toSec();
            planning_time -= 2.0;
            std::cout << "planning_time: " << planning_time << std::endl;

            file << planning_time << std::endl;
            file.close();
        }
        else
        {
            ROS_ERROR("Unable to open file");
        }
    }
};


Logger logger;

void velCallback(const visualization_msgs::Marker::ConstPtr &msg)
{
    if(msg->scale.x == 0.0)
    {
        return;
    }
    static bool init = false;
    if(!init)
    {
        logger.init();
        init = true;
    }
    double vel = msg->scale.x;
    logger.addVel(vel);
}

void pathCallback(const nav_msgs::Path::ConstPtr &msg)
{

    vector<Eigen::Vector3d> path;
    for(int i=0;i<msg->poses.size();i++)
    {
        path.push_back(Eigen::Vector3d(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z));
    }
    logger.setPath(path);
}


int main(int argc, char **argv)
{

    ros::init(argc, argv, "data_analyzer");
    ros::NodeHandle nh("~");

    ros::Subscriber vel_sub = nh.subscribe<visualization_msgs::Marker>("vel", 1, velCallback);
    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>("path", 1, pathCallback);

    ros::spin();

    return 0;
}