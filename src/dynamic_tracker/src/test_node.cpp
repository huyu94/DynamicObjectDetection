#include <iostream>
#include <Eigen/Eigen>
#include <munkres.h>
#include <time.h>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
using namespace std;

void func(float x,float y, float z){
    cout << x << " " << y << " " << z << endl;

}
int findPointPyramidHorizontalIndexInSensorFrame(float x,float y,float z)
{

}
int findPointPyramidVerticalIndexInSensorFrameV1(float &x,float &y,float &z)
{
    float sinf = z / sqrt(pow(x,2)+pow(y,2)+pow(z,2));
    float f = asin(sinf);
    int verticalIndex = std::floor(f / 0.1);
}

int findPointPyramidVerticalIndexInSensorFrameV2(float x,float y,float z)
{
    float vertical_rad = atan2(z,sqrtf(powf(x,2) + powf(y,2)));
    int vertical_index = floor(vertical_rad + 0.1 / 0.1);
}

int main(int argc, char** argv){

    ros::Time::init();
    float x = 1.0;
    float y = 2.0;
    float z = 3.0;

    ros::Time t1 = ros::Time::now();
    findPointPyramidVerticalIndexInSensorFrameV1(x,y,z);
    ROS_INFO("v1 cost time: %llu", (ros::Time::now() - t1).toNSec());

    ros::Time t2 = ros::Time::now();
    // findPointPyramidVerticalIndexInSensorFrameV2(x,y,z);
    
    ROS_INFO("v2 cost time: %llu", (ros::Time::now() - t2).toNSec());




    return 0;

}