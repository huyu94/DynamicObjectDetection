#include <iostream>
#include <Eigen/Eigen>
#include <munkres.h>
#include <time.h>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
using namespace std;


int half_fov_vertical_ = 50;
int half_fov_horizontal = 180;
int observation_pyramid_num_horizontal_ = 180;
int observation_pyramid_num_vertical_ = 50;
double one_degree_rad_ = M_PI / 180;

bool inPyramidsAreaInSensorFrame(float x, float y, float z)
{
    float vertical_degree = atan2(z,sqrtf(powf(x,2) + powf(y,2))) * 180 / M_PI;
    if(vertical_degree > -half_fov_vertical_ && vertical_degree <= half_fov_vertical_)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int findPointPyramidHorizontalIndexInSensorFrame(float x, float y, float z)
{
    // float horizontal_rad = fmod(atan2(y,x) + 2 * M_PI, 2*M_PI);

    // int horizontal_index = std::floor(horizontal_rad / one_degree_rad_);
    int horizontal_index = std::floor(fmod(atan2(y,x) + 2 * M_PI, 2*M_PI) / one_degree_rad_);

    if(horizontal_index >= 0 && horizontal_index < 2 * observation_pyramid_num_horizontal_)
    {
        return horizontal_index;
    }
    ROS_INFO("!!!!!! Please use Function ifInPyramidsArea() to filter the points first before using findPointPyramidHorizontalIndex()");
    return -1;
}

int findPointPyramidVerticalIndexInSensorFrame(float x,float y,float z)
{
    float vertical_rad = atan2(z,sqrtf(powf(x,2) + powf(y,2)));
    int vertical_index = floor((vertical_rad + half_fov_vertical_ * M_PI / 180) / one_degree_rad_);
    if(vertical_index >= 0 && vertical_index < 2 * observation_pyramid_num_vertical_ )
    {
        return vertical_index;
    }
    ROS_INFO("vertical index : %d",vertical_index);
    ROS_INFO("!!!!!! Please use Function ifInPyramidsAreaInSensorFrame() to filter the points first before using findPyramidVerticalIndexInSensorFrame()");
    return -1;
}


int main(int argc, char** argv){

/* 1. sensor point -> world point : */
    // Eigen::Vector3d sensor_point(-2.0, 4.0, 0.0);
    // // Eigen::Vector3d world_point(1.0, 2.0, 0);

    // // 定义传感器的位置和姿态
    // Eigen::Vector3d sensor_position(5, 4, 0);
    // Eigen::Quaterniond sensor_orientation;  // 默认姿态，可根据实际情况修改
    // sensor_orientation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) * 
	// 						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * 
	// 						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    // // Eigen::Vector3d sensor_point;
    // Eigen::Vector3d world_point;

    // world_point = sensor_orientation * sensor_point + sensor_position;
    // // sensor_point = sensor_orientation * (sensor_point - sensor_position);

    // Eigen::Vector3d rotated_sensor_point;
    // rotated_sensor_point = sensor_orientation.inverse() * (world_point - sensor_position);
    // // Eigen::Vector3d rotated_world_point;
    // // rotated_world_point = sensor_orientation * sensor_point + sensor_position;

    // std::cout << "传感器坐标系下的点P: " << sensor_point.transpose() << std::endl;
    // std::cout << "世界坐标系下的点P: " << world_point.transpose() << std::endl;
    // std::cout << "旋转后，传感器坐标系下的点P: " << rotated_sensor_point.transpose() << std::endl;


    /* 2. world point -> sensor point */    
    // Eigen::Vector3d world_point(1,2,0);
    // Eigen::Vector3d world_position(-4,5,0);

    // Eigen::Quaterniond world_orientation =  Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()) * 
    //                                         Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * 
    //                                         Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

    // Eigen::Vector3d sensor_point;
    

    // sensor_point = world_orientation * world_point + world_position;

    // std::cout << "世界坐标系下的点P: " << world_point.transpose() << std::endl;
    // std::cout << "传感器坐标系下的点P: " << sensor_point.transpose() << std::endl;

    /* 3. try affine */

    // Eigen::Vector3d sensor_point(-2.0, 4.0, 0.0);

    // // 定义传感器的位置和姿态
    // Eigen::Vector3d sensor_position(5, 4, 0);
    // Eigen::Quaterniond sensor_orientation;  // 默认姿态，可根据实际情况修改
    // sensor_orientation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) * 
	// 						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * 
	// 						Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

    // Eigen::Affine3d transform;
    // transform.linear() = sensor_orientation.toRotationMatrix();
    // transform.translation() = sensor_position;
    // Eigen::Vector3d world_point;

    // // world_point = sensor_orientation * sensor_point + sensor_position;
    // world_point = transform * sensor_point;

    // Eigen::Vector3d rotated_sensor_point;
    // // rotated_sensor_point = sensor_orientation.inverse() * (world_point - sensor_position);
    // rotated_sensor_point = transform.inverse() * world_point;


    // std::cout << "传感器坐标系下的点P: " << sensor_point.transpose() << std::endl;
    // std::cout << "世界坐标系下的点P: " << world_point.transpose() << std::endl;
    // std::cout << "旋转后，传感器坐标系下的点P: " << rotated_sensor_point.transpose() << std::endl;


    /* 4. quanterniond inverse rotation */
//     Eigen::Quaterniond sensor_orientation = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) * 
// 							Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * 
// 							Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

//     Eigen::Quaterniond sensor_orientation_inverse = Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitZ()) * 
// 							Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * 
// 							Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());

//     // std::cout << (sensor_orientation.inverse() == sensor_orientation_inverse) ;

// std::cout << sensor_orientation.inverse().toRotationMatrix() << std::endl;
// std::cout << sensor_orientation_inverse.toRotationMatrix() << std::endl;

    /* 5. core */
    // using namespace Eigen;
    // Vector3d world_point = Vector3d(1,2,0);
    // Vector3d sensor_position = Vector3d(5,4,0);
    // Quaterniond sensor_rotation = AngleAxisd(M_PI_2,Vector3d::UnitZ())*
    //                                 AngleAxisd(0.0, Vector3d::UnitY()) * 
    //                                 AngleAxisd(0.0, Vector3d::UnitX());

    // Affine3d transform;
    // transform.translation() << sensor_position;
    // transform.linear() = sensor_rotation.toRotationMatrix();

    // Vector3d sensor_point =  transform.inverse() * world_point;

    // Vector3d rotated_world_point = transform * sensor_point;
    // std::cout << "世界坐标系下的点P: " << world_point.transpose() << std::endl;
    // std::cout << "传感器坐标系下的点P: " << sensor_point.transpose() << std::endl;
    // std::cout << "旋转后,世界坐标系下的点P: " << rotated_world_point.transpose() << std::endl;

    

    // double radius = 5.0;
    // Eigen::Vector3d center(0,0,0);
    // for(double i = -M_PI;i<M_PI;i+=M_PI/200)
    // {
    //     for(double j=-M_PI/2;j<M_PI/2;j+=M_PI/200)
    //     {
    //         double x = radius * cos(i) * cos(j);
    //         double y = radius * sin(i) * cos(j);
    //         double z = radius * sin(j);
    //         if(inPyramidsAreaInSensorFrame(x,y,z))
    //         {
    //             std::cout << "x: " << x << " y: " << y << " z: " << z << std::endl;
    //             std::cout << "horizontal index: " << findPointPyramidHorizontalIndexInSensorFrame(x,y,z) << std::endl;
    //             std::cout << "vertical index: " << findPointPyramidVerticalIndexInSensorFrame(x,y,z) << std::endl;
    //             std::cout << "i:" << i * 180 / M_PI << ", j: " << j * 180 / M_PI << std::endl;
    //         }
    //     }
    
    // }


    /* test vertical index*/
    // int radius = 5;
    // Eigen::Vector3d center(0,0,0);
    // double rad_7 = -7 * M_PI / 180;
    // for(double i=-M_PI/2;i<M_PI/2;)
    // {
    //     double x = center[0] + radius * cos(i);
    //     double z = center[1] + radius * sin(i);
    //     int vertical_index = floor((atan2(z,x) - rad_7) / M_PI * 180);
    //     std::cout << i << " " << vertical_index << std::endl;
    //     i+=M_PI/200;
    // }
    // std::cout << rad_7 << std::endl;

    /* test horizontal index */
    int radius = 5;
    Eigen::Vector2d center(0,0); 
    for(double i=-M_PI;i<M_PI;i+=M_PI/200)
    {
        double x = center[0] + radius * cos(i);
        double y = center[1] + radius * sin(i);
        // int horizontal_index = std::floor(fmod(atan2(y,x) + 2 * M_PI, 2*M_PI) / one_degree_rad_);
        int horizontal_index = findPointPyramidHorizontalIndexInSensorFrame(x,y,0);
        if(horizontal_index < 0 || horizontal_index >= 360)
        {
            std::cout << "horizontal index: " << horizontal_index << std::endl;
            std::cout << "i : " << i / M_PI * 180 << std::endl;
        }
        // std::cout << i << " " << horizontal_index << std::endl;
    }
    

    return 0;

}