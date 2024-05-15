#include <geometry_msgs/PoseStamped.h>
#include <random>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>

using namespace std;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using Eigen::Quaterniond;

ros::Publisher ellipse_pub, capsule_pub,point_pub;
ros::Timer timer;

struct Cluster
{
    Vector3d center;
    Vector3d velocity;
    Vector3d length;
    ros::Time start_time;
    Cluster(){}
    Cluster(Vector3d c, Vector3d v, Vector3d l)
    {
        center = c;
        velocity = v;
        length = l;
    }
    void display()
    {
        cout << "cluster : " << std::endl;
        cout << "center: " << center.transpose() << endl;
        cout << "velocity: " << velocity.transpose() << endl;
        cout << "length: " << length.transpose() << endl;
    }

};

struct Cube
{
    Vector3d center;
    Vector3d length;
    Quaterniond q;
    Cube(){}
    Cube(Vector3d c, Vector3d l, Quaterniond r)
    {
        center = c;
        length = l;
        q = r;
    }

    void display()
    {
        cout << "capsule : " << std::endl;
        cout << "center: " << center.transpose() << endl;
        cout << "length: " << length.transpose() << endl;
        cout << "rotation: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    
    }
};





visualization_msgs::Marker generateEllipse(const Vector3d &pos, const Vector3d &len, Quaterniond q, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "test";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = pos.z();

    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = len.x() * 2;
    marker.scale.y = len.y() * 2;
    marker.scale.z = len.z() * 2;
    marker.color.r = 1.0; 
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    return marker;
}

visualization_msgs::Marker generateCubeMarker(const Vector3d &pos, const Vector3d &len, Quaterniond q, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "test";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pos.x();
    marker.pose.position.y = pos.y();
    marker.pose.position.z = pos.z();

    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = len.x() * 2;
    marker.scale.y = len.y() * 2;
    marker.scale.z = len.z() * 2;
    marker.color.r = 0.0; 
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    return marker;
}


visualization_msgs::Marker generatePoint(Vector3d p, Eigen::Vector4d color, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "test";
    marker.id = id;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point pt1;
    pt1.x = p.x();
    pt1.y = p.y();
    pt1.z = p.z();
    marker.points.push_back(pt1);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = color(0); 
    marker.color.g = color(1);
    marker.color.b = color(2);
    marker.color.a = color(3);
    return marker;
}

visualization_msgs::Marker generateGradLine(Vector3d p1, Vector3d p2, int id)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "test";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    geometry_msgs::Point pt1;
    pt1.x = p1.x();
    pt1.y = p1.y();
    pt1.z = p1.z();
    marker.points.push_back(pt1);
    geometry_msgs::Point pt2;
    pt2.x = p2.x();
    pt2.y = p2.y();
    pt2.z = p2.z();
    marker.points.push_back(pt2);

    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 1.0; 
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.5;
    return marker;
}


Cube generateCube(const Cluster &input, double t)
{

    Vector3d p0 = input.center;
    Vector3d pt = input.center + input.velocity * t;
    // 1. 扩大椭圆为圆形
    double radius = input.length.maxCoeff();
    double low_r = input.length.minCoeff();
    // Vector3d len = Vector3d::Constant(input.length.maxCoeff());
    
    // 1. 椭球的中心
    Vector3d cube_center = (p0 + pt) / 2;

    Matrix3d rotation;
    Vector3d xtf,ytf,ztf, downward(0,0,-1);
    xtf = (pt - p0).normalized();
    std::cout << "D:" << downward.normalized() << std::endl;
    // cornor case : xtf is parallel to downward
    if(xtf.cross(downward).norm() < 1e-6)
    {
        std::cout << "no" << std::endl;
        xtf(0) += 0.01;
    }
    else
    {
        std::cout << "yes" << std::endl;
        xtf(1) += 0.01;
    }
    
    ytf = xtf.cross(downward).normalized();
    ztf = xtf.cross(ytf).normalized();
    rotation.col(0) = xtf;
    rotation.col(1) = ytf;
    rotation.col(2) = ztf;
    std::cout << "rotation : " << rotation << std::endl;
    Vector3d el_len(0,0,0);
    Vector3d ax(input.length(0),0,0);
    Vector3d ay(0,input.length(1),0);
    Vector3d az(0,0,input.length(2));
    Vector3d al[3] = {ax,ay ,az };

    for(int i=0; i<3;i++)
    {   
        el_len(0) = max(abs(xtf.dot(al[i])),el_len(0));
        el_len(1) = max(abs(ytf.dot(al[i])),el_len(1));
        el_len(2) = max(abs(ztf.dot(al[i])),el_len(2));
        std::cout << "el_len : " << el_len.transpose() << std::endl;
        // el_len(0) += abs(xtf.dot(al[i]));
        // el_len(1) += abs(ytf.dot(al[i]));
        // el_len(2) += abs(ztf.dot(al[i]));
    }
    // el_len(2) += 0.5 * el_len(0) *el_len(0);

    // 2. 椭球的半长轴
    Vector3d cube_len;
    // cube_len(0) = (pt - p0).norm() / 2 + el_len(0) + (pt-p0).norm() * 0.2;
    // cube_len(1) = el_len(1);
    // cube_len(2) = el_len(2) + 0.5 * el_len(0) *el_len(0);
    cube_len(0) = (pt - p0).norm() / 2 + el_len(0);
    cube_len(1) = el_len(1);
    cube_len(2) = el_len(2);
    std::cout << "cube_len : " << cube_len.transpose() << std::endl;
    // double l1 = (pt - p0).norm() / 2 + radius;
    // double l2 = radius;
    // double l3 = l2;

    // double c = (pt - p0).norm() / 2;
    // double a = c + cube_len(0);
    // double b = sqrt(pow(a, 2) - pow(c, 2));

    // std::cout << "l1 : " << l1 << std::endl;
    // std::cout << "l2 : " << l2  << std::endl;
    // std::cout << "a : " << a << std::endl;
    // std::cout << "b : " << b << std::endl;
    // std::cout << "c : " << c << std::endl;
    Cube cube;
    cube.center = cube_center;
    // capsule.length = Vector3d(l1,l2,l2);
    // double scale_factor = radius / low_r;
    // capsule.length = Vector3d(a,b*scale_factor,b*scale_factor);
    cube.length = cube_len;
    cube.q = Quaterniond(rotation);


    return cube;
}

void pushOutofCube(Cube cube, Vector3d &p)
{
    double cost = 0.0;  
    double clearance = 1.1;
    Vector3d x = p - cube.center;
    Vector3d grad(0,0,0);
    Vector3d dist_vec = (p - cube.center);
    Vector3d inv_len = Vector3d(1,1,1).cwiseQuotient(cube.length.cwiseProduct(cube.length));
    
    // = -6 * pow(1 - x.transpose() * theta * x,2) * theta * x;
    double ellip_dist2 = dist_vec(0) * dist_vec(0) * inv_len(0) + dist_vec(1) * dist_vec(1) * inv_len(1) + dist_vec(2) * dist_vec(2) * inv_len(2);
    
    double clearance2 = clearance * clearance;
    double dist2_err = clearance2 - ellip_dist2;
    double dist2_err2 = dist2_err * dist2_err;
    double dist2_err3 = dist2_err2 * dist2_err;
    if(dist2_err3 > 0)
    {
        cost += dist2_err3;
        grad = 3 * dist2_err2 * (-2) * ellip_dist2 * Eigen::Vector3d(dist_vec(0) * inv_len(0), dist_vec(1) * inv_len(1), dist_vec(2) * inv_len(2));
    }

    // grad = x;
    std::cout << "grad : " << grad.transpose() << std::endl;

    p -= grad;

}

bool checkPointInCube(Cube cube, Vector3d p)
{
    Vector3d dist_vec = (p - cube.center);
    Vector3d inv_len = Vector3d(1,1,1).cwiseQuotient(cube.length.cwiseProduct(cube.length));
    double ellip_dist2 = dist_vec(0) * dist_vec(0) * inv_len(0) + dist_vec(1) * dist_vec(1) * inv_len(1) + dist_vec(2) * dist_vec(2) * inv_len(2);
    std::cout << "ellip_dist2 : " << ellip_dist2 << std::endl;
    if(ellip_dist2 <= 1)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// void visCallback(const ros::TimerEvent&)
// {
//     visualization_msgs::MarkerArray ellipse_array, capsule_array;
//     Cluster cluster(Vector3d(0,0,0), Vector3d(1,1,1), ros::Time::now());
//     Capsule capsule = generateCapsule(1, cluster);
//     visualization_msgs::Marker ellipse = generateEllipse(cluster.center, cluster.velocity, 0);
//     visualization_msgs::Marker capsule_marker = generateEllipse(capsule.center, capsule.length, 0);
//     ellipse_array.markers.push_back(ellipse);
//     capsule_array.markers.push_back(capsule_marker);
//     ellipse_pub.publish(ellipse_array);
//     capsule_pub.publish(capsule_array);

// }

Eigen::Vector3d randomPointInCube(Vector3d center, Vector3d length) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> x_rand(center(0) - length(0)/2 ,center(0) + length(0)/2);
    std::uniform_real_distribution<> y_rand(center(1) - length(1)/2 ,center(1) + length(1)/2);
    std::uniform_real_distribution<> z_rand(center(2) - length(2)/2 ,center(2) + length(2)/2);

    Eigen::Vector3d point;
    point << x_rand(gen), y_rand(gen), z_rand(gen);
    return point;
}


class HighPrecisionTimer {
public:
    void start() {
        start_time_ = std::chrono::high_resolution_clock::now();
    }

    void stop() {
        end_time_ = std::chrono::high_resolution_clock::now();
    }

    double elapsedMilliseconds() const {
        return std::chrono::duration<double, std::milli>(end_time_ - start_time_).count();
    }

private:
    std::chrono::high_resolution_clock::time_point start_time_;
    std::chrono::high_resolution_clock::time_point end_time_;
};

int main(int argc, char ** argv)
{
    /*
    ros::init(argc, argv, "test_stCapsule");
    ros::NodeHandle nh("~");

    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
    // timer = nh.createTimer(ros::Duration(0.05), visCallback);
    ellipse_pub = nh.advertise<visualization_msgs::MarkerArray>("ellipse", 1);
    capsule_pub = nh.advertise<visualization_msgs::MarkerArray>("capsule", 1); 
    point_pub = nh.advertise<visualization_msgs::MarkerArray>("point", 1);
    double t=1;
    Vector3d v = Vector3d(1,1,0);
    Cluster cl(Vector3d(0,2,2), v, Vector3d(0.2,0.3,0.4));
    Cluster c2(Vector3d(0,2,2) + v * t, Vector3d(1,1,0), Vector3d(0.2,0.3,0.4));
    HighPrecisionTimer timer;
    timer.start();
    Cube cb = generateCube(cl, t);
    timer.stop();
    std::cout << "Elapsed time: " << timer.elapsedMilliseconds() << " ms" << std::endl;



    visualization_msgs::MarkerArray ellipse_array, capsule_array,point_array;

    

    visualization_msgs::Marker ellipse_1 = generateEllipse(cl.center, cl.length, Quaterniond::Identity(), 0);
    visualization_msgs::Marker ellipse_2 = generateEllipse(c2.center, c2.length, Quaterniond::Identity(), 1);

    visualization_msgs::Marker cube_marker = generateCubeMarker(cb.center, cb.length, cb.q, 0);

    ellipse_array.markers.push_back(ellipse_1);
    ellipse_array.markers.push_back(ellipse_2);
    capsule_array.markers.push_back(cube_marker);
    cl.display();
    c2.display();
    cb.display();
    // 在cube内部随机生成一个点


    Vector3d p = randomPointInCube(cb.center, cb.length);
    std::cout << "p : " << p.transpose() << std::endl;
    int count = 0;
    visualization_msgs::Marker ori_point = generatePoint(p,Eigen::Vector4d(1.0,0.0,0.0,1.0),count++);
    point_array.markers.push_back(ori_point);
    while(checkPointInCube(cb, p))
    {
        std::cout << "herer " << std::endl;
        Vector3d ori = p;
        pushOutofCube(cb, p);    
        visualization_msgs::Marker push_point = generatePoint(p,Eigen::Vector4d(0.0,1.0,1.0,1.0), count++);
        visualization_msgs::Marker push_line = generateGradLine(ori, p , count ++);
        point_array.markers.push_back(push_point);
        point_array.markers.push_back(push_line);
        if(count > 200)
        {
            break;
        }
    }

    geometry_msgs::PoseStamped pose = geometry_msgs::PoseStamped();
    pose.header.frame_id = "world";
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = cb.center.x();
    pose.pose.position.y = cb.center.y();
    pose.pose.position.z = cb.center.z();
    pose.pose.orientation.x = cb.q.x();
    pose.pose.orientation.y = cb.q.y();
    pose.pose.orientation.z = cb.q.z();
    pose.pose.orientation.w = cb.q.w();


    ros::Rate loop_rate(20);
    while(ros::ok())
    {
        pose_pub.publish(pose); 
        ellipse_pub.publish(ellipse_array);
        capsule_pub.publish(capsule_array);
        point_pub.publish(point_array);
        loop_rate.sleep();
    }

    // ros::spin();
    */

    Vector3d p0(0,0,0);
    Vector3d l0(1,1,1);
    Quaterniond q0(1,0,0,0);
    



    return 0;
}