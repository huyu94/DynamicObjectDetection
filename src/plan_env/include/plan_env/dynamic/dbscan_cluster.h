#ifndef DBSCAN_H_
#define DBSCAN_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <memory>

#define UN_PROCESSED 0
#define PROCESSING 1
#define PROCESSED 2


/* 初始化，
* -1 : 噪声点
* 0 : 初始化的点
* 1-n : 簇号
*/
class DBSCANCluster
{
private:
    //圆中至少有多少个点，那么圆心这个点才可以是核心点
    unsigned int pts_;
    // 圆的半径
    float min_eps_;
    float max_eps_;
    float default_eps_;
    static const float rad;
    // unsigned int clusterNum_; //分成了多少个簇
    // int min_pts_per_cluster_{1}; //一个簇中最少有多少个点
    // int max_pts_per_cluster_{std::numeric_limits<int>::max()}; // 一个簇中最多有多少个点
    // pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    void core(unsigned int minPoints, std::vector<float>& eps_list,std::vector<pcl::PointIndices>& cluster_indices);
    bool checkInputs(float epsilon);
    void boundEpsilon(float &epsilon);
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_;


public:


public:
    DBSCANCluster(unsigned int pts,float min_eps,float max_eps,float default_eps):pts_(pts),min_eps_(min_eps),max_eps_(max_eps),default_eps_(default_eps){};
    DBSCANCluster():pts_(5),min_eps_(0.5),max_eps_(2.0),default_eps_(1.0){};
    ~DBSCANCluster(){};

    void setInputCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);
    int radiusSearch(const pcl::PointXYZI &searchPoint,const double radius,std::vector<int> &pointIdxRadiusSearch,std::vector<float> &pointRadiusSquaredDistance);
    void run(std::vector<pcl::PointIndices>& cluster_indices,bool adptive);
    // int getTotalpointSize(){return point_cloud_->points.size();}

    //set
    void setCorePointPts(unsigned int minPts){pts_ = minPts;}
    void setMinEpsilonSize(float min_eps){min_eps_=min_eps;}
    void setMaxEpsilonSize(float max_eps){max_eps_=max_eps;}
    // void setMinClusterSize(int min_pts_per_cluster){min_pts_per_cluster_=min_pts_per_cluster;}
    // void setMaxClusterSize(int max_pts_per_cluster){max_pts_per_cluster_=max_pts_per_cluster_;}


    //get
    int getCorePointPts(){return pts_;}
    int getMinEpsilonSize(){return min_eps_;}
    int getMaxEpsilonSize(){return max_eps_;}
    // int getMinClusterSize(){return min_pts_per_cluster_;}
    // int getMaxClusterSize(){return max_pts_per_cluster_;}

    // pcl::PointCloud<pcl::PointXYZL>::Ptr getLabeledPointCloud(){return pointCloudwithLabel;}
    
    typedef std::unique_ptr<DBSCANCluster> Ptr;
};


#endif