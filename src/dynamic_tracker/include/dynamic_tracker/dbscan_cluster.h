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

namespace dbscan
{
/* 初始化，
* -1 : 噪声点
* 0 : 初始化的点
* 1-n : 簇号
*/
    class DBSCANCluster
    {
    private:
        unsigned int minPts_; //圆中至少有多少个点，那么圆心这个点才可以是核心点
        float eps_; // 圆的半径
        unsigned int clusterNum_; //分成了多少个簇
        int min_pts_per_cluster_{1}; //一个簇中最少有多少个点
        int max_pts_per_cluster_{std::numeric_limits<int>::max()}; // 一个簇中最多有多少个点
        // pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    public:


    public:
        DBSCANCluster();
        ~DBSCANCluster(){};

        void init(unsigned int minPts, float eps);

        int radiusSearch(const pcl::PointXYZ &searchPoint,const double radius,std::vector<int> &pointIdxRadiusSearch,std::vector<float> &pointRadiusSquaredDistance);
        void cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud,std::vector<int> &clusterIds);
        // int getTotalpointSize(){return point_cloud_->points.size();}

        //set
        void setCorePointMinPts(unsigned int minPts){minPts_= minPts;}
        void setEpsilonSize(float eps){eps_=eps;}
        void setMinClusterSize(int min_pts_per_cluster){min_pts_per_cluster_=min_pts_per_cluster;}
        void setMaxClusterSize(int max_pts_per_cluster){max_pts_per_cluster_=max_pts_per_cluster_;}


        //get
        int getCorePointMinPts(){return minPts_;}
        int getEpsilonSize(){return eps_;}
        int getMinClusterSize(){return min_pts_per_cluster_;}
        int getMaxClusterSize(){return max_pts_per_cluster_;}

        // pcl::PointCloud<pcl::PointXYZL>::Ptr getLabeledPointCloud(){return pointCloudwithLabel;}
        
        typedef std::unique_ptr<DBSCANCluster> Ptr;
    };
}























#endif