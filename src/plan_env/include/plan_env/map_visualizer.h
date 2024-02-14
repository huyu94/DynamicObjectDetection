#ifndef _MAP_VISUALIZER_H_
#define _MAP_VISUALIZER_H_

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>


class Color : public std_msgs::ColorRGBA {
 public:
  Color() : std_msgs::ColorRGBA() {}
  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
  Color(double red, double green, double blue, double alpha) : Color() {
    r = red;
    g = green;
    b = blue;
    a = alpha;
  }

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 0.7, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color SteelBlue() { return Color(0.4, 0.7, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};



struct VisualCluster
{
  Eigen::Vector3d centroid;
  Eigen::Vector3d length;
  Eigen::Vector3d min_bound,max_bound;

  VisualCluster(Eigen::Vector3d c,Eigen::Vector3d l,Eigen::Vector3d min,Eigen::Vector3d max) : 
                    centroid(Eigen::Vector3d::Zero()), length(Eigen::Vector3d::Zero()), 
                    min_bound(Eigen::Vector3d::Zero()), max_bound(Eigen::Vector3d::Zero()) {}
};


class MapVisualizer
{
public:
    MapVisualizer();
    MapVisualizer(const ros::NodeHandle& node);

    void visualizeClusterResult(std::vector<VisualCluster> &visual_cluster);
    void visualizeSegmentationResult();
    void visualizeKMResult();
    void visualizeMovingObjectBox();
    void visualizeMovingObjectTraj();

    typedef std::shared_ptr<MapVisualizer> Ptr;

private:
    ros::NodeHandle node_;
    ros::Publisher cluster_result_pub_;
    ros::Publisher segmentation_result_pub_;
    ros::Publisher km_result_pub_;
    ros::Publisher moving_object_box_pub_;
    ros::Publisher moving_object_traj_pub_;

};



#endif