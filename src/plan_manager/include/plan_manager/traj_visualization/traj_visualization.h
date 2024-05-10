#ifndef _TRAJ_VISUALIZATION_H_
#define _TRAJ_VISUALIZATION_H_


#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>

using Vector3d = Eigen::Vector3d;


class Color : public std_msgs::ColorRGBA
{
public:
    Color() : std_msgs::ColorRGBA(){}
    Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
    Color(double red, double green, double blue, double alpha) : Color()
    {
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



class VisualRviz
{

public:
    VisualRviz(const ros::NodeHandle& nh);
    void visualizeCollision(const Vector3d& collision, ros::Time local_time);

    void visualizeStartAndGoal(const Vector3d& start, const Vector3d& goal, ros::Time local_time);


typedef std::shared_ptr<VisualRviz> Ptr;


private:
    VisualRviz();
    ros::NodeHandle nh_;

    /* pubisher */
}


#endif