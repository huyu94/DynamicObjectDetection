#ifndef _TRAJ_VISUALIZATION_H_
#define _TRAJ_VISUALIZATION_H_


#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <algorithm>

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
    static const Color Black() { return Color(0.0, 0.0, 0.0); } // 
    static const Color Gray() { return Color(0.5, 0.5, 0.5); } // 优化失败轨迹
    static const Color Red() { return Color(1.0, 0.0, 0.0); } // 优化成功轨迹
    static const Color Green() { return Color(0.0, 0.7, 0.0); }
    static const Color Blue() { return Color(0.0, 0.0, 1.0); } // Kino traj color 
    static const Color SteelBlue() { return Color(0.4, 0.7, 1.0); }
    static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
    static const Color Orange() { return Color(1.0, 0.5, 0.0); }
    static const Color Purple() { return Color(0.5, 0.0, 1.0); }
    static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
    static const Color Teal() { return Color(0.0, 1.0, 1.0); }
    static const Color Pink() { return Color(1.0, 0.0, 0.5); }
    static const Color Cyan() { return Color(0.0, 1.0, 1.0); }
    static const Color Magenta() { return Color(1.0, 0.0, 1.0); }
    static const Color Gold() { return Color(1.0, 0.84, 0.0); }
    static const Color Olive() { return Color(0.5, 0.5, 0.0); }
    static const Color Lime() { return Color(0.0, 1.0, 0.0); }
    static const Color Indigo() { return Color(0.29, 0.0, 0.51); }
};



class VisualRviz
{

public:
    VisualRviz();

    void init(const ros::NodeHandle& nh);

    void visualizeCollision(const Vector3d& collision, ros::Time local_time);

    void visualizeStartAndGoal(const Vector3d& start, const Vector3d& goal, ros::Time local_time);

    void visualizeMultiTopoTrajs(const std::vector<std::vector<Vector3d>>& topo_trajs, std::vector<bool>& success, ros::Time local_time);

    void visualizeKinodynamicTraj(const std::vector<Vector3d>& kino_traj, ros::Time local_time);
    
    void visualizeOptimalTraj(const std::vector<Vector3d>& optimal_traj, ros::Time local_time);


typedef std::shared_ptr<VisualRviz> Ptr;


private:


    ros::NodeHandle nh_;

    std::vector<Color> topoColorMap = { Color::Purple(), Color::Chartreuse(), Color::Teal(),Color::Pink(), Color::Cyan(), 
                                        Color::Magenta(), Color::Gold(), Color::Olive(), Color::Lime(), Color::Indigo()};


    void visualizeMarkerList(ros::Publisher& pub, const std::vector<Vector3d>& list, double scale,
                             Color color, int id, bool show_sphere /* = true */);

    /* pubisher */
    ros::Publisher pub_multi_topo_trajs_; // id [0,9] 
    std::vector<int> multi_topo_trajs_id = {0,1,2,3,4,5,6,7,8,9}; 
    ros::Publisher pub_kino_traj_; // id 10;
    const int kino_traj_id_ = 10;
    ros::Publisher pub_opti_traj_; // id 11;
    const int opti_traj_id_ = 11;
    ros::Publisher pub_collision_;
    const int collision_id_ = 12;
    ros::Publisher pub_start_and_goal_;
    const int start_and_goal_id_ = 13;

};

#endif