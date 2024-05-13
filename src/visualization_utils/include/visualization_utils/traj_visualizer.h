#ifndef _TRAJ_VISUALIZATION_H_
#define _TRAJ_VISUALIZATION_H_


#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <algorithm>

#include <visualization_utils/color.hpp>

using Vector3d = Eigen::Vector3d;



class TrajVisualizer
{

public:
    TrajVisualizer();

    void init(const ros::NodeHandle& nh);

    void visualizeCollision(const Vector3d& collision, ros::Time local_time);

    void visualizeStartAndGoal(const Vector3d& start, const Vector3d& goal, ros::Time local_time);

    void visualizeMultiTopoTrajs(const std::vector<std::vector<Vector3d>>& topo_trajs, std::vector<bool>& success, ros::Time local_time);

    void visualizeKinodynamicTraj(const std::vector<Vector3d>& kino_traj, ros::Time local_time);
    
    void visualizeOptimalTraj(const std::vector<Vector3d>& optimal_traj, ros::Time local_time);


typedef std::shared_ptr<TrajVisualizer> Ptr;


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