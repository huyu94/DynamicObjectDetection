#include <traj_utils/traj_utils.h>
#include <ros/ros.h>
#include <queue>
#include <string>
#include "traj_visualization.h"


VisualRviz::VisualRviz(const ros::NodeHandle& nh) : nh_(nh)
{
    // publisher
    pub_collision_ = nh_.advertise<visualization_msgs::Marker>("collision", 1);
    pub_start_goal_ = nh_.advertise<visualization_msgs::MarkerArray>("start_goal", 1);
}


void VisualRviz::visualizeCollision(const Vector3d& collision, ros::Time local_time)
{
    if(pub_collision_.getNumSubscribers == 0)
    {
        return ;
    }
    visualization_msgs::Marker collision_point;
    geometry_msgs::Point p;
    p.x = collision(0);
    p.y = collision(1);
    p.z = collision(2);
    collision_point.points.push_back(p);
    collision_point.header.frame_id = "map";
    collision_point.header.stamp = local_time;
    collision_point.ns = "collision";
    collision_point.action = visualization_msgs::Marker::ADD;
    collision_point.life_time = ros::Duration(0); // always exist
    collision_point.pose.orientation.w = 1.0;
    collision_point.id = 400;
    collision_point.type = visualization_msgs::Marker::SPHERE_LIST;
    collision_point.scale.x = 0.2;
    collision_point.scale.y = 0.2;
    collision_point.scale.z = 0.2;
    collision_point.color = Color::Red();
    collision_point.color.a = 1.0;
    pub_collision_.publish(collision_point);    
}

void VisualRviz::visualizeStartAndGoal(const Vector3d &start, const Vector3d &goal, ros::Time local_time)
{
    if(pub_start_and_goal_.getNumSubscribers == 0)
    {
        return;
    }
    visualization_msgs::Marker pos_point;

    geometry_msgs::Point p;
    p.x = start[0];
    p.y = start[1];
    p.z = start[2];
    pos_point.points.push_back(p);
    p.x = goal[0];
    p.y = goal[1];
    p.z = goal[2];
    pos_point.points.push_back(p);

    pos_point.header.frame_id = "map";
    pos_point.header.stamp = local_time;
    pos_point.ns = "s-g";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.life_time = ros::Duration(0); // always exist
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 11;
    pos_point.type = visualization_msgs::Marker::POINTS;
    pos_point.scale.x = 0.4;
    pos_point.scale.y = 0.4;
    pos_point.color = Color::Red();
    pos_point.color.a = 1.0;

    pub_start_and_goal_.publish(pos_point);

}

void VisualRviz::visualizeMultiTopoTrajs(const std::vector<std::vector<Vector3d>> &topo_trajs, ros::Time local_time)
{
    if(pub_multi_topo_trajs_.getNumSubscribers == 0)
    {
        return;
    }

    static int last_nums = 0;
    for(int id=0; id<last_nums; id++)
    {
        Vector4d color(0,0,0,0);
        vector<Vector3d> blank;
        // visualizeMarkerList(blank, id, color, local_time);
        ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for(int id=0; id<topo_trajs.size(); id++)
    {
        Vector4d color;
        if(success[id])
        {
            color = colorMap[id];
        }
    }

}
