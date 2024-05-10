#include <plan_manager/traj_visualization/traj_visualization.h>
#include <ros/ros.h>
#include <queue>
#include <string>
#include "traj_visualization.h"


VisualRviz::VisualRviz(const ros::NodeHandle& nh) : nh_(nh)
{
    // publisher
    pub_collision_ = nh_.advertise<visualization_msgs::Marker>("collision", 1);
    pub_start_and_goal_ = nh_.advertise<visualization_msgs::MarkerArray>("start_goal", 1);

}


void VisualRviz::visualizeCollision(const Vector3d& collision, ros::Time local_time)
{
    if(pub_collision_.getNumSubscribers() == 0)
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
    collision_point.lifetime = ros::Duration(0); // always exist
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
    if(pub_start_and_goal_.getNumSubscribers() == 0)
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
    pos_point.lifetime = ros::Duration(0); // always exist
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = 11;
    pos_point.type = visualization_msgs::Marker::POINTS;
    pos_point.scale.x = 0.4;
    pos_point.scale.y = 0.4;
    pos_point.color = Color::Red();
    pos_point.color.a = 1.0;

    pub_start_and_goal_.publish(pos_point);

}

void VisualRviz::visualizeMultiTopoTrajs(const std::vector<std::vector<Vector3d>> &topo_trajs, std::vector<bool>& success, ros::Time local_time)
{
    if(pub_multi_topo_trajs_.getNumSubscribers() == 0)
    {
        return;
    }

    static int last_nums = 0;
    for(int id=0; id<last_nums; id++)
    {
        Eigen::Vector4d color(0,0,0,0);
        std::vector<Vector3d> blank;
        // visualizeMarkerList(blank, id, color, local_time);
        ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for(int id=0; id<topo_trajs.size(); id++)
    {
        if(id >= topoColorMap.size())
        {
            ROS_ERROR("ColorMap is not enough!");
            break;
        }
        Color color;
        if(success[id])
        {
            color = topoColorMap[id];
        }
        else
        {
            color = Color::Gray();
        }
        // displayMarkerList();
        ros::Duration(0.001).sleep();
        last_nums++;
    }

}

// real ids used: {id, id+1000}
void VisualRviz::visualizeMarkerList(ros::Publisher &pub, const std::vector<Vector3d> &list, double scale, Color color, int id, bool show_sphere)
{
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "map";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;

    sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
    sphere.color.r = line_strip.color.r = color.r;
    sphere.color.g = line_strip.color.g = color.g;
    sphere.color.b = line_strip.color.b = color.b;
    sphere.color.a = line_strip.color.a = color.a > 1e-5 ? color.a : 1.0;
    sphere.scale.x = scale;
    sphere.scale.y = scale;
    sphere.scale.z = scale;
    line_strip.scale.x = scale/2;
    geometry_msgs::Point pt;
    for(int i=0; i < int(list.size()); i++)
    {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        if(show_sphere) sphere.points.push_back(pt);
        line_strip.points.push_back(pt);
    }

    if(show_sphere) pub.publish(sphere);
    pub.publish(line_strip);

    pub

}
