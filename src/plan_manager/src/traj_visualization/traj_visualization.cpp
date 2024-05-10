#include <traj_utils/traj_utils.h>
#include <ros/ros.h>
#include <queue>
#include <string>


VisualRviz::VisualRviz(const ros::NodeHandle& nh) : nh_(nh)
{
    // publisher
    pub_collision_ = nh_.advertise<visualization_msgs::Marker>("collision", 1);
    pub_start_goal_ = nh_.advertise<visualization_msgs::MarkerArray>("start_goal", 1);
}


void VisualRviz::visualizeCollision(const Vector3d& collision, ros::Time local_time)
{
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


void VisualRviz::visualize