#include <visualization_utils/traj_visualizer.h>
#include <ros/ros.h>
#include <queue>
#include <string>


TrajVisualizer::TrajVisualizer()
{

}

void TrajVisualizer::init(const ros::NodeHandle& nh)
{
    // publisher
    pub_collision_ = nh_.advertise<visualization_msgs::Marker>("collision", 1);
    pub_start_and_goal_ = nh_.advertise<visualization_msgs::Marker>("start_goal", 1);
    pub_kino_traj_ = nh_.advertise<visualization_msgs::Marker>("kino_traj",1);
    pub_multi_topo_trajs_ = nh_.advertise<visualization_msgs::Marker>("multi_topo_trajs",1);
    pub_opti_traj_ = nh_.advertise<visualization_msgs::Marker>("opti_traj",1);
    pub_bspline_traj_ = nh_.advertise<visualization_msgs::Marker>("bspline_traj",1);
    pub_astar_pathes_ = nh_.advertise<visualization_msgs::Marker>("astar_pathes",1);
}



void TrajVisualizer::visualizeCollision(const Vector3d& collision, bool keep)
{
    if(pub_collision_.getNumSubscribers() == 0)
    {
        return ;
    }
    static int collision_id = 0;
    if(keep)
    {
        collision_id++;
    }
    visualization_msgs::Marker collision_point;
    geometry_msgs::Point p;
    p.x = collision(0);
    p.y = collision(1);
    p.z = collision(2);
    collision_point.points.push_back(p);
    collision_point.header.frame_id = "world";
    collision_point.header.stamp = ros::Time::now();
    collision_point.ns = "collision";
    collision_point.action = visualization_msgs::Marker::ADD;
    collision_point.lifetime = ros::Duration(0); // always exist
    collision_point.pose.orientation.w = 1.0;
    collision_point.id = collision_id;
    collision_point.type = visualization_msgs::Marker::SPHERE_LIST;
    collision_point.scale.x = 0.2;
    collision_point.scale.y = 0.2;
    collision_point.scale.z = 0.2;
    collision_point.color = Color::Red();
    collision_point.color.a = 1.0;
    pub_collision_.publish(collision_point);    
}

void TrajVisualizer::visualizeStartAndGoal(const Vector3d &start, const Vector3d &goal, bool keep)
{
    if(pub_start_and_goal_.getNumSubscribers() == 0)
    {
        return;
    }
    visualization_msgs::Marker pos_point;

    static int point_id = 0;
    if(keep)
    {
        point_id ++;
    }

    geometry_msgs::Point p;
    p.x = start[0];
    p.y = start[1];
    p.z = start[2];
    pos_point.points.push_back(p);
    p.x = goal[0];
    p.y = goal[1];
    p.z = goal[2];
    pos_point.points.push_back(p);

    pos_point.header.frame_id = "world";
    pos_point.header.stamp = ros::Time::now();
    pos_point.ns = "s-g";
    pos_point.action = visualization_msgs::Marker::ADD;
    pos_point.lifetime = ros::Duration(0); // always exist
    pos_point.pose.orientation.w = 1.0;
    pos_point.id = point_id;
    pos_point.type = visualization_msgs::Marker::POINTS;
    pos_point.scale.x = 0.4;
    pos_point.scale.y = 0.4;
    pos_point.color = Color::Red();
    pos_point.color.a = 1.0;

    pub_start_and_goal_.publish(pos_point);

}

void TrajVisualizer::visualizeMultiTopoTrajs(const std::vector<std::vector<Vector3d>> &topo_trajs, std::vector<bool>& success, bool keep, bool show_sphere)
{
    if(pub_multi_topo_trajs_.getNumSubscribers() == 0)
    {
        return;
    }

    std::string ns = "topo_trajs";
    static int last_nums = 0;
    if(last_nums > topo_trajs.size())
    {
        ROS_WARN_STREAM("last_nums: " << last_nums << " topo_trajs.size(): " << topo_trajs.size());
        return ;
    }
    for(int i = 0; i < last_nums ; i++)
    {
        int scale = 0.2;
        Color color(0,0,0,0);
        std::vector<Vector3d> blank;
        visualizeTrajectoryAndSamplePoints(pub_multi_topo_trajs_,blank,0.2,color,ns,i);
        ros::Duration(0.001).sleep();
    }
    last_nums = 0;

    for(int i=0; i<topo_trajs.size(); i++)
    {
        if(i >= topoColorMap.size())
        {
            ROS_ERROR("ColorMap is not enough!");
            break;
        }
        Color color;
        if(success[i])
        {
            color = topoColorMap[i];
        }
        else
        {
            color = Color::Gray();
        }
        visualizeTrajectoryAndSamplePoints(pub_multi_topo_trajs_,topo_trajs[i],0.2,color,ns,i);
        ros::Duration(0.001).sleep();
        last_nums++;
    }

}


void TrajVisualizer::visualizeKinodynamicTraj(const std::vector<Vector3d> &kino_traj, bool keep, bool show_sphere)
{
    if(pub_kino_traj_.getNumSubscribers() == 0)
    {
        return;
    }
    string ns = "kino_trajs";
    Color green = Color::Green();
    static int kino_id = 0;
    if(keep)
    {
        kino_id++;
    }

    if(show_sphere)
    {
        visualizeTrajectoryAndSamplePoints(pub_kino_traj_,kino_traj,0.2,green,ns,kino_id);
    }else
    {
        visualizeTrajectory(pub_kino_traj_,kino_traj,0.2,green,ns,kino_id);
    }
}


void TrajVisualizer::visualizeOptimalTraj(const std::vector<Vector3d> &optimal_traj, bool keep, bool show_sphere)
{
    if(pub_opti_traj_.getNumSubscribers() == 0)
    {
        return;
    }

    string ns = "opti_trajs";
    Color red = Color::Red();
    static int opti_id = 0;
    if(keep)
    {
        opti_id ++;
    }    

    if(show_sphere)
    {
        visualizeTrajectoryAndSamplePoints(pub_opti_traj_,optimal_traj,0.2,red,ns,opti_id);
    }else
    {
        visualizeTrajectory(pub_opti_traj_,optimal_traj,0.2,red,ns,opti_id);
    }
}


void TrajVisualizer::visualizeBsplineTraj(const std::vector<Vector3d> &bspline, const MatrixXd& ctrl_pts, bool keep)
{
    if(pub_bspline_traj_.getNumSubscribers() == 0)
    {
        return;
    }

    Color blue = Color::Blue();
    string ns = "bspline";
    static int bspline_id = 0;
    if(keep)
    {
        bspline_id++;
    }
    visualizeTrajectoryAndControlPoints(pub_bspline_traj_,bspline,ctrl_pts,0.1,blue,ns,bspline_id);
}

void TrajVisualizer::visualizeAstarPath(const std::vector<std::vector<Vector3d>> &astar_pathes, bool keep)
{
    if(pub_astar_pathes_.getNumSubscribers() == 0)
    {
        return;
    }

    static string ns = "astar_pathes";
    static int last_nums = 0;
    
    for(int i = 0; i < last_nums; i++)
    {
        Color color(0,0,0,0);
        std::vector<Vector3d> blank;
        visualizeTrajectoryAndSamplePoints(pub_astar_pathes_,blank,0.2,color,"astar_pathes",i);
    }
    last_nums = 0;

    for(size_t i = 0; i < astar_pathes.size(); i++)
    {
        Color color = Color::SteelBlue();
        visualizeTrajectoryAndSamplePoints(pub_astar_pathes_,astar_pathes[i],0.2,color,ns,i);
        last_nums ++;
        ros::Duration(0.001).sleep();
    }

}


void TrajVisualizer::visualizeTrajectory(ros::Publisher &pub, const vector<Vector3d> &traj, double scale, Color color, const string ns, int id)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.header.stamp = ros::Time::now();
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.id = id + 1000;
    line_strip.ns = ns;

    line_strip.pose.orientation.w = 1.0;
    line_strip.color.r = color.r;
    line_strip.color.g = color.g;
    line_strip.color.b = color.b;
    line_strip.color.a = color.a > 1e-5 ? color.a : 1.0;
    line_strip.scale.x = scale/2;
    geometry_msgs::Point pt;
    for(int i=0; i < int(traj.size()); i++)
    {
        pt.x = traj[i](0);
        pt.y = traj[i](1);
        pt.z = traj[i](2);
        line_strip.points.push_back(pt);
    }
    pub.publish(line_strip);
}

void TrajVisualizer::visualizeTrajectoryAndSamplePoints(ros::Publisher &pub, const vector<Vector3d> &traj, double scale, Color color, const string ns, int id)
{
    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;
    sphere.ns = line_strip.ns = ns;

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
    for(int i=0; i < int(traj.size()); i++)
    {
        pt.x = traj[i](0);
        pt.y = traj[i](1);
        pt.z = traj[i](2);
        sphere.points.push_back(pt);
        line_strip.points.push_back(pt);
    }

    pub.publish(sphere);
    pub.publish(line_strip);
}

void TrajVisualizer::visualizeTrajectoryAndControlPoints(ros::Publisher &pub, const vector<Vector3d> &traj, const MatrixXd &ctrl_pts, double scale, Color color, string ns, int id)
{

    visualization_msgs::Marker sphere, line_strip;
    sphere.header.frame_id = line_strip.header.frame_id = "world";
    sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
    sphere.type = visualization_msgs::Marker::SPHERE_LIST;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
    sphere.id = id;
    line_strip.id = id + 1000;
    sphere.ns = line_strip.ns = ns;
    
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
    for(size_t i = 0; i < traj.size(); i++)
    {
        pt.x = traj[i](0);
        pt.y = traj[i](1);
        pt.z = traj[i](2);
        line_strip.points.push_back(pt);
    }
    for(int i = 0; i < ctrl_pts.cols(); i++)
    {
        pt.x = ctrl_pts(0,i);
        pt.y = ctrl_pts(1,i);
        pt.z = ctrl_pts(2,i);
        sphere.points.push_back(pt);
    }

    pub.publish(sphere);
    pub.publish(line_strip);

}
