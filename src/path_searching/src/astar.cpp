// #include "path_searching/dynamic_astar.hpp
#include "path_searching/astar.hpp"

using namespace std;
using namespace Eigen;


AStar::~AStar()
{
    for(int i=0;i<pool_size_(0);i++)
    {
        for(int j=0;j<pool_size_(1);j++)
        {
            for(int k=0;k<pool_size_(2);k++)
            {
                delete grid_node_map_[i][j][k];
            }
        }
    }
}

void AStar::initGridMap(GridMap::Ptr grid_map,const Eigen::Vector3i pool_size)
{
    pool_size_ = pool_size;
    center_idx_ = pool_size_/2;

    grid_node_map_ = new GridNodePtr **[pool_size_(0)];
    for(int i=0;i<pool_size_(0);i++)
    {
        grid_node_map_[i] = new GridNodePtr *[pool_size_(1)];
        for(int j=0;j<pool_size_(1);j++)
        {
            grid_node_map_[i][j] = new GridNodePtr[pool_size_(2)];
            for(int k=0;k<pool_size_(2);k++)
            {
                grid_node_map_[i][j][k] = new GridNode;
            }
        }
    }
    grid_map_ = grid_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx,dy),dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if(dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy,dz) + 1.0 * abs(dy - dz);
    }
    if(dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx,dz) + 1.0 * abs(dx -dz);
    }
    if(dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx,dy) + 1.0 * abs(dx - dy);
    }
    return h;
}


double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}


double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while(current->cameFrom != nullptr)
    {
        current = current->cameFrom;
        path.push_back(current);
    }
    return path;
}

bool AStar::ConvertToGridIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt,Vector3i &start_idx, Vector3i &end_idx)
{
    if(!Coord2Index(start_pt,start_idx) || !Coord2Index(end_pt,end_idx))
    {
        return false;
    }

    if(checkOccupancy(Index2Coord(start_idx)))
    {
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if(!Coord2Index(start_pt,start_idx))
            {
                return false;
            }
        }while(checkOccupancy(Index2Coord(start_idx)));
    }

    if(checkOccupancy(Index2Coord(end_idx)))
    {
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if(!Coord2Index(end_pt,end_idx))
            {
                return false;
            }
        }while(checkOccupancy(Index2Coord(end_idx)));
    }
}


bool AStar::AStarSearch(const double step_size, Vector3d start_pt,Vector3d end_pt)
{
    ros::Time time_1 = ros::Time::now();
    ++ rounds_;

    step_size_ = step_size;
    inv_step_size_ = 1 / step_size;
    center_ = (start_pt + end_pt) / 2.0;
    
    Vector3i start_idx,end_idx;
    if(!ConvertToGridIndexAndAdjustStartEndPoints(start_pt,end_pt,start_idx,end_idx))
    {
        ROS_ERROR("Failed to convert start and end points to grid index");
        return false;
    }

    GridNodePtr start_ptr = grid_node_map_[start_idx(0)][start_idx(1)][start_idx(2)];
    GridNodePtr end_ptr = grid_node_map_[end_idx(0)][end_idx(1)][end_idx(2)];

    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
    open_set_.swap(empty);

    GridNodePtr neighbor_ptr = nullptr;
    GridNodePtr current_ptr = nullptr;

    start_ptr->index = start_idx;
    start_ptr->rounds = rounds_;
    start_ptr->gScore = 0;
    start_ptr->fScore = getHeu(start_ptr,end_ptr);
    start_ptr->state = GridNode::OPENSET;
    start_ptr->cameFrom = nullptr;
    open_set_.push(start_ptr);

    double tentative_gScore;
    int num_iter = 0;

    while(!open_set_.empty())
    {
        num_iter ++;
        current_ptr = open_set_.top();
        open_set_.pop();

        if(current_ptr->index(0) == end_ptr->index(0) && current_ptr->index(1) == end_ptr->index(1) && current_ptr->index(2) == end_ptr->index(2))
        {
            grid_path_ = retrievePath(current_ptr);
            return true;
        }
        current_ptr->state = GridNode::CLOSEDSET;

        for(int dx=-1; dx<=1; dx ++)
        {
            for(int dy=-1;dy<=1;dy++)
            {
                for(int dz=-1;dz<=1;dz++)
                {
                    if(dx == 0 && dy == 0 && dz == 0)
                    {
                        continue;
                    }

                    Vector3i neighbor_idx;
                    neighbor_idx(0) = (current_ptr->index)(0) + dx;
                    neighbor_idx(1) = (current_ptr->index)(1) + dy;
                    neighbor_idx(2) = (current_ptr->index)(2) + dz;
                    
                    if (neighbor_idx(0) < 1 || neighbor_idx(0) >= pool_size_(0) - 1 || neighbor_idx(1) < 1 || neighbor_idx(1) >= pool_size_(1) - 1 || neighbor_idx(2) < 1 || neighbor_idx(2) >= pool_size_(2) - 1)
                    {
                        continue;
                    }

                    neighbor_ptr = grid_node_map_[neighbor_idx(0)][neighbor_idx(1)][neighbor_idx(2)];
                    neighbor_ptr->index = neighbor_idx;

                    bool flag_explored = neighbor_ptr->rounds == rounds_;


                    if (flag_explored && neighbor_ptr->state == GridNode::CLOSEDSET)
                    {
                        continue; //in closed set.
                    }

                    neighbor_ptr->rounds = rounds_;

                    if (checkOccupancy(Index2Coord(neighbor_ptr->index)))
                    {
                        continue;
                    }

                    double static_cost = sqrt(dx * dx + dy * dy + dz * dz);
                    tentative_gScore = current_ptr->gScore + static_cost;

                    if (!flag_explored)
                    {
                        //discover a new node
                        neighbor_ptr->state = GridNode::OPENSET;
                        neighbor_ptr->cameFrom = current_ptr;
                        neighbor_ptr->gScore = tentative_gScore;
                        neighbor_ptr->fScore = tentative_gScore + getHeu(neighbor_ptr, end_ptr);
                        open_set_.push(neighbor_ptr); //put neighbor in open set and record it.
                    }
                    else if (tentative_gScore < neighbor_ptr->gScore)
                    { //in open set and need update
                        neighbor_ptr->cameFrom = current_ptr;
                        neighbor_ptr->gScore = tentative_gScore;
                        neighbor_ptr->fScore = tentative_gScore + getHeu(neighbor_ptr, end_ptr);
                    }
                }
            }
        }

        ros::Time time_2 = ros::Time::now();
        if ((time_2 - time_1).toSec() > 0.2)
        {
            ROS_WARN("Failed in A star path searching !!! 0.2 seconds time limit exceeded.");
            return false;
        }
    }
    ros::Time time_2 = ros::Time::now();

    if ((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in A star path finding is %.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

    return false;

}

vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : grid_path_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}

bool AStar::checkOccupancy(const Eigen::Vector3d &pos)
{
    return (bool)particle_map_->getOccupancy(pos);
}