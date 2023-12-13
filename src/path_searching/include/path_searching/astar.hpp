#ifndef _DYNAMIC_ASTAR_H_
#define _DYNAMIC_ASTAR_H_

#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <plan_env/particle_map.hpp>
#include <plan_env/grid_map.h>
#include <queue>
#include <memory>

constexpr double inf = 1 >> 20;

struct GridNode;
typedef GridNode *GridNodePtr;  

struct GridNode
{
    enum node_state
    {
        OPENSET = 1,
        CLOSEDSET = 2,
        UNDEFINED = 3
    };

    int rounds{0};
    enum node_state state{UNDEFINED};

    Eigen::Vector3i index;
    double gScore{inf}, fScore{inf};
    GridNodePtr cameFrom{nullptr};
};


class NodeComparator
{
public:
    bool operator()(GridNodePtr node1, GridNodePtr node2)
    {
        return node1->fScore > node2->fScore;
    }

};


class AStar
{
private:
    GridMap::Ptr grid_map_;
    particle_map::ParticleMap::Ptr particle_map_;
    inline void coord2gridIndexFast(const double x, const double y, const double z, int &id_x, int &id_y, int &id_z);

    double getDiagHeu(GridNodePtr node1, GridNodePtr node2);
    double getManhHeu(GridNodePtr node1, GridNodePtr node2);
    double getEuclHeu(GridNodePtr node1, GridNodePtr node2);
    inline double getHeu(GridNodePtr node1, GridNodePtr node2);

    bool ConvertToGridIndexAndAdjustStartEndPoints(const Eigen::Vector3d start_pt, const Eigen::Vector3d end_pt,
                                                        Eigen::Vector3i &start_index, Eigen::Vector3i &end_index);

    inline Eigen::Vector3d Index2Coord(const Eigen::Vector3i index) const;

    inline bool Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &index) const;
    bool checkOccupancy(const Eigen::Vector3d &pos);
    
    std::vector<GridNodePtr> retrievePath(GridNodePtr current);

    double step_size_, inv_step_size_;
    Eigen::Vector3d center_;
    Eigen::Vector3i center_idx_,pool_size_;
    const double tie_breaker_ = 1.0 + 1.0 /10000;
    std::vector<GridNodePtr> grid_path_;
    
    GridNodePtr ***grid_node_map_;
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_set_;

    int rounds_{0};

public:
    typedef std::shared_ptr<AStar> Ptr;
    AStar(){};
    ~AStar();

    void initGridMap(GridMap::Ptr grid_map, const Eigen::Vector3i pool_size);
    void initParticleMap(particle_map::ParticleMap::Ptr particle_map, const Eigen::Vector3i pool_size);

    bool AStarSearch(const double step_size, Eigen::Vector3d start_pt, Eigen::Vector3d end_pt);

    std::vector<Eigen::Vector3d> getPath();

};

inline double AStar::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    return tie_breaker_ * getDiagHeu(std::move(node1),std::move(node2));
}

inline Eigen::Vector3d AStar::Index2Coord(const Eigen::Vector3i index) const
{
    return ((index - center_idx_).cast<double>() * step_size_)+ center_;
}

inline bool AStar::Coord2Index(const Eigen::Vector3d &pt, Eigen::Vector3i &index) const
{
	index = ((pt - center_) * inv_step_size_ + Eigen::Vector3d(0.5, 0.5, 0.5)).cast<int>() + center_idx_;

	if (index(0) < 0 || index(0) >= pool_size_(0) || index(1) < 0 || index(1) >= pool_size_(1) || index(2) < 0 || index(2) >= pool_size_(2))
	{
		ROS_ERROR("Ran out of pool, index=%d %d %d", index(0), index(1), index(2));
		return false;
	}

	return true;

}


#endif