#ifndef _TOPO_PRM_H
#define _TOPO_PRM_H


#include <plan_env/static/grid_map.h>
#include <plan_env/static/raycast.h>
#include <plan_env/pos_checker.h>
#include <random>
#include <memory>
#include <list> // Add missing include for 'list'
#include <vector> // Add missing include for 'vector'
#include <Eigen/Core> // Add missing include for 'Eigen'
#include <ros/ros.h> // Add missing include for 'ros'



class GraphNode
{
private:
    /* data */


public:
    enum NODE_TYPE { Guard = 1 , Connector = 2};

    enum NODE_STATE { NEW = 1, CLOSE = 2, OPEN = 3 };

    GraphNode(/* args */) {
    }

    GraphNode(Eigen::Vector3d pos, NODE_TYPE type, int id) {
        pos_ = pos;
        type_ = type;
        state_ = NEW;
        id_ = id;
    }
    ~GraphNode() {
    }

    std::vector<std::shared_ptr<GraphNode>> neighbors_;
    Eigen::Vector3d pos_;
    NODE_TYPE type_;
    NODE_STATE state_;
    int id_;
    typedef std::shared_ptr<GraphNode> Ptr;    

};


class TopoPRM
{
private:
    // DspMap::Ptr dsp_map_;
    PosChecker::Ptr pos_checker_;

    // sampling generator
    std::random_device rd_; // Add missing 'std::' namespace
    std::default_random_engine eng_; // Add missing 'std::' namespace
    std::uniform_real_distribution<double> rand_pos_; // Add missing 'std::' namespace

    Eigen::Vector3d sample_r_;
    Eigen::Vector3d translation_;
    Eigen::Matrix3d rotation_;


    // roadmap data structure, 0:start, 1:goal, 2-n: others
    std::list<GraphNode::Ptr> graph_; // Add missing 'std::' namespace
    std::vector<std::vector<Eigen::Vector3d>> raw_paths_; // 在graph基础上搜索出来的路径
    std::vector<std::vector<Eigen::Vector3d>> short_paths_; // 经过shorten的路径
    std::vector<std::vector<Eigen::Vector3d>> final_paths_; // 最终路径
    std::vector<Eigen::Vector3d> start_pts_, end_pts_;

    // raycasting
    std::vector<RayCaster> casters_;
    Eigen::Vector3d offset_;


    // parameter
    double max_sample_time_;
    int max_sample_num_;
    int max_raw_path_, max_raw_path2_; // max_raw_path2_ : 最大搜索的路径数量
    int short_cut_num_;
    Eigen::Vector3d sample_inflate_;
    double resolution_;


    double ratio_to_short_;
    int reserve_num_;

    bool parallel_shortcut_; // 是否并行减枝

    
    /* create topological roadmap */
    /* path searching, shortening, pruning and merging */
    std::list<GraphNode::Ptr> createGraph(Eigen::Vector3d start, Eigen::Vector3d end);
    std::vector<std::vector<Eigen::Vector3d>> searchPaths();
    void shortcutPaths();
    std::vector<std::vector<Eigen::Vector3d>> pruneEquivalent(std::vector<std::vector<Eigen::Vector3d>>& paths);
    std::vector<std::vector<Eigen::Vector3d>> selectShortPaths(std::vector<std::vector<Eigen::Vector3d>>& paths, int step);

    /* ---------- helper ---------- */
    inline Eigen::Vector3d getSample();
    std::vector<GraphNode::Ptr> findVisibGuard(Eigen::Vector3d pt);  // find pairs of visibile guard
    bool needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2,
                        Eigen::Vector3d pt);  // test redundancy with existing
                                                // connection between two guard
    bool lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, 
                    Eigen::Vector3d& pc, int caster_id = 0);
    bool triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2);
    void pruneGraph();

    void depthFirstSearch(std::vector<GraphNode::Ptr>& vis);

    std::vector<Eigen::Vector3d> discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2);
    std::vector<std::vector<Eigen::Vector3d>> discretizePaths(std::vector<std::vector<Eigen::Vector3d>>& path);

    std::vector<Eigen::Vector3d> discretizePath(std::vector<Eigen::Vector3d> path);
    void shortcutPath(std::vector<Eigen::Vector3d> path, int path_id, int iter_num = 1);

    std::vector<Eigen::Vector3d> discretizePath(const std::vector<Eigen::Vector3d>& path, int pt_num);
    bool sameTopoPath(const std::vector<Eigen::Vector3d>& path1, const std::vector<Eigen::Vector3d>& path2,
                        double thresh);
    Eigen::Vector3d getOrthoPoint(const std::vector<Eigen::Vector3d>& path);

    int shortestPath(std::vector<std::vector<Eigen::Vector3d>>& paths);


public:
    // double risk_thresh_;

    TopoPRM(/* args */);
    ~TopoPRM();

    void init(const ros::NodeHandle& nh);

    // void setEnvironment(const DspMap::Ptr dsp_map);
    void setPosChecker(const PosChecker::Ptr pos_checker);

    void getBox(Eigen::Vector3d &pt, Eigen::Vector3d &scale, Eigen::Quaterniond &quat);
    void findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end, std::vector<Eigen::Vector3d> start_pts,
                        std::vector<Eigen::Vector3d> end_pts, std::list<GraphNode::Ptr>& graph,
                        std::vector<std::vector<Eigen::Vector3d>>& raw_paths,
                        std::vector<std::vector<Eigen::Vector3d>>& filtered_paths,
                        std::vector<std::vector<Eigen::Vector3d>>& select_paths);

    double pathLength(const std::vector<Eigen::Vector3d>& path);
    std::vector<Eigen::Vector3d> pathToGuidePts(std::vector<Eigen::Vector3d>& path, int pt_num);


};



#endif