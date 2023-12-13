#include "path_searching/topo_rpm.hpp"
#include "topo_rpm.hpp"

namespace topo_risk_planner
{
    
void TopoPRM::init(const ros::NodeHandle& nh)
{
    graph_.clear();
    eng_ = default_random_engine(rd_());
    rand_pos_ = uniform_real_distribution<double>(-1.0, 1.0);

    //init parameter
    nh.param("topo_prm/sample_inflate_x", sample_inflate_(0), -1.0); // sample radius
    nh.param("topo_prm/sample_inflate_y", sample_inflate_(1), -1.0);
    nh.param("topo_prm/sample_inflate_z", sample_inflate_(2), -1.0);
    nh.param("topo_prm/clearance", clearance_, -1.0); // 
    nh.param("topo_prm/short_cut_num", short_cut_num_, -1);
    nh.param("topo_prm/reserve_num", reserve_num_, -1);
    nh.param("topo_prm/ratio_to_short", ratio_to_short_, -1.0);
    nh.param("topo_prm/max_sample_num", max_sample_num_, -1);
    nh.param("topo_prm/max_sample_time", max_sample_time_, -1.0);
    nh.param("topo_prm/max_raw_path", max_raw_path_, -1);
    nh.param("topo_prm/max_raw_path2", max_raw_path2_, -1);
    nh.param("topo_prm/parallel_shortcut", parallel_shortcut_, false);

    resolution_ = particle_map_->getVoxelResolution();
    offset_ = Eigen::Vector3d(0.5,0.5,0.5) - particle_map_->getOrigin() / resolution_;

    for(int i = 0; i < max_raw_path_;i++)
    {
        casters_.push_back(RayCaster());
    }
}

void TopoPRM::setEnvironment(const particle_map::ParticleMap::Ptr particle_map)
{
    this->particle_map_ = particle_map;
}



void TopoPRM::findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end, 
                            vector<Eigen::Vector3d> start_pts, vector<Eigen::Vector3d> end_pts, 
                            list<GraphNode::Ptr> &graph, vector<vector<Eigen::Vector3d>> &raw_paths vetcor, 
                            vector<vector<Eigen::Vector3d>> &raw_paths vector<vector<Eigen::Vector3d>> &filtered_paths, 
                            vector<vector<Eigen::Vector3d>> &select_paths)
{
    ros::Time t1, t2;
    double graph_time, search_time, short_time, prune_time, select_time;
      /* ---------- create the topo graph ---------- */
    t1 = ros::Time::now();

    start_pts_ = start_pts;
    end_pts_ = end_pts;

    graph = createGraph(start, end);

    graph_time = (ros::Time::now() - t1).toSec();

    /* ---------- search paths in the graph ---------- */
    t1 = ros::Time::now();

    raw_paths = searchPaths();

    search_time = (ros::Time::now() - t1).toSec();

    /* ---------- path shortening ---------- */
    // for parallel, save result in short_paths_
    t1 = ros::Time::now();

    shortcutPaths();

    short_time = (ros::Time::now() - t1).toSec();

    /* ---------- prune equivalent paths ---------- */
    t1 = ros::Time::now();

    filtered_paths = pruneEquivalent(short_paths_);

    prune_time = (ros::Time::now() - t1).toSec();
    // cout << "prune: " << (t2 - t1).toSec() << endl;

    /* ---------- select N shortest paths ---------- */
    t1 = ros::Time::now();

    select_paths = selectShortPaths(filtered_paths, 1);

    select_time = (ros::Time::now() - t1).toSec();

    final_paths_ = select_paths;

    double total_time = graph_time + search_time + short_time + prune_time + select_time;

    std::cout << "\n[Topo]: total time: " << total_time << ", graph: " << graph_time
            << ", search: " << search_time << ", short: " << short_time << ", prune: " << prune_time
            << ", select: " << select_time << std::endl;

}


double TopoPRM::pathLength(const vector<Eigen::Vector3d> &path) 
{
    double length = 0.0;
    if(path.size() < 2) return length;

    for(int i=0;i<path.size()-1;i++)
    {
        length += (path[i+1] - path[i]).norm();
    }
    return length;
}

vector<Eigen::Vector3d> TopoPRM::pathToGuidePts(vector<Eigen::Vector3d> &path, int pt_num)
{
    return discretizePath(path,pt_num);
}

inline Eigen::Vector3d TopoPRM::getSample()
{
    Eigen::Vector3d pt;
    pt(0) = rand_pos_(eng_) * sample_r_(0);
    pt(1) = rand_pos_(eng_) * sample_r_(1);
    pt(2) = rand_pos_(eng_) * sample_r_(2);
    pt = rotation_ * pt + translation_;
    return pt;
}

vector<GraphNode::Ptr> TopoPRM::findVisibGuard(Eigen::Vector3d pt)
{
    vector<GraphNode::Ptr> visib_guards;
    Eigen::Vector3d pc;
    int visib_num = 0;

    /* find visible GUARD from pt */
    for(list<GraphNode::Ptr>::iterator iter = graph_.begin; iter != graph_.end();++iter)
    {
        if((*iter)->type_ == GraphNode::Connector) continue;

        if(lineVisib(pt,(*iter)->pos_,resolution_,pc))
        {
            visib_guards.push_back(*iter);
            ++visib_num;
            if(visib_num >= 2) break;
        }
    }

    return visib_guards;
}
bool TopoPRM::needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt)
{
    vector<Eigen::Vetcor3d> path1(3),path2(3);
    // 当前路径
    path1[0] = g1->pos_;
    path1[1] = pt;
    path1[2] = g2->pos_;

    // 搜索路径，path2[1]空着
    path2[0] = g1->pos_;
    path2[2] = g2->pos_;

    vector<Eigen::Vecto3d> connect_pts;
    bool has_connect = false;
    // 找g1 和 g2的共同邻居
    for(int i=0;i < g1->neighbors_.size();i++)
    {
        for(int j=0;j<g2->neighbors_.size();j++)
        {
            if(g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_)
            {
                // 找到共同邻居
                path2[1] = g1->neighbors_[i]->pos_;
                bool same_topo = sameTopoPath(path1,path2,0.0); // 判断是否同拓扑
                if(same_topo) // 如果是
                {
                    // get shorter connection ?
                    if(pathLength(path1) < pathLength(path2)) // 如果当前路径还比找到的路径短，那么就这条路径也连上，后面做处理。
                    {
                        g1->neighbors_[i]->pos_ = pt;
                    }
                    return false;
                }
            }
        }
    }
    return true;
}
bool TopoPRM::lineVisib(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, double thresh, Eigen::Vector3d &pc, int caster_id)
{
    Eigen::Vector3d ray_pt;
    Eigen::Vector3i pt_id;
    double risk;

    casters_[caster_id].setInput(p1/resolution_, p2/resolution_);

    while(casters_[caster_id].step(ray_pt))
    {
        pt_id(0) = ray_pt(0) + offset_(0);
        pt_id(1) = ray_pt(1) + offset_(1);
        pt_id(2) = ray_pt(2) + offset_(2);
        
        risk = particle_map_->getVoxelRisk(pt_id);
        if(risk > thresh)
        {
            pc = particle_map_->globalIdx2Pos(pt_id);
            return false;
        }
    }
    return true;
}

bool TopoPRM::triangleVisib(Eigen::Vector3d pt, Eigen::Vector3d p1, Eigen::Vector3d p2)
{
    // get the traversing points along p1-p2
    vector<Eigen::Vector3d> pts;
    Eigen::Vector3d dir = p2 - p1;
    double len = dir.norm();
    int seg_num = ceil(length / resolution_);

    Eigen::Vector3d pt1;
    for(int i=1;i<seg_num;i++)
    {
        pt1 = p1 + dir * double(i) / double(seg_num);
        pts.push_back(pt1);
    }

    //test visibility
    for(int i=0;i<pts.size();i++)
    {
        return false;
    }

    return true;
}
void TopoPRM::pruneGraph()
{
    /* prune useless node */
    if(graph_.size() > 2)
    {
        for(list<GraphNode::Ptr>::iterator iter1 = grpah_.begin();iter1 != graph_.end() && graph_.size() > 2; ++iter1)
        {
            if((*iter1)->id_ <= 1) continue;
            
            /*core*/
            if((*iter1)->neighbors_.size() <= 1)
            {
                // delete this node from others' neighbor
                for(list<GraphNode::Ptr>::iterator iter2 = graph_.begin();iter2 != graph_.end(); iter2++)
                {
                    for(vector<GraphNode::Ptr>::iterator it_nb = (*iter2)->neighbors_.begin(); it_nb != (*iter2)->neighbors_.end(); ++ it_nb)
                    {
                        if((*it_nb)->id_ == (*iter1)->id_)
                        {
                            (*iter2)->neighbors_.erase(it_nb);
                            break;
                        }
                    }
                }

                // delete this node from graph, restart checking
                graph_.erase(iter1);
                iter1 = graph_.begin();
            }

        }
    }
}
void TopoPRM::depthFirstSearch(vector<GraphNode::Ptr> &vis)
{
    GraphNode::Ptr cur = vis.back();

    for(int i=0;i<cur->neighbors_.size();i++)
    {
        // check reach goal
        if(cur->neighbors_[i]->id_ == 1)
        {
            // add this path to paths set
            vector<Eigen::Vector3d> path;
            for(int j=0;j<vis.size();j++)
            {
                path.push_back(vis[j]->pos_);
            }
            path.push_back(cur->neighbors_[i]->pos_);

            raw_paths_.push_back(path);
            if(raw_paths_.size() >= max_raw_path_) return;

            break;
        }
    }

    for(int i=0;i<cur->neighbors_.size();i++)
    {
        // skip reach goal
        if(cur->neighbors_[i]->id_ == 1) continue;

        // skip already visited node
        bool revisit = false;
        for(int j=0;j<vis.size();j++)
        {
            if(cur->neighbors_[i]->id_ == vis[j]->id_)
            {
                revisit = true;
                break;
            }
        }
        if(revisit) continue;

        // recursive search
        vis.push_back(cur->neighbors_[i]);
        depthFirstSearch(vis);
        if(raw_paths_.size() >= max_raw_path_) return;
        vis.pop_back();
    }
}
vector<Eigen::Vector3d> TopoPRM::discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2)
{
    Eigen::Vector3d dir = p2 - p1;
    double len = dir.norm();
    int seg_num = ceil(len / resolution_);

    vector<Eigen::Vector3d> line_pts;
    if(seg_num <= 0){
        return line_pts;
    }

    for(int i=0;i<=seg_num;i++) line_pts.push_back(p1 + dir * double(i) / double(seg_num));

    return line_pts;
}

vector<Eigen::Vector3d> TopoPRM::discretizePath(vector<Eigen::Vector3d> path)
{
    vector<Eigen::Vetcor3d> dis_path,segment;

    if(path.size() < 2)
    {
        ROS_ERROR("what path ?");
        return dis_path;
    }

    for(int i=0;i<path.size()-1;i++)
    {
        segment = discretizeLine(path[i],path[i+1]);

        if(segment.size() < 1) continue;

        dis_path.insert(dis_path.end(),segment.begin(),segment.end());

        if(i != path.size() - 2) dis_path.pop_back(); // 取消重复，除了最后一个点
    }
    return dis_path;
}


vector<vector<Eigen::Vector3d>> TopoPRM::discretizePaths(vector<vector<Eigen::Vector3d>> &path)
{
    vector<vector<Eigen::Vector3d>> dis_paths;
    vector<Eigen::Vector3d> dis_path;

    for(int i=0;i<path.size();i++)
    {
        dis_path = discretizePath(path[i],0.0);
        
        if(dis_path.size() > 0) dis_paths.push_back(dis_path);
    }
    return dis_paths;
}


void TopoPRM::shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num)
{
    vector<Eigen::Vector3d> short_path = path;
    vector<Eigen::Vector3d> last_path;

    for(int k=0;k<iter_num;k++)
    {
        last_path = short_path;

        vector<Eigen::Vector3d> dis_path = discretizePath(short_path);
        
        if(dis_path.size() < 2)
        {
            short_paths_[path_id] = dis_path;
            return ;
        }

        /* visibility path shortening */
        Eigen::Vector3d collision_pt, grad, dir, push_dir;
        double dist;
        short_path.clear();
        short_path.push_back(dis_path.front());
        for(int i=1;i<dis_path.size();i++)
        {

            // 如果可见，在我们这就是风险小于阈值，
            if(lineVisib(short_path.back(),dis_path[i],resolution_,collision_pt,path_id)) continue;

            // 如果不可见，我们就直接把当前这个中风险点当作下一个点了，去处理ESDF的推力
            short_path.push_back(collision_pt);
        }
    }
    short_path.push_back(dis_path.back());

    /* break if no shortcut */
    double len1 = pathLength(last_path);
    double len2 = pathLength(short_path);
    if(len2 > len1)
    {
        ROS_WARN("pause shortcut, l1: %lf, l2: %lf, iter: %d", len1, len2, k + 1);
        short_path = last_path;
        break;
    }
}

vector<Eigen::Vector3d> TopoPRM::discretizePath(const vector<Eigen::Vector3d> &path, int pt_num)
{
    vector<double> len_list;
    len_list.push_back(0.0);

    for(int i=0;i<path.size() - 1;i++)
    {
        double inc_l = (path[i+1] - path[i]).norm();
        len_list.push_back(len_list[i] + inc_l);
    }

    // calc pt_num points along the path
    double len_total = len_list.back();
    double dl = len_total / double(pt_num - 1);
    double cur_l;

    vector<Eigen::Vector3d> dis_path;
    for(int i=0;i<pt_num;i++)
    {
        cur_l = double(i) * dl;

        // find the range cur_l in
        int idx = -1;
        for(int j=0;j<len_list.size()-1;j++)
        {
            if(cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j+1] + 1e-4)
            {
                idx = j;
                break;
            }
        }

        // find lambda and interpolate
        double lambda = (cur_l - len_list[idx]) / (len_list[idx+1] - len_list[idx]);
        Eigen::Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx+1];
        dis_path.push_back(inter_pt);
    }

    return dis_path;
}

bool TopoPRM::sameTopoPath(const vector<Eigen::Vector3d> &path1, const vector<Eigen::Vector3d> &path2, double thresh)
{
    double len1 = pathLength(path1);
    double len2 = pathLength(path2);

    double max_len = max(len1,len2);

    int pt_num = ceil(max_len / resolution_);

    vector<Eigen::Vector3d> pts1 = discretizePath(path1,pt_num);
    vector<Eigen::Vector3d> pts2 = discretizePath(path2,pt_num);

    Eigen::Vector3d pc;
    for(int i=0;i<pt_num;i++)
    {
        if(!lineVisib(pts1[i],pts2[i],thresh,pc))
        {
            return false;
        }
    }

    return true;
}
Eigen::Vector3d TopoPRM::getOrthoPoint(const vector<Eigen::Vector3d> &path)
{
    Eigen::Vector3d x1 = path.front();
    Eigen::VEctor3d x2 = path.back();

    Eigen::Vector3d dir = (x2 - x1).normalized();
    Eigen::VEctor3d mid = 0.5 *( x1 + x2);

    double min_cos = 1000.0;
    Eigen::Vector3d pdir;
    Eigen::Vector3d ortho_pt;

    for(int i=1 ; i < path.size() - 1;i++)
    {
        pdir = (path[i] - mid).normalized();
        double cos  =fabs(pdir.dot(dir));

        if(cos < min_cos)
        {
            min_cos = cos;
            ortho_pt = path[i];
        }
    }
    return ortho_pt;
}
int TopoPRM::shortestPath(vector<vector<Eigen::Vector3d>> &paths)
{
    int short_id = -1;
    double min_len = 100000000;
    for(int i=0;i<paths.size();i++)
    {
        double len = pathLength(paths[i]);
        if(len < min_len)
        {
            min_len = len;
            short_id = i;
        }
    }
    return length;
}
}