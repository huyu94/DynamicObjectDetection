#include "path_searching/topo_prm.h"
#include <thread>





TopoPRM::TopoPRM(){}

TopoPRM::~TopoPRM(){}  

void TopoPRM::init(const ros::NodeHandle& nh)
{
    
    graph_.clear();
    eng_ = default_random_engine(rd_());
    
    rand_pos_ = uniform_real_distribution<double>(-1.0, 1.0);

    // init parameter
    nh.param("topo_prm/sample_inflate_x", sample_inflate_(0), -1.0);
    nh.param("topo_prm/sample_inflate_y", sample_inflate_(1), -1.0);
    nh.param("topo_prm/sample_inflate_z", sample_inflate_(2), -1.0);
    nh.param("topo_prm/short_cut_num", short_cut_num_, -1);
    nh.param("topo_prm/reserve_num", reserve_num_, -1);
    nh.param("topo_prm/ratio_to_short", ratio_to_short_, -1.0);
    nh.param("topo_prm/max_sample_num", max_sample_num_, -1);
    nh.param("topo_prm/max_sample_time", max_sample_time_, -1.0);
    nh.param("topo_prm/max_raw_path", max_raw_path_, -1);
    nh.param("topo_prm/max_raw_path2", max_raw_path2_, -1);
    nh.param("topo_prm/parallel_shortcut", parallel_shortcut_, false);

    // nh.param("topo_prm/risk_thresh", risk_thresh_, 0.5);
    // resolution_ = dsp_map_->getResolution();
    resolution_ = pos_checker_->getResolution();
    
    // offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - dsp_map_->getOrigin() / resolution_;
    // offset_ = Eigen::Vector3d(0.5,0.5,0.5) 
    
    for (int i = 0; i < max_raw_path_; ++i) {
        casters_.push_back(RayCaster());
    }
    ROS_INFO("Topo prm init done");
}


void TopoPRM::setPosChecker(PosChecker::Ptr pos_checker)
{
    pos_checker_ = pos_checker;
}

void TopoPRM::findTopoPaths(Eigen::Vector3d start, Eigen::Vector3d end,
                                vector<Eigen::Vector3d> start_pts, vector<Eigen::Vector3d> end_pts,
                                list<GraphNode::Ptr>& graph, vector<vector<Eigen::Vector3d>>& raw_paths,
                                vector<vector<Eigen::Vector3d>>& filtered_paths,
                                vector<vector<Eigen::Vector3d>>& select_paths) 
{
    ros::Time t1, t2;

    double graph_time, search_time, short_time, prune_time, select_time;
    /* ---------- create the topo graph ---------- */
    t1 = ros::Time::now();

    start_pts_ = start_pts;
    end_pts_ = end_pts;

    graph = createGraph(start, end);
    ROS_INFO("create graph finished");
    graph_time = (ros::Time::now() - t1).toSec();
    cout << "create graph: " << (t2 - t1).toSec() << endl;
    /* ---------- search paths in the graph ---------- */
    t1 = ros::Time::now();

    raw_paths = searchPaths();

    search_time = (ros::Time::now() - t1).toSec();
    cout << "search path: " << (t2 - t1).toSec() << endl;
    /* ---------- path shortening ---------- */
    // for parallel, save result in short_paths_
    t1 = ros::Time::now();

    shortcutPaths();

    short_time = (ros::Time::now() - t1).toSec();
    cout << "short path: " << (t2 - t1).toSec() << endl;
    /* ---------- prune equivalent paths ---------- */
    t1 = ros::Time::now();

    filtered_paths = pruneEquivalent(short_paths_);

    prune_time = (ros::Time::now() - t1).toSec();
    cout << "prune: " << (t2 - t1).toSec() << endl;

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



list<GraphNode::Ptr> TopoPRM::createGraph(Eigen::Vector3d start, Eigen::Vector3d end)
{
    static int count = 0;
    std::cout << endl
            << "[create graph]: -------------------------------------" << count++ << std::endl;
    graph_.clear();
    
    GraphNode::Ptr start_node = GraphNode::Ptr(new GraphNode(start, GraphNode::Guard, 0));
    GraphNode::Ptr end_node = GraphNode::Ptr(new GraphNode(end, GraphNode::Guard, 1));

    graph_.push_back(start_node);
    graph_.push_back(end_node);

    /* 确定一个采样空间， 一个椭球体的长度和姿态*/

    // sample region
    sample_r_(0) = 0.5 * (end - start).norm() + sample_inflate_(0); // x-axis length
    sample_r_(1) = sample_inflate_(1); // y-axis length 
    sample_r_(2) = sample_inflate_(2); // z-axis length

    // transformation
    translation_ = 0.5 * (start + end); // center 

    Eigen::Vector3d xtf, ytf, ztf, downward(0, 0, -1);
    xtf = (end - translation_).normalized(); // x方向
    ytf = xtf.cross(downward).normalized(); // y方向
    ztf = xtf.cross(ytf);

    rotation_.col(0) = xtf;
    rotation_.col(1) = ytf;
    rotation_.col(2) = ztf;
    
    int node_id = 1;


    /* -------------- algrithm main loop,use two type of nodes to create graph -------------------- */
    int sample_num = 0;
    double sample_time = 0.0;
    Eigen::Vector3d pt;
    ros::Time t1, t2;
    // ROS_INFO("start in [create graph]");
    while (sample_time < max_sample_time_ && sample_num < max_sample_num_) {
        t1 = ros::Time::now();

        pt = getSample();
        ++sample_num;
        int collision_id;
        if(pos_checker_->checkCollisionInSlideBox(pt,collision_id))
        {
            sample_time += (ros::Time::now() - t1).toSec();
            continue;
        }

        /* find visible guard */
        vector<GraphNode::Ptr> visib_guards = findVisibGuard(pt);
        if (visib_guards.size() == 0)  // 如果没有可连接的点，将该点定义为guard
        {
            GraphNode::Ptr guard = GraphNode::Ptr(new GraphNode(pt, GraphNode::Guard, ++node_id));
            graph_.push_back(guard);
        } else if (visib_guards.size() == 2) { // 如果找到可连接的点
            /* try adding new connection between two guard */
            // vector<pair<GraphNode::Ptr, GraphNode::Ptr>> sort_guards =
            // sortVisibGuard(visib_guards);
            // 尝试连接            ROS_INFO("end by 2");
            bool need_connect = needConnection(visib_guards[0], visib_guards[1], pt);
            if (!need_connect) { // 连接未成功就终止
                sample_time += (ros::Time::now() - t1).toSec();
                continue;
            }

            // new useful connection needed, add new connector， 定义为connector
            GraphNode::Ptr connector = GraphNode::Ptr(new GraphNode(pt, GraphNode::Connector, ++node_id));
            graph_.push_back(connector);

            // connect guards
            visib_guards[0]->neighbors_.push_back(connector);
            visib_guards[1]->neighbors_.push_back(connector);

            connector->neighbors_.push_back(visib_guards[0]);
            connector->neighbors_.push_back(visib_guards[1]);
        }
        sample_time += (ros::Time::now() - t1).toSec();
    }
      /* print record */
    // std::cout << "[Topo]: sample num: " << sample_num;

    /* 修剪，缩短路径 */
    pruneGraph();
    // std::cout << "[Topo]: node num: " << graph_.size() << std::endl;

    return graph_;
}


vector<GraphNode::Ptr> TopoPRM::findVisibGuard(Eigen::Vector3d pt)
{
    vector<GraphNode::Ptr> visib_guards;
    Eigen::Vector3d pc;

    int visib_num = 0;

    /* find visible GUARD from pt */
    for(list<GraphNode::Ptr>::iterator iter = graph_.begin(); iter != graph_.end(); ++ iter)
    {
        if((*iter)->type_ == GraphNode::Connector) continue; // 不连connector

        if(lineVisib(pt,(*iter)->pos_,pc))
        {
            visib_guards.push_back(*iter);
            ++visib_num;
        }
    }
    return visib_guards;
}



bool TopoPRM::needConnection(GraphNode::Ptr g1, GraphNode::Ptr g2, Eigen::Vector3d pt)
{
    vector<Eigen::Vector3d> path1(3), path2(3);

    path1[0] = g1->pos_;
    path1[1] = pt;
    path1[2] = g2->pos_;

    path2[0] = g1->pos_;

    path2[2] = g2->pos_;

    vector<Eigen::Vector3d> connect_pts;
    bool has_connect = false;
    for(size_t i=0; i< g1->neighbors_.size();i++)
    {
        for(size_t j=0; j < g2->neighbors_.size();j++)
        {
            if(g1->neighbors_[i]->id_ == g2->neighbors_[j]->id_) // 如果g1和g2 有相同的邻居
            {
                path2[1] = g1->neighbors_[i]->pos_;
                bool same_topo = sameTopoPath(path1, path2, 0.0);
                if(same_topo)
                {
                    if(pathLength(path1) < pathLength(path2))
                    {
                        g1->neighbors_[i]->pos_ = pt;
                    }
                }
                return false;
            }
        }
    }
    return true;
}


Eigen::Vector3d TopoPRM::getSample()
{
    Eigen::Vector3d pt;
    pt(0) = rand_pos_(eng_) * sample_r_(0);
    pt(1) = rand_pos_(eng_) * sample_r_(1);
    pt(2) = rand_pos_(eng_) * sample_r_(2);

    pt = rotation_ * pt + translation_;
    return pt;
}

bool TopoPRM::lineVisib(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, Eigen::Vector3d& pc, int caster_id)
{
    Eigen::Vector3d ray_pt;
    Eigen::Vector3d pt_id;

    casters_[caster_id].setInput(p1 / resolution_, p2 /resolution_);
    while(casters_[caster_id].step(ray_pt))
    {
        pt_id = (ray_pt + Eigen::Vector3d(0.5,0.5,0.5)) * resolution_;
        // pt_id(0) = (ray_pt(0) + Eigen::Vector3d(0.5,0.5,0.5)) * resolution_;
        // pt_id(1) = (ray_pt(1) + Eigen::Vector3d(0.5,0.5,0.5)) * resolution_;
        // pt_id(2) = (ray_pt(2) + Eigen::Vector3d(0.5,0.5,0.5)) * resolution_;
        // pt_id(0) = ray_pt(0) + offset_(0);
        // pt_id(1) = ray_pt(1) + offset_(1);
        // pt_id(2) = ray_pt(2) + offset_(2);
        int collision_id;
        if(pos_checker_->checkCollisionInSlideBox(pt_id,collision_id)) // 占据
        {
            return false;
        }
    }
    return true;
}


void TopoPRM::pruneGraph()
{
    /* prune useless node */
    if(graph_.size() > 2 )
    {
        for(list<GraphNode::Ptr>::iterator iter1 = graph_.begin(); iter1 != graph_.end() && graph_.size() > 2; ++iter1)
        {
            if((*iter1)->id_ <= 1) continue;

            /* core */
            if((*iter1)->neighbors_.size() <= 1)
            {
                // delete this node from others' neighbor
                for(list<GraphNode::Ptr>::iterator iter2 = graph_.begin(); iter2 != graph_.end(); ++iter2)
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

vector<vector<Eigen::Vector3d>> TopoPRM::pruneEquivalent(vector<vector<Eigen::Vector3d>>& paths)
{
    vector<vector<Eigen::Vector3d>> pruned_paths;
    if(paths.size() < 1) return pruned_paths;

    /* ---------- prune topo equivalent path ---------- */
    // output: pruned_paths
    vector<int> exist_paths_id;
    exist_paths_id.push_back(0);

    for(int i=0; i < paths.size();i++)
    {
        // compare with exsit paths
        bool new_path = true;
        for (int j = 0; j < exist_paths_id.size(); ++j) 
        {
            // compare with one path
            bool same_topo = sameTopoPath(paths[i], paths[exist_paths_id[j]], 0.0); //判断是否存在同拓扑路径

            if (same_topo) 
            {
                new_path = false;
                break;
            }
        }
        if (new_path) { // 如果没有同拓扑路径，就加入
            exist_paths_id.push_back(i);
        }
    }

    // save pruned paths
    for(int i=0;i<exist_paths_id.size();i++)
    {
        pruned_paths.push_back(paths[exist_paths_id[i]]);
    }

    std::cout << "[Topo]: prune equivalent paths: " << paths.size() << " -> " << pruned_paths.size() << std::endl;

    return pruned_paths;
}

vector<vector<Eigen::Vector3d>> TopoPRM::selectShortPaths(vector<vector<Eigen::Vector3d>>& paths,int step)
{
    /* ---------- only reserve top short path ---------- */
    vector<vector<Eigen::Vector3d>> short_paths;
    vector<Eigen::Vector3d> short_path;
    double min_len;

    for(int i = 0 ; i < reserve_num_ && paths.size() > 0 ; i++)
    {
        int path_id = shortestPath(paths);
        if(i == 0) // 如果是第一条，就直接加入，因为short paths里面是空的
        {
            short_paths.push_back(paths[path_id]);
            min_len = pathLength(paths[path_id]);
            paths.erase(paths.begin() + path_id);
        }
        else
        { // 否则就和最短路径比，如果路径长度超出了太多，就不加入
            double rat = pathLength(paths[path_id]) / min_len; 
            if(rat < ratio_to_short_)
            {
                short_paths.push_back(paths[path_id]);
                paths.erase(paths.begin() + path_id);
            }
            else
            {
                break;
            }
        }
    }

    std::cout << "in [Select short path], select path num: " << short_paths.size();

    for(int i=0; i< short_paths.size(); i++)
    {
        short_paths[i].insert(short_paths[i].begin(), start_pts_.begin(), start_pts_.end());
        short_paths[i].insert(short_paths[i].end(), end_pts_.begin(), end_pts_.end());
    }
    for (int i = 0; i < short_paths.size(); ++i) 
    {
        shortcutPath(short_paths[i], i, 5); // 5 表示迭代次数
        short_paths[i] = short_paths_[i];
    }

    short_paths = pruneEquivalent(short_paths);

    return short_paths;

}

bool TopoPRM::sameTopoPath(const vector<Eigen::Vector3d>& path1,const vector<Eigen::Vector3d>& path2, double thresh)
{
    // calc the length
    double len1 = pathLength(path1);
    double len2 = pathLength(path2);

    double max_len = max(len1, len2);

    int pt_num = ceil(max_len / resolution_);

    // std::cout << "pt num: " << pt_num << std::endl;

    vector<Eigen::Vector3d> pts1 = discretizePath(path1, pt_num);
    vector<Eigen::Vector3d> pts2 = discretizePath(path2, pt_num);

    Eigen::Vector3d pc;
    for (int i = 0; i < pt_num; ++i) 
    {
        if (!lineVisib(pts1[i], pts2[i], pc)) 
        {
            return false;
        }
    }


    return true;
}

int TopoPRM::shortestPath(vector<vector<Eigen::Vector3d>>& paths) {
    int short_id = -1;
    double min_len = 100000000;
    for (int i = 0; i < paths.size(); ++i) 
    {
        double len = pathLength(paths[i]);
        if (len < min_len) 
        {
            short_id = i;
            min_len = len;
        }
    }
    return short_id;
}
double TopoPRM::pathLength(const vector<Eigen::Vector3d>& path) {
    double length = 0.0;
    if (path.size() < 2) return length;

    for (int i = 0; i < path.size() - 1; ++i) 
    {
        length += (path[i + 1] - path[i]).norm();
    }
    return length;
}

vector<Eigen::Vector3d> TopoPRM::discretizePath(const vector<Eigen::Vector3d>& path, int pt_num)
{
    vector<double> len_list;
    len_list.push_back(0.0);

    for (int i = 0; i < path.size() - 1; ++i) {
        double inc_l = (path[i + 1] - path[i]).norm();
        len_list.push_back(inc_l + len_list[i]);
    }   

  // calc pt_num points along the path
    double len_total = len_list.back();
    double dl = len_total / double(pt_num - 1);
    double cur_l;

    vector<Eigen::Vector3d> dis_path;
    for (int i = 0; i < pt_num; ++i) 
    {
        /* 线性插值求中间路径 */
        cur_l = double(i) * dl; 

        // find the range cur_l in
        int idx = -1;
        for (int j = 0; j < len_list.size() - 1; ++j) 
        {
            if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) 
            {
                idx = j;
                break;
            }
        }

        // find lambda and interpolate
        double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
        Eigen::Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
        dis_path.push_back(inter_pt);
    }

    return dis_path;
}



vector<Eigen::Vector3d> TopoPRM::pathToGuidePts(vector<Eigen::Vector3d>& paths, int pt_num)
{
    return discretizePath(paths, pt_num);
}


void TopoPRM::shortcutPath(vector<Eigen::Vector3d> path, int path_id, int iter_num)
{
    /* 缩短路径，原路径曲折，通过ESDF缩短，*/
    vector<Eigen::Vector3d> short_path = path;
    vector<Eigen::Vector3d> last_path;


    for(int k=0; k< iter_num ; k++)
    {
        last_path = short_path;

        vector<Eigen::Vector3d> dis_path = discretizePath(short_path);

        if(dis_path.size() < 2)
        {
            short_paths_[path_id] = dis_path;
            return ;
        }


        /* visibiliry path shortening */

        // Eigen::Vector3d last_pt, new_pt;
        Eigen::Vector3d colli_pt, grad,dir,push_dir, temp_dir;
        int last_dis_id = 1;
        double risk;
        short_path.clear();
        short_path.push_back(dis_path.front()); // 先加上起始点
        
        for(int i=1;i<dis_path.size();i++)
        {
            if(lineVisib(short_path.back(),dis_path[i], colli_pt, path_id))
            {
                continue;
            }

            
            dir = (dis_path[i] - short_path.back()).normalized();
            double min_cos = 1000.0;
            int min_id = -1;
            for(int j = last_dis_id; j < i; j++)
            {
                temp_dir = (dis_path[j] - short_path.back()).normalized();
                double cos = fabs(dir.dot(temp_dir));
                if(cos < min_cos)
                {
                    min_cos = cos;
                    min_id = j;
                }
            }
            last_dis_id = min_id;
            push_dir = temp_dir;
            colli_pt = colli_pt + resolution_ * push_dir; 
            
            short_path.push_back(colli_pt);            
        }
        short_path.push_back(dis_path.back()); // 最后加上末尾点

        /* break if no shortcut */      
        double len1 = pathLength(last_path);
        double len2 = pathLength(short_path);
        if (len2 > len1) {
            // ROS_WARN("pause shortcut, l1: %lf, l2: %lf, iter: %d", len1, len2, k +
            // 1);
            short_path = last_path;
            break;
        }
    }
    short_paths_[path_id] = short_path;

}

void TopoPRM::shortcutPaths()
{
    short_paths_.resize(raw_paths_.size());
    if (parallel_shortcut_) 
    {
        vector<thread> short_threads;
        for (int i = 0; i < raw_paths_.size(); ++i)
        {
            short_threads.push_back(std::thread(&TopoPRM::shortcutPath, this, raw_paths_[i], i, 1));
        }
        for (int i = 0; i < raw_paths_.size(); ++i) 
        {
            short_threads[i].join();
        }
    } else {
        for (int i = 0; i < raw_paths_.size(); ++i)
        {
            shortcutPath(raw_paths_[i], i);
        }
    }

}

vector<Eigen::Vector3d> TopoPRM::discretizeLine(Eigen::Vector3d p1, Eigen::Vector3d p2) 
{
    Eigen::Vector3d dir = p2 - p1;
    double len = dir.norm();
    int seg_num = ceil(len / resolution_);

    vector<Eigen::Vector3d> line_pts;
    if (seg_num <= 0) 
    {
        return line_pts;
    }

    for (int i = 0; i <= seg_num; ++i)
    {
        line_pts.push_back(p1 + dir * double(i) / double(seg_num));
    } 
    return line_pts;
}

vector<Eigen::Vector3d> TopoPRM::discretizePath(vector<Eigen::Vector3d> path) 
{
    vector<Eigen::Vector3d> dis_path, segment;

    if (path.size() < 2) 
    {
        ROS_ERROR("what path? ");
        return dis_path;
    }

    for (int i = 0; i < path.size() - 1; ++i) 
    {
        segment = discretizeLine(path[i], path[i + 1]);

        if (segment.size() < 1)
        {
            continue;
        } 
        dis_path.insert(dis_path.end(), segment.begin(), segment.end());
        if (i != path.size() - 2)
        {
            dis_path.pop_back();
        } 
    }
    return dis_path;
}

vector<vector<Eigen::Vector3d>> TopoPRM::discretizePaths(vector<vector<Eigen::Vector3d>>& path) 
{
    vector<vector<Eigen::Vector3d>> dis_paths;
    vector<Eigen::Vector3d> dis_path;

    for (int i = 0; i < path.size(); ++i) {
        dis_path = discretizePath(path[i]);

        if (dis_path.size() > 0)
        {
            dis_paths.push_back(dis_path);
        }
    }   

    return dis_paths; 
}

vector<vector<Eigen::Vector3d>> TopoPRM::searchPaths()
{
    raw_paths_.clear();

    vector<GraphNode::Ptr> visited;
    visited.push_back(graph_.front());

    depthFirstSearch(visited);

    // sort the path by node number 
    int min_node_num = 10000, max_node_num = 1;
    vector<vector<int>> path_list(100);

    for(int i=0;i<raw_paths_.size();i++)
    {
        if(int(raw_paths_[i].size()) > max_node_num)
        {
            max_node_num = raw_paths_[i].size();
        }
        if(int(raw_paths_[i].size()) < min_node_num)
        {
            min_node_num = raw_paths_[i].size();
        }
        if(int(raw_paths_[i].size()) < 100)
        {
            path_list[int(raw_paths_[i].size())].push_back(i);
        }
        else{
            ROS_WARN("in [TopoPRM::searchPaths], path size: %d", int(raw_paths_[i].size()));
        }
        // path_list[int(raw_paths_[i].size())].push_back(i); // 这样不会内存非法访问吗？
    }

    // select paths with less nodes
    vector<vector<Eigen::Vector3d>> filter_raw_paths;
    for(int i= min_node_num; i<=max_node_num;i++)
    {
        bool reach_max = false;
        for(int j=0;j < path_list[i].size();j++)
        {
            filter_raw_paths.push_back(raw_paths_[path_list[i][j]]);
            if(filter_raw_paths.size() >= max_raw_path2_) // 如果路径过多,超出了限制,就break;
            {
                reach_max = true;
                break;
            }
        }
        if (reach_max) break;
    }

    std::cout << ", raw path num: " << raw_paths_.size() << ", " << filter_raw_paths.size();

    raw_paths_ = filter_raw_paths;

    return raw_paths_;

}


void TopoPRM::depthFirstSearch(vector<GraphNode::Ptr>& vis)
{
    GraphNode::Ptr cur_node = vis.back();
    
    // check reach goal
    for(int i=0; i<cur_node->neighbors_.size();i++)
    {
        if(cur_node->neighbors_[i]->id_ == 1)
        {      
            // add this path to paths set
            vector<Eigen::Vector3d> path;
            for(int j=0 ; j <vis.size(); j++)
            {
                path.push_back(vis[j]->pos_);
            }
            path.push_back(cur_node->neighbors_[i]->pos_); // last point 

            raw_paths_.push_back(path);
            if(raw_paths_.size() >= max_raw_path_) return;

            break;

        }
    }

    for(int i=0 ; i < cur_node->neighbors_.size(); i++)
    {
        // skip reach goal
        if(cur_node->neighbors_[i]->id_ == 1) continue;

        // skip already visited node
        bool revisit = false;
        for(int j=0; j <vis.size();j++)
        {
            if(cur_node->neighbors_[i]->id_ == vis[j]->id_)
            {
                revisit = true;
                break;
            }
        }

        if(revisit) continue;

        // recursive search
        vis.push_back(cur_node->neighbors_[i]);
        depthFirstSearch(vis); 
        if(raw_paths_.size() >= max_raw_path_) return; // 出来以后检测

        vis.pop_back();
    }

}


void TopoPRM::getBox(Eigen::Vector3d &pt, Eigen::Vector3d &scale, Eigen::Quaterniond &quat)
{
    pt = translation_;
    scale = sample_r_;
    quat = rotation_;
}
