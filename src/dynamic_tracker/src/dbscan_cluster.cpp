#include "dynamic_tracker/dbscan_cluster.hpp"
#include <queue>


namespace dbscan
{
    const float DBSCANCluster::rad = 0.017453292;

    int DBSCANCluster::radiusSearch(const pcl::PointXYZI &searchPoint,const double radius,std::vector<int> &pointIdxRadiusSearch,std::vector<float> &pointRadiusSquaredDistance)
    {
        return kdtree.radiusSearch(searchPoint,radius,pointIdxRadiusSearch,pointRadiusSquaredDistance);
    };

    bool DBSCANCluster::checkInputs(float epsilon)
    {
        if(epsilon < 0.5 || epsilon > 2.0){
            return false;
        }
        return true;
    }

    void DBSCANCluster::core(unsigned int minPts,std::vector<float>& eps_list,pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,std::vector<int> &clusterIds)
    {
        int clusterIndex = 0;
        // 表示点云属于哪个簇，-1表示噪声，0是初始的，一开始全是噪声
        // std::vector<int> clusterIds(input_cloud->points.size(),0);
        kdtree.setInputCloud(input_cloud);
        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_distances;
        std::vector<bool> visited(input_cloud->points.size(),false); //在while循环里标记这个点有没有被加入到数组中过，每次while循环结束则全部置为false。
        clusterIds.resize(input_cloud->points.size());
        for(int i= 0;i<input_cloud->points.size();i++)
        {
            // !=0 说明该点已经被分配到了某个簇内，或者是噪音点，对于噪音点，我们也不将其加入到后续的访问队列中，所以visited[i]=true
            if(clusterIds[i] != 0)
            {
                visited[i] = true; // 减少了判断
                continue;
            }
            // 近邻搜索
            int neighbor_size = radiusSearch(input_cloud->points[i],eps_list[i],neighbor_indices,neighbor_distances);
            //density check 如果这个点周围的点大于等于min_pts_,被判定为核心点，这个时候需要将它的邻居加入到队列里，同时将它赋到簇；如果小于，则是边界点，我们不看它的邻居。
            if(neighbor_size < minPts) // 圆内邻居点不够,噪音点
            {
                clusterIds[i] = -1; // 标记为噪声点
                continue;
            }
            else{
                //圈内点数够，标记为核心点 并赋予簇号
                clusterIndex ++;
                clusterIds[i] = clusterIndex;

                //seed_queue用来存放我们后续需要操作的点
                std::queue<int> seed_queue;

                // 对于核心点的所有邻居，我们后续都要检查，所以先放进seed_queue中
                for(int j=0;j<neighbor_size;j++) 
                {
                    if(neighbor_indices[j] != i)
                    {
                        if(!visited[neighbor_indices[j]] && clusterIds[neighbor_indices[j]] == 0) // 如果这个点
                        {
                            seed_queue.push(neighbor_indices[j]);
                            visited[neighbor_indices[j]] = true;
                        }
                    }
                }
                //对周围邻居点进行核心点检查
                while(!seed_queue.empty())
                {   
                    // 取出邻居之一
                    int neighbor_point = seed_queue.front();
                    seed_queue.pop();

                    //不管怎么样，先把邻居加到这个簇内
                    clusterIds[neighbor_point] = clusterIndex;

                    // 判断这个邻居是不是核心点
                    neighbor_size = radiusSearch(input_cloud->points[neighbor_point],eps_list[neighbor_point],neighbor_indices,neighbor_distances);
                    if(neighbor_size >= minPts) //如果是核心点
                    {
                        for(int j=0;j<neighbor_size;j++)
                        {
                            if(!visited[neighbor_indices[j]])
                            {
                                seed_queue.push(neighbor_indices[j]);
                                visited[neighbor_indices[j]] = true;                            
                            }
                            // std::cout << 1 << std::endl;
                        }
                    }
                    // std::cout << seed_queue.size() << std::endl;
                    //不然就是边界点，加到簇里就不管它了
                }
                std::fill(visited.begin(),visited.end(),false);
            }
        }

    }


    void DBSCANCluster::boundEpsilon(float& epsilon)
    {
        epsilon = std::min(std::max(epsilon,min_eps_),max_eps_);
    }


    void DBSCANCluster::run(pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,std::vector<int> &clusterIds,bool adptive)
    {           
        unsigned int minPts = pts_;
        std::vector<float> eps_list(input_cloud->size());
        if(adptive){
            for(size_t i = 0;i < input_cloud->size();i++){
                eps_list[i] = input_cloud->points[i].intensity * sqrt(2) * rad * 8;
                std::cout << eps_list[i] << " ";

                boundEpsilon(eps_list[i]);
            }
            std::cout << std::endl;



        }
        else{
            for(size_t i = 0;i < input_cloud->size();i++){
                eps_list[i] = default_eps_;
            }
        }
        // std::cout << "min_pts : " << minPts << std::endl;
        // for(size_t i = 0;i < input_cloud->size();i++){
        //     std::cout << eps_list[i] << " ";
        // }
        // std::cout << std::endl;
        core(minPts,eps_list,input_cloud,clusterIds);
    }

} // namespace DBSCAN
