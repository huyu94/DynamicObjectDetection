#include <iostream>
#include "plan_env/ikd-Tree/ikd_Tree.h"


using namespace std;


// struct PointType : public ikdTree_PointType
// {
//     float x,y,z;
//     // float label;
//     // float visited; // used for DBSCAN cluster 
//     // float temp;
//     PointType (float px = 0.0f, float py = 0.0f, float pz = 0.0f): 
//                 ikdTree_PointType(px,py,pz){
//     }
// };

using PointType = ikdTree_PointType;
using PointVector = KD_TREE<PointType>::PointVector;

struct Tcc
{
    int a;
    int b;
    int c;
};

struct Point 
{
    int x;
    int y;
    int z;
};
int main()
{
    // KD_TREE<PointType>::Ptr kd_tree_ptr(new KD_TREE<PointType>);
    // PointType p(1,2,3);
    // PointVector v;
    // v.push_back(p);
    // kd_tree_ptr->Build(v);
    // PointVector result;
    // kd_tree_ptr->Radius_Search(p,2,result);
    // for(auto &p : result)
    // {
    //     cout << p.x << " " << p.y << " " << p.z << endl;
    //     cout << p.visited << endl;
    //     cout << p.label << endl;
    // }


    
    return 0;
}