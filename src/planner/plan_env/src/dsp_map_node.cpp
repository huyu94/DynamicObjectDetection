#include <iostream>
#include "plan_env/dsp_map.h"
#include <plan_env/particle_map.hpp>

using namespace std;
using namespace ego_planner;
int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_dspmap");
    ros::NodeHandle nh("~");
    ParticleMap::Ptr map;
    map.reset(new ParticleMap);
    map->initMap(nh);

    // DspMap::Ptr mymap;
    // mymap.reset(new DspMap);
    // mymap->initMap(nh);

    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}