#include <iostream>
#include "plan_env/dsp_map.h"
#include "plan_env/particle_map.h"


using namespace std;
using namespace dsp_map;
using namespace particle_map;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_dspmap");
    ros::NodeHandle nh("~");
    ROS_INFO("HELLO ");
    // DspMap::Ptr mymap;
    // mymap.reset(new DspMap);
    // mymap->initMap(nh);
    ParticleMap::Ptr particlemap;
    particlemap.reset(new ParticleMap);
    particlemap->initMap(nh);

    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}