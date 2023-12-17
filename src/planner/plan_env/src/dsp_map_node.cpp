#include <iostream>
#include "plan_env/dsp_map.h"


using namespace std;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"test_dspmap");
    ros::NodeHandle nh("~");
    DspMap::Ptr mymap;
    mymap.reset(new DspMap);
    mymap->initMap(nh);

    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}