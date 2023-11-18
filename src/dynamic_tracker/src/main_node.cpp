#include <dynamic_tracker/dynamic_tracker.h>
#include <iostream>

using namespace std;
using namespace dynamic_tracker;


int main(int argc,char ** argv){

    // initialize node
    ros::init(argc, argv, "dynamic_tracker");
    ros::NodeHandle nh;

    DynamicTracker::Ptr dt_ptr;
    dt_ptr.reset(new DynamicTracker);
    dt_ptr->init(nh);



    ros::spin();

    return 0;
}