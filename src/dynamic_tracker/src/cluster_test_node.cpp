#include <dynamic_tracker/cluster_tester.hpp>
#include <iostream>

using namespace std;
using namespace cluster_tester;


int main(int argc,char ** argv){

    setlocale(LC_ALL,"");
    // initialize node
    ros::init(argc, argv, "cluster_test_node");
    ros::NodeHandle nh;

    ClusterTester::Ptr ct_ptr;
    ct_ptr.reset(new ClusterTester());
    ct_ptr->init(nh);



    ros::spin();

    return 0;
}