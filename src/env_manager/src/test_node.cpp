#include <iostream>

#include "env_manager/env_manager.h"



int main(int argc, char ** argv)
{
    ros::init(argc, argv, "env_node");
    ros::NodeHandle nh("~");

    EnvManager::Ptr env_manager_ptr;
    env_manager_ptr.reset(new EnvManager());
    env_manager_ptr->init(nh);


    ros::spin();

    return 0;
}