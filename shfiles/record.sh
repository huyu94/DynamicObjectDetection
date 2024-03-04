rosbag record --tcpnodelay \
/drone_0_odom_visualization/path \
/drone_0_node/init_list \
/drone_0/odometry \
/dynamic_env/global_cylinders_state \
/drone_0_node/slide_box \
/drone_0_node/kalman_filter \
/drone_0_odom_visualization/robot \
/drone_0_node/optimal_list \
-o ~/bags/chapter3/traj.bag