# depth2gridmapROS
Visualize depth and color images on ROS rviz as 3d point cloud and gridmap

# sample data
data/rgb: data_odometry_rgb sequence 0 of kitti dataset (http://www.cvlibs.net/datasets/kitti/)<br />
data/depth: depth map generated from struct2depth. (https://github.com/tensorflow/models/tree/master/research/struct2depth)<br />
data/data_odometry_poses: from http://www.cvlibs.net/datasets/kitti/

# Run
After build, modify launch file and run : roslaunch depth2gridmap depthpose2grid.launch