# data-verification
This project aim to check data for lidar slam algorithm (lio-sam)
A tool box to verify data set, use checklist.yaml to set every data's path.

Data set being checked can include the following data:
1. ROSBAG. At most include pointcloud msg, imu msg and gnss msg;
2. PCD files. Pointcloud data types in file;
3. CONFIG FILE. Like lio-sam params.yaml.

Including modules:
1. common parts. This part include an enum problem struct, which can be easily matched. And several pointcloud type for XYZIRT;
2. gnss check class. This part check rosbag gnss msg status and data quality;
3. imu check class. This part check rosbag imu msg status and data quality;
4. pcd check class. This part check pcd files' quality;
5. rosbag check class. This part check rosbag pointcloud msg and data quality;
6. yaml check class. This part print msg information as written in param.yaml.


usage:
1. catkin_make
2. source devel/setup.bash
3. rosrun data_check data_check checklist.yaml
