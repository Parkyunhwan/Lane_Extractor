#include <ros/ros.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>


int main(int argc, char** argv){
//USAGE : rosrun package_name node_name PCD_file_path, new_file_name, min_intensity, max_intensity//
  ros::init(argc, argv, "Intensity_cloud_saver");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::io::loadPCDFile<pcl::PointXYZI> (argv[1], *cloud);
  ROS_INFO("Cloud Loading Complete!");

    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName ("intensity");
    pass.setFilterLimits ( FLT_MIN, atof(argv[3]) );//atof(argv[3])
    //pass.setFilterLimits ( FLT_MAX, atof(argv[4]) );
    pass.setNegative(true);
    pass.filter(*cloud);

     pcl::PCDWriter writer;
     writer.writeBinaryCompressed(argv[2], *cloud);

    ROS_INFO("Intensity Passthrough [%.2f < 'intensity' < %.2f]  Passthrough Complete!",atof(argv[3]),atof(argv[4]));
}