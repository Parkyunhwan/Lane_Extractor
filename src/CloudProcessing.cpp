#include <lane_extractor/CloudProcessing.h>

namespace lane_extractor
{
    CloudProcessing::CloudProcessing()
    : cloud(new pcl::PointCloud<pcl::PointXYZI>)
    , cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>)
    , Intesity_Cloud(new pcl::PointCloud<pcl::PointXYZI>)
    , leaf_size(1.0)
    {

        map_pub = nh.advertise<sensor_msgs::PointCloud2> ("Point_map", 10);
        lane_pub = nh.advertise<sensor_msgs::PointCloud2> ("lane_pub", 10);

        nh.getParam("/file_path", file_path);
        CloudLoader();

       // MapDownsampling(false);
        map_publish();
    }

    void CloudProcessing::CloudLoader(){
            pcl::io::loadPCDFile<pcl::PointXYZI> (file_path.c_str(), *cloud);
            ROS_INFO("Cloud Loading Complete!");
    }

    void CloudProcessing::passthrough()
    {
        pcl::PassThrough<pcl::PointXYZI> pass;
        pass.setInputCloud(cloud);
        // pass.setFilterFieldName ("x");
        // pass.setFilterLimits (-150.0, 150.0);
        // pass.filter(*cloud);
            
        // pass.setFilterFieldName ("y");
        // pass.setFilterLimits (-100.0, 300.0);
        // pass.filter(*cloud);

        pass.setFilterFieldName ("z");
        pass.setFilterLimits (-1.0, 1.0);
        pass.filter(*cloud);

        ROS_INFO("Z axis -25 / +25(cm) Passthrough Complete!");
    }

    void CloudProcessing::VoxelGridFilter(){
            pcl::VoxelGrid<pcl::PointXYZI> voxel;
            voxel.setInputCloud(cloud);
            voxel.setLeafSize(leaf_size,leaf_size,leaf_size);
            voxel.filter(*cloud);

            ROS_INFO("%fM VoxelGridFilter Complete!",leaf_size);
    }

    void CloudProcessing::MapDownsampling(bool voxel){
              passthrough();
              
            //   if(voxel){
            //    VoxelGridFilter();
            //   }
            //   ROS_INFO("downsampling Complete!");
    }
    void CloudProcessing::fromMsgToCloud(const sensor_msgs::PointCloud2 &msg){
        sensor_msgs::PointCloud2 c_msg = msg;
        c_msg.fields[3].name = "intensity";
        pcl::fromROSMsg(c_msg, *cloud);
    }

    void CloudProcessing::map_publish(){
                pcl::toROSMsg(*cloud, cloud_msg);
                cloud_msg.header.frame_id = "map";
                map_pub.publish(cloud_msg);
    }

    void CloudProcessing::lane_publish(){
                pcl::toROSMsg(*Intesity_Cloud, Intensity_msg);
                Intensity_msg.header.frame_id="map";
                lane_pub.publish(Intensity_msg);
    }

    // void CloudProcessing::CloudSaver(float intensity_min, float intensity_max)
    // {
    //     strcat(name,"");
    //     pcl::PCDWriter writer;
    //     writer.writeBinaryCompressed("",cloud);
    //     // pcl::io::savePCDFileASCII ("test_pcd.pcd", cloud);
    // }

    // void laneExtract::lane_exclusion_publish(){
    //             pcl::toROSMsg(*cloud_filtered, a_msg);
    //             a_msg.header.frame_id="map";
    //             a_pub.publish(a_msg);
    // }   

}