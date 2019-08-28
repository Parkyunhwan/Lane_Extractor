#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/io/pcd_io.h>
#include <sstream>

namespace lane_extractor
{

    typedef struct SearchInfo{
        float radius;
        double min_intensity;
        double max_intensity;
    }SearchInfo;
    class CloudProcessing
    {

        public:
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud; //Cloud with source map
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered; //Points before center of gravity processing for debugging
            pcl::PointCloud<pcl::PointXYZI>::Ptr Intesity_Cloud; //lane cloud
            std::string file_path; //pcd file path
            SearchInfo searchinfo; //radius , intensity_bound information
            pcl::PointXYZI searchPoint[5]; // lane_search_point buffer

            CloudProcessing(); 
            void CloudLoader();
            void MapDownsampling(bool voxel);
            void fromMsgToCloud(const sensor_msgs::PointCloud2 &cloud);
            void map_publish();
            void lane_publish();
            void cloud_filtered_publish();
            void marker_publish(tf::Transform &pose); //for debugging
            bool CloudSaver(int num);

        private:
            void passthrough();
            void VoxelGridFilter();
            sensor_msgs::PointCloud2 cloud_msg;
            sensor_msgs::PointCloud2 Intensity_msg;
            sensor_msgs::PointCloud2 cloud_filtered_msg;
            ros::Publisher map_pub;
            ros::Publisher lane_pub;
            ros::Publisher cloud_filtered_pub;
            ros::Publisher marker_pub;

            ros::NodeHandle nh;

            float leaf_size; //for voxel filter
    };

    
}