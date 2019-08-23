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
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud; 
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;
            pcl::PointCloud<pcl::PointXYZI>::Ptr Intesity_Cloud;
        //pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered1;
        //pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
            sensor_msgs::PointCloud2 cloud_msg;
            sensor_msgs::PointCloud2 Intensity_msg;
            sensor_msgs::PointCloud2 cloud_filtered_msg;
            ros::Publisher map_pub;
            ros::Publisher lane_pub;
            ros::Publisher cloud_filtered_pub;
            ros::Publisher marker_pub;

            ros::NodeHandle nh;

            float leaf_size;
            std::string file_path;
            SearchInfo searchinfo;
            pcl::PointXYZI searchPoint[5];
            //pcl::PointXYZI CurrentPointBox[5];

        public:
            CloudProcessing();
            void CloudLoader();
            void passthrough();
            void VoxelGridFilter();
            void MapDownsampling(bool voxel);

            void fromMsgToCloud(const sensor_msgs::PointCloud2 &cloud);

            void map_publish();
            void lane_publish();
            void cloud_filtered_publish();
            void marker_publish(tf::Transform &pose);
            // void lane_exclusion_publish();

    };

    
}