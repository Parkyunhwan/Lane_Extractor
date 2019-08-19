#include <ros/ros.h>
#include <lane_extractor/CloudProcessing.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/extract_indices.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <lane_extractor/srvinfo.h>
#include <pcl/common/centroid.h>
#include <queue>


namespace lane_extractor
{

class LaneExtractor
{
    public:
        LaneExtractor();
        virtual ~LaneExtractor();
        void mapCallback(const sensor_msgs::PointCloud2ConstPtr& pc);
        void ndtPoseCallback(const geometry_msgs::PoseStampedConstPtr& pc);
        bool FindLaneService(lane_extractor::srvinfo::Request  &req, lane_extractor::srvinfo::Response &res);

        void tfTransform(const geometry_msgs::PoseStampedConstPtr& ptr);
        void RadiusSearch(int SearchNum);
        void set_searchPoint();
        double ToEulerAngles(tf::Quaternion q);
        double pointAndLineDistance2D (const Eigen::Vector2f &point_arg, const Eigen::Vector3d &line_arg);
        double getPointToDistance(const Eigen::Vector2f &poseP, const double &yaw, const Eigen::Vector2f &targetP );
        double getSlope (const double &dir_arg);
        bool getStraightLineEquation2D (const Eigen::Vector2f &point_arg,
                                                                const double &slope_arg,
                                                                Eigen::Vector3d &ret_line_arg);
        double getDeviation (const Eigen::Vector2f &start_pt, const Eigen::Vector2f &candidate_pt, const Eigen::Vector3d &line_coef);
        CloudProcessing cp;
        geometry_msgs::Pose _pose;
        std::queue<geometry_msgs::Pose> ndt_pose;
        tf::Transform last_pose;
        int rot;
        double tan_yaw;
        pcl::PointXYZI save_point;
};


}