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
#include <lane_extractor/saveinfo.h>
#include <pcl/common/centroid.h>
#include <queue>


namespace lane_extractor
{

class LaneExtractor
{
    public:
        LaneExtractor(); //Constructor
        virtual ~LaneExtractor();
        void mapCallback(const sensor_msgs::PointCloud2ConstPtr& pc);
        void ndtPoseCallback(const geometry_msgs::PoseStampedConstPtr& pc);
        bool FindLaneService(lane_extractor::srvinfo::Request  &req, lane_extractor::srvinfo::Response &res);
        bool SaveLaneService(lane_extractor::saveinfo::Request &req,lane_extractor::saveinfo::Response &res);
        void tfBroadcaster(const tf::Transform &transform,const std::string &from_link,const std::string &to_link);
        int RadiusSearch(int SearchLine,float rad);
        tf::Transform setSearchEv();
        void setSearchPoint(const tf::Transform &tr,const int &line);
        double ToEulerAngles(tf::Quaternion q);
        double pointAndLineDistance2D (const Eigen::Vector2f &point_arg, const Eigen::Vector3d &line_arg);
        double getPointToDistance(const Eigen::Vector2f &poseP, const double &yaw, const Eigen::Vector2f &targetP );
        double getSlope (const double &dir_arg);
        bool getStraightLineEquation2D (const Eigen::Vector2f &point_arg, const double &slope_arg, Eigen::Vector3d &ret_line_arg);
        double getDeviation (const Eigen::Vector2f &start_pt, const Eigen::Vector2f &candidate_pt, const Eigen::Vector3d &line_coef);
        double calDiffbtQuarternion(const tf::Quaternion &q1,const tf::Quaternion &q2,int &rot_direction);
        int lineRadiusSearch(pcl::PointXYZI &centerpoint,std::vector<pcl::PointXYZI> &s_point,int searchlane);
        int CreateSideLane(tf::Transform &poseTransform,int left, int right);

    private:
        CloudProcessing cp;
        geometry_msgs::Pose _pose;
        std::queue<geometry_msgs::Pose> ndt_pose;
        int rotation_direction;
        double tan_yaw;
        int L_lane_break;
        int R_lane_break;
        int L_continuos_line;
        int R_continuos_line;
        enum { CENTER=0,LEFT=1,RIGHT=2,MULTILEFT=3,MULTIRIGHT=4,LANE_BREAK_LIMIT=3};
        // static const int CENTER =0;
        // static const int LEFT =1;
        // static const int RIGHT =2;
        // static const int MULTILEFT =3;
        // static const int MULTIRIGHT =4;
        // static const int LANE_BREAK_LIMIT =3;
        pcl::PointXYZI save_point;
};


}