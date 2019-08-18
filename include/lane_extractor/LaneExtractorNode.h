#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <lane_extractor/LaneExtractor.h>
#include <pcl/common/centroid.h>
// #include <tf2_ros/transform_broadcaster.h>
namespace lane_extractor
{
/**
 * @brief Wrapper class for FootstepPlanner, providing callbacks for
 * the node functionality.
 */
class LaneExtractorNode
{
public:
  LaneExtractorNode();
  LaneExtractorNode(char* file);
  virtual ~LaneExtractorNode();

protected:
  LaneExtractor le;
  ros::NodeHandle nh;

  ros::Subscriber MapSub;
//   ros::Subscriber leftPoseSub;
//   ros::Subscriber rightPoseSub;
  ros::Subscriber PoseSub;

  ros::ServiceServer FindLaneService;
//   ros::ServiceServer ivFootstepPlanFeetService;
};
}