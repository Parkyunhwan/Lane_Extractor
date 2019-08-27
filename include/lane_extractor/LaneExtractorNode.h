#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <lane_extractor/LaneExtractor.h>
#include <pcl/common/centroid.h>

namespace lane_extractor
{

class LaneExtractorNode
{
public:
  LaneExtractorNode();
  LaneExtractorNode(char* file);
  virtual ~LaneExtractorNode();

protected:
  LaneExtractor le;
  ros::NodeHandle nh;

  ros::Subscriber MapSub;//Map Messages Subscriber
  ros::Subscriber PoseSub;//Pose Messages Subscriber
  ros::ServiceServer FindLaneService;//Lane find service
  ros::ServiceServer SaveLaneService;//Lane save service

};
}