#include <lane_extractor/LaneExtractorNode.h>

namespace lane_extractor
{
LaneExtractorNode::LaneExtractorNode()
{
  ros::NodeHandle nh;
  
  // provide callbacks
  MapSub = nh.subscribe<sensor_msgs::PointCloud2>("map", 100, &LaneExtractor::mapCallback, &le);
  PoseSub = nh.subscribe<geometry_msgs::PoseStamped>("ndt_pose", 1, &LaneExtractor::ndtPoseCallback, &le);

  FindLaneService = nh.advertiseService("find_lane", &LaneExtractor::FindLaneService, &le);
  SaveLaneService = nh.advertiseService("save_lane", &LaneExtractor::SaveLaneService, &le);

}

LaneExtractorNode::~LaneExtractorNode()
{}
}