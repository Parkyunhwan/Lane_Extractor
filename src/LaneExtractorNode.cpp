#include <lane_extractor/LaneExtractorNode.h>

namespace lane_extractor
{
LaneExtractorNode::LaneExtractorNode()
{
  ros::NodeHandle nh;
  // provide callbacks to interact with the footstep planner:
  MapSub = nh.subscribe<sensor_msgs::PointCloud2>("map", 100, &LaneExtractor::mapCallback, &le);
  PoseSub = nh.subscribe<geometry_msgs::PoseStamped>("ndt_pose", 1, &LaneExtractor::ndtPoseCallback, &le);
//   PoseSub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, &FootstepPlanner::startPoseCallback, &laneExtractor);
//   leftPoseSub = 
//   // service:
  FindLaneService = nh.advertiseService("find_lane", &LaneExtractor::FindLaneService, &le);
  SaveLaneService = nh.advertiseService("save_lane", &LaneExtractor::SaveLaneService, &le);
//   ivFootstepPlanFeetService = nh.advertiseService("plan_footsteps_feet", &FootstepPlanner::planFeetService, &ivFootstepPlanner);
}

LaneExtractorNode::~LaneExtractorNode()
{}
}