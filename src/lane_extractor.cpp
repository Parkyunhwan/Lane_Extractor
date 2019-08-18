#include <lane_extractor/LaneExtractorNode.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_extractor");
  ros::NodeHandle nh;

  if(argc==2){
    std::string str(argv[1]);
    nh.param<std::string>("file_path",str,"/home/autoware/shared_dir/20190420_kcity_0.2.pcd");
  }
  else{
    std::string str;
    nh.setParam("file_path","/home/autoware/Autoware/ros/test450.pcd");
  }
    
    
    
  lane_extractor::LaneExtractorNode extractor;
   while(nh.ok()){
        
        ros::spinOnce();
    }
  return 0;
}