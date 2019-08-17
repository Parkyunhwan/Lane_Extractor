#include <lane_extractor/LaneExtractor.h>
// #include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

namespace lane_extractor
{
    LaneExtractor::LaneExtractor()
    {
       ros::NodeHandle nh;
       std::cout << "aa" << std::endl;
    }
    LaneExtractor::~LaneExtractor()
    {}
    

    void LaneExtractor::mapCallback(const sensor_msgs::PointCloud2ConstPtr& pc)
    {
        cp.fromMsgToCloud(*pc);
        //pcl::fromROSMsg(*pc, *cloud);
        //cp.MapDownsampling(false); // Voxel true or false.. (param)
        cp.map_publish();
    }

    void LaneExtractor::ndtPoseCallback(const geometry_msgs::PoseStampedConstPtr& ptr)
    {   
        std::cout << "ndt_pose subscribe.." << std::endl;
        static int i=0;
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::TransformStamped transformMaptoleftlink;
        

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "pose_link";
        transformStamped.transform.translation.x = ptr->pose.position.x;
        transformStamped.transform.translation.y = ptr->pose.position.y;
        transformStamped.transform.translation.z = ptr->pose.position.z;

        transformStamped.transform.rotation = ptr->pose.orientation;
        br.sendTransform( transformStamped);
        tf::Transform final_transform;

        tf::Vector3 poseT(ptr->pose.position.x, ptr->pose.position.y, ptr->pose.position.z);
        tf::Quaternion poseQ(ptr->pose.orientation.x,ptr->pose.orientation.y,ptr->pose.orientation.z,ptr->pose.orientation.w);
        tf::Transform poseTransform(poseQ,poseT);

        tf::Vector3 leftT(0.0, 1.7, 0.0);
        tf::Transform leftTransform(tf::Quaternion(0.0, 0.0, 0.0) ,leftT);
        final_transform = poseTransform * leftTransform;

        tf::TransformBroadcaster broadcaster;
        broadcaster.sendTransform(
        tf::StampedTransform(final_transform,ros::Time::now(),
        "map", "left_link"));

        tf::Vector3 rightT(0.0, -1.7, 0.0);
        tf::Transform rightTransform(tf::Quaternion(0.0, 0.0, 0.0) ,rightT);
        final_transform = poseTransform * rightTransform;

        tf::TransformBroadcaster broadcaster1;
        broadcaster1.sendTransform(
        tf::StampedTransform(final_transform,ros::Time::now(),
        "map", "right_link"));

        if(i%10==0)
        {
        geometry_msgs::Pose current_pose = ptr->pose;
        //current_pose.position.z = 0.0;
        ndt_pose.push(current_pose);
        
        pcl::PointXYZI searchPoint;
        searchPoint.x = ndt_pose.back().position.x;
        searchPoint.y = ndt_pose.back().position.y;
        searchPoint.z = ndt_pose.back().position.z;

    //std::cout << "search num -> "<< i << endl;
        std::cout << searchPoint.x << "  "
         << searchPoint.y << "  "
         << searchPoint.z << " end \n";
        }
        i++;
    }

    bool LaneExtractor::FindLaneService(lane_extractor::srvinfo::Request  &req,
                                        lane_extractor::srvinfo::Response &res)
    {
        //setting serachinfo
        std::cout << "start "<< std::endl;
        cp.searchinfo.radius = req.rad;
        cp.searchinfo.min_intensity = req.minIntensity;
        cp.searchinfo.max_intensity = req.maxIntensity;
        
        /////////////////////////////////search
        while(!ndt_pose.empty()){
            set_searchPoint(); // search point 설정
            RadiusSearch(1);
            RadiusSearch(2);
            RadiusSearch(3);
            RadiusSearch(4);
            cp.lane_publish();
        }

                std::cout << "Ridius Search Done.. "<< std::endl;
        return true;
    }
    void LaneExtractor::RadiusSearch(int SearchNum)
    {
        if(SearchNum<0 && SearchNum>4){
            ROS_INFO("SearchNum Error...");
            return;
        }
        if(SearchNum==1){
            std::cout << "left_lane search.." <<std::endl;
        }
        else if(SearchNum==2){
            std::cout << "right_lane search.." <<std::endl;
        }


        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(cp.cloud);

        // pcl::ExtractIndices<pcl::PointXYZI> extract;
        // extract.setInputCloud(cp.cloud);
         //kdtree에 cloud 설정
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
                if( kdtree.radiusSearch(cp.searchPoint[SearchNum], cp.searchinfo.radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) //하나라도 search된경우
                {
                    //  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
                        int k=0;
                        
                       for(size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
                       {
                        //    std::cout << "intensity : "<< cp.cloud->points[ pointIdxRadiusSearch[i] ].intensity << std::endl;
                        //    std::cout << "min_intensity : "<< cp.searchinfo.min_intensity << std::endl;
                           if((cp.cloud->points[pointIdxRadiusSearch[i]].intensity > cp.searchinfo.min_intensity) )//&& (cp.cloud->points[pointIdxRadiusSearch[i]].intensity < cp.searchinfo.max_intensity)
                              {
                                 cp.Intesity_Cloud->points.push_back(cp.cloud->points[ pointIdxRadiusSearch[i] ]);
                                k++;//  inliers->indices.push_back(pointIdxRadiusSearch[i]);
                              }
                              
                       }
                        std::cout << "---intensity size---   "<< k << std::endl;
                        std::cout << "---Intensity Cloud size---   "<< cp.Intesity_Cloud->points.size() << std::endl;

                    //    extract.setIndices(inliers);
                    //    extract.setNegative(true);
                    //    extract.filter(*cp.cloud);
                    //    inliers->indices.clear();
                    std::cout << "---search size---   "<< pointIdxRadiusSearch.size() << std::endl << std::endl;
                }
    }


    void LaneExtractor::set_searchPoint()
    {
    static int i = 0;

        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        geometry_msgs::TransformStamped transformMaptoleftlink;
        
        //center_search
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "center_search";
        transformStamped.transform.translation.x = ndt_pose.front().position.x;
        transformStamped.transform.translation.y = ndt_pose.front().position.y;;
        transformStamped.transform.translation.z = ndt_pose.front().position.z;;

        transformStamped.transform.rotation =  ndt_pose.front().orientation;
        br.sendTransform( transformStamped); 

        tf::Transform final_Ltransform;
        tf::Transform final_Rtransform;
        tf::Transform final_LLtransform;
        tf::Transform final_RRtransform;

        tf::Vector3 poseT( ndt_pose.front().position.x,  ndt_pose.front().position.y,  ndt_pose.front().position.z);
        tf::Quaternion poseQ( ndt_pose.front().orientation.x, ndt_pose.front().orientation.y, ndt_pose.front().orientation.z, ndt_pose.front().orientation.w);
        tf::Transform poseTransform(poseQ,poseT);

        //left_search
        tf::Vector3 leftT(0.0, 1.7, 0.0);
        tf::Transform leftTransform(tf::Quaternion(0.0, 0.0, 0.0) ,leftT);
        final_Ltransform = poseTransform * leftTransform;

        tf::TransformBroadcaster broadcaster;
        broadcaster.sendTransform(
        tf::StampedTransform(final_Ltransform,ros::Time::now(),
        "map", "left_search"));

        //right_search
        tf::Vector3 rightT(0.0, -1.7, 0.0);
        tf::Transform rightTransform(tf::Quaternion(0.0, 0.0, 0.0) ,rightT);
        final_Rtransform = poseTransform * rightTransform;

        tf::TransformBroadcaster broadcaster1;
        broadcaster1.sendTransform(
        tf::StampedTransform(final_Rtransform,ros::Time::now(),
        "map", "right_search"));

        //leftleft_search
        tf::Vector3 leftleftT(0.0, (1.7+3.4), 0.0);
        tf::Transform leftleftTransform(tf::Quaternion(0.0, 0.0, 0.0) ,leftleftT);
        final_LLtransform = poseTransform * leftleftTransform;

        tf::TransformBroadcaster broadcaster2;
        broadcaster.sendTransform(
        tf::StampedTransform(final_LLtransform,ros::Time::now(),
        "map", "leftleft_search"));

        //rightright_search
        tf::Vector3 rightrightT(0.0, (-1.7-3.4), 0.0);
        tf::Transform rightrightTransform(tf::Quaternion(0.0, 0.0, 0.0) ,rightrightT);
        final_RRtransform = poseTransform * rightrightTransform;

        tf::TransformBroadcaster broadcaster3;
        broadcaster1.sendTransform(
        tf::StampedTransform(final_RRtransform,ros::Time::now(),
        "map", "rightright_search"));



    cp.searchPoint[0].x = ndt_pose.front().position.x;
    cp.searchPoint[0].y = ndt_pose.front().position.y;
    cp.searchPoint[0].z = ndt_pose.front().position.z;
    cp.searchPoint[1].x = final_Ltransform.getOrigin().getX();
    cp.searchPoint[1].y = final_Ltransform.getOrigin().getY();
    cp.searchPoint[1].z = final_Ltransform.getOrigin().getZ();
    cp.searchPoint[2].x = final_Rtransform.getOrigin().getX();
    cp.searchPoint[2].y = final_Rtransform.getOrigin().getY();
    cp.searchPoint[2].z = final_Rtransform.getOrigin().getZ();
    cp.searchPoint[3].x = final_LLtransform.getOrigin().getX();
    cp.searchPoint[3].y = final_LLtransform.getOrigin().getY();
    cp.searchPoint[3].z = final_LLtransform.getOrigin().getZ();
    cp.searchPoint[4].x = final_RRtransform.getOrigin().getX();
    cp.searchPoint[4].y = final_RRtransform.getOrigin().getY();
    cp.searchPoint[4].z = final_RRtransform.getOrigin().getZ();
    ndt_pose.pop();

    std::cout << "search num -> "<< i << std::endl;
    std::cout << "center point -> "<< i << std::endl;
    std::cout << cp.searchPoint[0].x << "  "
              << cp.searchPoint[0].y << "  "
              << cp.searchPoint[0].z << " end \n";
    std::cout << "left point -> "<< i << std::endl;
    std::cout << cp.searchPoint[1].x << "  "
              << cp.searchPoint[1].y << "  "
              << cp.searchPoint[1].z << " end \n";
    std::cout << "right point -> "<< i << std::endl;
    std::cout << cp.searchPoint[2].x << "  "
              << cp.searchPoint[2].y << "  "
              << cp.searchPoint[2].z << " end \n";
    i++;
    }

    void LaneExtractor::tfTransform(const geometry_msgs::PoseStampedConstPtr& ptr)
    {
    geometry_msgs::PointStamped laser_point;
    laser_point.header.frame_id = "pose_link";
    //we'll just use the most recent transform available
    laser_point.header.stamp = ros::Time();
    laser_point.point.x = ptr->pose.position.x;
    laser_point.point.y = ptr->pose.position.y;
    laser_point.point.z = ptr->pose.position.z;

    tf::Vector3 poseT(laser_point.point.x, laser_point.point.y, laser_point.point.z);
    tf::Quaternion poseQ(ptr->pose.orientation.x, ptr->pose.orientation.y, ptr->pose.orientation.z, ptr->pose.orientation.w);
    tf::Transform poseTransform(poseQ,poseT);

    tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(
       tf::StampedTransform(poseTransform,ros::Time::now(),
       "map", "pose_link"));
    

    // LEFT LINK TRANSFORM
    tf::Vector3 leftT(0.0, -1.8, 0.0);
    tf::Transform leftTransform(tf::Quaternion(0.0, 0.0, 0.0) ,leftT);
    leftTransform = poseTransform * leftTransform;
    tf::TransformBroadcaster broadcaster1;
    broadcaster1.sendTransform(
       tf::StampedTransform(leftTransform,ros::Time::now(),
       "map", "left_link"));

    // RIGHT LINK TRANSFORM
    tf::Vector3 rightT(0.0, 1.8, 0.0);
    tf::Transform rightTransform(tf::Quaternion(0.0, 0.0, 0.0) ,rightT);
    rightTransform = poseTransform * rightTransform;
    tf::TransformBroadcaster broadcaster2;
    broadcaster2.sendTransform(
        tf::StampedTransform(rightTransform,ros::Time::now(),
        "map", "right_link"));

      ROS_INFO("tfTransform_success..");
    }


    

}