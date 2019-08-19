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
            set_searchPoint(); // search point ?Ñ§?†ï
            RadiusSearch(1);
            RadiusSearch(2);
            RadiusSearch(3);
            RadiusSearch(4);
            cp.lane_publish();
            cp.cloud_filtered_publish();
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
            // if(rot==1)
            //     cp.searchinfo.radius = cp.searchinfo.radius*2;
            
            std::cout << "left_lane search.." <<std::endl;
        }
        else if(SearchNum==2){
            // if(rot==2) cp.searchinfo.radius = cp.searchinfo.radius*2; 
            std::cout << "right_lane search.." <<std::endl;
        }


        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        kdtree.setInputCloud(cp.cloud);

        // pcl::ExtractIndices<pcl::PointXYZI> extract;
        // extract.setInputCloud(cp.cloud);
         //kdtree?óê cloud ?Ñ§?†ï
            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            pcl::CentroidPoint<pcl::PointXYZI> centroidpoint;
            int chance=5;
                if( kdtree.radiusSearch(cp.searchPoint[SearchNum], cp.searchinfo.radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) //?ïò?Çò?ùº?èÑ search?êúÍ≤ΩÏö∞
                {
                    //  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
                        int k=0;
                        
                       for(size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
                       {
                        //    std::cout << "intensity : "<< cp.cloud->points[ pointIdxRadiusSearch[i] ].intensity << std::endl;
                        //    std::cout << "min_intensity : "<< cp.searchinfo.min_intensity << std::endl;
                           if((cp.cloud->points[pointIdxRadiusSearch[i]].intensity > cp.searchinfo.min_intensity) )//&& (cp.cloud->points[pointIdxRadiusSearch[i]].intensity < cp.searchinfo.max_intensity)
                              {
                                 cp.cloud_filtered->points.push_back(cp.cloud->points[ pointIdxRadiusSearch[i] ]);
                                 centroidpoint.add(cp.cloud->points[ pointIdxRadiusSearch[i]]);
                                 Eigen::Vector2f poseP(cp.searchPoint[0].x,cp.searchPoint[0].y);
                                 Eigen::Vector2f targetP(cp.cloud->points[ pointIdxRadiusSearch[i]].x, cp.cloud->points[ pointIdxRadiusSearch[i]].y);
                                 double distance = getPointToDistance(poseP, tan_yaw, targetP);

                                if(SearchNum==3 || SearchNum==4){
                                 if(distance<4.8f || distance>5.4f) chance--;
                                 if(chance==0) break;
                                }
                                 k++;//  inliers->indices.push_back(pointIdxRadiusSearch[i]);
                              }
                              
                       }
                       pcl::PointXYZI point;
                       centroidpoint.get(point);
                        if(chance!=0)
                            cp.Intesity_Cloud->points.push_back(point);
                        //std::cout << "---centroid point---   "<< point << std::endl;
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

        
        tan_yaw = ToEulerAngles(poseQ);
        ////
        // if(i==0)last_pose = poseTransform;
        rot=0;
        if(i>0){
            
            double yaw1 = ToEulerAngles(poseQ);
            double yaw_degrees1 = yaw1 * 180.0 / M_PI; // conversion to degrees
            if( yaw_degrees1 < 0 ) yaw_degrees1 += 360.0; // convert negative to positive angles
            double yaw2 = ToEulerAngles(last_pose.getRotation());
            double yaw_degrees2 = yaw2 * 180.0 / M_PI; // conversion to degrees
            if( yaw_degrees2 < 0 ) yaw_degrees2 += 360.0; // convert negative to positive angles
 
            double yaw_diff = fabs(yaw_degrees1 - yaw_degrees2);
            if(yaw_diff > 180) yaw_diff = 360 - yaw_diff;
            
            tf::TransformBroadcaster broadcaster;
            if(yaw_diff > 10)  
            {
                broadcaster.sendTransform(tf::StampedTransform(poseTransform,ros::Time::now(),
                    "map", "Angles"));
            if(yaw_degrees2 > yaw_degrees1 ) rot=1; //right
            else if (yaw_degrees2 < yaw_degrees1) rot=2; //left

            }

            std::cout << "yaw degree1 : " << yaw_degrees1 << std::endl;
            std::cout << "yaw degree2 : " << yaw_degrees2 << std::endl;
            std::cout << "yaw_diff : " << yaw_diff << std::endl;
            
        }
        last_pose = poseTransform;

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

    double LaneExtractor::ToEulerAngles(tf::Quaternion q)
    {
    //EulerAngles angles;
    double yaw;

        // roll (x-axis rotation)
        // double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
        // double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
        // angles.roll = atan2(sinr_cosp, cosr_cosp);

        // // pitch (y-axis rotation)
        // double sinp = +2.0 * (q.w * q.y - q.z * q.x);
        // if (fabs(sinp) >= 1)
        //     angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        // else
        //     angles.pitch = asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
        yaw = atan2(siny_cosp, cosy_cosp);

    return yaw;//angles;
    }


    double LaneExtractor::getPointToDistance(const Eigen::Vector2f &poseP, const double &yaw, const Eigen::Vector2f &targetP )
    {
        Eigen::Vector3d ret_line;
        getStraightLineEquation2D(poseP,getSlope(yaw),ret_line);
        return getDeviation(poseP,targetP,ret_line); 
    }


    double LaneExtractor::getSlope (const double &dir_arg)
    {
    double ret_slope;

    const float min_value = std::numeric_limits<double>::epsilon ();
    double theta = dir_arg;//getApproxTheta (dir_arg);


    if ( std::abs (theta - M_PI/2) < min_value || 
        std::abs (theta - 3*M_PI/2) < min_value )
    {
        ret_slope = std::numeric_limits<double>::max();
    }
    else
        ret_slope = tan (theta);

    return ret_slope;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool LaneExtractor::getStraightLineEquation2D (const Eigen::Vector2f &point_arg,
                                                            const double &slope_arg,
                                                            Eigen::Vector3d &ret_line_arg)
    {
    
    double min_value = std::numeric_limits<double>::epsilon();
    
    if (std::abs (slope_arg - std::numeric_limits<double>::max()) > min_value)
    {
        ret_line_arg = Eigen::Vector3d (slope_arg, -1, -slope_arg * point_arg.x() + point_arg.y());
        return true;
    }
    else 
    { 
        std::cout << "NO LINE" << std::endl;
        ret_line_arg = Eigen::Vector3d (std::numeric_limits< double >::max(), 
                                        std::numeric_limits< double >::max(), 
                                        std::numeric_limits< double >::max());
        return false;
    }
    }

    double LaneExtractor::pointAndLineDistance2D (const Eigen::Vector2f &point_arg, const Eigen::Vector3d &line_arg)
    {
    /*
    if (std::abs (point_arg(0) - std::numeric_limits<double>::max()) < std::numeric_limits<double>::epsilon())
    return std::numeric_limits<double>::max();
    else  
    */
        return (std::abs (line_arg(0) * point_arg.x() + line_arg(1) * point_arg.y() + line_arg(2))) / 
                sqrt (pow (line_arg(0), 2) + pow (line_arg(1), 2));
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double LaneExtractor::getDeviation (const Eigen::Vector2f &start_pt, const Eigen::Vector2f &candidate_pt, const Eigen::Vector3d &line_coef)
    {
    double candidate_deviation;

    if (std::abs (line_coef (0) - std::numeric_limits<double>::max ()) < std::numeric_limits<double>::epsilon ())
        candidate_deviation = std::abs (start_pt.x() - candidate_pt.x());
    else
        candidate_deviation = pointAndLineDistance2D (Eigen::Vector2f (candidate_pt.x(), candidate_pt.y()), line_coef);  

    return candidate_deviation;
    }


    

}