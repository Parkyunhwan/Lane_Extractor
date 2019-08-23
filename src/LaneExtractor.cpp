#include <lane_extractor/LaneExtractor.h>
// #include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#define CENTER 0
#define LEFT 1
#define RIGHT 2
#define MULTILEFT 3
#define MULTIRIGHT 4



namespace lane_extractor
{
    LaneExtractor::LaneExtractor()
    {
       ros::NodeHandle nh;
       L_lane_break=3;
       R_lane_break=3;
       continuos_line=0;
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

        if(i%5==0)
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
        
        cp.searchinfo.radius = req.rad;
        cp.searchinfo.min_intensity = req.minIntensity;
        cp.searchinfo.max_intensity = req.maxIntensity;
        /////////////////////////////////search
        while(!ndt_pose.empty()){
            setSearchEv(); // search point
            RadiusSearch(LEFT);
            RadiusSearch(RIGHT);
            RadiusSearch(MULTILEFT);
            RadiusSearch(MULTIRIGHT);
            cp.lane_publish();
            cp.cloud_filtered_publish();
        }
            //     static std::vector<pcl::PointXYZI> Multi_left_point; empty rjatkaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
            // static std::vector<pcl::PointXYZI> Multi_right_point; 
        std::cout << "Ridius Search Finish..<Put New Pose Data> "<< std::endl;
        return true;
    }

    void LaneExtractor::RadiusSearch(int SearchLine)
    {     
            static std::vector<pcl::PointXYZI> Multi_left_point;
            static std::vector<pcl::PointXYZI> Multi_right_point; 
            std::vector<pcl::PointXYZI> s_point;
            int chance=4;
            float z=0.3;
            pcl::PointXYZI cetroidpoint;
            if(rotation_direction==LEFT||rotation_direction==RIGHT){
                chance = 6;
                z=0.3;
                cp.searchinfo.radius = cp.searchinfo.radius + 0.4f;
                cp.searchinfo.min_intensity = cp.searchinfo.min_intensity+150;
            }
            int size = lineRadiusSearch(cetroidpoint,s_point,SearchLine);
            if( size > 0 )
            {
                if(SearchLine==MULTILEFT || SearchLine==MULTIRIGHT)
                {
                    if(size <= 3){ s_point.clear() , chance = 0; }                             //Under 3 point remove;
                        while(!s_point.empty()){
                            Eigen::Vector2f poseP(cetroidpoint.x, cetroidpoint.y);
                            pcl::PointXYZI target_p = s_point.back();
                            s_point.pop_back();
                            Eigen::Vector2f targetP(target_p.x, target_p.y);
                            double distance = getPointToDistance(poseP, tan_yaw, targetP);
                            if(distance>0.25f) chance--;
                            if(chance==0) break;
                        }
                        s_point.clear();

                    if(chance != 0 && ( fabs(cetroidpoint.z-save_point.z) > z)){ // pass -> extract
                        if(SearchLine==MULTILEFT) {
                            Multi_left_point.push_back(cetroidpoint);        //just save Multi lnae
                            if( Multi_left_point.size()==15){
                                while(!Multi_left_point.empty()){
                                    cp.Intesity_Cloud->points.push_back(Multi_left_point.back());
                                    Multi_left_point.pop_back();
                                }
                            }
                        }
                        else if(SearchLine==MULTIRIGHT) {
                            Multi_right_point.push_back(cetroidpoint); //just save Multi lnae
                            if( Multi_right_point.size()==10){
                                while(!Multi_right_point.empty()){
                                    cp.Intesity_Cloud->points.push_back(Multi_right_point.back());
                                    Multi_right_point.pop_back();
                                }
                            }
                        }
                        continuos_line = 1; // continuous multiline
                    }
                    else if(continuos_line == 1){ // extract type change
                        if(SearchLine==MULTILEFT) {
                            L_lane_break--;
                            if(L_lane_break==0){
                                Multi_left_point.clear();
                                L_lane_break=3;
                            }
                        }
                        else if(SearchLine==MULTIRIGHT) {
                            R_lane_break--;
                            if(L_lane_break==0){
                                Multi_right_point.clear();
                                R_lane_break=3;
                            }
                        }
                        continuos_line = 0; // non-continuous multiline
                    }
                }
                else{ 
                    cp.Intesity_Cloud->points.push_back(cetroidpoint);
                }
            }
            // if(chance!=0)
            //     cp.Intesity_Cloud->points.push_back(cetroidpoint);
            //     if( (SearchLine==MULTILEFT || SearchLine==MULTIRIGHT) && (chance!=0) && ( fabs(cetroidpoint.z-save_point.z) > z ) )// || fabs(point.z-save_point.z) > 0.1
            //         cp.Intesity_Cloud->points.pop_back();
 
            //         save_point = cetroidpoint;
            // }
                cp.searchinfo.radius = 0.6;
                cp.searchinfo.min_intensity = 430;
                cp.searchinfo.max_intensity = 1700;
    }
                       //std::cout << "---centroid point---   "<< point << std::endl;
                    //std::cout << "---search size---   "<< pointIdxRadiusSearch.size() << std::endl << std::endl;
    int LaneExtractor::lineRadiusSearch(pcl::PointXYZI &centerpoint,std::vector<pcl::PointXYZI> &s_point,int SearchLine){
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        kdtree.setInputCloud(cp.cloud);
        pcl::CentroidPoint<pcl::PointXYZI> centroidpoint;
        int size = kdtree.radiusSearch(cp.searchPoint[SearchLine], cp.searchinfo.radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);  
                if(size > 0)
                {
                       for(size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
                       {
                           if((cp.cloud->points[pointIdxRadiusSearch[i]].intensity > cp.searchinfo.min_intensity) )//&& (cp.cloud->points[pointIdxRadiusSearch[i]].intensity < cp.searchinfo.max_intensity)
                              {
                                 cp.cloud_filtered->points.push_back(cp.cloud->points[ pointIdxRadiusSearch[i] ]);
                                 centroidpoint.add(cp.cloud->points[ pointIdxRadiusSearch[i]]);
                                 s_point.push_back(cp.cloud->points[ pointIdxRadiusSearch[i]]);
                              }  
                       }
                       centroidpoint.get(centerpoint);
                       return size;
                }
                else {
                    ROS_INFO("No point could be found..");
                    return 0;
                }
    }


    void LaneExtractor::setSearchEv()
    {
        static int i = 0;
        static tf::Transform last_pose;
        tf::Vector3 poseT( ndt_pose.front().position.x,  ndt_pose.front().position.y,  ndt_pose.front().position.z);
        tf::Quaternion poseQ( ndt_pose.front().orientation.x, ndt_pose.front().orientation.y, ndt_pose.front().orientation.z, ndt_pose.front().orientation.w);
        tf::Transform poseTransform(poseQ,poseT);
        tfBroadcaster(poseTransform,"map","center_search");
        
        tan_yaw = ToEulerAngles(poseQ);
        if(i>0)
        {
            double yaw_diff=calDiffbtQuarternion(poseQ,last_pose.getRotation(),rotation_direction);
            if(yaw_diff > 3.0)  tfBroadcaster(poseTransform,"map","Angle");
            else rotation_direction=CENTER;

            std::cout << "yaw_diff : " << yaw_diff << std::endl;
            if(rotation_direction==CENTER)
            std::cout << "rotation_direction : CENTER" << std::endl;
            if(rotation_direction==LEFT)
            std::cout << "rotation_direction : LEFT" << std::endl;
            else if(rotation_direction==RIGHT)
            std::cout << "rotation_direction : RIGHT" << std::endl;
        }
        last_pose = poseTransform;

        tf::Transform final_Ltransform, final_Rtransform, final_MLtransform, final_MRtransform;
        //left_search
        tf::Transform leftTransform(tf::Quaternion(0.0, 0.0, 0.0) ,tf::Vector3(0.0, 1.7, 0.0));
        final_Ltransform = poseTransform * leftTransform;
        tfBroadcaster(final_Ltransform,"map","left_search");
        //right_search
        tf::Transform rightTransform(tf::Quaternion(0.0, 0.0, 0.0) ,tf::Vector3(0.0, -1.7, 0.0));
        final_Rtransform = poseTransform * rightTransform;
        tfBroadcaster(final_Rtransform,"map","right_search");
        //Multileft_search     
        tf::Transform MultileftTransform(tf::Quaternion(0.0, 0.0, 0.0) ,tf::Vector3(0.0, (1.7+3.4), 0.0));
        final_MLtransform = poseTransform * MultileftTransform;
        tfBroadcaster(final_MLtransform,"map","Multileft_search");
        //Multiright_search
        tf::Transform MultirightTransform(tf::Quaternion(0.0, 0.0, 0.0) ,tf::Vector3(0.0, (-1.7-3.4), 0.0));
        final_MRtransform = poseTransform * MultirightTransform;
        tfBroadcaster(final_MRtransform,"map","Multiright_search");

        setSearchPoint( poseTransform    ,CENTER );
        setSearchPoint( final_Ltransform ,LEFT );
        setSearchPoint( final_Rtransform ,RIGHT );
        setSearchPoint( final_MLtransform,MULTILEFT );
        setSearchPoint( final_MRtransform,MULTIRIGHT );
        
        ndt_pose.pop();
        i++;
    }

    double LaneExtractor::calDiffbtQuarternion(const tf::Quaternion &q1,const tf::Quaternion &q2,int &rot_direction)
    {
            double yaw1 = ToEulerAngles(q1);
            double yaw_degrees1 = yaw1 * 180.0 / M_PI; // conversion to degrees
            if( yaw_degrees1 < 0 ) yaw_degrees1 += 360.0; // convert negative to positive angles
            double yaw2 = ToEulerAngles(q2);
            double yaw_degrees2 = yaw2 * 180.0 / M_PI; // conversion to degrees
            if( yaw_degrees2 < 0 ) yaw_degrees2 += 360.0; // convert negative to positive angles
 
            double yaw_diff = fabs(yaw_degrees1 - yaw_degrees2);
            if(yaw_diff > 180) yaw_diff = 360 - yaw_diff;

            if(yaw_degrees2 > yaw_degrees1 ) rot_direction = RIGHT; //right
            else if (yaw_degrees2 < yaw_degrees1) rot_direction = LEFT; //left

            return yaw_diff;
    }
    void LaneExtractor::setSearchPoint(const tf::Transform &tr,const int &line)
    {
        cp.searchPoint[line].x = tr.getOrigin().getX();
        cp.searchPoint[line].y = tr.getOrigin().getY();
        cp.searchPoint[line].z = tr.getOrigin().getZ();
    }

    void LaneExtractor::tfBroadcaster(const tf::Transform &transform,const std::string &from_link,const std::string &to_link)
    {
            tf::TransformBroadcaster broadcaster;

            broadcaster.sendTransform(
            tf::StampedTransform(transform,ros::Time::now(),
            from_link, to_link));
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