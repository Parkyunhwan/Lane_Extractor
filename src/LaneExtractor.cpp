#include <lane_extractor/LaneExtractor.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
// #define CENTER 0
// #define LEFT 1
// #define RIGHT 2
// #define MULTILEFT 3
// #define MULTIRIGHT 4
// #define LANE_BREAK_LIMIT 3


namespace lane_extractor
{
    LaneExtractor::LaneExtractor()
    {
       ros::NodeHandle nh;
       L_lane_break=LANE_BREAK_LIMIT;
       R_lane_break=LANE_BREAK_LIMIT;
       L_continuos_line=0;
       R_continuos_line=0;
    }
    LaneExtractor::~LaneExtractor()
    {}
    

    void LaneExtractor::mapCallback(const sensor_msgs::PointCloud2ConstPtr& pc)
    {
        cp.fromMsgToCloud(*pc);
        cp.map_publish();
    }

    void LaneExtractor::ndtPoseCallback(const geometry_msgs::PoseStampedConstPtr& ptr)
    {   
        std::cout << "ndt_pose subscribe.." << std::endl;
        static int i=0;
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;

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

        if(i%5==0) //Save once every five times --> you can change this parameter.
        {
        geometry_msgs::Pose current_pose = ptr->pose;
        //current_pose.position.z = 0.0;
        ndt_pose.push(current_pose);
        
        pcl::PointXYZI searchPoint;
        searchPoint.x = ndt_pose.back().position.x;
        searchPoint.y = ndt_pose.back().position.y;
        searchPoint.z = ndt_pose.back().position.z;

        std::cout << "[SEARCH Q]" << i << std::endl;
        }
        i++;
    }

    bool LaneExtractor::FindLaneService(lane_extractor::srvinfo::Request  &req,
                                        lane_extractor::srvinfo::Response &res)
    {
        
        cp.searchinfo.radius = req.rad;
        cp.searchinfo.min_intensity = req.minIntensity;
        cp.searchinfo.max_intensity = req.maxIntensity;
        tf::Transform poseTransform;
        /////////////////////////////////search
        while(!ndt_pose.empty()){
            poseTransform=setSearchEv(); // Env setting
            int left = RadiusSearch(LEFT, req.rad);
            int right= RadiusSearch(RIGHT, req.rad);
            CreateSideLane(poseTransform,left, right);
            RadiusSearch(MULTILEFT, req.rad);
            RadiusSearch(MULTIRIGHT, req.rad);
            cp.lane_publish();
            cp.cloud_filtered_publish();
        }
        std::cout << "Ridius Search Finish..<Put New Pose Data> "<< std::endl;
        return true;
    }

    bool LaneExtractor::SaveLaneService(lane_extractor::saveinfo::Request &req,lane_extractor::saveinfo::Response &res)
    {
        const char* name = req.data.c_str();
        if(cp.CloudSaver(name))
            std::cout << "Current Extract Lane saved!!" << std::endl;
        else
            std::cout << "Lane Saved failed.." << std::endl;
    }

    int LaneExtractor::CreateSideLane(tf::Transform &poseTransform,int left, int right)
    {
            pcl::PointXYZI lane_position;
            if(left && !right){
                tf::Transform final_Ltransform;
                tf::Transform leftTransform(tf::Quaternion(0.0, 0.0, 0.0) ,tf::Vector3(0.0, -1.7, 0.0));
                final_Ltransform = poseTransform * leftTransform;
                lane_position.x = final_Ltransform.getOrigin().getX();
                lane_position.y = final_Ltransform.getOrigin().getY();
                lane_position.z = final_Ltransform.getOrigin().getZ();
                cp.Intesity_Cloud->points.push_back(lane_position);
            }
            else if(right && !left){
                tf::Transform final_Rtransform;
                tf::Transform rightTransform(tf::Quaternion(0.0, 0.0, 0.0) ,tf::Vector3(0.0, 1.7, 0.0));
                final_Rtransform = poseTransform * rightTransform;
                lane_position.x = final_Rtransform.getOrigin().getX();
                lane_position.y = final_Rtransform.getOrigin().getY();
                lane_position.z = final_Rtransform.getOrigin().getZ();
                cp.Intesity_Cloud->points.push_back(lane_position);
            }
    }
    int LaneExtractor::RadiusSearch(int SearchLine,float rad)
    {     

            static std::vector<pcl::PointXYZI> Multi_left_point;   //Multi left  point candidate group
            static std::vector<pcl::PointXYZI> Multi_right_point;  //Multi right point candidate group
            std::vector<pcl::PointXYZI> s_point;
            pcl::PointXYZI cetroidpoint;
            int chance=5; //Number of opportunities to inspect for out of line range
            float z=0.3; //Using the height difference of points (not useful)
            cp.searchinfo.radius = rad;

            if(rotation_direction==LEFT||rotation_direction==RIGHT){
                L_lane_break=LANE_BREAK_LIMIT;
                R_lane_break=LANE_BREAK_LIMIT;
                cp.searchinfo.radius = cp.searchinfo.radius + 0.4f;
                // cp.searchinfo.min_intensity = cp.searchinfo.min_intensity+change; intensity change
            }

            int size = lineRadiusSearch(cetroidpoint,s_point,SearchLine);

            if( size > 0 )
            {
                //Apply to Multi-Rain Only
                if(SearchLine==MULTILEFT || SearchLine==MULTIRIGHT) 
                {
                    if(size <= 3){ s_point.clear() , chance = 0; }//Under 3 point remove;
                        while(!s_point.empty()){
                            Eigen::Vector2f poseP(cetroidpoint.x, cetroidpoint.y);
                            pcl::PointXYZI target_p = s_point.back();
                            s_point.pop_back();
                            Eigen::Vector2f targetP(target_p.x, target_p.y);
                            double distance = getPointToDistance(poseP, tan_yaw, targetP);
                            if(distance>0.30f) chance--;
                            if(chance==0) break;
                        }
                        s_point.clear();

                    if(chance != 0 && ( fabs(cetroidpoint.z-save_point.z) > z)){ // pass -> extract
                        if(SearchLine==MULTILEFT) {
                            Multi_left_point.push_back(cetroidpoint);        //just save Multi lnae
                            cp.cloud_filtered->points.push_back(cetroidpoint);
                            L_continuos_line = 1; // continuous multiline
                        }
                        else if(SearchLine==MULTIRIGHT) {
                            Multi_right_point.push_back(cetroidpoint); //just save Multi lnae
                            cp.cloud_filtered->points.push_back(cetroidpoint);
                            R_continuos_line = 1; // continuous multiline
                        }
                    }
                    else if(L_continuos_line == 1 || R_continuos_line==1){ // extract type change
                        if(SearchLine==MULTILEFT) {
                            L_lane_break--;
                            if( Multi_left_point.size()>=8){
                                while(!Multi_left_point.empty()){
                                    cp.Intesity_Cloud->points.push_back(Multi_left_point.back());
                                    Multi_left_point.pop_back();
                                }
                                L_lane_break=LANE_BREAK_LIMIT;
                            }
                            else if(L_lane_break==0){
                                Multi_left_point.clear();
                                L_lane_break=LANE_BREAK_LIMIT;
                            }
                            L_continuos_line = 0; // non-continuous multiline
                        }
                        else if(SearchLine==MULTIRIGHT) {
                            R_lane_break--;
                            if( Multi_right_point.size()>=8){
                                while(!Multi_right_point.empty()){
                                    cp.Intesity_Cloud->points.push_back(Multi_right_point.back());
                                    Multi_right_point.pop_back();
                                }
                                R_lane_break=3;
                            }
                            else if(R_lane_break==0){
                                Multi_right_point.clear();
                                R_lane_break=3;
                            }
                            R_continuos_line = 0; // non-continuous multiline
                        }
                        
                    }
                }//Apply to Multi-Rain Only

                else{ 
                    cp.Intesity_Cloud->points.push_back(cetroidpoint); //Not Multi-lane
                }

                return SearchLine; //Return the lane number if lane found
            }

            else return 0; //If you don't find the lane, return 0.
    }

    int LaneExtractor::lineRadiusSearch(pcl::PointXYZI &centerpoint,std::vector<pcl::PointXYZI> &s_point,int SearchLine){
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;//Set up your search tree
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        kdtree.setInputCloud(cp.cloud);
        pcl::CentroidPoint<pcl::PointXYZI> centroidpoint;
        //                            (   setting search_point  ), (  setting radius  ), 
        int size = kdtree.radiusSearch(cp.searchPoint[SearchLine], cp.searchinfo.radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);  
                if(size > 0)
                {
                       for(size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
                       {
                           if((cp.cloud->points[pointIdxRadiusSearch[i]].intensity > cp.searchinfo.min_intensity) )//&& (cp.cloud->points[pointIdxRadiusSearch[i]].intensity < cp.searchinfo.max_intensity)
                              {
                                 cp.cloud_filtered->points.push_back(cp.cloud->points[ pointIdxRadiusSearch[i] ]); //Insert all points found for debugging
                                 centroidpoint.add(cp.cloud->points[ pointIdxRadiusSearch[i]]); //Insert all points found to obtain the center of gravity
                                 s_point.push_back(cp.cloud->points[ pointIdxRadiusSearch[i]]); //Insert all points found for condition determination
                              }  
                       }
                       centroidpoint.get(centerpoint); //get center of gravity point;
                       return size;
                }
                else {
                    ROS_INFO("No point could be found..");
                    return 0;
                }
    }

    //Setting up your point search preferences
    tf::Transform LaneExtractor::setSearchEv() 
    {
        static int i = 0;
        static tf::Transform last_pose;

        //Pull out ndt_pose and apply poseTransform
        tf::Vector3 poseT( ndt_pose.front().position.x,  ndt_pose.front().position.y,  ndt_pose.front().position.z);
        tf::Quaternion poseQ( ndt_pose.front().orientation.x, ndt_pose.front().orientation.y, ndt_pose.front().orientation.z, ndt_pose.front().orientation.w);
        tf::Transform poseTransform(poseQ,poseT);
        tfBroadcaster(poseTransform,"map","center_search");

        tan_yaw = ToEulerAngles(poseQ); //Use quarterion information to obtain yaw value

        if(i>0) //Extract direction compared to previous pose
        {
            double yaw_diff=calDiffbtQuarternion(poseQ,last_pose.getRotation(),rotation_direction); //Find yaw difference
            if(yaw_diff > 2.5)  tfBroadcaster(poseTransform,"map","Angle"); //If the difference is above 2.5, acknowledge the direction.
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

        //Set the tf position that depends on the pose
        tf::Transform final_Ltransform, final_Rtransform, final_MLtransform, final_MRtransform;
        //left_search
        tf::Transform leftTransform(tf::Quaternion(0.0, 0.0, 0.0) ,tf::Vector3(0.0, 1.7, 0.0)); //-1.7M to the left of the pose
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
        
        //Set points in the set position
        setSearchPoint( poseTransform    ,CENTER );
        setSearchPoint( final_Ltransform ,LEFT );
        setSearchPoint( final_Rtransform ,RIGHT );
        setSearchPoint( final_MLtransform,MULTILEFT );
        setSearchPoint( final_MRtransform,MULTIRIGHT );
        
        ndt_pose.pop();//Delete used pose information
        i++;
        return poseTransform; //return poseTransform
    }

    //Compare the two quarternion values and return the yaw difference
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

    //Setting up search points for each line
    void LaneExtractor::setSearchPoint(const tf::Transform &tr,const int &line)
    {
        cp.searchPoint[line].x = tr.getOrigin().getX();
        cp.searchPoint[line].y = tr.getOrigin().getY();
        cp.searchPoint[line].z = tr.getOrigin().getZ();
    }

    //tf Broadcaster
    void LaneExtractor::tfBroadcaster(const tf::Transform &transform,const std::string &from_link,const std::string &to_link)
    {
            tf::TransformBroadcaster broadcaster;

            broadcaster.sendTransform(
            tf::StampedTransform(transform,ros::Time::now(),
            from_link, to_link));
    }
    
    //Convert from Quartinion to Euler Angle
    double LaneExtractor::ToEulerAngles(tf::Quaternion q)
    {
    //EulerAngles angles;
        double yaw;
        double siny_cosp = +2.0 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = +1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());  
        yaw = atan2(siny_cosp, cosy_cosp);

    return yaw;//angles;
    }

    //Calculate the distance from a given point to a point by drawing a line
    double LaneExtractor::getPointToDistance(const Eigen::Vector2f &poseP, const double &yaw, const Eigen::Vector2f &targetP )
    {
        Eigen::Vector3d ret_line;
        getStraightLineEquation2D(poseP,getSlope(yaw),ret_line);
        return getDeviation(poseP,targetP,ret_line); 
    }

    //
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