Lane_Extractor
======================

A Project to extract a lane from the pointcloud map and save it back as a pcd file.
-----------------------------------
Point Cloud -> extract -> Vector_lane

## Execution screen

![lane_extractor_1](https://user-images.githubusercontent.com/47476276/91465844-44e2b480-e8c9-11ea-9e56-9750e2de484e.gif)

👆 <0sec ~ 30sec>

![lane_extractor_2](https://user-images.githubusercontent.com/47476276/91465906-5b890b80-e8c9-11ea-83e9-97561afca8f4.gif)

👆 <30sec ~ 60sec>

## Preparations
Available in environments with [PCL & ROS kinetic] installed.

> <http://www.pointclouds.org/downloads/linux.html> [PCL download] \
> <http://wiki.ros.org/kinetic/Installation/Debian> [ROS kinetic download]

### Map processing


	$ rosrun lane_extractor intensity_cloud_saver [Original pcd file path] [Intensity Processing pcd file name] [min_intensity]  
You can set the desired intensity coefficients.  
If you put 450 in [min_intensity], create a pcd file, leaving only the point with an intensity value of 450 or more.  
Search speed is extremely fast when processing applying.  


### Visual debugging


Run rviz and open the conforming file stored in the rviz folder in the Lane_extractor package.




## Run
### 1. run lane_extractor_node

**There are two ways to run the Lane Extractor.**

*First,*

	$ rosrun lane_extractor lane_extractor_node  
  > Running without giving a argument behind it will be executed by loading the pcd file in the path set to Default in the code.
	
*Second,*

	$ rosrun lane_extractor lane_extractor_nod [pcd file path]  
  > When you pass the path of a pcd file to a factor, you use a pcd file that is passed to a factor.

### 2. run rosbag file
	$ rosbag play [ndt_pose path] or  
	$ rosbag play -r [speed rate]  [ndt_pose path] 

When you run the command, the line_extractor node receives the message ndt_pose and saves it to the queue.

### 3. run /find_lane rosservice to get lane point
*User can set a value.*

	$ rosservice call /find_lane "rad: [value] minIntensity: [value] maxIntensity: [value]" 
If you have run all of the robag files, use the ros service to have the lane_extractor node find lane.

### 4. run /save_lane rosservice to get pcd files

	$ rosservice call /save_lane "num : [int]" 실행
Save the lane points found so far to a file.
(File saved in the location of the lane_extractor node.)

## Contact
If you interested in my project.
Send me an email.

Thanks!
