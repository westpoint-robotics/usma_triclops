# usma_triclops --Single Node version
This package uses the Point Grey BumbleBee camera to generate pointclouds of red objects, blue objects, and white lines.

This version of the code uses a single ROS node to publish the point clouds for the red flags, blue falgs, and white lines. It also publish the color rectified image and the disparity image. Under the branch called threeNodeVersion is a version that uses a seperate nodes for the cmarea driver, filters, and point clouds. 
 

This node:
1. Is the device driver for a Point Grey Bumblebee2 camera. It detects the camera and displays the camera information upon start up. This node processes the raw images from the camera using the PGR Flycapture API. It then produces these images into a rectified color image and a disparity image using the PGR Triclops API. These are then published as ROS topics. 

2. Conducts line_filtering: This node detects white lines and prodcues a white line mask (mono8) for consumption by the pointcloud producing code. 

3. Conducts red and blue pixel filtering: The node detects red and blue pixels using HSV and produces a mask (mono8) for consumption by the pointcloud producing code. TODO: Integrate blog detection to filter out noise, currently it does not appear needed, but would improve accuracy in noisy environments.

4. Vision3d produces point clouds based on image masks provided: This node turns a disparity image into a 3d point cloud. Provided a mask, the disparity image and triclops context it produces a point cloud with each point arranged in 3d space.

## To Install
1. Download the latest libraries from Point Grey Website. Currently (March2016) they are called:
    - flycapture2-2.9.3.13-amd64-pkg.tgz
    - triclops-3.4.0.8-amd64-pkg.tgz
    
2. Extract Flycapture files and install according to the README that can be found
once you have extracted the files.

3. Extract Triclops files and install according to the README that can be found
once you have extracted the files. 

4. Clone this package into the catkin_ws.

    - `cd ~/catkin_ws/src`
    - `git clone https://github.com/westpoint-robotics/usma_triclops.git`
    - `cd ..`
    - `catkin_make`

5. Test the code.
    
   - `roslaunch usma_triclops bumblebee.launch` 

