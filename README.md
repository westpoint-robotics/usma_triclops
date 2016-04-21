# usma_triclops --TODO THIS NEEDS REWRITING TO REFLECT THE NEW CODE
This package uses the Point Grey BumbleBee camera to generate a pointcloud. It is capable of producing a pointcloud with pixels organized in 3d space and with their original color. It also can produce a filtered point cloud that produces only points allowed thru a filter, such as white line filter. The code is still in need of added functionality, documentation, and is still very much a work in progress.

This package creates three nodes:
- camera_system: This node acts as the device driver for a Point Grey Bumblebee2 camera. It automatically detects the camera and displays the camera information upon start up. This node processes the raw images from the camera and produces several images that are then published as ROS topics. The images produced include: left camera BGR, right camera BGR, rectified color BGR, and disparity image.

- line_filter: This node detects white lines. It subscribes to a BGR Image and publishes a Mono8 image that is all black except the the white pixels that belong to the white lines. It opens a OpenCv highGui interface that allows you to tune the variables that are used in the white line detector.

- vision3d: This node turns a disparity image into a 3d point cloud. If it is provided the rectified color image this node can produce a complete point cloud with each point arranged in 3d space as well as the color of the pixel ( this done by changing method calls and recompiling. The method that does this is called: doPointCloud). This node can also subscribe to a mono8 image and use it as a mask to display only certian pixels in the pointcloud, such as a white line ( this done by changing method calls and recompiling. The method that does this is called: maskToPointCloud).

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
    
   - `roslaunch usma_triclops triclops_camera2.launch` 

