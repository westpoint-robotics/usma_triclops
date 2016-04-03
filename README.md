# usma_triclops
This package uses the Point Grey BumbleBee camera to generate a pointcloud. It is capable of producing a pointcloud with pixels organized in 3d space and with their original color. It also can produce a filtered point cloud that produces only points allowed thru a filter, such as white line filter.

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

