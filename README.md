zed_cpu_ros
===========
A simple zed camera driver which only use CPU and only publish left and right raw images and its camera info.

# Useage:
1. git the packge into your working space

    ```
    cd catkin_ws/src
    git clone https://github.com/transcendrobotics/zed_cpu_ros
    cd ..
    catkin_make
    ```
2. Get your calibration files:
    You can get your calibration files from zed or do a calibration your self by using ROS camera calibration package.
    
    (1). From zed:
    
    Find your zed calibration files in
    ```
    cd /usr/local/zed/settings
    ```
    or download from:
    http://calib.stereolabs.com/?SN=XXXX

    Note: XXXX is your last four digit S/N of your camera, make sure to change it!!

    put the .conf file into zed_cpu_ros/config folder

    update launch file configuration file name in zed_cpu_ros.launch into your SNXXXX.conf
    ```
    roscd zed_cpu_ros/launch
    gedit zed_cpu_ros.launch
    ```
    change XXXX into the .conf file you have, for example 1010
    ```
    <arg name="config_file_location" default="$(find zed_cpu_ros)/config/SN1010.conf"/>
    ```

    (2). Do a calibration yourself:
    
    This option is suggested. Reference: http://wiki.ros.org/camera_calibration
    ```
    roslaunch zed_cpu_ros camera_calibration.launch
    ```
    After calibration:
    Find the left.yaml and right.yaml in the tar file and put them into the zed_cpu_ros/config folder.
    The calibration file will be loaded if you turn <load_zed_config> off in the launch file.

3. launch the code
    ```
    roslaunch zed_cpu_ros zed_cpu_ros.launch
    ```
## Launch file parameters

 Parameter                    |           Description                                       |              Value          
------------------------------|-------------------------------------------------------------|-------------------------           
 resolution                   | ZED Camera resolution                                       | '0': HD2K                   
 _                            | _                                                           | '1': HD1080                 
 _                            | _                                                           | '2': HD720                  
 _                            | _                                                           | '3': VGA                                    
 frame_rate                   | Rate at which images are published                          | int                                             
 left_frame_id                | Left Frame ID                                               | string        
 right_frame_id               | Right Frame ID                                              | string        
 config_file_location         | The location of ZED calibration file                        | string        
 show_image                   | Whether to use opencv show image                            | bool        

# TODO:

1. add the launch file for stereo_proc.
2. add the nodelet functionality.

# Transcend Robotics:
Patented articulated traction control ARTI technology for stair climbing and obstacle traversal without complex software or controls
http://transcendrobotics.com/

# Authour:
Di Zeng 