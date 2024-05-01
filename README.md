# ROS HSV Tester

Simple ROS node for displaying affect of HSV values on a static image pulled from the 'usb_cam/image_raw' topic.
</br>

## Install
```bash
$ ~/catkin_ws/src
$ git clone https://github.com/A-Crawford/ros_hsv_tester.git
$ cd ~/catkin_ws/ && catkin_make
```

Source the workspace after install
```bash
$ cd /catkin_ws/
$ source devel/setup.bash
```

</br>

## Starting the node

```bash
roslaunch ros_hsv_tester ros_hsv_tester.launch
```

