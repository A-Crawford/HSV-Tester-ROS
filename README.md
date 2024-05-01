# ROS HSV Tester

Simple ROS node for displaying affect of HSV values on a static image pulled from a specified ROS image topic. Defaults to '/usb_cam/image_raw'.
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
roslaunch ros_hsv_tester ros_hsv_tester.launch <args>
```

#### Args
`camera_topic` used for specifying the topic for the node to subscribe to. Defaults to `/usb_cam/image_raw/`.

Example usuage:
```bash
roslaunch ros_hsv_tester ros_hsv_tester.launch camera_topic:="/camera/color/image_raw"
```