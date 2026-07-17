# ros2_astra_camera

A ROS2 driver for Orbbec 3D cameras

## Install dependencies

### ROS2

● Please refer to the
official [ROS 2 installation guide](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)

### other dependencies

● Install dependencies (be careful with your ROS distribution)

```bash
 Assuming you have sourced the ros environment, same below
sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager\
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev

```

● Install libuvc

```bash
git clone https://github.com/libuvc/libuvc.git
cd libuvc
mkdir build && cd build
cmake .. && make -j4
sudo make install
sudo ldconfig # Refreshing the link library
```

### Getting Started

#### Use cases

● Create a new ros2 workspace

```bash
mkdir -p ~/ros2_ws/src
```

● Extract and copy openNISDk_ROS2_xxx.tar.gz to ~/ros2_ws/src/
● Install libusb rules

```bash
cd ~/ros2_ws/src/ros2_astra_camera/astra_camera/scripts
sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

● Compiling

```bash
 cd ~/ros2_ws
source /opt/ros/galactic/setup.bash 
colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
```

Starting the camera

● In terminal 1

```bash
source /opt/ros/galactic/setup.bash 
source ./install/setup.bash 
ros2 launch astra_camera astra.launch.xml
```

● In terminal 2

```bash
source /opt/ros/galactic/setup.bash 
source ./install/setup.bash 
rviz2
```

● List topics / services/ parameters (open a new terminal)

```bash
source /opt/ros/galactic/setup.bash 
ros2 topic list
ros2 service list 
ros2 param list
```

Select the topic you want to display

● Get camera parameters

```bash
ros2 service call /camera/get_camera_info astra_camera_msgs/srv/GetCameraInfo '{}'
```

● Check camera parameters, please refer to the ROS documentation for the meaning of the specific fields of the camera
parameters  [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)

```bash
ros2 topic echo /camera/depth/camera_info
ros2 topic echo /camera/color/camera_info
```

● Check device information

```bash
ros2 service call /camera/get_device_info astra_camera_msgs/srv/GetDeviceInfo '{}'
```

● Get the SDK version

```bash
ros2 service call /camera/get_sdk_version  astra_camera_msgs/srv/GetString "{}"
```

● Set/get (auto) exposure

```bash
 # Auto exposure switch. For setting the exposure manually, please turn off the auto exposure first
 ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool '{data: false}' 
 ros2 service call /camera/set_uvc_auto_exposure std_srvs/srv/SetBool '{data: false}'
 ros2 service call /camera/set_ir_auto_exposure std_srvs/srv/SetBool "{data: false}"
    
# Setting exposure values
ros2 service call /camera/set_ir_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
ros2 service call /camera/set_color_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
ros2 service call /camera/set_uvc_exposure astra_camera_msgs/srv/SetInt32 "{data: 2000}"
 # Get exposure
 ros2 service call /camera/get_ir_exposure astra_camera_msgs/srv/GetInt32 "{}"
 ros2 service call /camera/get_color_exposure astra_camera_msgs/srv/GetInt32 "{}"
 ros2 service call /camera/get_uvc_exposure astra_camera_msgs/srv/GetInt32 "{}"
```

● Set/get gain

```bash
# Get Gain
ros2 service call /camera/get_color_gain astra_camera_msgs/srv/GetInt32 '{}' # OpenNI camera
ros2 service call /camera/get_ir_gain astra_camera_msgs/srv/GetInt32 '{}' # OpenNI camera
ros2 service call /camera/get_uvc_gain astra_camera_msgs/srv/GetInt32 "{}" # UVC camera
 # Setting the gain
ros2 service call /camera/set_color_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
ros2 service call /camera/set_ir_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
ros2 service call /camera/set_uvc_gain astra_camera_msgs/srv/SetInt32 "{data: 200}"
```

● Set mirror mode

```bash
ros2 service call /camera/set_color_mirror std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_ir_mirror std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_depth_mirror std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_uvc_color_mirror std_srvs/srv/SetBool '{data: true}'
```

● Set/get (auto) white balance

```bash
ros2 service call /camera/get_color_auto_white_balance astra_camera_msgs/srv/GetInt32 '{}' # Return 0 or 1
ros2 service call /camera/set_color_auto_white_balance std_srvs/srv/SetBool '{data: false}'
```

● Turn on/off laser

```bash
ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: true}' # Turn on
ros2 service call /camera/set_laser_enable std_srvs/srv/SetBool '{data: false}' #Turn off
```

● Turn on/off fan

```bash
 ros2 service call /camera/set_fan_mode std_srvs/srv/SetBool '{data: true}' #Turn on
 ros2 service call /camera/set_fan_mode std_srvs/srv/SetBool '{data: false}' # Turn off
```

● Turn LDP on/off

```bash
ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /camera/set_ldp_enable std_srvs/srv/SetBool '{data: false}'
```

● Turn sensors on/off

```bash
ros2 service call  /camera/toggle_ir std_srvs/srv/SetBool "{data: true}"
ros2 service call  /camera/toggle_color std_srvs/srv/SetBool "{data: true}"
ros2 service call  /camera/toggle_depth std_srvs/srv/SetBool "{data: true}"
ros2 service call /camera/toggle_uvc_camera std_srvs/srv/SetBool "{data : true}"
```

### Multi-Camera

● First, get the serial number of the camera, plug in the camera and run

```bash
ros2 run astra_camera list_devices_node
```

● Set the parameter device_num to the number of cameras

● Go to the `ros2_astra_camera/launch/multi_xxx.launch`   and change the serial number. Currently, different cameras can
only be distinguished by the serial number,

```xml
<launch>
    <!-- unique camera name-->
    <arg name="camera_name" default="camera"/>
    <!-- Hardware depth registration -->
    <arg name="3d_sensor" default="astra"/>
    <!-- stereo_s_u3, astrapro, astra -->
    <arg name="camera1_prefix" default="01"/>
    <arg name="camera2_prefix" default="02"/>
    <arg name="camera1_serila_number" default="ADA611300CE"/>
    <arg name="camera2_serila_number" default="sn123456789"/>
    <arg name="device_num" default="2"/>
    <node name="camera" pkg="astra_camera" type="cleanup_shm_node" output="screen"/>
    <include file="$(find-pkg-share astra_camera)/launch/$(arg 3d_sensor).launch.xml">
        <arg name="camera_name" value="$(var camera_name)_$(var camera1_prefix)"/>
        <arg name="serial_number" value="$(var camera1_serila_number)"/>
        <arg name="device_num" value="$(var device_num)"/>
    </include>

    <include file="$(ind-pkg-share astra_camera)/launch/$(var 3d_sensor).launch.xml">
        <arg name="camera_name" value="$(var camera_name)_$(var camera2_prefix)"/>
        <arg name="serial_number" value="$(var camera2_serila_number)"/>
        <arg name="device_num" value="$(var device_num)"/>
    </include>
    <node pkg="tf2_ros" exec="static_transform_publisher" name="camera_tf"
          args="0 0 0 0 0 0 camera01_link camera02_link"/>
</launch>
```

* astra camera will use semaphore to do process synchronization,
if the camera start fails, the semaphore file may be left in the `/dev/shm`,
causing the next start to be stuck.
Before launch, please run

```bash
ros2 run astra_camera cleanup_shm_node 
```

to clean up `/dev/shm/`.

* Launch

```bash
ros2 launch astra_camera multi_astra.launch.xml
```

## Use calibration camera parameter

* Set camera info url, Go to `xxx.launch`

```xml
<launch>
    <!--...-->
    <arg name="ir_info_url" default="file:///you_ir_camera_calib_path/depth_camera.yaml"/>
    <arg name="color_info_url" default="file:///you_depth_camera_calib_path/rgb_camera.yaml"/>
    <!--...-->
</launch>
```

* Calibration file should be like

> Be careful the camera name The color camera is rgb_camera, the depth/IR camera name is ir_camera

```yaml
image_width: 640
image_height: 480
# The camera name is fixed. The color camera is rgb_camera, the depth/IR camera name is ir_camera
camera_name: rgb_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [517.301, 0, 326.785, 0, 519.291, 244.563, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.41527, 0.31874, -0.00197, 0.00071, 0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [0.999973, 0.00612598, -0.00406652, -0.00610201, 0.999964, 0.00588094, 0.0041024, -0.00585596, 0.999974 ]
projection_matrix:
  rows: 3
  cols: 4
  data: [517.301, 0, 326.785, -25.3167, 0, 519.291, 244.563, 0.282065, 0, 0, 1, 0.0777703]

```

## Launch parameters

* `connection_delay`, The delay time for reopening the device in seconds. Some devices would take longer time to
  initialize, such as Astra mini, so reopening the device immediately would causes firmware crashes when hot plug.

* `enable_point_cloud`, Whether to enable point cloud.
* `enable_colored_point_cloud`, Whether to enable RGB point cloud.
* `point_cloud_qos`, `[color|depth|ir]_qos`,`[color|depth|ir]_camera_info_qos`, ROS2 Message QoS,
  can be set to `SYSTEM_DEFAULT`, `DEFAULT`,`PARAMETER_EVENTS` ,`SERVICES_DEFAULT`, `PARAMETERS`, `SENSOR_DATA`,
  corresponds to `rmw_qos_profile_system_default`, `rmw_qos_profile_default`, `rmw_qos_profile_parameter_events`
 `rmw_qos_profile_services_default`,`rmw_qos_profile_parameters`, `SENSOR_DATA`,Case-insensitive.
* `enable_d2c_viewer`, Publish D2C overlay image (For Test only).
* `device_num`,The number of devices, You need to fill in the number of devices when you need multiple cameras.
* `color_width`， `color_height`， `color_fps`， color stream resolution and frame rate.
* `ir_width`， `ir_height`， `ir_fps`，IR stream resolution and frame rate
* `depth_width`， `depth_height`， `depth_fps` depth stream resolution and frame rate
* `enable_color`， Whether to enable RGB camera, this parameter has no effect when the RGB camera is UVC protocol
* `enable_depth` , Whether to enable depth camera
* `enable_ir`, Whether to enable IR camera
* `depth_registration`, Enables hardware depth to color alignment, requires RGB point cloud to open
* `depth_scale`, Depth image zoom scale, e.g. set to 2 means aligning depth 320x240 to RGB 640x480
* `color_roi_x`， `color_roi_y`， `color_roi_width`， `color_roi_height`, Whether to crop RGB images, the default is -1,
  which is only used when the RGB resolution is greater than the depth resolution and needs to be aligned. For example,
  if you need to align the depth 640x400 to RGB 640x480, you need to set color_roi_x: 0, color_roi_y: 0,
  color_roi_width: 640, color_roi_height: 400. roi_height: 400, which will crop the top 400 pixels of the RGB with a
  corresponding depth ROI.
* `color_depth_synchronization`，Enable synchronization of RGB with depth
* `use_uvc_camera`，if the RGB camera is UVC protocol, setting as true, UVC is the protocol that currently includes
  dabai, dabai_dcw etc.
* `uvc_product_id`，pid of UVC camera
* `uvc_camera_format`，Image format for uvc camera
* `uvc_retry_count` sometimes the UVC protocol camera does not reconnect successfully when hot-plug, requiring many
  times to retry.
* `enable_publish_extrinsic` Enable publish camera extrinsic.
* `oni_log_level`, Log levels for OpenNI verbose/ info /warning/ error /none
* `oni_log_to_console`, Whether to output OpenNI logs to the console
* `oni_log_to_file`, Whether to output OpenNI logs to a file, by default it will save in Log folder under the path of
  the currently running program

## DDS Tuning

The default DDS settings (Galactic) may not be efficient for data transmission. Different DDS settings will have
different performance. Here we take CycloneDDS as an example. For detailed information, please refer
to[ROS DDS Tuning](https://docs.ros.org/en/humble/How-To-Guides/DDS-tuning.html)。

● Edit cyclonedds configuration file

```bash
sudo gedit /etc/cyclonedds/config.xml
```

Add

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/confighttps://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain id="any">
        <General>
            <NetworkInterfaceAddress>lo</NetworkInterfaceAddress>
            <AllowMulticast>false</AllowMulticast>
        </General>
        <Internal>
            <MinimumSocketReceiveBufferSize>16MB</MinimumSocketReceiveBufferSize>
        </Internal>
        <Discovery>
            <ParticipantIndex>auto</ParticipantIndex>
            <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
            <Peers>
                <Peer address="localhost"/>
            </Peers>
        </Discovery>
    </Domain>
</CycloneDDS>
```

● Set the environment variables, add to .zshrc or .bashrc

```bash
export ROS_DOMAIN_ID=42 # Numbers from 0 to 232
export ROS_LOCALHOST_ONLY=1
export CYCLONEDDS_URI=file:///etc/cyclonedds/config.xml
```

Tip：to understand why the maximum ROS_DOMAIN_ID is 232, please
visit [The ROS DOMAIN ID](https://docs.ros.org/en/humble/Concepts/About-Domain-ID.html)
● Increase UDP receive buffer size
Edit

```bash
/etc/sysctl.d/10-cyclone-max.conf
```

Add

```bash
net.core.rmem_max=2147483647
net.core.rmem_default=2147483647
```

## Frequently Asked Questions

● No point cloud or image displayed on Rviz2

* The default QoS for outgoing messages is sense_data, i.e. Relaiblity Policy = Best Effort, please check if the
  Relaiblity Policy is set to Best Effort on Rviz2, and also check if there is any problem with the QoS setting of the
  message via `ros2 topic info -v /xxx/topic`

● No picture from multiple cameras

* It is possible that the power supply is insufficient, do not connect all cameras to the same hub, and a powered hub is
  recommended

* It is also possible that the resolution is too high, try turning it lower
* If the color camera is an UVC camera, the serial number of the UVC may not be set

## License

Copyright 2023 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

**Other names and brands may be claimed as the property of others**
