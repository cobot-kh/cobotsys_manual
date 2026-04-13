
1. Source ROS Environment
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
2. Start ROS Core
roscore
3. Launch Camera

Orbbec Gemini 330 기준:

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch orbbec_camera gemini_330_series.launch depth_registration:=true
4. Check Camera Topics
rostopic list | egrep "color|depth|aligned|camera_info"
5. Check Camera Info
rostopic echo -n 1 /camera/color/camera_info/header
rostopic echo -n 1 /camera/depth/camera_info/header
rostopic echo -n 1 /camera/color/camera_info | head -20
rostopic echo -n 1 /camera/depth/camera_info | head -20

확인 포인트:

frame_id 동일 여부
width, height 동일 여부
K 값 동일 여부
6. Run Button Detector Node

YOLO + depth + camera frame 3D point publish

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg button_detector_ros.py
7. Check Detection Topics
rostopic echo /vision/target_point_red
rostopic echo /vision/target_valid_red
rostopic echo /vision/target_point_green
rostopic echo /vision/target_valid_green
8. Check TF Tree
rosrun tf tf_echo base_link camera_color_optical_frame

연결이 안 되어 있으면 unconnected trees 에러가 발생할 수 있습니다.

9. Connect Camera TF Temporarily

이 터미널은 계속 켜둡니다.

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_base camera_link
10. Verify TF Connection
rosrun tf tf_echo base_link camera_color_optical_frame

정상이라면 translation / rotation 값이 계속 출력됩니다.

11. Run TF Transformer Node

camera frame 기준 점을 base_link 기준 점으로 변환합니다.

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg button_point_tf_transformer.py
12. Check Base Frame Topics
rostopic info /vision/target_point_red_base
rostopic echo /vision/target_point_red_base
rostopic echo /vision/target_point_green_base
13. Run Marker Node for Base Target Point

red target marker 기준:

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_point_marker_pub.py \
  _input_topic:=/vision/target_point_red_base \
  _marker_topic:=/vision/target_marker_red_base \
  _marker_lifetime:=0 \
  _marker_scale:=0.08 \
  _text_scale:=0.06
14. Run RViz
rviz
RViz Settings
Fixed Frame = base_link
Add → Marker
Topic = /vision/target_marker_red_base
15. Run Target Lock / Pre-Press / Press Node

최근 N프레임 평균으로 목표점을 안정화하고, pre_press / press 점을 생성합니다.

source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_lock_prepress_pub.py
16. Check Lock / Pre-Press / Press Topics
rostopic echo /vision/locked_target_point_red_base
rostopic echo /vision/pre_press_target_point_red_base
rostopic echo /vision/press_target_point_red_base
rostopic echo /vision/locked_target_valid_red_base
17. Run Marker Node for Pre-Press Target
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_point_marker_pub.py \
  _input_topic:=/vision/pre_press_target_point_red_base \
  _marker_topic:=/vision/pre_press_target_marker_red_base \
  _color_r:=0.0 \
  _color_g:=0.0 \
  _color_b:=1.0 \
  _marker_lifetime:=0 \
  _marker_scale:=0.06
RViz Settings
Add → Marker
Topic = /vision/pre_press_target_marker_red_base
18. Run Marker Node for Press Target
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_point_marker_pub.py \
  _input_topic:=/vision/press_target_point_red_base \
  _marker_topic:=/vision/press_target_marker_red_base \
  _color_r:=1.0 \
  _color_g:=0.5 \
  _color_b:=0.0 \
  _marker_lifetime:=0 \
  _marker_scale:=0.06
RViz Settings
Add → Marker
Topic = /vision/press_target_marker_red_base
19. Final Debug Commands
rostopic list | grep vision
rostopic hz /vision/target_point_red
rostopic hz /vision/target_point_red_base
rostopic hz /vision/pre_press_target_point_red_base
rosnode list | grep button
Recommended Terminal Layout
Terminal 1 - ROS Core
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
Terminal 2 - Camera
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch orbbec_camera gemini_330_series.launch depth_registration:=true
Terminal 3 - Button Detector
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg button_detector_ros.py
Terminal 4 - Static TF
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_base camera_link
Terminal 5 - TF Transformer
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg button_point_tf_transformer.py
Terminal 6 - Base Target Marker
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_point_marker_pub.py \
  _input_topic:=/vision/target_point_red_base \
  _marker_topic:=/vision/target_marker_red_base \
  _marker_lifetime:=0 \
  _marker_scale:=0.08 \
  _text_scale:=0.06
Terminal 7 - Target Lock / Pre-Press / Press
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_lock_prepress_pub.py
Terminal 8 - Pre-Press Marker
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_point_marker_pub.py \
  _input_topic:=/vision/pre_press_target_point_red_base \
  _marker_topic:=/vision/pre_press_target_marker_red_base \
  _color_r:=0.0 \
  _color_g:=0.0 \
  _color_b:=1.0 \
  _marker_lifetime:=0 \
  _marker_scale:=0.06
Terminal 9 - Press Marker
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_point_marker_pub.py \
  _input_topic:=/vision/press_target_point_red_base \
  _marker_topic:=/vision/press_target_marker_red_base \
  _color_r:=1.0 \
  _color_g:=0.5 \
  _color_b:=0.0 \
  _marker_lifetime:=0 \
  _marker_scale:=0.06
Terminal 10 - RViz
rviz
