개요
이 문서는 ROS1 환경에서 버튼을 YOLO로 검출하고, depth를 이용해 카메라 기준 3D 좌표를 만든 뒤, TF로 base_link 기준으로 변환하고, RViz Marker 및 pre_press / press 목표점까지 생성하는 전체 실행 순서를 정리한 문서입니다.

사전 조건
ROS Noetic 설치
Orbbec Gemini 330 카메라 사용
YOLO 모델 파일 준비 완료
패키지 your_pkg 안에 아래 노드가 있어야 함

```bash
button_detector_ros.py
button_point_tf_transformer.py
target_point_marker_pub.py
target_lock_prepress_pub.py
```

주의
your_pkg, ~/catkin_ws 는 실제 환경에 맞게 수정해서 사용
camera_base -> camera_link static TF는 현재 임시값 0 0 0 기준임
정확한 버튼 누르기 전에는 실제 카메라 장착 위치 오프셋 보정 필요

실행 체크리스트

[ ] ROS source
[ ] roscore 실행
[ ] 카메라 launch 실행
[ ] color / depth / camera_info 확인
[ ] button_detector_ros.py 실행
[ ] /vision/target_point_red 또는 green 확인
[ ] static TF 연결
[ ] button_point_tf_transformer.py 실행
[ ] /vision/target_point_red_base 확인
[ ] RViz marker 확인
[ ] target_lock_prepress_pub.py 실행
[ ] /vision/pre_press_target_point_red_base 확인

ROS 환경 source

```bash
source devel/setup.bash
```
roscore 실행

roscore

카메라 실행

```bash
roslaunch orbbec_camera gemini_330_series.launch depth_registration:=true
```


카메라 토픽 확인
```bash
rostopic list | egrep "color|depth|aligned|camera_info"
```

camera_info 확인

```bash
rostopic echo -n 1 /camera/color/camera_info/header
rostopic echo -n 1 /camera/depth/camera_info/header
rostopic echo -n 1 /camera/color/camera_info | head -20
rostopic echo -n 1 /camera/depth/camera_info | head -20
```

확인 포인트
frame_id 동일 여부
width, height 동일 여부
K 값 동일 여부

버튼 검출 노드 실행

```bash
rosrun your_pkg button_detector_ros.py
```

버튼 검출 토픽 확인

```bash
rostopic echo /vision/target_point_red
rostopic echo /vision/target_valid_red
rostopic echo /vision/target_point_green
rostopic echo /vision/target_valid_green
```

TF 연결 상태 확인

```bash
rosrun tf tf_echo base_link camera_color_optical_frame
```

연결이 안 되어 있으면 unconnected trees 에러 발생 가능

카메라 TF 임시 연결
이 터미널은 계속 켜둠

```bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_base camera_link
```

TF 연결 확인

```bash
rosrun tf tf_echo base_link camera_color_optical_frame
```

정상일 경우 translation / rotation 값이 계속 출력됨

camera frame 기준 점을 base_link 기준 점으로 변환하는 노드 실행

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg button_point_tf_transformer.py
```

base 기준 토픽 확인

```bash
rostopic info /vision/target_point_red_base
rostopic echo /vision/target_point_red_base
rostopic echo /vision/target_point_green_base
```

base 기준 target point를 RViz Marker로 표시하는 노드 실행
red marker 기준

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_point_marker_pub.py _input_topic:=/vision/target_point_red_base _marker_topic:=/vision/target_marker_red_base _marker_lifetime:=0 _marker_scale:=0.08 _text_scale:=0.06
```

RViz 실행

```bash
rviz
```

RViz 설정

Fixed Frame = base_link
Add -> Marker
Topic = /vision/target_marker_red_base

target lock + pre_press + press 목표점 생성 노드 실행

```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_lock_prepress_pub.py
```

lock / pre_press / press 토픽 확인

```bash
rostopic echo /vision/locked_target_point_red_base
rostopic echo /vision/pre_press_target_point_red_base
rostopic echo /vision/press_target_point_red_base
rostopic echo /vision/locked_target_valid_red_base
```

pre_press Marker 노드 실행

```bash
rosrun your_pkg target_point_marker_pub.py _input_topic:=/vision/pre_press_target_point_red_base _marker_topic:=/vision/pre_press_target_marker_red_base _color_r:=0.0 _color_g:=0.0 _color_b:=1.0 _marker_lifetime:=0 _marker_scale:=0.06
```

pre_press Marker RViz 설정

Add -> Marker
Topic = /vision/pre_press_target_marker_red_base

press Marker 노드 실행

```bash
rosrun your_pkg target_point_marker_pub.py _input_topic:=/vision/press_target_point_red_base _marker_topic:=/vision/press_target_marker_red_base _color_r:=1.0 _color_g:=0.5 _color_b:=0.0 _marker_lifetime:=0 _marker_scale:=0.06
```

press Marker RViz 설정

Add -> Marker
Topic = /vision/press_target_marker_red_base

최종 점검 명령

```bash
rostopic list | grep vision
rostopic hz /vision/target_point_red
rostopic hz /vision/target_point_red_base
rostopic hz /vision/pre_press_target_point_red_base
rosnode list | grep button
```

권장 터미널 구성

Terminal 1
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roscore
```

Terminal 2
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
roslaunch orbbec_camera gemini_330_series.launch depth_registration:=true
```
Terminal 3
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg button_detector_ros.py
```

Terminal 4
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_base camera_link
```

Terminal 5
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg button_point_tf_transformer.py
```

Terminal 6
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_point_marker_pub.py _input_topic:=/vision/target_point_red_base _marker_topic:=/vision/target_marker_red_base _marker_lifetime:=0 _marker_scale:=0.08 _text_scale:=0.06
```

Terminal 7
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_lock_prepress_pub.py
```

Terminal 8
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_point_marker_pub.py _input_topic:=/vision/pre_press_target_point_red_base _marker_topic:=/vision/pre_press_target_marker_red_base _color_r:=0.0 _color_g:=0.0 _color_b:=1.0 _marker_lifetime:=0 _marker_scale:=0.06
```

Terminal 9
```bash
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
rosrun your_pkg target_point_marker_pub.py _input_topic:=/vision/press_target_point_red_base _marker_topic:=/vision/press_target_marker_red_base _color_r:=1.0 _color_g:=0.5 _color_b:=0.0 _marker_lifetime:=0 _marker_scale:=0.06
```

Terminal 10
```bash
rviz
```
트러블슈팅

TF transform failed: unconnected trees
카메라 프레임과 base_link 트리가 연결되지 않은 상태

확인
```bash
rosrun tf tf_echo base_link camera_color_optical_frame
```

임시 연결
```bash
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_base camera_link
```

Marker가 RViz에 안 보임
Fixed Frame = base_link 인지 확인
Marker display를 추가했는지 확인
Topic 이름이 정확한지 확인
marker_lifetime:=0 으로 실행했는지 확인
/vision/pre_press_target_point_red_base 는 보이는데 RViz에 안 보임
PointStamped 토픽은 RViz Marker display에 직접 안 뜸
별도 marker publisher 노드 실행 필요
