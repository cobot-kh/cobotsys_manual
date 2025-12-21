# 마커를 이용한 대상물 잡기 케이스  

## 소개  

1. AprilTag(Aruco Marker)를 활ㅇ요하여 좌표계의 대상위치를 얻음
2. 대상물의 좌표 값에 따라 왼손사용 또는 오른손 사용을 개별적으로 판단
3. 팔의 각 관절 목표 각도는 IK 리버스 역학을 통해서 계산
4. 대상물에 도달하는 좌표 값 외에 움직임을 사전 지정된 좌표계로 이동

## 목적  
1. 비전과 팔의 움직임에 대한 연동을 함께 고려

## 로봇 자동 프로세스
1. 로봇 헤드의 정위치에서 지표면 방향으로 고개를 내린 후 좌표값에 대한 탐색(마커 서칭)을 싫시함
2. 좌표가 결정되면 움직임 실시
3. 그립 후 지정된 위치로 이동
4. 움직인 팔의 원위치 복귀후 로봇 헤드의 정위치 복귀하면 싸이클 끝남


## Tag(마커) 준비

[마커 제작 사이트](https://chev.me/arucogen/)  

사용된 마커는 Apritag 36h11 타입 
현재 셋팅은 42mm (0.042)로 설정되어 있음

## 상위기의 준비
마커 Reading을 위한 config 파일 의 조정   
  - 파일 위치   
``kuavo_ros_application/src/ros_vision/detection_apriltag/apriltag_ros/config/tags.yaml``
  - 예시
```yaml
standalone_tags:
[
  {id: 0, size: 0.042, name: 'tag_0'},
  {id: 1, size: 0.042, name: 'tag_1'},
  ...
]
# ***인쇄된 Tag 크기 확인 및 config 파일과의 일치 필요 ***
```
## 하위기 준비
  - 실행할 데모의 위치는
```kuavo-ros-opensource/src/demo/arm_capture_apriltag```

  - 최초 catkin(ros1) Build 실행
```bash
cd kuavo-ros-opensource
sudo su
catkin build kuavo_sdk
```
  - IK 패키지 빌드
```bash
catkin build motion_capture_ik
```
  - 로봇 구동 ** 주의 H12 리모콘을 이용해 로봇의 기동 시킨경우 이단계는 실행하면 안됨
```bash
roslaunch humanoid_controllers load_kuavo_real.launch cali:=true
```

  - IK Solver 서비스 확인 / 실행
```bash
rosnode list | grep ik
```
  - ``/arms_ik_node 존재`` → 다음 단계로  

  - 존재하지 않음 → 아래 실행  
```bash
cd kuavo-ros-opensource
sudo su
source devel/setup.bash
roslaunch motion_capture_ik ik_node.launch
```

## 상위기에서의 로봇 헤드 구동 및 센서 기동
  - 준비
```bash
cd kuavo_ros_application
source /opt/ros/noetic/setup.bash
```
  - 디스플레이 설정 옵션(원격, 직접 연결 기동시)
```bash
# Host PC (i7)
export DISPLAY=:1.0

# ORIN-NX / AGX-Orin
export DISPLAY=:2.0
```
  - 헤드 센서 기동
```bash
source devel/setup.bash

# 구버전(4, 4Pro)
roslaunch dynamic_biped load_robot_head.launch

# 표준 / 전시 / 고급
roslaunch dynamic_biped load_robot_head.launch use_orbbec:=true

# Max
roslaunch dynamic_biped load_robot_head.launch use_orbbec:=true enable_wrist_camera:=true
```
## 하위기 에서의 센서 데이터 수신 확인
```bash
rostopic list | grep tag
rostopic echo /robot_tag_info
```
  - 데이터 도착 확인 후 다음단계로 실행
## 하위기에서 메인 로직 실행
```bash
python3 src/demo/arm_capture_apriltag/arm_capture_apriltag.py --offset_start True
```

## Offset 파라미터 정리
```python
if offset_start == True:
    offset_z = -0.10     # 태그 바로 아래 → 음수
    temp_x_l = -0.035
    temp_y_l =  0.035
    temp_x_r = -0.045
    temp_y_r =  0.035
else:
    offset_z = temp_x_l = temp_y_l = temp_x_r = temp_y_r = 0.0

offset_angle = 1.00
```
  - 오른손 튜닝 가이드  
    A. 높이(z) 튜닝: offset_z
       - 잡는 점이 너무 위에서 멈춘다 → offset_z를 더 낮게(더 음수로)  
         예: ``-0.10 → -0.12``  
       - 잡는 점이 너무 아래로 박는다/충돌한다 → offset_z를 올린다(0쪽으로)  
         예: ``-0.10 → -0.08``  
         팁 : 실물은 태그가 “병뚜껑 위” 같은 곳에 있으면, 실제 파지점은 태그 아래로 내려가야 해서 offset_z 음수가 거의 필수로 나옴

    B. 앞/뒤(x) 튜닝: temp_x_r
       - 손이 너무 앞으로 나가서 목표를 지나친다(앞쪽으로 오버) → temp_x_r 줄이기(더 음수로)
         ``예: -0.045 → -0.055``
       - 손이 덜 나가서 목표보다 뒤에서 멈춘다(앞쪽으로 부족) → temp_x_r 늘리기(0쪽으로)
         ``예: -0.045 → -0.035``
    
    C. 좌/우(y) 튜닝: temp_y_r
       - 오른손은 오른쪽 영역을 잡으러 가는 경우가 많아서, temp_y_r은 “오른쪽으로 맞추는 보정”에 자주 쓰임.
       - 잡는 점이 오른쪽으로 치우쳤다 → temp_y_r 감소
       - 잡는 점이 왼쪽으로 치우쳤다 → temp_y_r 증가
       - 팁 : temp_y_r은 보통 0.005~0.01 단위로 움직여도 체감이 큼. 한 번에 0.03씩 바꾸면 “원인 파악”이 어려워짐.
    
    D. 회전(yaw) 튜닝: offset_angle + roll/pitch 고정
       - 손목이 태그 방향에 비해 각도가 계속 비틀어진다 → offset_angle을 1.00 기준으로 조금씩 조정
         ```예: 1.00 → 0.98 또는 1.02```

  - 왼손 튜닝 가이드  
    왼손은 원리 동일하고, 차이는 **y 보정 방향(좌측 영역 보정)**이 더 크게 들어가는 경우가 많음  
    A. 높이(z) 튜닝 : 오른손과 완전히 동일 규칙  
    B. 앞/뒤(x): temp_x_l  
       - 너무 앞으로 나감(오버) → temp_x_l 더 음수로.  
       - 덜 나감(언더) → temp_x_l 0쪽으로.  
       - (오른손 x 규칙과 동일).
   
    C. 좌/우(y): temp_y_l  
       - y는 항상 양수, 그리고 “왼쪽으로 갈수록 증가”    
       - 잡는 점이 왼쪽으로 너무 붙는다(좌측 오버) → temp_y_l 감소    
       - 잡는 점이 오른쪽으로 밀린다(좌측이 부족) → temp_y_l 증가    

       - 왼손은 대칭이라 생각하고 temp_y_r 규칙을 그대로 뒤집으면 되는데,  
       - 실물에서는 카메라 설치/헤드 각도 때문에 왼손 y 보정이 오른손보다 더 크게 필요한 경우가 잦음.  
       - 그래서 기본값도 예시처럼 temp_y_l=0.035로 시작하는 게 무난.  
