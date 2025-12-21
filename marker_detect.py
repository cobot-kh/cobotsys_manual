#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import AprilTagDetectionArray
import math
import numpy as np  # import numpy library for numerical computation

import time
import argparse
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
from kuavo_msgs.srv import twoArmHandPoseCmdSrv
from kuavo_msgs.msg import twoArmHandPoseCmd, ikSolveParam

from kuavo_msgs.msg import robotHandPosition
from kuavo_msgs.msg import robotHeadMotionData


####################################################################

# Custom IK parameters
use_custom_ik_param = True
# Use default joint angles as the initial guess for IK
joint_angles_as_q0 = False
# Create ikSolverParam object
ik_solve_param = ikSolveParam()
# Set parameters for ikSolveParam
ik_solve_param.major_optimality_tol = 1e-3
ik_solve_param.major_feasibility_tol = 1e-3
ik_solve_param.minor_feasibility_tol = 1e-3
ik_solve_param.major_iterations_limit = 500
ik_solve_param.oritation_constraint_tol = 1e-3
ik_solve_param.pos_constraint_tol = 1e-3
ik_solve_param.pos_cost_weight = 0.0

# Hand open/close control
close_hand = [100, 100, 80, 75, 75, 75]    # catch pose
open_hand = [0, 100, 0, 0, 0, 0]           # open pose

# Head up/down control
def set_head_target(yaw, pitch):
    """
    Set head target position and publish message
    :param yaw: head yaw angle, range [-30, 30] degrees
    :param pitch: head pitch angle, range [-25, 25] degrees
    """

    # Create a publisher to publish to '/robot_head_motion_data'
    # Use robotHeadMotionData message type, queue size 10
    pub_head_pose = rospy.Publisher('/robot_head_motion_data', robotHeadMotionData, queue_size=10)
    rospy.sleep(0.5)  # Ensure Publisher is registered

    # Create a robotHeadMotionData message object
    head_target_msg = robotHeadMotionData()

    # Set joint data, containing yaw and pitch
    # Ensure yaw within [-30, 30], pitch within [-25, 25]
    head_target_msg.joint_data = [yaw, pitch]

    # Publish message to target topic
    pub_head_pose.publish(head_target_msg)

    # Print log info, showing the published head target
    rospy.loginfo(f"Published head target: yaw={yaw}, pitch={pitch}")

######################## IK solving section ############################################

# Get robot version/parameters
def get_parameter(param_name):
    try:
        # Get parameter value
        param_value = rospy.get_param(param_name)
        rospy.loginfo(f"Parameter {param_name} value: {param_value}")
        return param_value
    except rospy.ROSException:
        rospy.logerr(f"Parameter {param_name} does not exist! Program exiting.")
        rospy.signal_shutdown("Failed to get parameter")
        return None

# IK inverse kinematics service
def call_ik_srv(eef_pose_msg):
    # Ensure the service is available
    rospy.wait_for_service('/ik/two_arm_hand_pose_cmd_srv')
    try:
        # Initialize service proxy
        ik_srv = rospy.ServiceProxy('/ik/two_arm_hand_pose_cmd_srv', twoArmHandPoseCmdSrv)
        # Call service and get response
        res = ik_srv(eef_pose_msg)
        # Return IK result
        return res
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False, []

# Set arm control mode
def set_arm_control_mode(mode):
    # Create service proxy to communicate with service
    arm_traj_change_mode_client = rospy.ServiceProxy("/arm_traj_change_mode", changeArmCtrlMode)

    # Create request object
    request = changeArmCtrlModeRequest()
    request.control_mode = mode  # Set requested control mode

    # Send request and receive response
    response = arm_traj_change_mode_client(request)

    if response.result:
        rospy.loginfo(f"Successfully changed arm control mode to {mode}: {response.message}")
    else:
        rospy.logwarn(f"Failed to change arm control mode to {mode}: {response.message}")

# Publish arm target poses
def publish_arm_target_poses(times, values):
    # Create Publisher object
    pub = rospy.Publisher('kuavo_arm_target_poses', armTargetPoses, queue_size=10)
    rospy.sleep(0.5)  # Ensure Publisher is registered

    # Create message object and set input times and values
    msg = armTargetPoses()
    msg.times = times
    msg.values = values

    rospy.loginfo("Publishing arm target poses to topic 'kuavo_arm_target_poses'")

    # Wait for subscriber connection
    rate = rospy.Rate(10)  # 10Hz
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("Waiting for subscribers to connect...")
        rate.sleep()

    # Publish message
    pub.publish(msg)
    rospy.loginfo("Message published.")

# Quaternion helper class
class Quaternion:
    def __init__(self):
        self.w = 0
        self.x = 0
        self.y = 0
        self.z = 0

# yaw (Z), pitch (Y), roll (X)
# Euler angles (Z-Y-X order) → rotation matrix → quaternion
def euler_to_rotation_matrix(yaw_adaptive=0, pitch_adaptive=0, roll_adaptive=0,
                            yaw_manual=0, pitch_manual=0, roll_manual=0):
    """
    Euler angles (Z-Y-X order) → rotation matrix
    Parameters:
        yaw (float): rotation around Z axis (radians)
        pitch (float): rotation around Y axis (radians)
        roll (float): rotation around X axis (radians)
    Returns:
        np.ndarray: 3x3 rotation matrix
    """
    # Compute trig values
    cy, sy = np.cos(yaw_adaptive), np.sin(yaw_adaptive)
    cp, sp = np.cos(pitch_adaptive), np.sin(pitch_adaptive)

    R = np.array([
        [cy * cp,   -sy,        cy * sp],
        [sy * cp,    cy,        sy * sp],
        [-sp,        0,         cp     ]
    ])

    # If custom parameters exist, apply secondary rotation
    if yaw_manual or pitch_manual or roll_manual:

        # Initialize as identity matrix
        R_manual = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ])
        if abs(yaw_manual) > 0.01:
            print("yaw_manual=",yaw_manual)
            c, s = np.cos(yaw_manual), np.sin(yaw_manual)
            R_manual = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]) @ R_manual

        if abs(pitch_manual) > 0.01:
            print("pitch_manual=",pitch_manual)
            c, s = np.cos(pitch_manual), np.sin(pitch_manual)
            R_manual = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ R_manual

        if abs(roll_manual) > 0.01:
            print("roll_manual=",roll_manual)
            c, s = np.cos(roll_manual), np.sin(roll_manual)
            R_manual = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) @ R_manual

        return R @ R_manual
    # No custom parameters, return rotation matrix directly
    else:
        return R

def rotation_matrix_to_quaternion(R):
    """
    Rotation matrix → quaternion
    Parameters:
        R (np.ndarray): 3x3 rotation matrix
    Returns:
        Quaternion: quaternion object with fields x,y,z,w
    """
    trace = np.trace(R)
    q = Quaternion()

    if trace > 0:
        q.w = math.sqrt(trace + 1.0) / 2
        q.x = (R[2, 1] - R[1, 2]) / (4 * q.w)
        q.y = (R[0, 2] - R[2, 0]) / (4 * q.w)
        q.z = (R[1, 0] - R[0, 1]) / (4 * q.w)
    else:
        # Handle case where w is near zero
        i = np.argmax([R[0, 0], R[1, 1], R[2, 2]])
        j = (i + 1) % 3
        k = (j + 1) % 3
        t = np.zeros(4)
        t[i] = math.sqrt(R[i, i] - R[j, j] - R[k, k] + 1) / 2
        t[j] = (R[i, j] + R[j, i]) / (4 * t[i])
        t[k] = (R[i, k] + R[k, i]) / (4 * t[i])
        t[3] = (R[k, j] - R[j, k]) / (4 * t[i])

        q.x, q.y, q.z, q.w = t  # reorder as [x, y, z, w]

    # Normalize (avoid numerical drift)
    norm = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
    if norm > 0:
        q.w /= norm
        q.x /= norm
        q.y /= norm
        q.z /= norm
    return q

def euler_to_quaternion_via_matrix(yaw_adaptive=0, pitch_adaptive=0, roll_adaptive=0,
                                    yaw_manual=0, pitch_manual=0, roll_manual=0):
    """
    Euler angles → rotation matrix → quaternion
    Parameters:
        yaw (float): rotation around Z (radians)
        pitch (float): rotation around Y (radians)
        roll (float): rotation around X (radians)
    Returns:
        Quaternion: quaternion object
    """
    R = euler_to_rotation_matrix(yaw_adaptive, pitch_adaptive, roll_adaptive,
                                yaw_manual, pitch_manual, roll_manual)
    return rotation_matrix_to_quaternion(R)

######################### AprilTag processing section ###########################################

class AprilTagProcessor:
    def __init__(self, is_init=False):
        """
        Initialize AprilTagProcessor class.
        :param is_init: whether to initialize ROS node (default False)
        """
        if is_init:
            rospy.init_node('tag_detections_listener', anonymous=True)

    def quaternion_to_euler(self, w, x, y, z):
        """
        Convert quaternion to Euler angles (yaw).
        :param w, x, y, z: quaternion components.
        :return: yaw angle in degrees.
        """
        # Compute roll, pitch, yaw
        sinr_cosp = 2 * (w * z + x * y)
        cosr_cosp = 1 - 2 * (y**2 + z**2)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        #print(f"roll: {math.degrees(roll)}")

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp) if abs(sinp) < 1 else math.copysign(math.pi / 2, sinp)
        #print(f"pitch: {math.degrees(pitch)}")

        siny_cosp = 2 * (w * x + y * z)
        cosy_cosp = 1 - 2 * (x**2 + y**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        #print(f"yaw: {math.degrees(yaw)}")
        return math.degrees(yaw)

    def get_apriltag_data(self):
        """
        Get AprilTag detection data from the specified ROS topic.
        :return: list containing each AprilTag info.
        """
        try:
            # Wait for AprilTagDetectionArray message from topic "/robot_tag_info"
            msg = rospy.wait_for_message("/robot_tag_info", AprilTagDetectionArray, timeout=5)
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to get AprilTag data: {e}")
            return None

        data_list = []
        for detection in msg.detections:
            id = detection.id[0]  # Get AprilTag ID
            quaternion = detection.pose.pose.pose.orientation  # Get quaternion pose
            pos = detection.pose.pose.pose.position  # Get position

            # Convert quaternion to yaw angle
            yaw_angle = self.quaternion_to_euler(quaternion.x, quaternion.y, quaternion.z, quaternion.w)

            # Build AprilTag data dict
            tag_data = {
                "id": id,
                "off_horizontal": round(pos.x, 3),
                "off_camera": round(pos.y, 3),
                "off_vertical": round(pos.z, 3),
                "yaw_angle": yaw_angle
            }
            data_list.append(tag_data)

        return data_list

    def get_apriltag_by_id(self, tag_id):
        """
        Get specific AprilTag data by ID.
        :param tag_id: AprilTag ID to find.
        :return: matched AprilTag dict.
        """
        all_tags = self.get_apriltag_data()
        if all_tags is None:
            rospy.logerr("Failed to get AprilTag data")
            return None

        for tag in all_tags:
            if tag["id"] == tag_id:
                return tag

        return None

    def get_averaged_apriltag_data(self, tag_id, num_samples=10):
        """
        Get averaged AprilTag position and pose data for specified ID.
        :param tag_id: AprilTag ID to find.
        :param num_samples: number of samples used for averaging (default 10).
        :return: dict containing averaged position and pose.
        """
        data_list = []

        while len(data_list) < num_samples:
            if rospy.is_shutdown():
                return None
            tag_data = self.get_apriltag_by_id(tag_id)
            if tag_data:
                data_list.append(tag_data)
            else:
                rospy.loginfo(f"AprilTag ID {tag_id} not detected, waiting...")
                time.sleep(0.1)

        # Use numpy to compute averages
        avg_off_horizontal = np.mean([tag["off_horizontal"] for tag in data_list])
        avg_off_camera = np.mean([tag["off_camera"] for tag in data_list])
        avg_off_vertical = np.mean([tag["off_vertical"] for tag in data_list])
        avg_yaw_angle = np.mean([tag["yaw_angle"] for tag in data_list])

        result = {
            "id": tag_id,
            "avg_off_horizontal": round(avg_off_horizontal, 3),
            "avg_off_camera": round(avg_off_camera, 3),
            "avg_off_vertical": round(avg_off_vertical, 3),
            "avg_yaw_angle": round(avg_yaw_angle, 3)
        }
        rospy.loginfo(
            f"AprilTag ID: {result['id']}, "
            f"Position: x={result['avg_off_horizontal']}, y={result['avg_off_camera']}, z={result['avg_off_vertical']}, "
            f"Tilt angle: {result['avg_yaw_angle']}"
        )
        return result

########################### Main function #########################################

def main():

    # Create AprilTagProcessor instance and initialize ROS node
    processor = AprilTagProcessor(is_init=True)

    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Whether to enable offset")
    parser.add_argument("--offset_start", type=str, choices=["False", "True"], required="True", help="Choose offset_start = True or False")
    parser.add_argument("--cost_weight", type=float, default=0.0, help="Pass pos_cost_weight")
    args = parser.parse_args()

    if args.cost_weight != 0.0:
        ik_solve_param.pos_cost_weight = args.cost_weight
        print(f"pos_cost_weight:{ik_solve_param.pos_cost_weight}")

    # offset_start="True" enables offsets, otherwise disables offsets
    if args.offset_start == "True":
        # Bias slightly to the side/back
        offset_z = -0.094  # Grasp point is directly below the tag
        temp_x_l = 0.08
        temp_y_l = 0.06
        temp_x_r = -0.0
        temp_y_r = 0.0
    else:
        offset_z = 0.00
        temp_x_l = 0.00
        temp_y_l = 0.00
        temp_x_r = 0.00
        temp_y_r = 0.00

    # Angle offset (correct rotation around z-axis)
    offset_angle = 1.00

    # Look down
    set_head_target(0, 20)
    print("head down")
    time.sleep(2)

##########################################   Find AprilTag   #########################################

    # Get averaged data for specified AprilTag ID
    tag_data = processor.get_averaged_apriltag_data(tag_id=0)

    # During tag detection, press ctrl+c to exit
    if rospy.is_shutdown():
        return None

##########################################   Parameter preparation   #########################################

    # Decide left or right hand; subsequent logic uses this flag
    # position_flag > 0: left hand, else right hand
    # To force a fixed hand, set position_flag here explicitly
    if False:
        position_flag = -1
    else:
        position_flag = tag_data['avg_off_camera'] * 100

    print(f"tag position: {position_flag}")

    # Get robot version
    robot_version = get_parameter('robot_version')
    # Initial positions for different robot models (robot coordinate frame)
    def start_with_version(version_number: int, series: int):
        """Check whether version number belongs to a series"""
        # PPPPMMMMN
        MMMMN_MASK = 100000
        return (version_number % MMMMN_MASK) == series

    if start_with_version(robot_version, 45) or start_with_version(robot_version, 49):
        robot_zero_x = -0.0173
        robot_zero_y = -0.2927
        robot_zero_z = -0.2837

    elif start_with_version(robot_version, 42):
        robot_zero_x = -0.0175
        robot_zero_y = -0.25886
        robot_zero_z = -0.20115

    else:
        print("Robot version number error, only supports 42 / 45 / 49 series")

    # Set arm motion mode to external control
    set_arm_control_mode(2)

    # Hand control API
    # Initialize topic publisher
    hand_control_pub = rospy.Publisher('/control_robot_hand_position', robotHandPosition, queue_size=10)
    # Create message object
    hand_control_msg = robotHandPosition()

########################################## Motion control - IK solving #########################################
    # Create request object
    eef_pose_msg = twoArmHandPoseCmd()
    # Set request params
    eef_pose_msg.ik_param = ik_solve_param
    eef_pose_msg.use_custom_ik_param = use_custom_ik_param
    eef_pose_msg.joint_angles_as_q0 = joint_angles_as_q0
    # When joint_angles_as_q0 is False, these two are not used (unit: radians)
    eef_pose_msg.hand_poses.left_pose.joint_angles = np.zeros(7)
    eef_pose_msg.hand_poses.right_pose.joint_angles = np.zeros(7)

    # Grasp position correction
    if position_flag > 0:
        # Set left end-effector position
        set_x = tag_data['avg_off_horizontal'] + temp_x_l
        set_y = tag_data['avg_off_camera'] + temp_y_l
        set_z = tag_data['avg_off_vertical'] + offset_z
    else:
        # Set right end-effector position
        set_x = tag_data['avg_off_horizontal'] + temp_x_r
        set_y = tag_data['avg_off_camera'] - temp_y_r
        set_z = tag_data['avg_off_vertical'] + offset_z

    # Compute IK targets based on left/right
    if position_flag > 0:
        # Set left end-effector position and orientation
        eef_pose_msg.hand_poses.left_pose.pos_xyz = np.array([set_x, set_y, set_z])
        # Compute relative angle
        relative_angle = math.atan((robot_zero_y - set_y) / (set_x - robot_zero_x))
        print(f"relative_angle: {relative_angle}")
        # Compute quaternion
        quat = euler_to_quaternion_via_matrix(relative_angle * offset_angle, -1.57, 0)
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = [quat.x, quat.y, quat.z, quat.w]  # with yaw angle
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)

        # Right hand stays at robot initial position
        eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array([robot_zero_x, robot_zero_y, robot_zero_z])
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = [0.0, 0.0, 0.0, 1.0]  # vertical state
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)
    else:
        # Left hand stays at robot initial position
        eef_pose_msg.hand_poses.left_pose.pos_xyz = np.array([robot_zero_x, -1 * robot_zero_y, robot_zero_z])
        eef_pose_msg.hand_poses.left_pose.quat_xyzw = [0.0, 0.0, 0.0, 1.0]  # vertical state
        eef_pose_msg.hand_poses.left_pose.elbow_pos_xyz = np.zeros(3)

        # Set right end-effector position and orientation
        eef_pose_msg.hand_poses.right_pose.pos_xyz = np.array([set_x, set_y, set_z])
        # Compute relative angle
        relative_angle = math.atan((set_y - robot_zero_y) / (set_x - robot_zero_x))
        print(f"relative_angle: {relative_angle}")
        # Compute quaternion
        quat = euler_to_quaternion_via_matrix(relative_angle * offset_angle, -1.57, 0)
        eef_pose_msg.hand_poses.right_pose.quat_xyzw = [quat.x, quat.y, quat.z, quat.w]  # with yaw angle
        eef_pose_msg.hand_poses.right_pose.elbow_pos_xyz = np.zeros(3)

    print("Grasp point x y z")
    print(set_x, " , ", set_y, " , ", set_z)

    # Call IK service
    res = call_ik_srv(eef_pose_msg)

    # IK success
    if (res.success):
########################################## Show IK results ####################################################

        l_pos = res.hand_poses.left_pose.pos_xyz
        l_pos_error = np.linalg.norm(l_pos - eef_pose_msg.hand_poses.left_pose.pos_xyz)
        r_pos = res.hand_poses.right_pose.pos_xyz
        r_pos_error = np.linalg.norm(r_pos - eef_pose_msg.hand_poses.right_pose.pos_xyz)

        # Print partial IK results
        print(f"time_cost: {res.time_cost:.2f} ms. left_pos_error: {1e3*l_pos_error:.2f} mm, right_pos_error: {1e3*r_pos_error:.2f} mm")
        print(f"left_joint_angles: {res.hand_poses.left_pose.joint_angles}")
        print(f"right_joint_angles: {res.hand_poses.right_pose.joint_angles}")
        print(f"res.q_arm: {res.q_arm}")

########################################## Motion control - Pre pose #########################################

        # Open hands
        hand_control_msg.left_hand_position = open_hand
        hand_control_msg.right_hand_position = open_hand
        hand_control_pub.publish(hand_control_msg)

        # Initial position
        print("move to position 0")
        publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                                         20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
        print("Return to initial position")

        time.sleep(0.5)

        # Move to waiting position
        if position_flag > 0:
            # Open arm span
            print("move to position 1")
            publish_arm_target_poses([1.5], [20.0, 60.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                             20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5)
            # Bend elbow
            print("move to position 2")
            publish_arm_target_poses([1.5], [0.0, 60.0, 0.0, -90.0, 0.0, 0.0, 0.0,
                                             20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5)
            print("move over")
        else:
            # Open arm span
            print("move to position 1")
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                                             20.0, -60.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            time.sleep(1.5)
            # Bend elbow
            print("move to position 2")
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                                             -10.0, -60.0, 0.0, -90.0, 35.0, 20.0, 0.0])
            time.sleep(1.5)
            print("move over")

########################################## Motion control - Execute IK result #########################################

        # 0.35 0.52
        if position_flag > 0:
            joint_end_angles = np.concatenate([res.hand_poses.left_pose.joint_angles, [0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0]])
        else:
            joint_end_angles = np.concatenate([[0.35, 0.0, 0.0, -0.52, 0.0, 0.0, 0.0], res.hand_poses.right_pose.joint_angles])

        degrees_list = [math.degrees(rad) for rad in joint_end_angles]
        publish_arm_target_poses([3], degrees_list)
        print("IK solved and moved to the target pose")
        time.sleep(5)

        print("IK finished")

########################################## Motion control - Water handover flow #########################################
        if position_flag > 0:
            # Grip
            hand_control_msg.left_hand_position = close_hand
            hand_control_msg.right_hand_position = open_hand
            hand_control_pub.publish(hand_control_msg)
            time.sleep(1)
            publish_arm_target_poses([1.5], [-60.0, 0.0, 0.0, -30.0, -20.0, 0.0, 0.0,
                                             20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(2.5)
            # Release
            hand_control_msg.left_hand_position = open_hand
            hand_control_msg.right_hand_position = open_hand
            hand_control_pub.publish(hand_control_msg)
            time.sleep(1)
        else:
            hand_control_msg.left_hand_position = open_hand
            hand_control_msg.right_hand_position = close_hand
            hand_control_pub.publish(hand_control_msg)
            time.sleep(1)
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                                             -60.0, 0.0, 0.0, -30.0, 20.0, 0.0, 0.0])
            time.sleep(2.5)
            # Release
            hand_control_msg.left_hand_position = open_hand
            hand_control_msg.right_hand_position = open_hand
            hand_control_pub.publish(hand_control_msg)
            time.sleep(1)

        print("Handover completed")

        # Wait one more second after releasing
        time.sleep(1)

########################################## Motion control - Post processing #########################################
        # Arm reset corresponding to earlier angles
        if position_flag > 0:
            publish_arm_target_poses([1.5], [0.0, 60.0, 0.0, -90.0, 0.0, 0.0, 0.0,
                                             20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5)
            publish_arm_target_poses([1.5], [20.0, 60.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                                             20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5)
        else:
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                                             0.0, -60.0, 0.0, -90.0, 0.0, 0.0, 0.0])
            time.sleep(1.5)
            publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                                             20.0, -60.0, 0.0, -30.0, 0.0, 0.0, 0.0])
            time.sleep(1.5)

    # IK failed
    else:
        print("IK failed, program exiting")

########################################## End flow - Post handling #########################################
    # Return to initial position
    publish_arm_target_poses([1.5], [20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0,
                                    20.0, 0.0, 0.0, -30.0, 0.0, 0.0, 0.0])

    # Head reset (look up)
    set_head_target(0, 0)
    print("head reset")

    time.sleep(1.5)

    # Restore arm control mode to auto swing while walking
    set_arm_control_mode(1)

if __name__ == '__main__':
    main()
