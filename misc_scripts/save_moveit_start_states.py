"""
Usage:
Move robot around in rviz
Whenever you press "plan", it'll save that state

Whatever planning_group is active the first time you press plan,
that group is assumed for this whole file.


"""


import argparse
parser = argparse.ArgumentParser()
parser.add_argument("outfile", type=argparse.FileType("w"))
args = parser.parse_args()

import moveit_msgs.msg as mm


import rospy
from time import time, sleep
import openravepy
import numpy as np

rospy.init_node("save_states",disable_signals = True)

np.set_printoptions(precision=4, suppress=True)

states = []

#env_file: bookshelves.env.xml
#robot_name: pr2
#joint_names: [bl_caster_rotation_joint, bl_caster_l_wheel_joint, bl_caster_r_wheel_joint, br_caster_rotation_joint, br_caster_l_wheel_joint, br_caster_r_wheel_joint, fl_caster_rotation_joint, fl_caster_l_wheel_joint, fl_caster_r_wheel_joint, fr_caster_rotation_joint, fr_caster_l_wheel_joint, fr_caster_r_wheel_joint, torso_lift_joint, head_pan_joint, head_tilt_joint, l_shoulder_pan_joint, l_shoulder_lift_joint, l_upper_arm_roll_joint, l_elbow_flex_joint, l_forearm_roll_joint, l_wrist_flex_joint, l_wrist_roll_joint, l_gripper_l_finger_joint, l_gripper_motor_slider_joint, l_gripper_motor_screw_joint, l_gripper_joint, laser_tilt_mount_joint, r_shoulder_pan_joint, r_shoulder_lift_joint, r_upper_arm_roll_joint, r_elbow_flex_joint, r_forearm_roll_joint, r_wrist_flex_joint, r_wrist_roll_joint, r_gripper_l_finger_joint, r_gripper_motor_slider_joint, r_gripper_motor_screw_joint, r_gripper_joint, torso_lift_motor_screw_joint]
#default_joint_values: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#default_base_pose: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#active_joints: [27, 28, 29, 30, 31, 32, 33]
#active_affine: 0


RIGHTARM_JOINTS = ["r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint"]
LEFTARM_JOINTS = ["l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint"]
RIGHTARM_TORSO_JOINTS = ["torso_lift_joint"] + RIGHTARM_JOINTS
LEFTARM_TORSO_JOINTS = ["torso_lift_joint"] + LEFTARM_JOINTS
ARMS_JOINTS = LEFTARM_JOINTS + RIGHTARM_JOINTS
WHOLE_BODY_JOINTS = ["torso_lift_joint"] + LEFTARM_JOINTS + RIGHTARM_JOINTS


virgin = True

joint_names = None
default_joint_values = None
default_base_pose = None
active_joint_names = None
active_joint_inds = None
active_affine = None
group_name = None

def repr(flist):
    s = np.array(flist).__repr__()
    return s[s.index("["):s.index("]")+1]

def callback(msg):
    global virgin, joint_names, default_joint_values, default_base_pose,\
           active_joint_names, active_joint_inds, active_affine, group_name

    joint_state = msg.goal.request.start_state.joint_state
    base_state = msg.goal.request.start_state.multi_dof_joint_state    

    tf = base_state.joint_transforms[0]
    wxyz_xyz = [tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.translation.x, tf.translation.y, tf.translation.z]

    assert isinstance(msg, mm.MoveGroupActionGoal)

    if virgin:
        
        joint_names = joint_state.name
        default_joint_values = list(joint_state.position)
        default_base_pose = wxyz_xyz
        active_joint_names = {"right_arm":RIGHTARM_JOINTS, "left_arm": LEFTARM_JOINTS, 
                              "right_arm_and_torso":RIGHTARM_TORSO_JOINTS, "left_arm_and_torso":LEFTARM_TORSO_JOINTS,
                              "arms":ARMS_JOINTS, "whole_body":WHOLE_BODY_JOINTS, "base":[]
                              }[msg.goal.request.group_name]
        group_name = msg.goal.request.group_name
        active_joint_inds = [joint_names.index(name) for name in active_joint_names]
        aff = openravepy.DOFAffine
        active_affine = (aff.RotationAxis | aff.X | aff.Y) if msg.goal.request.group_name in ["whole_body","base"] else 0
        
        virgin = False
        
    assert joint_state.name == joint_names
        
    state = [joint_state.position[i] for i in active_joint_inds] + openravepy.RaveGetAffineDOFValuesFromTransform(wxyz_xyz, active_affine).tolist()

    states.append(state)
    print "added state", repr(state)


    

rospy.sleep(1)
sub = rospy.Subscriber("/move_group/goal", mm.MoveGroupActionGoal, callback)
rospy.sleep(1)


try:
    sleep(10000)
except KeyboardInterrupt:
    pass


#env_file: bookshelves.env.xml
#robot_name: pr2
#joint_names: [bl_caster_rotation_joint, bl_caster_l_wheel_joint, bl_caster_r_wheel_joint, br_caster_rotation_joint, br_caster_l_wheel_joint, br_caster_r_wheel_joint, fl_caster_rotation_joint, fl_caster_l_wheel_joint, fl_caster_r_wheel_joint, fr_caster_rotation_joint, fr_caster_l_wheel_joint, fr_caster_r_wheel_joint, torso_lift_joint, head_pan_joint, head_tilt_joint, l_shoulder_pan_joint, l_shoulder_lift_joint, l_upper_arm_roll_joint, l_elbow_flex_joint, l_forearm_roll_joint, l_wrist_flex_joint, l_wrist_roll_joint, l_gripper_l_finger_joint, l_gripper_motor_slider_joint, l_gripper_motor_screw_joint, l_gripper_joint, laser_tilt_mount_joint, r_shoulder_pan_joint, r_shoulder_lift_joint, r_upper_arm_roll_joint, r_elbow_flex_joint, r_forearm_roll_joint, r_wrist_flex_joint, r_wrist_roll_joint, r_gripper_l_finger_joint, r_gripper_motor_slider_joint, r_gripper_motor_screw_joint, r_gripper_joint, torso_lift_motor_screw_joint]
#default_joint_values: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#default_base_pose: [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
#active_joints: [r_shoulder_pan_joint, r_shoulder_lift_joint, r_upper_arm_roll_joint, r_elbow_flex_joint, r_forearm_roll_joint, r_wrist_flex_joint, r_wrist_roll_joint]
#active_affine: 0

#states:
    #- state0: &state0
        #[0,0,0,-0.2,0,-0.1,0]
    #- state1: &state1
        #[.1,0,0,-0.2,0,-0.1,0]
d = {}        
d["env_file"] = "FIXME"
d["robot_name"] = "pr2"
d["joint_names"] = joint_names
d["default_joint_values"] = default_joint_values
d["default_base_pose"] = default_base_pose
d["active_joints"] = active_joint_names
d["active_affine"] = active_affine


import yaml
yaml.dump(d, args.outfile)

args.outfile.write(
"""states:
""")
for (i,state) in enumerate(states):
    args.outfile.write("""
    - state%i: &state%i
        %s
     """%(i,i,repr(state)))
