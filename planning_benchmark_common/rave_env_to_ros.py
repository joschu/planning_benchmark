import moveit_msgs.msg as mm
import geometry_msgs.msg as gm
import roslib.message as rm
import yaml
import os.path as osp
import sys, os
sys.path.insert(1, os.path.join(sys.path[0], '..')); import planning_benchmark_common as pbc
import shape_msgs.msg as sm
import geometry_msgs.msg as gm
import rospy
import openravepy
import func_utils as fu

ROS_JOINT_NAMES =  ['fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint', 'br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'torso_lift_joint', 'head_pan_joint', 'head_tilt_joint', 'laser_tilt_mount_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_motor_slider_joint', 'r_gripper_motor_screw_joint', 'r_gripper_l_finger_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_motor_slider_joint', 'l_gripper_motor_screw_joint', 'l_gripper_l_finger_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_joint', 'torso_lift_motor_screw_joint']



@fu.once
def make_planning_scene_pub():
    rospy.sleep(1)
    out = rospy.Publisher("/planning_scene", mm.PlanningScene)
    rospy.sleep(1)
    return out

def RosPoseFromRaveMatrix(mat):
    out = gm.Pose()
    out.position = gm.Point(*mat[:3,3])
    qw,qx,qy,qz = openravepy.quatFromRotationMatrix(mat[:3,:3])
    out.orientation = gm.Quaternion(qx,qy,qz, qw)
    return out
def RosTransformFromRaveMatrix(mat):
    out = gm.Transform()
    out.translation = gm.Vector3(*mat[:3,3])
    qw,qx,qy,qz = openravepy.quatFromRotationMatrix(mat[:3,:3])
    out.rotation = gm.Quaternion(qx,qy,qz, qw)
    return out

def ros_joints_to_rave(robot, ros_joint_names, ros_joint_vals):
    assert len(ros_joint_names) == len(ros_joint_vals)
    rave_joint_names = (joint.GetName() for joint in robot.GetJoints())
    rave_joint_inds, rave_joint_vals = [], []
    for i, name in enumerate(rave_joint_names):
        if name in ros_joint_names:
            rave_joint_inds.append(i)
            rave_joint_vals.append(ros_joint_vals[ros_joint_names.index(name)])
    return rave_joint_vals, rave_joint_inds

def rave_env_to_ros(env):
    
    pub = make_planning_scene_pub()
    
    ps=mm.PlanningScene()    
    with open(osp.join(pbc.miscdata_dir,"planning_scene_prototype.yaml"),"r") as fh: 
        d = yaml.load(fh)
    rm.fill_message_args(ps, d)
    
    cos = ps.world.collision_objects
        
    for body in env.GetBodies():
        if body.IsRobot():
            
            
            rstate = ps.robot_state
            
            rave_joint_vals = body.GetDOFValues()
            rave_joint_names = [joint.GetName() for joint in body.GetJoints()]
            
            ros_joint_names = []
            ros_joint_values = []
            for name in ROS_JOINT_NAMES:
                if name in rave_joint_names:
                    i = rave_joint_names.index(name)
                    ros_joint_values.append(rave_joint_vals[i])
                    ros_joint_names.append(name)
                    
            rstate.joint_state.position = ros_joint_values
            rstate.joint_state.name = ros_joint_names

            rstate.multi_dof_joint_state.header.frame_id = 'odom_combined'
            rstate.multi_dof_joint_state.header.stamp = rospy.Time.now()
            rstate.multi_dof_joint_state.joint_names =  ['world_joint']                        
            rstate.multi_dof_joint_state.joint_transforms = [ RosTransformFromRaveMatrix(body.GetTransform())]

        else:
            
            for link in body.GetLinks():
                co = mm.CollisionObject()
                co.operation = co.ADD
                co.id = body.GetName()
                co.header.frame_id = "odom_combined"
                co.header.stamp = rospy.Time.now()
                for geom in link.GetGeometries():
                    trans = RosPoseFromRaveMatrix(body.GetTransform().dot(link.GetTransform().dot(geom.GetTransform())))
                    if geom.GetType() == openravepy.GeometryType.Trimesh:
                        mesh = sm.Mesh()
                        rave_mesh = geom.GetCollisionMesh()
                        for pt in rave_mesh.vertices:
                            mesh.vertices.append(gm.Point(*pt))
                        for tri in rave_mesh.indices:
                            mt = sm.MeshTriangle()
                            mt.vertex_indices = tri
                            mesh.triangles.append(mt)
                        co.meshes.append(mesh)
                        co.mesh_poses.append(trans)
                    else:
                        shape = sm.SolidPrimitive()
                        shape.type = shape.BOX
                        shape.dimensions = geom.GetBoxExtents()*2
                        co.primitives.append(shape)
                        co.primitive_poses.append(trans)
                cos.append(co)

    pub.publish(ps)
    return ps
            
if __name__ == "__main__":
    rospy.init_node("rave_env_to_ros")
    env = openravepy.Environment()
    env.Load("robots/pr2-beta-static.zae")
    env.Load(sys.argv[1])
    robot  = env.GetRobots()[0]
    tf = robot.GetTransform()
    tf[:3,3]=[1,1,0]
    robot.SetTransform(tf)
    rave_env_to_ros(env)
    rospy.sleep(.5)
