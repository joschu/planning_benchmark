import argparse,sys
parser = argparse.ArgumentParser()
parser.add_argument("problemfile",type=argparse.FileType("r"))
parser.add_argument("-o","--outfile",type=argparse.FileType("w"))


parser.add_argument("planner", choices=["trajopt", "ompl"])

parser.add_argument("--interactive", action="store_true")
parser.add_argument("--multi_init", type=bool, default=True)


parser.add_argument("--ompl_planner_id", default = "", choices = [
    "",
    "SBLkConfigDefault",
    "LBKPIECEkConfigDefault",
    "RRTkConfigDefault",
    "RRTConnectkConfigDefault",
    "ESTkConfigDefault",
    "KPIECEkConfigDefault",
    "BKPIECEkConfigDefault",
    "RRTStarkConfigDefault"])
parser.add_argument("--max_planning_time", type=float, default=10)
args = parser.parse_args()

if args.outfile is None: args.outfile = sys.stdout

import yaml, openravepy, trajoptpy
import os
import os.path as osp
sys.path.insert(1, os.path.join(sys.path[0], '..')); import planning_benchmark_common as pbc
from trajoptpy.check_traj import traj_is_safe, traj_collisions
from planning_benchmark_common.rave_env_to_ros import rave_env_to_ros
import trajoptpy.math_utils as mu
from time import time
import numpy as np
import json
import planning_benchmark_common.func_utils as fu

LEFT_POSTURES = [
    [-0.243379, 0.103374, -1.6, -2.27679, 3.02165, -2.03223, -1.6209], #chest fwd
    [-1.68199, -0.088593, -1.6, -2.08996, 3.04403, -0.41007, -1.39646],# side fwd
    [-0.0428341, -0.489164, -0.6, -1.40856, 2.32152, -0.669566, -2.13699],# face up
    [0.0397607, 1.18538, -0.8, -0.756239, -2.84594, -1.06418, -2.42207]# floor down
]
def mirror_arm_joints(x):
    "mirror image of joints (r->l or l->r)"
    return [-x[0],x[1],-x[2],x[3],-x[4],x[5],-x[6]]
def get_postures(group_name):
    if group_name=="left_arm": return LEFT_POSTURES
    if group_name=="right_arm": return [mirror_arm_joints(posture) for posture in LEFT_POSTURES]
    raise Exception
def animate_traj(traj, robot, pause=True, restore=True):
    """make sure to set active DOFs beforehand"""
    if restore: _saver = openravepy.RobotStateSaver(robot)
    viewer = trajoptpy.GetViewer(robot.GetEnv())
    for (i,dofs) in enumerate(traj):
        print "step %i/%i"%(i+1,len(traj))
        robot.SetActiveDOFValues(dofs)
        if pause: viewer.Idle()
        else: viewer.Step()

@fu.once
def get_ompl_service():
    import rospy
    import moveit_msgs.srv as ms
    svc = rospy.ServiceProxy('plan_kinematic_path', ms.GetMotionPlan)    
    print "waiting for plan_kinematic_path"
    svc.wait_for_service()
    print "ok"
    return svc

    
def setup_ompl(env):        
    import rospy
    rospy.init_node("benchmark_ompl",disable_signals=True)    
    get_ompl_service()
    rave_env_to_ros(env)
    

def trajopt_plan(robot, group_name, active_joint_names, active_affine, end_joints):
    
    start_joints = robot.GetActiveDOFValues()
    
    n_steps = 11
    coll_coeff = 10
    dist_pen = .02
    
    waypoint_step = (n_steps - 1)// 2
    joint_waypoints = [(np.asarray(start_joints) + np.asarray(end_joints))/2]
    
    if group_name in ["right_arm", "left_arm"]:
        if args.multi_init:        
            joint_waypoints.extend(get_postures(group_name))
    
    success = False
    t_start = time()
    t_verify = 0
    t_opt = 0
    
    for (i_init,waypoint) in enumerate(joint_waypoints):
        d = {
            "basic_info" : {
                "n_steps" : n_steps,
                "manip" : "active",
                "start_fixed" : True
            },
            "costs" : [
            {
                "type" : "joint_vel",
                "params": {"coeffs" : [1]}
            },            
            {
                "type" : "collision",
                "params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen], "continuous":True}
            },
            {
                "type" : "collision",
                "params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen], "continuous":False}
            }
            ],
            "constraints" : [
                {"type" : "joint", "params" : {"vals" : end_joints}}
            ],
            "init_info" : {
                "type" : "given_traj"
            }
        }
        
        if args.multi_init:        
            # print "trying with midpoint", waypoint
            inittraj = np.empty((n_steps, robot.GetActiveDOF()))
            inittraj[:waypoint_step+1] = mu.linspace2d(start_joints, waypoint, waypoint_step+1)
            inittraj[waypoint_step:] = mu.linspace2d(waypoint, end_joints, n_steps - waypoint_step)
        else:
            inittraj = mu.linspace2d(start_joints, end_joints, n_steps)                            
        d["init_info"]["data"] = [row.tolist() for row in inittraj]
                        
        s = json.dumps(d)
        prob = trajoptpy.ConstructProblem(s, robot.GetEnv())
        
        t_start_opt = time()
        result = trajoptpy.OptimizeProblem(prob)
        t_opt += time() - t_start_opt
                
        traj = result.GetTraj()
        
        prob.SetRobotActiveDOFs()
        t_start_verify = time()
        is_safe = traj_is_safe(traj, robot)
        t_verify += time() - t_start_verify
        
        if not is_safe:                       
            print "optimal trajectory has a collision. trying a new initialization"
        else:
            print "planning successful after %s initialization"%(i_init+1)
            success = True
            break
    if success == False: 
        print "fail"
    t_total = time() - t_start
    return success, t_total, [row.tolist() for row in traj]

def ompl_plan(robot, group_name, active_joint_names, active_affine, target_dof_values):
    
    import moveit_msgs.msg as mm
    from planning_benchmark_common.rave_env_to_ros import rave_env_to_ros
    import rospy
    ps = rave_env_to_ros(robot.GetEnv())
    msg = mm.MotionPlanRequest()
    msg.group_name = group_name
    msg.planner_id = args.ompl_planner_id
    msg.allowed_planning_time = 10
    c = mm.Constraints()
    joints = robot.GetJoints()
    #joint_inds = robot.GetManipulator(manip_name).GetArmIndices()

    if active_affine == 0:
        base_joint_names = []
    elif active_affine == 11:
        base_joint_names = ["world_joint/x", "world_joint/y", "world_joint/theta"]
    else:
        raise Exception
    for (name, val) in zip(active_joint_names+base_joint_names, target_dof_values):
        c.joint_constraints.append(mm.JointConstraint(joint_name=name, position = val,weight=1,tolerance_above=.0001, tolerance_below=.0001))
                                   
    jc = mm.JointConstraint()

    msg.start_state = ps.robot_state
    msg.goal_constraints = [c]
    #msg.allowed_planning_time = rospy.Duration(args.max_planning_time)
    svc = get_ompl_service()
    try:
        t_start = time()
        svc_response = svc.call(msg)
        response = svc_response.motion_plan_response
        print "planner succeeded"
        traj = [list(point.positions) for point in response.trajectory.joint_trajectory.points]
        #is_safe = traj_is_safe(traj, robot)
        #if not is_safe: 
            #print "Openrave thinks it's not safe!!! animating"
            #col_times = traj_collisions(traj, robot)
            #print "col times:",col_times
            #n=100
            #traj_up = mu.interp2d(np.linspace(0,1,n), np.linspace(0,1,len(traj)), traj)            
            #animate_traj(traj_up, robot)
        return True, response.planning_time, traj
    except rospy.service.ServiceException:
        print "planner failed"
        return False, np.nan, []

def main():

    problemset = yaml.load(args.problemfile)
    env = openravepy.Environment()
    env.StopSimulation()
    
    if args.planner == "trajopt":
        if args.interactive: trajoptpy.SetInteractive(True)      
        plan_func = trajopt_plan
    elif args.planner == "ompl":
        setup_ompl(env)
        plan_func = ompl_plan
        
    
    env.Load(osp.join(pbc.envfile_dir,problemset["env_file"]))
    robot2file = {
        "pr2":"robots/pr2-beta-static.zae"
    }
    env.Load(robot2file[problemset["robot_name"]])
    robot = env.GetRobots()[0]
    
    robot.SetTransform(openravepy.matrixFromPose(problemset["default_base_pose"]))
    rave_joint_names = [joint.GetName() for joint in robot.GetJoints()]
    rave_inds, rave_values = [],[]
    for (name,val) in zip(problemset["joint_names"], problemset["default_joint_values"]):
        if name in rave_joint_names:
            i = rave_joint_names.index(name)
            rave_inds.append(i)
            rave_values.append(val)
                        
    robot.SetDOFValues(rave_values, rave_inds)
    active_joint_inds = [rave_joint_names.index(name) for name in problemset["active_joints"]]
        
    results = []

    robot.SetActiveDOFs(active_joint_inds, problemset["active_affine"])

    problem_joints = []
    for prob in problemset["problems"]:
        # special "all_pairs" problem type
        if "all_pairs" in prob:
          states = prob["all_pairs"]["active_dof_values"]
          for i in range(len(states)):
            for j in range(i+1, len(states)):
              problem_joints.append((states[i], states[j]))
          continue

        if "active_dof_values" not in prob["start"] or "active_dof_values" not in prob["goal"]:
          raise NotImplementedError
        problem_joints.append((prob["start"]["active_dof_values"], prob["goal"]["active_dof_values"]))

    for start, goal in problem_joints:
        robot.SetActiveDOFValues(start)
        success, t_total, traj = plan_func(robot, problemset["group_name"], problemset["active_joints"], problemset["active_affine"], goal)
        results.append(
            {"success": success,
             "time": t_total,
             #"traj": traj
             })

    print "success rate: %i/%i"%(np.sum(result["success"] for result in results), len(results))
    yaml.dump(results, args.outfile)
        
    
             

if __name__ == "__main__":
    main()
