import argparse,sys
parser = argparse.ArgumentParser()
parser.add_argument("problemfile",type=argparse.FileType("r"))
parser.add_argument("-o","--outfile",type=argparse.FileType("w"))
parser.add_argument("--record_failed_problems", type=argparse.FileType("w"))
parser.add_argument("--problems", type=argparse.FileType("r"), help="ignore the problems in problemfile and use these only (good for output from --record_failed_problems)")

parser.add_argument("planner", choices=["trajopt", "ompl", "chomp"])

parser.add_argument("--animate_all", action="store_true", help="animate solutions to every problem after solving")
parser.add_argument("--multi_init", action="store_true", help="use multiple initializations, for trajopt and chomp")
parser.add_argument("--max_planning_time", type=float, default=10, help="max planning time for chomp and ompl")

# trajopt options
parser.add_argument("--interactive", action="store_true")
parser.add_argument("--trajopt_dist_pen", type=float, default=.02)
parser.add_argument("--trajopt_no_selfcoll_first", action="store_true")

# ompl options
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

# chomp options
parser.add_argument("--chomp_argstr", default="comp")

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
    #[-1.19257663,  0.73967304, -1.6, -1.17798376,  1.23031085, -0.38983174,  2.85648962],
    #[0.0024,  0.6091, -1.4913, -1.9477,  2.4689, -1.9924, -1.685], #tuck
    [-0.243379, 0.103374, -1.6, -2.27679, 3.02165, -2.03223, -1.6209], #chest fwd
    [-1.68199, -0.088593, -1.6, -2.08996, 3.04403, -0.41007, -1.39646],# side fwd
    [-0.0428341, -0.489164, -0.6, -1.40856, 2.32152, -0.669566, -2.13699],# face up
    [0.0397607, 1.18538, -0.8, -0.756239, -2.84594, -1.06418, -2.42207]# floor down
]

KITCHEN_WAYPOINTS = [
      [ 0.    ,  0.0436,  0.8844,  1.493 , -0.2914,  2.6037, -0.4586,
        0.6602,  0.0155,  0.8421, -2.0777, -0.544 , -2.5683, -0.4762,
       -1.5533,  1.4904, -0.4271,  1.8619],
      [ 0.    ,  0.0436,  0.8844,  1.493 , -0.2914,  2.6037, -0.4586,
        0.6602,  0.0155,  0.8421, -2.0777, -0.544 , -2.5683, -0.4762,
       -1.5533,  2.1866,  2.4017, -2.285 ]
]

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'

    def disable(self):
        self.HEADER = ''
        self.OKBLUE = ''
        self.OKGREEN = ''
        self.WARNING = ''
        self.FAIL = ''
        self.ENDC = ''


def mirror_arm_joints(x):
    "mirror image of joints (r->l or l->r)"
    return [-x[0],x[1],-x[2],x[3],-x[4],x[5],-x[6]]
def get_postures(group_name):
    if group_name=="left_arm": return LEFT_POSTURES
    if group_name=="right_arm": return [mirror_arm_joints(posture) for posture in LEFT_POSTURES]
    if group_name=="whole_body": return KITCHEN_WAYPOINTS
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
def traj_to_array(traj):
    return traj.GetWaypoints(0, traj.GetNumWaypoints()).reshape((traj.GetNumWaypoints(), -1))
def array_to_traj(robot, a, dt=1):
    #spec = robot.GetActiveConfigurationSpecification()
    spec = openravepy.ConfigurationSpecification()
    name = "joint_values %s %s" % (robot.GetName(), ' '.join(map(str, robot.GetActiveDOFIndices())))
    spec.AddGroup(name, robot.GetActiveDOF(), interpolation="linear")
    spec.AddDeltaTimeGroup()
    traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), '')
    traj.Init(spec)
    for i, joints in enumerate(a):
      pt = np.zeros(spec.GetDOF())
      spec.InsertJointValues(pt, joints, robot, robot.GetActiveDOFIndices(), 0)
      spec.InsertDeltaTime(pt, 0 if i == 0 else dt)
      traj.Insert(i, pt)
    return traj
def traj_no_self_collisions(traj, robot):
    with robot:
        for joints in traj:
            robot.SetActiveDOFValues(joints)
            if robot.CheckSelfCollision():
                return False
    return True
def hash_env(env):
    # hash based on non-robot object transforms and geometries
    import hashlib
    return hashlib.sha1(','.join(str(body.GetTransform()) + body.GetKinematicsGeometryHash() for body in env.GetBodies() if not body.IsRobot())).hexdigest()

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

@fu.once
def get_chomp_module(env):
    from orcdchomp import orcdchomp
    CHOMP_MODULE_PATH = '/home/jonathan/build/chomp/liborcdchomp.so'
    openravepy.RaveLoadPlugin(CHOMP_MODULE_PATH)
    m_chomp = openravepy.RaveCreateModule(env, 'orcdchomp')
    env.Add(m_chomp, True, 'blah_load_string')
    orcdchomp.bind(m_chomp)
    return m_chomp

def setup_chomp(env):
    get_chomp_module(env)


def gen_init_trajs(robot, group_name, n_steps, start_joints, end_joints):
    waypoint_step = (n_steps - 1)// 2
    joint_waypoints = [(np.asarray(start_joints) + np.asarray(end_joints))/2]
    if args.multi_init:
        if group_name in ["right_arm", "left_arm", "whole_body"]:
            joint_waypoints.extend(get_postures(group_name))
    trajs = []
    for i, waypoint in enumerate(joint_waypoints):
        if i == 0:
            inittraj = mu.linspace2d(start_joints, end_joints, n_steps)
        else:
            # print "trying with midpoint", waypoint
            inittraj = np.empty((n_steps, robot.GetActiveDOF()))
            inittraj[:waypoint_step+1] = mu.linspace2d(start_joints, waypoint, waypoint_step+1)
            inittraj[waypoint_step:] = mu.linspace2d(waypoint, end_joints, n_steps - waypoint_step)
        trajs.append(inittraj)
    return trajs

def make_trajopt_request(n_steps, coll_coeff, dist_pen, end_joints, inittraj, use_discrete_collision):
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
        ],
        "constraints" : [
            {"type" : "joint", "params" : {"vals" : end_joints}}
        ],
        "init_info" : {
            "type" : "given_traj",
            "data" : [row.tolist() for row in inittraj]
        }
    }
    if use_discrete_collision:
        d["costs"].append({
            "name": "discrete_collision",
            "type" : "collision",
            "params" : {"coeffs" : [coll_coeff],"dist_pen" : [dist_pen], "continuous":False}
        })
    return json.dumps(d)


def trajopt_plan(robot, group_name, active_joint_names, active_affine, end_joints):
    start_joints = robot.GetActiveDOFValues()

    n_steps = 11
    coll_coeff = 10
    dist_pen = args.trajopt_dist_pen

    def single_trial(inittraj, use_discrete_collision):
        s = make_trajopt_request(n_steps, coll_coeff, dist_pen, end_joints, inittraj, use_discrete_collision)
        prob = trajoptpy.ConstructProblem(s, robot.GetEnv())
        result = trajoptpy.OptimizeProblem(prob)
        traj = result.GetTraj()
        prob.SetRobotActiveDOFs()
        return traj, traj_is_safe(traj, robot) #and (use_discrete_collision or traj_no_self_collisions(traj, robot))

    init_trajs = gen_init_trajs(robot, group_name, n_steps, start_joints, end_joints)
    success = False
    msg = ''
    t_start = time()
    for (i_init,inittraj) in enumerate(init_trajs):
        if args.trajopt_no_selfcoll_first:
            traj, is_safe = single_trial(inittraj, False)
            if is_safe:
                msg = "planning successful after %s initialization (no self collisions)"%(i_init+1)
                success = True
                break
            traj, is_safe = single_trial(inittraj, True)
            if is_safe:
                msg = "planning successful after %s initialization (with self collisions)"%(i_init+1)
                success = True
                break
        else:
            traj, is_safe = single_trial(inittraj, True)
            if is_safe:
                msg = "planning successful after %s initialization"%(i_init+1)
                success = True
                break
    t_total = time() - t_start

    return success, t_total, [row.tolist() for row in traj], msg

def ompl_plan(robot, group_name, active_joint_names, active_affine, target_dof_values):
    
    import moveit_msgs.msg as mm
    from planning_benchmark_common.rave_env_to_ros import rave_env_to_ros
    import rospy
    ps = rave_env_to_ros(robot.GetEnv())
    msg = mm.MotionPlanRequest()
    msg.group_name = group_name
    msg.planner_id = args.ompl_planner_id
    msg.allowed_planning_time = args.max_planning_time
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
    #req.allowed_planning_time = rospy.Duration(args.max_planning_time)
    svc = get_ompl_service()
    try:
        t_start = time()
        svc_response = svc.call(msg)
        response = svc_response.motion_plan_response
        # success
        traj = [list(point.positions) for point in response.trajectory.joint_trajectory.points]
        #is_safe = traj_is_safe(traj, robot)
        #if not is_safe: 
            #print "Openrave thinks it's not safe!!! animating"
            #col_times = traj_collisions(traj, robot)
            #print "col times:",col_times
            #n=100
            #traj_up = mu.interp2d(np.linspace(0,1,n), np.linspace(0,1,len(traj)), traj)            
            #animate_traj(traj_up, robot)
        return True, response.planning_time, traj, ''
    except rospy.service.ServiceException:
        # failure
        return False, np.nan, [], ''

def chomp_plan(robot, group_name, active_joint_names, active_affine, target_dof_values):
    datadir = 'chomp_data'
    n_points = 11

    if active_affine != 0:
        raise NotImplementedError("chomp probably doesn't work with affine dofs")
    openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Warn)
    m_chomp = get_chomp_module(robot.GetEnv())

    env_hash = hash_env(robot.GetEnv())
    start_joints = robot.GetActiveDOFValues()

    # load distance field
    j1idxs = [m.GetArmIndices()[0] for m in robot.GetManipulators()]
    for link in robot.GetLinks():
        for j1idx in j1idxs:
            if robot.DoesAffect(j1idx, link.GetIndex()):
                link.Enable(False)
    try:
        m_chomp.computedistancefield(kinbody=robot, aabb_padding=1.0,
          cache_filename='%s/chomp-sdf-%s.dat' % (datadir, env_hash))
    except Exception, e:
        print 'Exception in computedistancefield:', repr(e)
    for link in robot.GetLinks():
        link.Enable(True)

    # build chomp command
    if args.chomp_argstr == 'comp':
        kwargs = dict(robot=robot, n_iter=1000000,
          max_time=args.max_planning_time,
          lambda_=100.0, no_collision_exception=True,
          no_collision_check=True, n_points=n_points)
    elif args.chomp_argstr.startswith('hmc-seed'):
        seed = int(args.chomp_argstr[len('hmc-seed'):])
        print 'Using seed:', seed
        kwargs = dict(robot=robot, n_iter=10000,
          max_time=args.max_planning_time,
          lambda_=100.0, no_collision_exception=True,
          use_momentum=True, use_hmc=True, seed=seed,
          no_collision_check=True, n_points=n_points)
    else:
        raise RuntimeError('must be chomp-seedXXXX')

    # run chomp
    msg = ''
    t_start = time()
    is_safe = False
    if args.multi_init:
        init_trajs = gen_init_trajs(robot, group_name, n_points, start_joints, target_dof_values)
        for i_init, inittraj in enumerate(init_trajs):
            t = kwargs["starttraj"] = array_to_traj(robot, inittraj)
            traj = traj_to_array(m_chomp.runchomp(**kwargs))
            if traj_is_safe(traj, robot):
                is_safe = True
                msg = "planning successful after %s initialization"%(i_init+1)
                break
    else:
        kwargs["adofgoal"] = target_dof_values
        traj = traj_to_array(m_chomp.runchomp(**kwargs))
        is_safe = traj_is_safe(traj, robot)
    t_total = time() - t_start


    return is_safe, t_total, traj, msg


def init_env(problemset):
    env = openravepy.Environment()
    env.StopSimulation()
    
    robot2file = {
        "pr2":"robots/pr2-beta-static.zae"
    }

    if args.planner == "trajopt":
        if args.interactive: trajoptpy.SetInteractive(True)      
        plan_func = trajopt_plan
    elif args.planner == "ompl":
        setup_ompl(env)
        plan_func = ompl_plan
    elif args.planner == "chomp":
        setup_chomp(env)
        robot2file["pr2"] = osp.join(pbc.envfile_dir, "pr2_with_spheres.robot.xml") # chomp needs a robot with spheres
        plan_func = chomp_plan

    env.Load(osp.join(pbc.envfile_dir,problemset["env_file"]))
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
    robot.SetActiveDOFs(active_joint_inds, problemset["active_affine"])

    return env, robot, plan_func

def main():
    problemset = yaml.load(args.problemfile)
    env, robot, plan_func = init_env(problemset)

    # enumerate the problems
    problem_joints = []
    problems = problemset["problems"] if args.problems is None else yaml.load(args.problems)
    for prob in problems:
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


    # solve the problems
    results = []
    failed = []
    for i, (start, goal) in enumerate(problem_joints):
        robot.SetActiveDOFValues(start)
        success, t_total, traj, msg = plan_func(robot, problemset["group_name"], problemset["active_joints"], problemset["active_affine"], goal)
        print '%s[%d/%d] %s%s' % (bcolors.OKGREEN if success else bcolors.FAIL, i+1, len(problem_joints), ('success' if success else 'FAILURE') + (': ' + msg if msg else ''), bcolors.ENDC)
        res = {"success": success, "time": t_total}
        if args.outfile is not sys.stdout:
            res["traj"] = traj
        results.append(res)

        if not success:
            failed.append({"start": {"active_dof_values":list(start)}, "goal": {"active_dof_values":list(goal)}})

        if args.animate_all:
            animate_traj(traj, robot)

    print "success rate: %i/%i"%(np.sum(result["success"] for result in results), len(results))
    times = np.asarray([float(result["time"]) for result in results])
    print "average time: %f" % np.mean(times[np.isfinite(times)])

    if args.record_failed_problems is not None:
        yaml.dump(failed, args.record_failed_problems)
        print 'Recorded %d failures' % len(failed)

    yaml.dump(results, args.outfile)


if __name__ == "__main__":
    main()
