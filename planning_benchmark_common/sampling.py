import numpy as np
import openravepy

def calc_world_bounds(env, ignore_names):
    inf = float('inf')
    lo, hi = np.array([inf, inf, inf]), -np.array([inf, inf, inf])
    for body in env.GetBodies():
        if body.GetName() in ignore_names: continue
        for link in body.GetLinks():
            for geom in link.GetGeometries():
                trans = body.GetTransform()#.dot(link.GetTransform().dot(geom.GetTransform()))
                aabb = geom.ComputeAABB(trans)
                lo = np.minimum(lo, aabb.pos() - aabb.extents())
                hi = np.maximum(hi, aabb.pos() + aabb.extents())
    return lo, hi

def sample_base_positions(robot, num=5, tucked=True):
    if tucked:
        joints = np.array([ 0.0976,  1.0257,  1.1795,  1.554 , -2.12  , -0.2533, -1.2496,
            0.7296, -0.5387,  1.1035, -1.7892, -1.8962,  0.1081, -1.4375,
           -1.3529,  1.705 ,  1.691 ,  0.7039])
    else:
        joints = np.asarray(robot.GetActiveDOFValues())
    env = robot.GetEnv()
    min_xyt, max_xyt = calc_world_bounds(env, ignore_names=[robot.GetName()])
    min_xyt[2], max_xyt[2] = 0, np.pi
    print 'world bounds', min_xyt, max_xyt
    out = []
    with robot:
        while len(out) < num:
            x, y, theta = np.random.rand(3)*(max_xyt-min_xyt) + min_xyt
            joints[-3:] = [x, y, theta]
            robot.SetActiveDOFValues(joints)
            if not env.CheckCollision(robot) and not robot.CheckSelfCollision():
                out.append(joints.copy())
    return np.asarray(out)

def sample_all(robot, num=5):
    env = robot.GetEnv()
    lim_lo, lim_hi = robot.GetActiveDOFLimits()
    for i in range(len(lim_lo)):
        if lim_lo[i] < -10: lim_lo[i] = 0
    for i in range(len(lim_hi)):
        if lim_hi[i] > 10: lim_hi[i] = 2*np.pi
    min_xyt, max_xyt = calc_world_bounds(env, ignore_names=[robot.GetName()])
    min_xyt[2], max_xyt[2] = 0, np.pi
    out = []
    with robot:
        while len(out) < num:
            joints = np.random.rand(len(lim_lo))*(lim_hi-lim_lo) + lim_lo
            joints[-3:] = np.random.rand(3)*(max_xyt-min_xyt) + min_xyt
            robot.SetActiveDOFValues(joints)
            if not env.CheckCollision(robot) and not robot.CheckSelfCollision():
                out.append(joints.copy())
    return np.asarray(out)

def main():
    import yaml
    import os; import os.path as osp
    import sys
    sys.path.insert(1, os.path.join(sys.path[0], '..')); import planning_benchmark_common as pbc

    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("problemfile", type=argparse.FileType("r"))
    parser.add_argument("--base_only", action="store_true")
    args = parser.parse_args()

    problemset = yaml.load(args.problemfile)
    assert problemset["active_affine"] == 11

    env = openravepy.Environment()
    env.StopSimulation()

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
    robot.SetActiveDOFs(active_joint_inds, problemset["active_affine"])

    env.SetViewer('qtcoin')
    env.UpdatePublishedBodies()

    if args.base_only:
      poses = sample_base_positions(robot, num=100)
    else:
      poses = sample_all(robot, num=100)
    while True:
      for i, p in enumerate(poses):
          robot.SetActiveDOFValues(poses[i])
          env.UpdatePublishedBodies()
          raw_input('showing %d/%d: joints = %s' % (i+1, len(poses), ', '.join(str(v) for v in poses[i])))
      print '=================='

if __name__ == '__main__':
    main()
