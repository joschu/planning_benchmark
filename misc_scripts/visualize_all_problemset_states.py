import openravepy
import yaml
import os; import os.path as osp
import sys
sys.path.insert(1, os.path.join(sys.path[0], '..')); import planning_benchmark_common as pbc

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("problemfile",type=argparse.FileType("r"))
parser.add_argument("--show_midpoints", action="store_true")
args = parser.parse_args()

problemset = yaml.load(args.problemfile)
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
import trajoptpy
viewer = trajoptpy.GetViewer(env)
handles = []
states = [state.values()[0] for state in problemset["states"]]

for i, s in enumerate(states):
    robot.SetActiveDOFValues(s)
    handles.append(viewer.PlotKinBody(robot))
    #handles[-1].SetTransparency(.4)
import numpy as np
away = np.eye(4)
away[2,3] = 100
robot.SetTransform(away)
viewer.Idle()                

