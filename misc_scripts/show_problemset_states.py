import openravepy
import yaml
import os; import os.path as osp
import sys
sys.path.insert(1, os.path.join(sys.path[0], '..')); import planning_benchmark_common as pbc

import argparse
parser = argparse.ArgumentParser()
parser.add_argument("problemfile",type=argparse.FileType("r"))
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

env.SetViewer('qtcoin')
while True:
  for prob in problemset["problems"]:
    # special "all_pairs" problem type
    if "all_pairs" in prob:
      states = prob["all_pairs"]["active_dof_values"]
      for i, s in enumerate(states):
        robot.SetActiveDOFValues(s)
        env.UpdatePublishedBodies()
        raw_input('showing state %d/%d' % (i+1, len(states)))
    else:
      robot.SetActiveDOFValues(prob["start"]["active_dof_values"])
      env.UpdatePublishedBodies()
      raw_input('showing start state: %s' % str(prob["start"]["active_dof_values"]))
      robot.SetActiveDOFValues(prob["goal"]["active_dof_values"])
      env.UpdatePublishedBodies()
      raw_input('showing goal state: %s' % str(prob["goal"]["active_dof_values"]))
    print '===================='
  raw_input('showed all states, now looping...')
