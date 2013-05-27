import argparse
parser = argparse.ArgumentParser()
parser.add_argument("infile", type = argparse.FileType("r"))
parser.add_argument("outfile", type = argparse.FileType("w"))
args = parser.parse_args()


import itertools as it
from copy import copy
import sys, random, yaml

input_info = yaml.load(args.infile)
problemset_info = {}
problemset_info["env_file"] = input_info["env_file"]
problemset_info["robot_name"] = input_info["robot_name"]
ACTIVE_JOINTS = problemset_info["active_joints"] = input_info["active_joints"]
ACTIVE_AFFINE = problemset_info["active_affine"] = input_info["active_affine"]
JOINT_NAMES = problemset_info["joint_names"] = input_info["joint_names"]
STATES = input_info["states"]
DEFAULT_JOINT_VALUES = input_info["default_joint_values"]
INITIAL_POSE = input_info["default_base_pose"]

problemset_info2 = {}
prob_infos = problemset_info2["problems"] = []
for (start, goal) in it.combinations(STATES, 2):
    prob_info = {}


    prob_info["start"] = {}
    initial_values = copy(DEFAULT_JOINT_VALUES)
    for (jind,val) in zip(ACTIVE_JOINTS, start):
        initial_values[jind] = val        
    prob_info["start"]["dof_values"] = initial_values    
    prob_info["start"]["base_pose"] = INITIAL_POSE
    
    prob_info["goal"] = {}
    prob_info["goal"]["type"] = "joint"
    prob_info["goal"]["dof_values"] = goal
    
    prob_info["id"] = random.randint(0,sys.maxint)
    
    prob_infos.append(prob_info)

Dumper = yaml.SafeDumper    
Dumper.ignore_aliases = lambda self, data: True

yaml.dump(problemset_info, args.outfile, Dumper = Dumper)
yaml.dump(problemset_info2, args.outfile, Dumper = Dumper)