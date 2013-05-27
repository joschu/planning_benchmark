import argparse
parser = argparse.ArgumentParser()
parser.add_argument("trajfile", help="json file with a bunch of trajectories",type=argparse.FileType('r'))
parser.add_argument("problemfile", help="json file with problems",type=argparse.FileType('r'))
parser.add_argument("outfile", help="json file with results",type=argparse.FileType('w'))
args = parser.parse_args()

import json

traj_infos = json.load(args.trajfile)
prob_infos = json.load(args.problemfile)

id2prob = dict([(prob_info["id"], prob_info) for prob_info in prob_infos])

class Globals:
    env_file = None
    robot = None

def verify_traj(traj, prob_info):
    """
    traj: 2d array
    prob_info: dictionary of problem info
    """    
    
    setup_env_for_problem(Globals.env, Globals.robot, prob_info)
    
    return traj_is_safe(traj, Globals.robot)
    
# Sort the problems by robot and env and collect the ones with the same robot and env
# Then     
    
json.dump(result_info, outfile)