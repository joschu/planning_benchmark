import argparse
import cPickle
import numpy as np
import os
import os.path as osp
import subprocess
import tempfile
import yaml

parser = argparse.ArgumentParser()
parser.add_argument("suitefile", type=argparse.FileType("r"), nargs="?")
parser.add_argument("-o","--outfile",type=argparse.FileType("w"))
parser.add_argument("--summarize", type=argparse.FileType("r"), help="take the input to be the output of a previous run, and display the summmary only")
args = parser.parse_args()


RUN_SCRIPT = 'benchmark_scripts/run_problemset.py'
PROBLEM_SET_DIR = 'problem_sets'

def filter_finite(a):
    return a[np.isfinite(a)]

def unwind(traj):
    #print traj
    if len(traj[0]) == 7 or len(traj[0]) == 15: inds = [2, 4, 6]
    elif len(traj[0]) == 15: traj = traj[:7]; inds = [2, 4, 6]
    elif len(traj[0]) == 18: inds = [1+2, 1+4, 1+6, 1+2+7, 1+4+7, 1+6+7, 17]
    else: raise NotImplementedError('dont know how to deal with %d dof' % len(traj[0]))
    for i in inds:
        traj[:,i] = np.unwrap(traj[:,i])
    return traj

def traj_len(traj):
    traj = traj.copy()
    traj = unwind(traj)
    return np.sqrt(((traj[1:,:] - traj[:-1,:])**2).sum(axis=1)).sum()

def run_suite(suite):
    results = {}
    i, num_runs = 0, len(suite["problem_sets"]) * len(suite["configurations"])
    for ps_id, pset_file in enumerate(suite["problem_sets"]):
        for cfg_id, cfg in enumerate(suite["configurations"]):
            # execute run_problemset.py and read its output
            i += 1
            fd, out_file_path = tempfile.mkstemp(); os.close(fd)
            cmd = ["python", RUN_SCRIPT, osp.join(PROBLEM_SET_DIR, pset_file), cfg["planner"], "--outfile=" + out_file_path]
            if "args" in cfg:
                cmd += cfg["args"]
            print ">>> [%d/%d] Running %s on problem set %s: %s" % (i, num_runs, cfg["name"], pset_file, ' '.join(cmd))
            subprocess.call(cmd)
            with open(out_file_path, "r") as f:
                output = yaml.load(f)
            os.unlink(out_file_path)

            results[(ps_id, cfg_id)] = output

    return results

def display_summary(suite, results):
    from collections import defaultdict
    for ps_id, pset_file in enumerate(suite["problem_sets"]):
        cfg2stats = defaultdict(dict)
        first = True
        for cfg_id, cfg in enumerate(suite["configurations"]):
            key = (ps_id, cfg_id)
            if key not in results: continue
            res = results[key]

            if first:
                print 'Problem set: %s, num problems: %d' % (pset_file, len(res))
                first = False

            cfg2stats[cfg_id]["success_frac"] = float(sum(run["success"] for run in res)) / len(res)

            times = np.asarray([float(run["time"]) for run in res])
            cfg2stats[cfg_id]["avg_time"] = np.mean(times[np.isfinite(times)])

            traj_lens = np.asarray([traj_len(np.asarray(run["traj"])) for run in res if len(run["traj"]) != 0])
            cfg2stats[cfg_id]["avg_abs_path_len"] = np.mean(traj_lens)

            all_traj_lens = np.asarray([traj_len(np.asarray(run["traj"])) if len(run["traj"]) != 0 and run["success"] else float('inf') for run in res])
            cfg2stats[cfg_id]["_all_abs_path_lens"] = all_traj_lens

        # for each pset, iterate through all cfgs, and normalize across corresponding path lens
        abs_lens = []
        for cfg_id, cfg in enumerate(suite["configurations"]):
            abs_lens.append(cfg2stats[cfg_id]["_all_abs_path_lens"])
            assert len(abs_lens) < 2 or len(abs_lens[-1]) == len(abs_lens[-2])
        abs_lens = np.asarray(abs_lens) # planners x problems
        normed = abs_lens / abs_lens.min(axis=0)[None,:]
        assert len(normed) == len(suite["configurations"])
        avg_normed = []
        for row in normed:
            avg_normed.append(np.mean(filter_finite(row)))
        for cfg_id, cfg in enumerate(suite["configurations"]):
            cfg2stats[cfg_id]["avg_normed_path_len"] = avg_normed[cfg_id]

        # print csv summary for the pset
        print ',' + ','.join(suite["configurations"][cfg_id]["name"] for cfg_id in cfg2stats)
        for field in cfg2stats[0]:
            if field[0] == "_": continue
            print field + ',' + ','.join(str(cfg2stats[cfg_id][field]) for cfg_id in cfg2stats)
        print

def main():
    if args.summarize is None:
        suite = yaml.load(args.suitefile)
        results = run_suite(suite)
    else:
        results = cPickle.load(args.summarize)
        suite = results["suite"] if "suite" in results else yaml.load(args.suitefile)

    print '\n===== Summary ========================\n'
    display_summary(suite, results)
    if args.outfile is not None and args.summarize is None:
        results["suite"] = suite
        cPickle.dump(results, args.outfile)

if __name__ == '__main__':
    main()
