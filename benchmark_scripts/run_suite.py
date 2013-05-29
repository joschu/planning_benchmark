import argparse
import os
import os.path as osp
import subprocess
import tempfile
import yaml

parser = argparse.ArgumentParser()
parser.add_argument("suitefile", type=argparse.FileType("r"))
args = parser.parse_args()


RUN_SCRIPT = 'benchmark_scripts/run_problemset.py'
PROBLEM_SET_DIR = 'problem_sets'

def run_suite(suite):
    results = {}
    for ps_id, pset_file in enumerate(suite["problem_sets"]):
        for cfg_id, cfg in enumerate(suite["configurations"]):
            # execute run_problemset.py and read its output
            print pset_file, cfg
            fd, out_file_path = tempfile.mkstemp(); os.close(fd)
            cmd = ["python", RUN_SCRIPT, osp.join(PROBLEM_SET_DIR, pset_file), cfg["planner"], "--outfile=" + out_file_path]
            if "args" in cfg:
                cmd += cfg["args"]
            print ">>> Running %s on problem set %s: %s" % (cfg["name"], pset_file, ' '.join(cmd))
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

            #cfg2stats[cfg_id]["succeeded"] = sum(run["success"] for run in res)
            #cfg2stats[cfg_id]["total"] = len(res)
            cfg2stats[cfg_id]["success_frac"] = float(sum(run["success"] for run in res)) / len(res)
            cfg2stats[cfg_id]["avg_time"] = sum(float(run["time"]) for run in res) / len(res)

        print ',' + ','.join(suite["configurations"][cfg_id]["name"] for cfg_id in cfg2stats)
        for field in cfg2stats[0]:
            print field + ',' + ','.join(str(cfg2stats[cfg_id][field]) for cfg_id in cfg2stats)
        print

def main():
    suite = yaml.load(args.suitefile)
    results = run_suite(suite)
    print '\n===== Summary ========================\n'
    display_summary(suite, results)

if __name__ == '__main__':
    main()
