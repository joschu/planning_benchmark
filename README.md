Motion Planning Benchmarks
==========================
for the RSS 2013 submission

**Finding Locally Optimal, Collision-Free Trajectories with Sequential Convex Optimization  
John Schulman, Jonathan Ho, Alex Lee, Ibrahim Awwal, Henry Bradlow and Pieter Abbeel**

This is a collection of scripts that runs planners--trajopt, OMPL/MoveIt, and CHOMP--on various
testing scenes in various configurations. All testing scenes use the PR2.

The benchmarks are structured as a suite of problem sets. Each problem set is defined as a
collection of DOF of the robot to plan, an environment description, and a set of start/goal
joint pairs, each of which constitutes a single planning problem. A suite of problem sets
is a set of configurations of planners (planner type and arguments) and a list of problem sets--
running a suite will run each of the planners on all of the configurations.

Setup
=====
The benchmarking code depends on the following packages:

- OpenRAVE, at least version 0.8 (see http://openrave.org/)

- Various Python packages: NumPy, YAML, JSON

Before running the benchmarks, you must install the planners:

- trajopt, the planner described in this paper (see http://rll.berkeley.edu/trajopt/)

- ROS, MoveIt (see http://moveit.ros.org/)

    sudo apt-get install ros-groovy-moveit-full ros-groovy-moveit-full-pr2

- CHOMP

  Note that the CHOMP implementation we use is not publicly available. If you do have access to it,
  however, then make sure that the `orcdchomp` Python package is available in your `$PYTHONPATH`.

Note that if any of these planners is not available, then it's still possible to run
the benchmarks excluding the unavailable ones.

Trajopt and CHOMP require no pre-benchmarking setup, but MoveIt/OMPL benchmarks need the
MoveIt ROS services to be running beforehand. To do this:

    roscore &
    roslaunch benchmark_scripts/moveit_without_retimer.launch

(note that we run MoveIt with the trajectory retimer disabled due to stability issues)

We also recommend setting the following environment variable:

    export TRAJOPT_LOG_THRESH=WARN

to suppress excessive logging output from trajopt.

Running a testing suite
=======================
We provide the following suite files:

- `problem_sets/suite_rightarm.yaml`: arm planning problems for trajopt, OMPL, and CHOMP
- `problem_sets/suite_fullbody.yaml`: full-body planning problems for trajopt and OMPL

(If you want to disable planner configurations for any reason, for example due to the unavailability of CHOMP,
simply remove or comment out the appropriate entries under the `configurations` item in the YAML file.)

To run a suite `SUITE_FILE.yaml` and save the planning results to `RESULTS_FILE.pkl`, execute the following command
from the root directory of this package:

    python benchmark_scripts/run_suite.py SUITE_FILE.yaml -o RESULTS_FILE.pkl

To analyze results in `RESULTS_FILE.pkl`, run:

    python benchmark_scripts/run_suite.py --summarize=RESULTS_FILE.pkl

An example analysis output will look like the following:

    Problem set: bookshelves_rightarm.yaml, num problems: 45
    ,trajopt,trajopt-multi_init,ompl-RRTConnect,ompl-LBKPIECE,chomp-hmc,chomp-hmc-multi_init
    success_frac,0.888888888889,1.0,1.0,0.866666666667,0.911111111111,0.955555555556
    avg_abs_path_len,3.95396221512,4.0252221864,5.06025580906,5.12745801789,6.46849317707,6.43896290886
    avg_normed_path_len,1.15462573366,1.1386438096,1.4550886838,1.55155654551,2.20245337917,2.21104783749
    avg_time,0.0762364970313,0.0979711161719,0.240608777778,0.862482378747,5.01627537939,6.0211640411

    Problem set: countertop_rightarm.yaml, num problems: 36
    ,trajopt,trajopt-multi_init,ompl-RRTConnect,ompl-LBKPIECE,chomp-hmc,chomp-hmc-multi_init
    success_frac,0.722222222222,0.916666666667,0.75,0.777777777778,0.805555555556,0.916666666667
    avg_abs_path_len,4.75977949645,5.29418064443,6.13255003221,6.26369261085,6.79648699092,6.71975814926
    avg_normed_path_len,1.09983039271,1.10239415556,1.62374799547,1.71360650077,1.64934807988,1.61262484416
    avg_time,0.180931780073,0.337876770231,0.30907774954,1.08488343256,5.02752073606,7.9616788427

Each paragraph shows the results in CSV format for a particular problem set in the suite.
The row names mean the following:

  - `success_frac`: Fraction of problems solved with valid, collision-free trajectories
  - `avg_abs_path_len`: Average joint-space path length among successful problems
  - `avg_normed_path_len`: Average normalized joint-space path length among successful problems.
    The normalized length of a trajectory for a single problem is defined as its absolute length
    divided by the minimum length for that problem across all planner configurations.
  - `avg_time`: Average time for planner to finish, for both successful and unsuccessful problems


Running individual problem sets
===============================
If you want to run an individual planning problem set
(note that this is what `run_suite.py` does under the hood for each configuration/problemset pair):

    python benchmark_scripts/run_problemset.py PROBLEM_SET_FILE PLANNER_NAME EXTRA_OPTIONS

Provided arm planning problem set files are

- `problem_sets/bookshelves_rightarm.yaml`
- `problem_sets/countertop_rightarm.yaml`
- `problem_sets/industrial_rightarm.yaml`
- `problem_sets/industrial_rightarm2.yaml`
- `problem_sets/tunnel_rightarm.yaml`

and provided full-body planning problem set files are

- `problem_sets/kitchen_fullbody.yaml`
- `problem_sets/kitchen_fullbody_10.yaml`
- `problem_sets/living_room.yaml` (environment courtesy of http://sketchup.google.com/3dwarehouse/details?mid=7eaee5ac64047976e7b3e201635a5735)

The supported planners are `trajopt`, `chomp`, and `ompl`. Extra options can be provided to
configure the behavior of the planners. For details, run

    python benchmark_scripts/run_problemset.py --help
