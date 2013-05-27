"""
Usage:


First run save_moveit_start_states.py
Suppose there are 10 states, numbered 0...9
Add a line
problems:
to the generateed yaml file.

Then call
make_pairs.py 10 >> blah.yaml

"""

s = """
  - start:
      active_dof_values: *state%i
    goal:
      active_dof_values: *state%i"""[1:]

import sys
n = int(sys.argv[1])
for i in xrange(n-1):
    for j in xrange(i+1,n):
        print s%(i,j)
        
