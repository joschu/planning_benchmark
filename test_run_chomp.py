# load module                   

import openravepy
m_chomp = prrave.rave.load_module(e, 'orcdchomp', 'blah_load_string')
orcdchomp.orcdchomp.bind(m_chomp)

# compute (and cache) distance field for this problem                                                                                                
print 'Loading distance field ...'
j1idxs = [m.GetArmIndices()[0] for m in r.GetManipulators()]
for link in r.GetLinks():
   for j1idx in j1idxs:
      if r.DoesAffect(j1idx, link.GetIndex()):
         link.Enable(False)
m_chomp.computedistancefield(kinbody=r, aabb_padding=1.0,cache_filename='%s/../chomp-sdf.dat'%datadir)
for link in r.GetLinks():
   link.Enable(True)

# run chomp                                                                                                                                          
t = m_chomp.runchomp(robot=ROBOT, n_iter=N_ITER, max_time=MAX_TIME,
   lambda_=100.0, adofgoal=goal, no_collision_exception=True,
   dat_filename='%s/summary.dat'%datadir,
   trajs_fileformstr='%s/traj-iter%%03d.xml'%datadir)
