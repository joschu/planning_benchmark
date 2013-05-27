def setup_env_for_problem(env, robot, prob_info):
    
    initial_names = prob_info["start"]["joint_names"]
    initial_vals = prob_info["start"]["joint_values"]
    initial_pose = prob_info["start"]["pose"]
    
    initial_inds = [robot.GetJoint(jname).GetIndex() for jname in initial_names]
    
    robot.SetDOFValues(initial_vals, initial_inds)
    robot.SetTransform(openravepy.matrixFromPose(initial_pose))    
    
    active_joints = prob_info["active_joints"]
    
    active_inds = [robot.GetJoint(jname).GetIndex() for jname in active_joints]
    robot.SetActiveDOFs(active_inds)
