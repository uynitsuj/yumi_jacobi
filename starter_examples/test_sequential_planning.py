from yumi_jacobi.interface import Interface
from autolab_core import RigidTransform, Point
from jacobi import Frame, CartesianWaypoint, LinearMotion, Motion, Studio, Trajectory

def run():
    interface = Interface(speed=0.26, file='/home/justinyu/multicable-decluttering/yumi_jacobi/starter_examples/ABB_YUMI_AUTOLAB.jacobi-project')
    print(interface.get_FK('left'))
    print(interface.get_FK('right'))
    interface.home()
    interface.calibrate_grippers()
    interface.open_grippers()

    wp1_l = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.4, 0.3, 0.2]
    )
    wp2_l = RigidTransform(
        rotation=[[-1.0000000, -0.0000000,  0.0000000], 
                  [-0.0000000,  0.9659258, -0.2588190], 
                  [-0.0000000, -0.2588190, -0.9659258]],
        translation=[0.4, 0.4, 0.1]
    )
    wp3_l = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.6, 0.3, 0.15]
    )
    wp1_r = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.3, -0.2, 0.15]
    )
    wp2_r = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.35, -0.05, 0.2]
    )
    wp3_r = RigidTransform(
        rotation=[[-0.7071068,  0.7071068,  0.0000000], 
                  [0.5000000,  0.5000000,  0.7071068], 
                  [0.5000000,  0.5000000, -0.7071068]],
        translation=[0.45, -0.08, 0.15]
    )
    
    
    # motions = [l_motion0, r_motion0, l_motion1, r_motion1, l_motion2]
    
    # traj = interface.planner.plan(motions)
    # print(f'Calculation duration: {interface.planner.last_calculation_duration:0.2f} [ms]')
    
    # interface.run_trajectories(traj)
    
    traj = interface.plan_linear_waypoints(l_targets = [wp1_l, wp2_l, wp3_l])
    import pdb; pdb.set_trace()
    print(traj)
    interface.run_trajectories(traj)
        
    import pdb; pdb.set_trace()
    
    interface.go_linear_single(l_target=wp1_l, r_target=wp1_r)
    interface.go_linear_single(l_target=wp2_l, r_target=wp2_r)
    interface.go_linear_single(l_target=wp3_l, r_target=wp3_r)
    interface.home()
    
    interface.go_delta([0, 0, -0.1])
    interface.go_delta([0, -0.1, 0], [0, 0.1, 0])
    interface.go_delta([0.1, -0.1, 0], [0.1, 0.1, 0])
    interface.home()

if __name__ == '__main__':
    run()