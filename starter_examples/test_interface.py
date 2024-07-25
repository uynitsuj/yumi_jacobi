from yumi_jacobi.interface import Interface
from autolab_core import RigidTransform, Point
# import asyncio
import time

def run():
    interface = Interface(speed=0.20, file='/home/justinyu/multicable-decluttering/yumi_jacobi/starter_examples/AUTOLAB_BWW_YuMi.jacobi-project')
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
        translation=[0.3, 0.3, 0.15]
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
    
    # Pre-plan then execute
    print('Pre-planning waypoints')
    motion_l1, motion_r1 = interface.plan_cartesian_waypoints(l_targets = [wp2_l, wp1_l, wp3_l], r_targets=[wp1_r, wp3_r, wp2_r])
    motion_l2, motion_r2 = interface.plan_cartesian_waypoints(l_targets = [wp3_l, wp2_l, wp2_l], r_targets=[wp2_r, wp3_r, wp1_r], starting_from_current_cfg=False)
    print("Executing waypoints")
    start_time = time.time()
    interface.blend_into(motion_l1, motion_r1, 3.0)
    print("One done in ", time.time() - start_time)
    print("Current left: ", interface.get_FK('left'))
    print("Next left: ", wp3_l)
    print("Current right: ", interface.get_FK('right'))
    print("Next right: ", wp2_r)
    start_time = time.time()
    interface.run_trajectory(motion_l2, motion_r2)
    print("Two done in ", time.time() - start_time)
    print("Current left: ", interface.get_FK('left'))
    print("Next left: ", wp2_l)
    print("Current right: ", interface.get_FK('right'))
    print("Next right: ", wp1_r)
    start_time = time.time()
    interface.run_trajectory(motion_l1, motion_r1)
    print("Three done in ", time.time() - start_time)
    print("Current left: ", interface.get_FK('left'))
    print("Next left: ", wp3_l)
    print("Current right: ", interface.get_FK('right'))
    print("Next right: ", wp2_r)

    interface.go_cartesian_waypoints(l_targets = [wp1_l, wp2_l, wp3_l], r_targets=[wp1_r, wp2_r, wp3_r])
    interface.go_linear_single(l_target=wp1_l, r_target=wp1_r)
    interface.go_linear_single(l_target=wp2_l, r_target=wp2_r)
    interface.go_linear_single(l_target=wp3_l, r_target=wp3_r)
    # print(interface.get_FK('left'))
    # print(interface.get_FK('right'))
    interface.home()
    
    interface.go_delta([0, 0, -0.1])
    interface.go_delta([0, -0.1, 0], [0, 0.1, 0])
    interface.go_delta([0.1, -0.1, 0], [0.1, 0.1, 0])
    # print(interface.get_FK('left'))
    # print(interface.get_FK('right'))
    interface.home()
    
    
if __name__ == '__main__':
    run()