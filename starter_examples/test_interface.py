from yumi_jacobi.interface import Interface
from autolab_core import RigidTransform, Point
# import asyncio

def run():
    interface = Interface(speed=0.23, file='/home/justinyu/multicable-decluttering/yumi_jacobi/starter_examples/AUTOLAB_BWW_YuMi.jacobi-project')
    print(interface.Frame2RT(interface.get_FK('left')))
    print(interface.Frame2RT(interface.get_FK('right')))
    interface.home()

    interface.calibrate_grippers()
    interface.open_grippers()

    wp1_l = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.4, 0.3, 0.2]
    )
    wp2_l = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.4, 0.4, 0.1]
    )
    wp3_l = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.3, 0.3, 0.15]
    )
    wp1_r = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.3, -0.3, 0.15]
    )
    wp2_r = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.45, -0.15, 0.2]
    )
    wp3_r = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.5, -0.1, 0.25]
    )
    interface.go_cartesian_waypoints(l_targets = [wp1_l, wp2_l, wp3_l], r_targets=[wp1_r, wp2_r, wp3_r])
    interface.home()
    
    interface.go_delta([0, 0, -0.1])
    interface.go_delta([0, -0.1, 0], [0, 0.1, 0])
    # print(interface.Frame2RT(interface.get_FK('left')))
    # print(interface.Frame2RT(interface.get_FK('right')))
    interface.go_delta([0.1, -0.1, 0], [0.1, 0.1, 0])
    interface.home()
    # await interface.go_linear(l_target=)

if __name__ == '__main__':
    run()