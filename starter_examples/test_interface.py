from yumi_jacobi.interface import Interface
from autolab_core import RigidTransform, Point
import asyncio

async def run():
    interface = Interface(file='/home/justinyu/multicable-decluttering/yumi_jacobi/starter_examples/AUTOLAB_BWW_YuMi.jacobi-project')

    await interface.home()

    await interface.calibrate_grippers()
    await interface.open_grippers()
    await interface.go_delta([0, 0, -0.1])
    await interface.go_delta([0, -0.1, 0], [0, 0.1, 0])
    print(interface.Frame2RT(interface.get_FK('left')))
    print(interface.Frame2RT(interface.get_FK('right')))
    await interface.go_delta([0.1, -0.1, 0], [0.1, 0.1, 0])
    await interface.home()

    wp1_l = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.4, 0.3, 0.2]
    )
    wp2_l = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.4, 0.4, 0.1]
    )
    wp1_r = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.3, -0.3, 0.15]
    )
    wp2_r = RigidTransform(
        rotation=[[-1, 0, 0], [0, 1, 0], [0, 0, -1]],
        translation=[0.35, -0.2, 0.2]
    )
    await interface.go_cartesian(l_targets = [wp1_l, wp2_l], r_targets=[wp1_r, wp2_r])
    await interface.home()

if __name__ == '__main__':
    asyncio.run(run())