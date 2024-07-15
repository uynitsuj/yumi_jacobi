from yumi_jacobi.interface import Interface
from autolab_core import RigidTransform, Point
import asyncio

async def run():
    interface = Interface()

    await interface.home()

    await interface.calibrate_grippers()
    await interface.open_grippers()
    await interface.go_delta([0, 0, -0.1])
    await interface.go_delta([0, -0.1, 0], [0, 0.1, 0])
    print(interface.Frame2RT(interface.get_FK('left')))
    print(interface.Frame2RT(interface.get_FK('right')))
    await interface.go_delta([0.1, -0.1, 0], [0.1, 0.1, 0])
    await interface.home()

if __name__ == '__main__':
    asyncio.run(run())