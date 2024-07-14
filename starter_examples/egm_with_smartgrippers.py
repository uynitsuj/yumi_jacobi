from jacobi import Planner, Frame, CartesianWaypoint, LinearMotion
from jacobi.robots import ABBYuMiIRB14000 as Yumi
from jacobi.drivers import ABBDriver
import time
import asyncio

async def run():
    yumi = Yumi()
    yumi.set_speed(0.14)

    planner = Planner(yumi)

    # From a fresh robot start, need to run twice to upload T_ROB_R before T_ROB_L starts

    module = ABBDriver.RapidModule(unit='ROB_L')

    module.upload = False #### TURN THIS TO FALSE OTHERWISE EDITED PENDANT MODULES GET OVERWRITTEN


    driver_left = ABBDriver(
        planner, yumi.left,
        host='192.168.125.1', port=6511,
        module=module, version=ABBDriver.RobotWareVersion.RobotWare6,
    )

    module = ABBDriver.RapidModule(unit='ROB_R')

    module.upload = False #### TURN THIS TO FALSE OTHERWISE EDITED PENDANT MODULES GET OVERWRITTEN


    driver_right = ABBDriver(
        planner, yumi.right,
        host='192.168.125.1', port=6512,
        module=module, version=ABBDriver.RobotWareVersion.RobotWare6,
    )

    print('Left current position', driver_left.current_joint_position)
    print('Right current position', driver_right.current_joint_position)

    # Move both arms one after the other
    home_left = [-0.967, -0.734, 0.748, -0.118, 1.365, 1.477, -1.6]
    home_right = [0.740, -0.319, -0.876, -0.310, -0.866, 1.276, -1.8]

    driver_left.move_to(home_left)
    driver_right.move_to(home_right)

    # Calculate tcp position in Task space
    print(yumi.left.calculate_tcp(home_left))
    print(yumi.right.calculate_tcp(home_right))

    # Move both arms synchronized
    trajectory = planner.plan(
        start={
            yumi.left: driver_left.current_joint_position,
            yumi.right: driver_right.current_joint_position,
        },
        goal={
            yumi.left: CartesianWaypoint(Frame(x=0.60, y=0.24, z=0.25, b=-3.1415, c=1.85)),
            yumi.right: CartesianWaypoint(Frame(x=0.50, y=-0.14, z=0.25, b=-3.1415, c=1.12)),
        },
    )

    result_left = driver_left.run_async(trajectory)
    result_right = driver_right.run_async(trajectory)
    await result_left
    await result_right

    # Linear motion of left arm
    motion = LinearMotion(
        yumi.left,
        driver_left.current_joint_position,
        Frame(x=0.60, y=0.14, z=0.25, b=-3.1415, c=1.0),
    )
    motion.robot.set_speed(0.05)

    tl = planner.plan(motion)
    result_left = driver_left.run_async(tl)

    # And stop after a timeout because we can control the robot on-the-fly
    time.sleep(3.0)
    driver_left.stop()

    # Back to home
    yumi.set_speed(0.15)
    result_left = driver_left.move_to_async(home_left)
    result_right = driver_right.move_to_async(home_right)
    await result_left
    await result_right


    #### BIMANUAL GRIPPER CONTROL ####

    l_g_comm = ['192.168.125.40', 'Hand_L']
    r_g_comm = ['192.168.125.30', 'Hand_R']

    def get_signal(driver, comm, signal):
        return driver.rws.get_signal(signal = signal, network = comm[0], device = comm[1])

    def set_signal(driver, comm, signal, value):
        return driver.rws.set_signal(signal = signal, value = str(value), network = comm[0], device = comm[1])

    ## cmd_GripperState_ controls ##
    #     COMMAND_NONE         := 0;
    #     COMMAND_INITIALIZE   := 1;
    #     COMMAND_CALIBRATE    := 2;
    #     COMMAND_MOVE_TO      := 3;
    #     COMMAND_GRIP_IN      := 4;
    #     COMMAND_GRIP_OUT     := 5;
    #     COMMAND_BLOW_ON_1    := 6;
    #     COMMAND_BLOW_ON_2    := 7;
    #     COMMAND_BLOW_OFF_1   := 8;
    #     COMMAND_BLOW_OFF_2   := 9;
    #     COMMAND_VACUUM_ON_1  := 10;
    #     COMMAND_VACUUM_ON_2  := 11;
    #     COMMAND_VACUUM_OFF_1 := 12;
    #     COMMAND_VACUUM_OFF_2 := 13;

    ## cmd_GripperPos_ is valid between 0 to 25 (Physical gripper travel range in [mm]) ##

    set_signal(driver_left, l_g_comm, 'cmd_GripperState_L', 5)
    set_signal(driver_left, l_g_comm, 'cmd_GripperPos_L', 0)

    print(get_signal(driver_left, l_g_comm, 'cmd_GripperState_L'))
    print(get_signal(driver_left, l_g_comm, 'cmd_GripperPos_L'))

    driver_left.rws.call_procedure('handleRunSGRoutine')

if __name__ == '__main__':
    asyncio.run(run())