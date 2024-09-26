from jacobi import Planner
from jacobi.robots import ABBYuMiIRB14000 as Yumi
from jacobi.drivers import ABBDriver


if __name__ == '__main__':
    yumi = Yumi()
    yumi.set_speed(0.14)

    planner = Planner(yumi)

    # From a fresh robot start, need to run twice to upload T_ROB_R before T_ROB_L starts
    module = ABBDriver.RapidModule(unit='ROB_L')
    driver_left = ABBDriver(
        planner, yumi.left,
        host='192.168.125.1', port=6511,
        module=module, version=ABBDriver.RobotWareVersion.RobotWare6,
    )

    module = ABBDriver.RapidModule(unit='ROB_R')
    driver_right = ABBDriver(
        planner, yumi.right,
        host='192.168.125.1', port=6512,
        module=module, version=ABBDriver.RobotWareVersion.RobotWare6,
    )

    io_name = 'hall_sensor_joint_1'

    value = driver_left.get_digital_input(io_name)
    if value is None:
        print(f"Could not read from address '{io_name}'.")
        exit()

    print(f"Value at digital input '{io_name}' is '{value}'.")

    # Set an output for a group of 8 bits
    success = driver_left.set_digital_output(io_name, 1)
    if not success:
        print(f"Could not write to address '{io_name}'.")
        exit()

    # Read value again
    value = driver_left.get_digital_input(io_name)
    print(f"Value at digital input '{io_name}' is now '{value}'.")