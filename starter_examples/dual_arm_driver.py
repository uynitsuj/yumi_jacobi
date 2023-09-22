from jacobi import Planner
from jacobi.drivers import ABBDriver
from jacobi.robots import ABBYuMiIRB14000


if __name__ == '__main__':
    # Set up robot
    yumi = ABBYuMiIRB14000()
    yumi.set_speed(0.2)

    # Set up planner
    planner = Planner(yumi)

    # Plan an example trajectory
    trajectory = planner.plan(
        start=[0, -30 * 3.1415/180, 0, 0, 0, 0, 0] + [0, -30 * 3.1415/180, 0, 0, 0, 0, 0],
        goal=[0, -50 * 3.1415/180, 0, 0, 0, 0, 0] + [0, -20 * 3.1415/180, 0, 0, 0, 0, 0],
    )
    print(f'Trajectory calculated with duration: {trajectory.duration:0.4f}')

    # Setup drivers for each arm separately
    driver_left = ABBDriver(planner, yumi.left, port=6511)
    driver_right = ABBDriver(planner, yumi.right, port=6512)

    # Run a trjactory in parallel
    result_left = driver_left.run_async(trajectory)
    result_right = driver_right.run_async(trajectory)

    result_left.wait()
    result_right.wait()

    # Or run them one after another
    # result_left = driver_left.run(trajectory)
    # result_right = driver_right.run(trajectory)
