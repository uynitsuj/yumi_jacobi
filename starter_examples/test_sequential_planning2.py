from jacobi import Box, Environment, Frame, Planner, CartesianWaypoint, Motion, Studio
from jacobi.robots import ABBYuMiIRB14000


if __name__ == '__main__':
    # yumi = ABBYuMiIRB14000()
    # yumi.left.name = 'Left'
    # yumi.right.name = 'Right'

    # planner = Planner(environment)
    planner = Planner.load_from_project_file('/home/justinyu/multicable-decluttering/yumi_jacobi/starter_examples/ABB_YUMI_AUTOLAB.jacobi-project')
    environment = planner.environment
    # print("Robot init")
    yumi = environment.get_robot()
    speed=0.14
    yumi.set_speed(speed)
        
    lstart = CartesianWaypoint(Frame.from_euler(0.556, 0.147, 0.324, 2.927, 0.579, -0.236))
    rstart = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    lmid = CartesianWaypoint(Frame.from_euler(0.456, 0.047, 0.224, 2.927, 0.579, -0.236))
    rgoal = CartesianWaypoint(Frame.from_euler(0.48, 0.1, 0.471, 2.382, 0.5, 0.815))
    lgoal = CartesianWaypoint(Frame.from_euler(0.3, 0.3, 0.2, 0.0, 0.0, 0.0))


    m1 = Motion("m1", yumi.left, lstart, lmid)
    m2 = Motion("m2", yumi.right, rstart, rgoal)
    m3 = Motion("m3", yumi.left, lmid, lgoal)

    ## The commented code would fail
    # planner.plan(m1)
    # planner.plan(m2)
    # planner.plan(m3)

    motions = [m1, m2, m3]
    print(f'Planning {len(motions)} motions')
    trajs = planner.plan(motions)

    print(f'Calculation duration: {planner.last_calculation_duration:0.2f} [ms]')

