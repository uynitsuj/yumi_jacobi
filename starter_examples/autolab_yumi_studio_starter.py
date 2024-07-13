# A small script to get started on visualization motions in Studio with the YuMi

from argparse import ArgumentParser

from jacobi import Planner, CartesianWaypoint, Frame, Studio


if __name__ == '__main__':
    parser = ArgumentParser('Load a jacobi-project file.')
    parser.add_argument('file')

    args = parser.parse_args()

    planner = Planner.load_from_project_file(args.file)
    environment = planner.environment
    yumi = environment.get_robot()

    yumi.set_speed(0.1)

    trajectory = planner.plan(
        start={
            yumi.left: [0.0, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
            yumi.right: [0.0, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0],
        },
        goal={
            yumi.left: CartesianWaypoint(Frame(x=0.60, y=0.24, z=0.25, b=-3.1415, c=1.85)),
            yumi.right: CartesianWaypoint(Frame(x=0.50, y=-0.14, z=0.25, b=-3.1415, c=1.12)),
        },
    )
    print(trajectory)

    studio = Studio()
    studio.run_trajectory(trajectory)
    