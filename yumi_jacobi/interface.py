import numpy as np
from autolab_core import RigidTransform, Point
from typing import List, Tuple, Union

from yumi_jacobi.yumi import YuMiArm
import asyncio
from jacobi import Frame, CartesianWaypoint, LinearMotion, Motion, Studio
from jacobi import Planner
from jacobi.robots import ABBYuMiIRB14000 as Yumi
from jacobi.drivers import ABBDriver
from yumi_jacobi.tcp import *
from copy import copy
import os

class Interface:
    # orientation where the gripper is facing downwards
    GRIP_DOWN_R = np.diag([1, -1, -1])
    GRIP_DOWN_L = np.diag([-1, 1, -1])
    GRIP_UP_R = np.diag([1, 1, 1])
    GRIP_SIDEWAYS = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
    GRIP_SIDEWAYS_L = RigidTransform.x_axis_rotation(-np.pi / 2) @ GRIP_DOWN_R
    GRIP_TILT_R = RigidTransform.x_axis_rotation(np.pi / 4) @ RigidTransform.z_axis_rotation(np.pi / 2) @ GRIP_DOWN_R
    GRIP_TILT_L = RigidTransform.x_axis_rotation(-np.pi / 4) @ RigidTransform.z_axis_rotation(np.pi / 2) @ GRIP_DOWN_R
    GRIP_TILT_R_Y = RigidTransform.y_axis_rotation(-np.pi / 8) @ RigidTransform.z_axis_rotation(np.pi / 2) @ GRIP_DOWN_R
    GRIP_TILT_L_Y = RigidTransform.y_axis_rotation(np.pi / 8) @ RigidTransform.z_axis_rotation(np.pi / 2) @ GRIP_DOWN_R
    L_ARMS_CLEAR_STATE = np.array([
        -0.5810662,
        -1.34913424,
        0.73567095,
        0.23, 
        1.46,
        1.25265177,
        2.940,
    ]
    )
    R_ARMS_CLEAR_STATE = np.array(
        [
            0.64224786,
            -1.34920282,
            -0.74859683,
            0.22,
            -1.535,
            1.20916355,
            -2.83024169,
        ]
    )
    def __init__(
        self,
        speed=0.14,
        l_tcp=ABB_WHITE.as_frames("l_tcp_frame", "l_tip_frame"),
        r_tcp=ABB_WHITE.as_frames("r_tcp_frame", "r_tip_frame"),
        file='/home/justinyu/multicable-decluttering/yumi_jacobi/starter_examples/AUTOLAB_BWW_YuMi.jacobi-project',
    ):
        
        self.speed = speed
        
        self.l_tcp = l_tcp
        self.r_tcp = r_tcp

        if os.path.exists(file):
            self.planner = Planner.load_from_project_file(file)
            self.environment = self.planner.environment
            self.yumi = self.environment.get_robot()
            self.yumi.set_speed(speed)
            self.studio = Studio()
        else:
            self.yumi = Yumi()
            self.yumi.set_speed(speed)
            self.planner = Planner(self.yumi)
            self.environment = None
            self.studio = None

        self.planner = Planner(self.yumi)

        module = ABBDriver.RapidModule(unit='ROB_L')
        module.upload = False
        self.driver_left = YuMiArm(
            self.planner, self.yumi.left,
            host='192.168.125.1', port=6511,
            module=module, version=ABBDriver.RobotWareVersion.RobotWare6,
        )

        module = ABBDriver.RapidModule(unit='ROB_R')
        module.upload = False
        self.driver_right = YuMiArm(
            self.planner, self.yumi.right,
            host='192.168.125.1', port=6512,
            module=module, version=ABBDriver.RobotWareVersion.RobotWare6,
        )

        self.set_TCP()

    def set_TCP(self):
        self.yumi.left.flange_to_tcp = self.RT2Frame(self.l_tcp)
        self.yumi.right.flange_to_tcp = self.RT2Frame(self.r_tcp)

    def get_FK(self, arm: str):
        if arm == 'left':
            return self.yumi.left.calculate_tcp(self.driver_left.current_joint_position)
        elif arm == 'right':
            return self.yumi.right.calculate_tcp(self.driver_right.current_joint_position)
        else:
            raise ValueError(f"Invalid arm {arm}, enter 'left' or 'right'")
    async def calibrate_grippers(self):
        await self.driver_left.calibrate_gripper(sync=True)
        await asyncio.sleep(0.1)
        await self.driver_right.calibrate_gripper()
        await asyncio.sleep(1)
        await self.driver_left.open_gripper(sync=True)
        await asyncio.sleep(0.1)
        await self.driver_right.open_gripper()
        await asyncio.sleep(1)
        return True
    
    async def close_grippers(self):
        await self.driver_left.close_gripper(sync=True)
        await asyncio.sleep(0.1)
        await self.driver_right.close_gripper()
        await asyncio.sleep(1)
        return True
    
    async def open_grippers(self):
        await self.driver_left.open_gripper(sync=True)
        await asyncio.sleep(0.1)
        await self.driver_right.open_gripper()
        await asyncio.sleep(1)
        return True

    async def grippers_move_to(self, left_dist = 0, right_dist = 0):
        # Jog grippers to the specified width [0, 25] (mm)
        # Default arguments are 0 mm open grippers
        await self.driver_left.move_gripper(left_dist, sync=True)
        await asyncio.sleep(0.1)
        await self.driver_right.move_gripper(right_dist)
        await asyncio.sleep(1)
        return True

    async def home(self):
        l_pos = self.yumi.left.calculate_tcp(self.driver_left.current_joint_position)
        r_pos = self.yumi.right.calculate_tcp(self.driver_right.current_joint_position)
        # print(l_pos)
        if l_pos.translation[2] < .08 or r_pos.translation[2] < .08:
            self.yumi.set_speed(0.1)
            await self.go_delta([0, 0, 0.12 - l_pos.translation[2]],
                                [0, 0, 0.12 - r_pos.translation[2]])
        self.yumi.set_speed(self.speed)

        await self.move_to(self.L_ARMS_CLEAR_STATE, self.R_ARMS_CLEAR_STATE)

    async def move_to(self, left_goal = None, right_goal=None):
        # Move both arms to the specified joint angles (radians)
        result_left = self.driver_left.move_to_async(left_goal)
        result_right = self.driver_right.move_to_async(right_goal)
        await result_left
        await result_right

    async def go_delta(self, left_delta: List = [0, 0, 0], right_delta: List = [0, 0, 0]):
        # Move both arms by the specified cartesian delta [x,y,z], meant for small motions
        l_curr_pos = self.yumi.left.calculate_tcp(self.driver_left.current_joint_position)
        l_goal_mat = l_curr_pos.matrix
        l_goal_mat[-4:-1] += np.array(left_delta)
        l_goal = Frame.from_matrix(l_goal_mat)

        r_curr_pos = self.yumi.right.calculate_tcp(self.driver_right.current_joint_position)
        r_goal_mat = r_curr_pos.matrix
        r_goal_mat[-4:-1] += np.array(right_delta)
        r_goal = Frame.from_matrix(r_goal_mat)

        trajectory = self.planner.plan(
        start={
            self.yumi.left: self.driver_left.current_joint_position,
            self.yumi.right: self.driver_right.current_joint_position,
        },
        goal={
            self.yumi.left: CartesianWaypoint(l_goal),
            self.yumi.right: CartesianWaypoint(r_goal),
        },
        )

        if self.studio is not None:
            self.studio.run_trajectory(trajectory)
        result_left = self.driver_left.run_async(trajectory)
        result_right = self.driver_right.run_async(trajectory)
        await result_left
        await result_right

    async def go_cartesian(self, l_targets: List[RigidTransform]=[], r_targets: List[RigidTransform]=[]):
        # Move both arms to waypoint [RigidTransform] or along specified waypoints (list of RigidTransforms)
        assert (len(l_targets) > 0 or len(r_targets) > 0), "No waypoints provided"
        if len(l_targets) > 0:
            motion = self.listRT2Motion(
                robot = self.yumi.left,
                start = self.driver_left.current_joint_position, 
                wp_list = l_targets
            )
            trajectory = self.planner.plan(motion)
            result_left = self.driver_left.run_async(trajectory)
            await result_left
        if len(r_targets) > 0:
            motion = self.listRT2Motion(
                robot = self.yumi.right,
                start = self.driver_right.current_joint_position, 
                wp_list = r_targets
            )
            trajectory = self.planner.plan(motion)
            result_right = self.driver_right.run_async(trajectory)
            await result_right

    def listRT2Motion(self, robot, start: List, wp_list: List[RigidTransform]) -> Motion:
        # Convert list of RigidTransform to Motion (Jacobi object) 
        # Currently supports up to 3 waypoints
        motion = Motion(
            robot, 
            start,
            self.RT2Frame(wp_list[-1])
        )
        motion.waypoints = [self.RT2Frame(q) for q in wp_list]
        return motion

    def RT2CW(self, transform: RigidTransform) -> CartesianWaypoint:
        # Convert RigidTransform (autolab_core) to CartesianWaypoint (Jacobi object)
        return CartesianWaypoint(
            self.RT2Frame(transform)
        )

    def RT2Frame(self, transform: RigidTransform) -> Frame:
        # Convert RigidTransform (autolab_core) to Frame (Jacobi object)
        return Frame(
            x=transform.translation[0],
            y=transform.translation[1],
            z=transform.translation[2],
            qw=transform.quaternion[0],
            qx=transform.quaternion[1],
            qy=transform.quaternion[2],
            qz=transform.quaternion[3],
        )
    
    def Frame2RT(self, frame: Frame) -> RigidTransform:
        # Convert Frame (Jacobi object) to RigidTransform (autolab_core)
        return RigidTransform(
            translation=frame.translation,
            rotation=RigidTransform.rotation_from_quaternion(frame.quaternion))