from jacobi import Planner, Frame, CartesianWaypoint, LinearMotion
from jacobi.robots import ABBYuMiIRB14000 as Yumi
from jacobi.drivers import ABBDriver
from autolab_core import RigidTransform

class YuMi_EGM(object):
    """
    YuMi interface with EGM implementation from Jacobi Robotics plus SmartGripper Support
    
    Jacobi Robotics Documentation:
    https://docs.jacobirobotics.com/index.html
    """
    def __init__(
        self,
        ip_address="192.168.125.1",
    ):
        self.l_g_comm = ['192.168.125.40', 'Hand_L']
        self.r_g_comm = ['192.168.125.30', 'Hand_R']

        self.yumi = Yumi()
        self.yumi.set_speed(0.14)

        module = self._instantiate_module('ROB_L')

        self.driver_left = ABBDriver(
            planner, yumi.left,
            host=ip_address, port=6511,
            module=module, version=ABBDriver.RobotWareVersion.RobotWare6,
        )
        self.gripper_left = YuMiSG(self.driver_left, 'left')

        module = self._instantiate_module('ROB_R')

        self.driver_right = ABBDriver(
            planner, yumi.right,
            host=ip_address, port=6512,
            module=module, version=ABBDriver.RobotWareVersion.RobotWare6,
        )
        self.gripper_right = YuMiSG(self.driver_right, 'right')

    def _instantiate_module(self, unit):
        module = ABBDriver.RapidModule(unit)
        module.upload = False #### FALSE OTHERWISE EDITED PENDANT MODULES GET OVERWRITTEN
        return module
    
    @property
    def connected(self):
        return self.driver_right.connected and self.driver_left.connected

    @property
    def motors_on(self):
        with self._lock:
            return self._iface.motors_on

    @motors_on.setter
    def motors_on(self, value):
        with self._lock:
            self._iface.set_motors_on() if value else self._iface.set_motors_off()

    @property
    def rapid_running(self):
        with self._lock:
            return self._iface.rapid_running

    @property
    def rw_version(self):
        return self.driver_right.rws.version

    def calibrate_grippers(self):
        for gripper in [self.gripper_left, self.gripper_right]:
            gripper.initialize_gripper()
            gripper.calibrate_gripper()
        time.sleep(3)

    def move_grippers(self, lpos, rpos):

    def close_grippers(self):

    def open_grippers(self):
        self.


class YuMiSG(object):
    "SmartGripper interface for YuMi robot using Jacobi I/O signal hooks and interface for calling RAPID procedures"
    def __init__(self, driver, side):
        self._driver = driver
        self._side = side # 'left' or 'right'
        if side == 'left':
            self._gripper_comm = ['192.168.125.40', 'Hand_L']
            self._gripper_state_IO = 'cmd_GripperState_L'
            self._gripper_pos_IO = 'cmd_GripperPos_L'
        elif side == 'right':
            self._gripper_comm = ['192.168.125.30', 'Hand_R']
            self._gripper_state_IO = 'cmd_GripperState_R'
            self._gripper_pos_IO = 'cmd_GripperPos_R'
        else:
            raise ValueError("Side must be either 'left' or 'right'")

    def _get_signal(self, signal):
        return self._driver.rws.get_signal(signal = signal, network = self._gripper_comm[0], device = self._gripper_comm[1])

    def _set_signal(self, signal, value):
        return self._driver.rws.set_signal(signal = signal, value = str(value), network = self._gripper_comm[0], device = self._gripper_comm[1])

    def initialize_gripper(self):
        self._set_signal(self._gripper_state_IO, 1)

    def calibrate_gripper(self):
        self._set_signal(self._gripper_state_IO, 2)

    def open_gripper(self):
        self._set_signal(self._gripper_state_IO, 5)

    def close_gripper(self):
        self._set_signal(self._gripper_state_IO, 4)

    def move_gripper(self, value):
        "Value is valid between 0 to 25 (Physical gripper travel range in [mm]), though pendant modules have extra check and saturate values"
        self._set_signal(self._gripper_pos_IO, value)
        self._set_signal(self._gripper_state_IO, 3)

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
