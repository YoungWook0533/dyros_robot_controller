import numpy as np
import dyros_robot_controller_cpp_wrapper as drc
from dyros_robot_controller.robot_data import MobileBase as DataMobileBase


class MobileControllerBase(drc.MobileControllerBase):
    def __init__(self, dt: float, robot_data: DataMobileBase):
        if not isinstance(robot_data, DataMobileBase):
            raise TypeError("robot_data must be an instance of the Python MobileBase wrapper")
        self._robot_data = robot_data
        self._dt = float(dt)
        super().__init__(self._dt, self._robot_data)

    def compute_wheel_vel(self, base_vel: np.ndarray) -> np.ndarray:
        return super().computeWheelVel(base_vel)

    def compute_IK_jacobian(self) -> np.ndarray:
        return super().computeIKJacobian()

    def velocity_command(self, desired_base_vel: np.ndarray) -> np.ndarray:
        return super().VelocityCommand(desired_base_vel)