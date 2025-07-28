import numpy as np
import dyros_robot_controller_cpp_wrapper as drc
from dyros_robot_controller import KinematicParam


class MobileBase(drc.MobileBase):
    def __init__(self, param: KinematicParam):
        if not isinstance(param, KinematicParam):
            raise TypeError("param must be a KinematicParam instance")
        self._param = param
        super().__init__(self._param.cpp())
        
    def update_state(self, wheel_pos: np.ndarray, wheel_vel: np.ndarray) -> bool:
        return bool(super().updateState(np.asarray(wheel_pos), np.asarray(wheel_vel)))
    
    # ================================ Compute Functions ================================
    def compute_base_vel(self, wheel_pos: np.ndarray, wheel_vel: np.ndarray) -> np.ndarray:
        return np.asarray(super().computeBaseVel(np.asarray(wheel_pos), np.asarray(wheel_vel)))

    def compute_fk_jacobian(self, wheel_pos: np.ndarray) -> np.ndarray:
        return np.asarray(super().computeFKJacobian(np.asarray(wheel_pos)))
    
    # ================================ Get Functions ================================
    def get_kine_param(self) -> KinematicParam:
        return self.kine_param
    
    def get_wheel_num(self) -> int:
        return int(super().getWheelNum())

    def get_wheel_pos(self) -> np.ndarray:
        return np.asarray(super().getWheelPosition())

    def get_wheel_vel(self) -> np.ndarray:
        return np.asarray(super().getWheelVelocity())

    def get_base_vel(self) -> np.ndarray:
        return np.asarray(super().getBaseVel())

    def get_FK_jacobian(self) -> np.ndarray:
        return np.asarray(super().getFKJacobian())