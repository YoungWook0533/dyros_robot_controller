import numpy as np
import dyros_robot_controller_cpp_wrapper as drc
from .robot_data import RobotData


class RobotController(drc.MobileManipulatorRobotController):
    def __init__(self, dt: float, robot_data: RobotData):
        if not isinstance(robot_data, RobotData):
            raise TypeError("robot_data must be an instance of the Python MobileManipulatorBase")
        self._dt = float(dt)
        self._robot_data = robot_data
        super().__init__(self._dt, self._robot_data)

    def set_manipulator_joint_gain(self, kp: np.ndarray, kv: np.ndarray):
        super().setManipulatorJointGain(kp, kv)

    def set_manipulator_joint_kp_gain(self, kp: np.ndarray):
        super().setManipulatorJointKpGain(kp)
        
    def set_manipulator_joint_kv_gain(self, kv: np.ndarray):
        super().setManipulatorJointKvGain(kv)

    def set_task_gain(self, kp: np.ndarray, kv: np.ndarray):
        super().setTaskGain(kp, kv)
        
    def set_task_kp_gain(self, kp: np.ndarray):
        super().setTaskKpGain(kp)
        
    def set_task_kv_gain(self, kv: np.ndarray):
        super().setTaskKpGain(kv)

    def move_manipulator_joint_position_cubic(self,
                                              q_mani_target: np.ndarray,
                                              qdot_mani_target: np.ndarray,
                                              q_mani_init: np.ndarray,
                                              qdot_mani_init: np.ndarray,
                                              current_time: float,
                                              init_time: float,
                                              duration: float,
                                              ) -> np.ndarray:
        return super().moveManipulatorJointPositionCubic(q_mani_target,
                                                         qdot_mani_target,
                                                         q_mani_init,
                                                         qdot_mani_init,
                                                         current_time,
                                                         init_time,
                                                         duration,
                                                         )

    # def move_manipulator_joint_torque_step(self, q_mani_target: np.ndarray, qdot_mani_target: np.ndarray) -> np.ndarray:
    #     return super().moveManipulatorJointTorqueStep(q_mani_target, qdot_mani_target)
    
    # def move_manipulator_joint_torque_step(self, qddot_mani_target: np.ndarray) -> np.ndarray:
    #     return super().moveManipulatorJointTorqueStep(qddot_mani_target)
    def move_manipulator_joint_torque_step(self,
                                           q_mani_target:     np.ndarray | None = None,
                                           qdot_mani_target:  np.ndarray | None = None,
                                           qddot_mani_target: np.ndarray | None = None,
                                           ) -> np.ndarray:
        if qddot_mani_target is not None:
            return super().moveManipulatorJointTorqueStep(qddot_mani_target)

        if q_mani_target is not None and qdot_mani_target is not None:
            return super().moveManipulatorJointTorqueStep(q_mani_target, qdot_mani_target)

    def move_manipulator_joint_torque_cubic(self,
                                            q_mani_target: np.ndarray,
                                            qdot_mani_target: np.ndarray,
                                            q_mani_init: np.ndarray,
                                            qdot_mani_init: np.ndarray,
                                            current_time: float,
                                            init_time: float,
                                            duration: float,
                                            ) -> np.ndarray:
        return super().moveManipulatorJointTorqueCubic(q_mani_target,
                                                       qdot_mani_target,
                                                       q_mani_init,
                                                       qdot_mani_init,
                                                       current_time,
                                                       init_time,
                                                       duration,
                                                       )

    def QPIK(self, xdot_target: np.ndarray, link_name: str) -> tuple[np.ndarray, np.ndarray]:
        return super().QPIK(xdot_target, link_name)

    def QPIK_step(self, x_target: np.ndarray, xdot_target: np.ndarray, link_name: str) -> tuple[np.ndarray, np.ndarray]:
        return super().QPIKStep(x_target, xdot_target, link_name)

    def QPIK_cubic(self,
                   x_target: np.ndarray,
                   xdot_target: np.ndarray,
                   x_init: np.ndarray,
                   xdot_init: np.ndarray,
                   current_time: float,
                   init_time: float,
                   duration: float,
                   link_name: str,
                   ) -> tuple[np.ndarray, np.ndarray]:
        return super().QPIKCubic(x_target,
                                 xdot_target,
                                 x_init,
                                 xdot_init,
                                 current_time,
                                 init_time,
                                 duration,
                                 link_name
                                 )

    def QPID(self, xddot_target: np.ndarray, link_name: str) -> tuple[np.ndarray, np.ndarray]:
        return super().QPID(xddot_target, link_name)

    def QPID_step(self, x_target: np.ndarray, xdot_target: np.ndarray, link_name: str) -> tuple[np.ndarray, np.ndarray]:
        return super().QPIDStep(x_target, xdot_target, link_name)

    def QPID_cubic(self,
                   x_target: np.ndarray,
                   xdot_target: np.ndarray,
                   x_init: np.ndarray,
                   xdot_init: np.ndarray,
                   current_time: float,
                   init_time: float,
                   duration: float,
                   link_name: str,
                   ) -> tuple[np.ndarray, np.ndarray]:
        return super().QPIDCubic(x_target,
                                 xdot_target,
                                 x_init,
                                 xdot_init,
                                 current_time,
                                 init_time,
                                 duration,
                                 link_name,
                                 )
