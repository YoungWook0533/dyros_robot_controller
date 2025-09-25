import numpy as np
import dyros_robot_controller_cpp_wrapper as drc
from dyros_robot_controller.robot_data import ManipulatorBase as DataManipulatorBase


class ManipulatorControllerBase(drc.ManipulatorControllerBase):
    def __init__(self, dt: float, robot_data: DataManipulatorBase):
        if not isinstance(robot_data, DataManipulatorBase):
            raise TypeError("robot_data must be an instance of the Python ManipulatorBase wrapper")

        self._dt = float(dt)
        self._robot_data = robot_data
        super().__init__(self._dt, self._robot_data)

    def set_joint_gain(self, kp: np.ndarray, kv: np.ndarray):
        super().setJointGain(kp, kv)

    def set_task_gain(self, kp: np.ndarray, kv: np.ndarray):
        super().setTaskGain(kp, kv)

    # ================================ Joint space Functions ================================
    def move_joint_position_cubic(self,
                                  q_target: np.ndarray,
                                  qdot_target: np.ndarray,
                                  q_init: np.ndarray,
                                  qdot_init: np.ndarray,
                                  current_time: float,
                                  init_time: float,
                                  duration: float,
                                  ) -> np.ndarray:
        return super().moveJointPositionCubic(q_target,
                                              qdot_target,
                                              q_init,
                                              qdot_init,
                                              current_time,
                                              init_time,
                                              duration, 
                                              )
        
    def move_joint_velocity_cubic(self,
                                  q_target: np.ndarray,
                                  qdot_target: np.ndarray,
                                  q_init: np.ndarray,
                                  qdot_init: np.ndarray,
                                  current_time: float,
                                  init_time: float,
                                  duration: float,
                                  ) -> np.ndarray:
        return super().moveJointVelocityCubic(q_target,
                                              qdot_target,
                                              q_init,
                                              qdot_init,
                                              current_time,
                                              init_time,
                                              duration, 
                                              )

    # def move_joint_torque_step(self, q_target: np.ndarray, qdot_target: np.ndarray) -> np.ndarray:
    #     return super().moveJointTorqueStep(q_target, qdot_target)
    
    # def move_joint_torque_step(self, qddot_target: np.ndarray) -> np.ndarray:
    #     return super().moveJointTorqueStep(qddot_target)
    
    def move_joint_torque_step(self,
                               q_target:     np.ndarray | None = None,
                               qdot_target:  np.ndarray | None = None,
                               qddot_target: np.ndarray | None = None,
                               ) -> np.ndarray:
        if qddot_target is not None:
            return super().moveJointTorqueStep(qddot_target)

        if q_target is not None and qdot_target is not None:
            return super().moveJointTorqueStep(q_target, qdot_target)

    def move_joint_torque_cubic(self,
                                q_target: np.ndarray,
                                qdot_target: np.ndarray,
                                q_init: np.ndarray,
                                qdot_init: np.ndarray,
                                current_time: float,
                                init_time: float,
                                duration: float,
                                ) -> np.ndarray:
        return super().moveJointTorqueCubic(q_target,
                                            qdot_target,
                                            q_init,
                                            qdot_init,
                                            current_time,
                                            init_time,
                                            duration,
                                            )

    # ================================ Task space Functions ================================
    def CLIK_step(self,
                  x_target: np.ndarray,
                  xdot_target: np.ndarray,
                  link_name: str,
                  null_qdot: np.ndarray | None = None,
                  ) -> np.ndarray:
        if null_qdot is None:
            return super().CLIKStep(x_target,
                                    xdot_target,
                                    link_name,
                                    )
        else:
            return super().CLIKStep(x_target,
                                    xdot_target,
                                    null_qdot,
                                    link_name,
                                    )

    def CLIK_cubic(self,
                   x_target: np.ndarray,
                   xdot_target: np.ndarray,
                   x_init: np.ndarray,
                   xdot_init: np.ndarray,
                   current_time: float,
                   init_time: float,
                   duration: float,
                   link_name: str,
                   null_qdot: np.ndarray | None = None,
                   ) -> np.ndarray:
        if null_qdot is None:
            return super().CLIKCubic(x_target, 
                                     xdot_target,
                                     x_init,   
                                     xdot_init,
                                     current_time, 
                                     init_time, 
                                     duration,
                                     link_name
                                     )
        else:
            return super().CLIKCubic(x_target, 
                                     xdot_target,
                                     x_init,   
                                     xdot_init,
                                     current_time, 
                                     init_time, 
                                     duration,
                                     null_qdot,
                                     link_name
                                     )


    def OSF(self,
            xddot_target: np.ndarray,
            link_name: str,
            null_torque: np.ndarray | None = None,
            ) -> np.ndarray:
        if null_torque is None:
            return super().OSF(xddot_target, link_name)
        else:
            return super().OSF(xddot_target, null_torque, link_name)

    def OSF_step(self,
                 x_target: np.ndarray,
                 xdot_target: np.ndarray,
                 link_name: str,
                 null_torque: np.ndarray | None = None,
                 ) -> np.ndarray:
        if null_torque is None:
            return super().OSFStep(x_target, xdot_target, link_name)
        else: 
            return super().OSFStep(x_target, xdot_target, null_torque, link_name)

    def OSF_cubic(self,
                  x_target: np.ndarray,
                  xdot_target: np.ndarray,
                  x_init: np.ndarray,
                  xdot_init: np.ndarray,
                  current_time: float,
                  init_time: float,
                  duration: float,
                  link_name: str,
                  null_torque: np.ndarray | None = None,
                  ) -> np.ndarray:
        if null_torque is None:
            return super().OSFCubic(x_target,
                                    xdot_target,
                                    x_init,
                                    xdot_init,
                                    current_time,
                                    init_time,
                                    duration,
                                    link_name,
                                    )
        else:
            return super().OSFCubic(x_target,
                                    xdot_target,
                                    x_init,
                                    xdot_init,
                                    current_time,
                                    init_time,
                                    duration,
                                    null_torque,
                                    link_name,
                                    )
    def QPIK(self, xdot_target: np.ndarray, link_name: str) -> np.ndarray:
        return super().QPIK(xdot_target, link_name)

    def QPIK_step(self, x_target: np.ndarray, xdot_target: np.ndarray, link_name: str) -> np.ndarray:
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
                   ) -> np.ndarray:
        return super().QPIKCubic(x_target,
                                 xdot_target,
                                 x_init,
                                 xdot_init,
                                 current_time,
                                 init_time,
                                 duration,
                                 link_name,
                                 )

    def QPID(self, xddot_target: np.ndarray, link_name: str) -> np.ndarray:
        return super().QPID(xddot_target, link_name)

    def QPID_step( self, x_target: np.ndarray, xdot_target: np.ndarray, link_name: str) -> np.ndarray:
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
                   ) -> np.ndarray:
        return super().QPIDCubic(x_target,
                                 xdot_target,
                                 x_init,
                                 xdot_init,
                                 current_time,
                                 init_time,
                                 duration,
                                 link_name,
                                 )