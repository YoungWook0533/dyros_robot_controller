from typing import Tuple
import numpy as np
from dyros_robot_controller import KinematicParam, JointIndex, ActuatorIndex
import dyros_robot_controller_cpp_wrapper as drc


class RobotData(drc.MobileManipulatorRobotData):
    def __init__(self,
                mobile_param: KinematicParam,
                joint_idx: JointIndex,
                actuator_idx: ActuatorIndex,
                urdf_path: str,
                srdf_path: str="",
                packages_path: str="",
                ):
        self._mobile_kine_param = mobile_param
        self._joint_idx = joint_idx
        self._actuator_idx = actuator_idx
        super().__init__(self._mobile_kine_param.cpp(),
                         self._joint_idx.cpp(),
                         self._actuator_idx.cpp(),
                         urdf_path,
                         srdf_path,
                         packages_path,
                         )
        
    def get_verbose(self) -> str:
        return super().getVerbose()

    def update_state(self,
                     q_virtual: np.ndarray,
                     q_mobile: np.ndarray,
                     q_mani: np.ndarray,
                     qdot_virtual: np.ndarray,
                     qdot_mobile: np.ndarray,
                     qdot_mani: np.ndarray,
                     ) -> bool:
        return super().updateState(q_virtual,
                                   q_mobile,
                                   q_mani,
                                   qdot_virtual,
                                   qdot_mobile,
                                   qdot_mani,
                                   )

    # ================================ Compute Functions ================================
    # Wholebody joint space
    def compute_mass_matrix(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray) -> np.ndarray:
        return super().computeMassMatrix(q_virtual, q_mobile, q_mani)

    def compute_gravity(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray) -> np.ndarray:
        return super().computeGravity(q_virtual, q_mobile, q_mani)

    def compute_coriolis(self,
                         q_virtual: np.ndarray,
                         q_mobile: np.ndarray,
                         q_mani: np.ndarray,
                         qdot_virtual: np.ndarray,
                         qdot_mobile: np.ndarray,
                         qdot_mani: np.ndarray,
                         ) -> np.ndarray:
        return super().computeCoriolis(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani)

    def compute_nonlinear_effects(self,
                                  q_virtual: np.ndarray,
                                  q_mobile: np.ndarray,
                                  q_mani: np.ndarray,
                                  qdot_virtual: np.ndarray,
                                  qdot_mobile: np.ndarray,
                                  qdot_mani: np.ndarray,
                                  ) -> np.ndarray:
        return super().computeNonlinearEffects(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani)

    def compute_mass_matrix_actuated(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray) -> np.ndarray:
        return super().computeMassMatrixActuated(q_virtual, q_mobile, q_mani)

    def compute_gravity_actuated(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray) -> np.ndarray:
        return super().computeGravityActuated(q_virtual, q_mobile, q_mani)

    def compute_coriolis_actuated(self,
                                  q_virtual: np.ndarray,
                                  q_mobile: np.ndarray,
                                  q_mani: np.ndarray,
                                  qdot_mobile: np.ndarray,
                                  qdot_mani: np.ndarray,) -> np.ndarray:
        return super().computeCoriolisActuated(q_virtual, q_mobile, q_mani, qdot_mobile, qdot_mani)

    def compute_nonlinear_effects_actuated(self,
                                           q_virtual: np.ndarray,
                                           q_mobile: np.ndarray,
                                           q_mani: np.ndarray,
                                           qdot_mobile: np.ndarray,
                                           qdot_mani: np.ndarray,
                                           ) -> np.ndarray:
        return super().computeNonlinearEffectsActuated(q_virtual, q_mobile, q_mani, qdot_mobile, qdot_mani)

    # Wholebody task space
    def compute_pose(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray, link_name: str) -> np.ndarray:
        return super().computePose(q_virtual, q_mobile, q_mani, link_name)

    def compute_jacobian(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray, link_name: str) -> np.ndarray:
        return super().computeJacobian(q_virtual, q_mobile, q_mani, link_name)

    def compute_jacobian_time_variation(self,
                                        q_virtual: np.ndarray,
                                        q_mobile: np.ndarray,
                                        q_mani: np.ndarray,
                                        qdot_virtual: np.ndarray,
                                        qdot_mobile: np.ndarray,
                                        qdot_mani: np.ndarray,
                                        link_name: str,
                                        ) -> np.ndarray:
        return super().computeJacobianTimeVariation(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani, link_name)

    def compute_velocity(self,
                         q_virtual: np.ndarray,
                         q_mobile: np.ndarray,
                         q_mani: np.ndarray,
                         qdot_virtual: np.ndarray,
                         qdot_mobile: np.ndarray,
                         qdot_mani: np.ndarray,
                         link_name: str,
                         ) -> np.ndarray:
        return super().computeVelocity(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani, link_name,)

    def compute_min_distance(self,
                             q_virtual: np.ndarray,
                             q_mobile: np.ndarray,
                             q_mani: np.ndarray,
                             qdot_virtual: np.ndarray,
                             qdot_mobile: np.ndarray,
                             qdot_mani: np.ndarray,
                             with_grad: bool,
                             with_graddot: bool,
                             verbose: bool = False,
                             )->Tuple[float, np.ndarray, np.ndarray]:
        min_result = super().computeMinDistance(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani, with_grad, with_graddot, verbose)
        return min_result.distance, min_result.grad, min_result.grad_dot
    
    def compute_selection_matrix(self, q_virtual: np.ndarray, q_mobile: np.ndarray) -> np.ndarray:
        return super().computeSelectionMatrix(q_virtual, q_mobile)

    def compute_jacobian_actuated(self, q_virtual: np.ndarray, q_mobile: np.ndarray, q_mani: np.ndarray, link_name: str) -> np.ndarray:
        return  super().computeJacobianActuated(q_virtual, q_mobile, q_mani, link_name)

    def compute_jacobian_time_variation_actuated(self,
                                                 q_virtual: np.ndarray,
                                                 q_mobile: np.ndarray,
                                                 q_mani: np.ndarray,
                                                 qdot_virtual: np.ndarray,
                                                 qdot_mobile: np.ndarray,
                                                 qdot_mani: np.ndarray,
                                                 link_name: str,
                                                 ) -> np.ndarray:
        return super().computeJacobianTimeVariationActuated(q_virtual, q_mobile, q_mani, qdot_virtual, qdot_mobile, qdot_mani, link_name)

    # Manipulator taskspace
    def compute_manipulability(self,
                               q_mani: np.ndarray,
                               qdot_mani: np.ndarray,
                               with_grad: bool,
                               with_graddot: bool,
                               link_name: str,
                               ):
        mani_result = super().computeManipulability(q_mani, qdot_mani, with_grad, with_graddot, link_name)
        return mani_result.manipulability, mani_result.grad, mani_result.grad_dot

    # Mobile
    def compute_mobile_FK_jacobian(self, q_mobile: np.ndarray) -> np.ndarray:
        return super().computeMobileFKJacobian(q_mobile)

    def compute_mobile_base_vel(self, q_mobile: np.ndarray, qdot_mobile: np.ndarray) -> np.ndarray:
        return super().computeMobileBaseVel(q_mobile, qdot_mobile)

    # ================================ Get Functions ================================
    def get_dof(self) -> int:
        return super().getDof()
    
    def get_actuator_dof(self) -> int:
        return int(super().getActuatordDof())

    def get_manipulator_dof(self) -> int:
        return int(super().getManipulatorDof())

    def get_mobile_dof(self) -> int:
        return int(super().getMobileDof())

    def get_joint_index(self) -> JointIndex:
        return self._joint_idx

    def get_actuator_index(self) -> ActuatorIndex:
        return self._actuator_idx
    
    def get_joint_position(self) -> np.ndarray:
        return super().getJointPosition()

    def get_joint_velocity(self) -> np.ndarray:
        return super().getJointVelocity()

    def get_joint_position_limit(self) -> Tuple[np.ndarray, np.ndarray]:
        return super().getJointPositionLimit()

    def get_joint_velocity_limit(self) -> Tuple[np.ndarray, np.ndarray]:
        return super().getJointVelocityLimit()

    def get_mobile_joint_position(self) -> np.ndarray:
        return super().getMobileJointPosition()

    def get_virtual_joint_position(self) -> np.ndarray:
        return super().getVirtualJointPosition()

    def get_manipulator_joint_position(self) -> np.ndarray:
        return super().getManiJointPosition()

    def get_joint_velocity_actuated(self) -> np.ndarray:
        return super().getJointVelocityActuated()

    def get_mobile_joint_velocity(self) -> np.ndarray:
        return super().getMobileJointVelocity()

    def get_virtual_joint_velocity(self) -> np.ndarray:
        return super().getVirtualJointVelocity()

    def get_manipulator_joint_velocity(self) -> np.ndarray:
        return super().getManiJointVelocity()

    def get_joint_position_actuated(self) -> np.ndarray:
        return super().getJointPositionActuated()

    #  Wholebody joint space
    def get_mass_matrix(self) -> np.ndarray:
        return super().getMassMatrix()

    def get_mass_matrix_inv(self) -> np.ndarray:
        return super().getMassMatrixInv()

    def get_coriolis(self) -> np.ndarray:
        return super().getCoriolis()

    def get_gravity(self) -> np.ndarray:
        return super().getGravity()

    def get_nonlinear_effects(self) -> np.ndarray:
        return super().getNonlinearEffects()
    
    def get_mass_matrix_actuated(self) -> np.ndarray:
        return super().getMassMatrixActuated()

    def get_mass_matrix_actuated_inv(self) -> np.ndarray:
        return super().getMassMatrixActuatedInv()

    def get_gravity_actuated(self) -> np.ndarray:
        return super().getGravityActuated()

    def get_coriolis_actuated(self) -> np.ndarray:
        return super().getCoriolisActuated()

    def get_nonlinear_effects_actuated(self) -> np.ndarray:
        return super().getNonlinearEffectsActuated()

    # Wholebody task space
    def get_pose(self, link_name: str) -> np.ndarray:
        return super().getPose(link_name)

    def get_jacobian(self, link_name: str) -> np.ndarray:
        return super().getJacobian(link_name)

    def get_jacobian_time_variation(self, link_name: str) -> np.ndarray:
        return super().getJacobianTimeVariation(link_name)

    def get_velocity(self, link_name: str) -> np.ndarray:
        return super().getVelocity(link_name)
    
    def get_min_distance(self, with_grad: bool, with_graddot: bool):
        min_result = super().getMinDistance(with_grad, with_graddot)
        return min_result.distance, min_result.grad, min_result.grad_dot
    
    def get_jacobian_actuated(self, link_name: str) -> np.ndarray:
        return super().getJacobianActuated(link_name)

    def get_jacobian_actuated_time_variation(self, link_name: str) -> np.ndarray:
        return super().getJacobianActuatedTimeVariation(link_name)

    def get_selection_matrix(self) -> np.ndarray:
        return super().getSelectionMatrix()

    def get_manipulability(self, with_grad: bool, with_graddot: bool, link_name:str):
        min_result = super().getManipulability(with_grad, with_graddot, link_name)
        return min_result.manipulability, min_result.grad, min_result.grad_dot
    
    # mobile getters
    def get_mobile_FK_jacobian(self) -> np.ndarray:
        return super().getMobileFKJacobian()

    def get_mobile_base_vel(self) -> np.ndarray:
        return super().getMobileBaseVel()