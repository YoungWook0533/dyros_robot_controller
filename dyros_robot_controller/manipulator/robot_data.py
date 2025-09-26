from typing import Tuple
import numpy as np
import dyros_robot_controller_cpp_wrapper as drc


class RobotData(drc.ManipulatorRobotData):
    def __init__(self, urdf_path: str, srdf_path: str, packages_path: str):
        super().__init__(urdf_path, srdf_path, packages_path)
        
    def get_verbose(self) -> str:
        return super().getVerbose()

    def update_state(self, q: np.ndarray, qdot: np.ndarray) -> bool:
        return super().updateState(q, qdot)

    # ================================ Compute Functions ================================
    # Joint space
    def compute_mass_matrix(self, q: np.ndarray) -> np.ndarray:
        return super().computeMassMatrix(q)

    def compute_gravity(self, q: np.ndarray) -> np.ndarray:
        return super().computeGravity(q)

    def compute_coriolis(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        return super().computeCoriolis(q, qdot)

    def compute_nonlinear_effects(self, q: np.ndarray, qdot: np.ndarray) -> np.ndarray:
        return super().computeNonlinearEffects(q, qdot)

    # Task space
    def compute_pose(self, q: np.ndarray, link_name: str) -> np.ndarray:
        return super().computePose(q, link_name)

    def compute_jacobian(self, q: np.ndarray, link_name: str) -> np.ndarray:
        return super().computeJacobian(q, link_name)

    def compute_jacobian_time_variation(self, q: np.ndarray, qdot: np.ndarray, link_name: str) -> np.ndarray:
        return super().computeJacobianTimeVariation(q, qdot, link_name)

    def compute_velocity(self, q: np.ndarray, qdot: np.ndarray, link_name: str) -> np.ndarray:
        return super().computeVelocity(q, qdot, link_name)
    
    def compute_min_distance(self, q: np.ndarray, qdot: np.ndarray, with_grad: bool, with_graddot: bool) -> Tuple[float, np.ndarray, np.ndarray]:
        min_result = super().computeMinDistance(q, qdot, with_grad, with_graddot)
        return min_result.distance, min_result.grad, min_result.grad_dot
    
    def compute_manipulability(self, q: np.ndarray, qdot: np.ndarray, with_grad: bool, with_graddot: bool, link_name: str) -> Tuple[float, np.ndarray, np.ndarray]:
        mani_result = super().computeManipulability(q, qdot, with_grad, with_graddot, link_name)
        return mani_result.manipulability, mani_result.grad, mani_result.grad_dot

    # ================================ Get Functions ================================
    def get_dof(self) -> int:
        return super().getDof()

    # Joint space
    def get_joint_position(self) -> np.ndarray:
        return super().getJointPosition()

    def get_joint_velocity(self) -> np.ndarray:
        return super().getJointVelocity()

    def get_joint_position_limit(self) -> Tuple[np.ndarray, np.ndarray]:
        return super().getJointPositionLimit()

    def get_joint_velocity_limit(self) -> Tuple[np.ndarray, np.ndarray]:
        return super().getJointVelocityLimit()

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

    # Task space
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
    
    def get_manipulability(self, with_grad: bool, with_graddot: bool, link_name:str):
        min_result = super().getManipulability(with_grad, with_graddot, link_name)
        return min_result.manipulability, min_result.grad, min_result.grad_dot
