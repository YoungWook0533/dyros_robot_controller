#ifndef FR3_ROBOT_DATA_HPP
#define FR3_ROBOT_DATA_HPP

#include <string>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <Eigen/Dense>
using namespace Eigen;

namespace FR3Controller
{
    typedef Eigen::Matrix<double,6,6> Matrix6d;
    typedef Eigen::Matrix<double,6,1> Vector6d;

    class RobotData
    {
        public:
            // @brief Constructor that loads robot model from URDF.
            // @param urdf_path Path to URDF file.
            RobotData(const std::string& urdf_path);

            // @brief Destructor.
            ~RobotData();

            // @brief Update robot state (joint positions, velocities, external torques).
            // @param q Joint position vector.
            // @param qdot Joint velocity vector.
            // @param tau_ext External torque vector.
            // @return True if successful.
            bool updateState(const VectorXd& q, const VectorXd& qdot, const VectorXd& tau_ext);

            // @brief Get list of joint names.
            // @return Vector of joint name strings.
            std::vector<std::string> getJointNames(){return joint_names_;}

            // @brief Compute pose (4x4 transformation matrix) of a given link.
            // @param q Joint position vector.
            // @param link_name Link name (default: end-effector).
            // @return 4x4 pose matrix.
            Matrix4d computePose(const VectorXd& q, const std::string& link_name=ee_name_);

            // @brief Compute spatial Jacobian of a given link.
            // @param q Joint position vector.
            // @param link_name Link name (default: end-effector).
            // @return 6 x nq Jacobian matrix.
            MatrixXd computeJacobian(const VectorXd& q, const std::string& link_name=ee_name_);

            // @brief Compute time derivative of Jacobian of a given link.
            // @param q Joint position vector.
            // @param qdot Joint velocity vector.
            // @param link_name Link name (default: end-effector).
            // @return 6 x nq time derivative of Jacobian.
            MatrixXd computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);

            // @brief Compute spatial velocity of a given link.
            // @param q Joint position vector.
            // @param qdot Joint velocity vector.
            // @param link_name Link name (default: end-effector).
            // @return 6D velocity vector.
            Vector6d computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);

            // @brief Compute joint-space mass (inertia) matrix.
            // @param q Joint position vector.
            // @return nq x nq mass matrix.
            MatrixXd computeMassMatrix(const VectorXd& q);

            // @brief Compute Coriolis and centrifugal forces.
            // @param q Joint position vector.
            // @param qdot Joint velocity vector.
            // @return nq-dimensional Coriolis vector.
            VectorXd computeCoriolis(const VectorXd& q, const VectorXd& qdot);

            // @brief Compute gravity torque vector.
            // @param q Joint position vector.
            // @return nq-dimensional gravity vector.
            VectorXd computeGravity(const VectorXd& q);

            // @brief Compute nonlinear effects (gravity + Coriolis).
            // @param q Joint position vector.
            // @param qdot Joint velocity vector.
            // @return nq-dimensional nonlinear effects vector.
            VectorXd computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot);

            // @brief Compute task-space mass matrix for a given link.
            // @param q Joint position vector.
            // @param link_name Link name (default: end-effector).
            // @return 6x6 task mass matrix.
            Matrix6d computeTaskMassMatrix(const VectorXd& q, const std::string& link_name=ee_name_);

            // @brief Compute task-space Coriolis force for a given link.
            // @param q Joint position vector.
            // @param qdot Joint velocity vector.
            // @param link_name Link name (default: end-effector).
            // @return 6D task-space Coriolis vector.
            Vector6d computeTaskCoriolis(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);

            // @brief Compute task-space gravity force for a given link.
            // @param q Joint position vector.
            // @param link_name Link name (default: end-effector).
            // @return 6D task-space gravity vector.
            Vector6d computeTaskGravity(const VectorXd& q, const std::string& link_name=ee_name_);

            // @brief Compute task-space nonlinear effects (gravity + Coriolis) for a given link.
            // @param q Joint position vector.
            // @param qdot Joint velocity vector.
            // @param link_name Link name (default: end-effector).
            // @return 6D task-space nonlinear effects vector.
            Vector6d computeTaskNonlinearEffects(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);

            // ---------------- Getters ----------------

            // @brief Get current joint positions.
            // @return nq-dimensional joint position vector.
            VectorXd getJointPosition(){return q_;}

            // @brief Get current joint velocities.
            // @return nq-dimensional joint velocity vector.
            VectorXd getJointVelocity(){return qdot_;}

            // @brief Get current external joint torques.
            // @return nq-dimensional external torque vector.
            VectorXd getExtTorque(){return tau_ext_;}

            // @brief Get cached pose of a given link.
            // @param link_name Link name (default: end-effector).
            // @return 4x4 pose matrix.
            Matrix4d getPose(const std::string& link_name=ee_name_);

            // @brief Get cached Jacobian of a given link.
            // @param link_name Link name (default: end-effector).
            // @return 6 x nq Jacobian matrix.
            MatrixXd getJacobian(const std::string& link_name=ee_name_);

            // @brief Get cached time derivative of Jacobian of a given link.
            // @param link_name Link name (default: end-effector).
            // @return 6 x nq time derivative of Jacobian.
            MatrixXd getJacobianTimeVariation(const std::string& link_name=ee_name_);

            // @brief Get cached velocity of a given link.
            // @param link_name Link name (default: end-effector).
            // @return 6D velocity vector.
            Vector6d getVelocity(const std::string& link_name=ee_name_);

            // @brief Get cached joint-space mass matrix.
            // @return nq x nq mass matrix.
            MatrixXd getMassMatrix();

            // @brief Get cached Coriolis and centrifugal forces.
            // @return nq-dimensional Coriolis vector.
            VectorXd getCoriolis();

            // @brief Get cached gravity vector.
            // @return nq-dimensional gravity vector.
            VectorXd getGravity();

            // @brief Get cached nonlinear effects.
            // @return nq-dimensional nonlinear effects vector.
            VectorXd getNonlinearEffects();

            // @brief Get cached task-space mass matrix.
            // @param link_name Link name (default: end-effector).
            // @return 6x6 task-space mass matrix.
            Matrix6d getTaskMassMatrix(const std::string& link_name=ee_name_);

            // @brief Get cached task-space Coriolis force.
            // @param link_name Link name (default: end-effector).
            // @return 6D task-space Coriolis vector.
            Vector6d getTaskCoriolis(const std::string& link_name=ee_name_);

            // @brief Get cached task-space gravity force.
            // @param link_name Link name (default: end-effector).
            // @return 6D task-space gravity vector.
            Vector6d getTaskGravity(const std::string& link_name=ee_name_);

            // @brief Get cached task-space nonlinear effects.
            // @param link_name Link name (default: end-effector).
            // @return 6D task-space nonlinear effects vector.
            Vector6d getTaskNonlinearEffects(const std::string& link_name=ee_name_);

        private:
            // @brief Internal function to update forward kinematics.
            // @param q Joint position vector.
            // @param qdot Joint velocity vector.
            // @return True if successful.
            bool updateKinematics(const VectorXd& q, const VectorXd& qdot);

            // @brief Internal function to update dynamics terms.
            // @param q Joint position vector.
            // @param qdot Joint velocity vector.
            // @return True if successful.
            bool updateDynamics(const VectorXd& q, const VectorXd& qdot);

            // pinocchio data
            pinocchio::Model model_;
            pinocchio::Data data_;

            std::vector<std::string> joint_names_;

            // Joint space state
            VectorXd q_;    // joint angle
            VectorXd qdot_; // joint velocity
            VectorXd tau_ext_;  // joint torque

            // Task space state
            static constexpr const char* ee_name_ = "fr3_link8"; // end-effector link name
            Matrix4d x_;     // pose of EE
            Vector6d xdot_;  // velocity of EE
            MatrixXd J_;     // jacobian of EE
            MatrixXd Jdot_;  // time derivative of jacobian of EE

            // Joint space Dynamics
            MatrixXd M_;     // inertia matrix
            MatrixXd M_inv_; // inverse of inertia matrix
            VectorXd g_;     // gravity forces
            VectorXd c_;     // centrifugal and coriolis forces
            VectorXd NLE_;   // nonlinear effects ( g_ + c_ )

            // Task space Dynamics
            Matrix6d M_ee_;     // inertia matrix
            Matrix6d M_ee_inv_; // inverse of inertia matrix
            Vector6d g_ee_;     // gravity forces
            Vector6d c_ee_;     // centrifugal and coriolis forces
            Vector6d NLE_ee_;   // nonlinear effects ( g_ + c_ )
    };
}

#endif // FR3_ROBOT_DATA_HPP