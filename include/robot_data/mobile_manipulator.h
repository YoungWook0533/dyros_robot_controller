#pragma once
#include "robot_data/robot_data_base.h"
#include <omp.h> 

using namespace Eigen;

class RobotDataMobileManipulator : public RobotDataBase
{
    public:
        static inline std::string ee_name_;      // end-effector link name

        struct JointIndex
        {
            const int virtual_start;
            const int mani_start;
            const int mobi_start;
        };

        struct ActuatorIndex
        {
            const int mani_start;
            const int mobi_start;
        };

        RobotDataMobileManipulator(const std::string& urdf_path, 
                                   const int& mani_dof, 
                                   const int& mobi_dof,
                                   const JointIndex& joint_idx,
                                   const ActuatorIndex& actuator_idx,
                                   const std::string& ee_name, 
                                   const bool verbose=false);
        
        RobotDataMobileManipulator(const std::string& urdf_path,
                                   const std::string& srdf_path, 
                                   const int& mani_dof, 
                                   const int& mobi_dof,
                                   const JointIndex& joint_idx,
                                   const ActuatorIndex& actuator_idx,
                                   const std::string& ee_name, 
                                   const bool verbose=false);
                                   

        Affine3d computePose(const VectorXd& q, const std::string& link_name=ee_name_);
        MatrixXd computeJacobian(const VectorXd& q, const std::string& link_name=ee_name_);
        MatrixXd computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);
        VectorXd computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);
        MatrixXd computeMassMatrix(const VectorXd& q);
        VectorXd computeGravity(const VectorXd& q);
        VectorXd computeCoriolis(const VectorXd& q, const VectorXd& qdot);
        VectorXd computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot);

        virtual MatrixXd computeMobileFKJacobian(const VectorXd& q_mobile) = 0;
        virtual MatrixXd computeSelectionMatrix(const VectorXd& q) = 0;
        virtual MatrixXd computeSelectionMatrixTimeVariation(const VectorXd& q, const VectorXd& qdot) = 0;
        MatrixXd computeJacobianActuated(const VectorXd& q_act, const std::string& link_name);
        MatrixXd computeMassMatrixActuated(const VectorXd& q_act);
        VectorXd computeGravityActuated(const VectorXd& q_act);
        VectorXd computeCoriolisActuated(const VectorXd& q_act, const VectorXd& qdot_act);
        VectorXd computeNonlinearEffectsActuated(const VectorXd& q_act, const VectorXd& qdot_act);

        double computeManipulability(const VectorXd& q_mani, const VectorXd& qdot_mani, VectorXd* grad, VectorXd* grad_dot, const std::string& link_name=ee_name_);
        
        double getDof() const {return total_dof_;}
        double getActuatordDof() const {return actuated_dof_;}
        double getManipulatorDof() const {return mani_dof_;}
        double getMobileDof() const {return mobi_dof_;}
        JointIndex getJointIndex() const {return joint_idx_;}
        ActuatorIndex getActuatorIndex() const {return actuator_idx_;}
        Affine3d getPose(const std::string& link_name=ee_name_) const;
        MatrixXd getJacobian(const std::string& link_name=ee_name_);
        MatrixXd getJacobianTimeVariation(const std::string& link_name=ee_name_); 
        VectorXd getVelocity(const std::string& link_name=ee_name_);
        MatrixXd getMassMatrix() const {return M_;}
        MatrixXd getMassMatrixInv() const {return M_inv_;}
        VectorXd getCoriolis() const {return c_;}
        VectorXd getGravity() const {return g_;}
        VectorXd getNonlinearEffects() const {return NLE_;}

        VectorXd getJointVelocityActuated() const {return qdot_actuated_;}
        VectorXd getJointPositionActuated() const {return q_actuated_;}
        MatrixXd getMobileFKJacobian() const {return J_mobile_;}
        MatrixXd getSelectionMatrix(){return S_;}
        MatrixXd getJacobianActuated(const std::string& link_name=ee_name_);
        MatrixXd getJacobianActuatedTimeVariation(const std::string& link_name=ee_name_);
        MatrixXd getMassMatrixActuated() const {return M_actuated_;}
        VectorXd getGravityActuated() const {return g_actuated_;}
        VectorXd getCoriolisActuated() const {return c_actuated_;}
        VectorXd getNonlinearEffectsActuated() const {return NLE_actuated_;}

        double getManipulability(VectorXd* grad, VectorXd* grad_dot, const std::string& link_name=ee_name_);

    protected:
        bool updateKinematics(const VectorXd& q, const VectorXd& qdot);
        bool updateDynamics(const VectorXd& q, const VectorXd& qdot);

        int mani_dof_;
        int mobi_dof_;
        int virtual_dof_{3};
        int total_dof_;
        int actuated_dof_;

        JointIndex joint_idx_;
        ActuatorIndex actuator_idx_;

        // Selection Matrix for virtual joints (Actuated Joint Velocity -> Total Joint Velocity)
        MatrixXd S_;
        MatrixXd Sdot_;
        MatrixXd J_mobile_; // wheel_vel -> mobile_base_vel (wrt base frame) 
        
        // Actuated Joint state
        VectorXd q_actuated_;        // joint angle
        VectorXd qdot_actuated_;     // joint velocity
        
        // Task space state
        pinocchio::FrameIndex ee_index_;  // end-effector link index
        Affine3d x_;                      // pose of EE
        VectorXd xdot_;                   // velocity of EE
        MatrixXd J_;                      // jacobian of EE
        MatrixXd Jdot_;                   // time derivative of jacobian of EE
        
        // Actuated Task space state
        MatrixXd J_actuated_;     // jacobian of EE
        MatrixXd J_actuated_dot_; // jacobian of EE
        
        // Joint space Dynamics
        MatrixXd M_;                 // inertia matrix
        MatrixXd M_inv_;             // inverse of inertia matrix
        VectorXd g_;                 // gravity forces
        VectorXd c_;                 // centrifugal and coriolis forces
        VectorXd NLE_;               // nonlinear effects ( g_ + c_ )
        
        // ActuatedJoint space Dynamics
        MatrixXd M_actuated_;     // inertia matrix
        MatrixXd M_inv_actuated_; // inverse of inertia matrix
        VectorXd g_actuated_;     // gravity forces
        VectorXd c_actuated_;     // centrifugal and coriolis forces
        VectorXd NLE_actuated_;   // nonlinear effects ( g_ + c_ )
    };