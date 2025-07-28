#pragma once
#include "robot_data/mobile_manipulator/base.h"
#include "math_type_define.h"
#include "QP/manipulator_IK.h"
#include "QP/manipulator_ID.h"
#include "QP/mobile_manipulator_IK.h"
#include "QP/mobile_manipulator_ID.h"

namespace RobotController
{
    namespace  MobileManipulator
    {
        class MobileManipulatorBase
        {
            public:
                MobileManipulatorBase(const double& dt,
                                      std::shared_ptr<RobotData::MobileManipulator::MobileManipulatorBase> robot_data);
                
                virtual void setManipulatorJointGain(const VectorXd& Kp, 
                                                     const VectorXd& Kv);
                virtual void setTaskGain(const VectorXd& Kp, 
                                         const VectorXd& Kv);

                virtual VectorXd moveManipulatorJointPositionCubic(const VectorXd& q_mani_target,
                                                                   const VectorXd& qdot_mani_target,
                                                                   const VectorXd& q_mani_init,
                                                                   const VectorXd& qdot_mani_init,
                                                                   const double& current_time,
                                                                   const double& init_time,
                                                                   const double& duration);

                virtual VectorXd moveManipulatorJointTorqueStep(const VectorXd& qddot_mani_target);
                virtual VectorXd moveManipulatorJointTorqueStep(const VectorXd& q_mani_target,
                                                                const VectorXd& qdot_mani_target);

                virtual VectorXd moveManipulatorJointTorqueCubic(const VectorXd& q_mani_target,
                                                                 const VectorXd& qdot_mani_target,
                                                                 const VectorXd& q_mani_init,
                                                                 const VectorXd& qdot_mani_init,
                                                                 const double& current_time,
                                                                 const double& init_time,
                                                                 const double& duration);

                virtual void QPIK(const VectorXd& xdot_target,
                                  const std::string& link_name,
                                  VectorXd& opt_qdot_mobile,
                                  VectorXd& opt_qdot_manipulator);

                virtual void QPIKStep(const Affine3d& x_target, 
                                      const VectorXd& xdot_target,
                                      const std::string& link_name,
                                      VectorXd& opt_qdot_mobile,
                                      VectorXd& opt_qdot_manipulator);


                virtual void QPIKCubic(const Affine3d& x_target,
                                       const VectorXd& xdot_target,
                                       const Affine3d& x_init,
                                       const VectorXd& xdot_init,
                                       const double& current_time,
                                       const double& init_time,
                                       const double& duration,
                                       const std::string& link_name,
                                       VectorXd& opt_qdot_mobile,
                                       VectorXd& opt_qdot_manipulator);


                virtual void QPID(const VectorXd& xddot_target,
                                  const std::string& link_name,
                                  VectorXd& opt_qddot_mobile,
                                  VectorXd& opt_torque_manipulator);

                virtual void QPIDStep(const Affine3d& x_target, 
                                      const VectorXd& xdot_target,
                                      const std::string& link_name,
                                      VectorXd& opt_qddot_mobile,
                                      VectorXd& opt_torque_manipulator);


                virtual void QPIDCubic(const Affine3d& x_target,
                                       const VectorXd& xdot_target,
                                       const Affine3d& x_init,
                                       const VectorXd& xdot_init,
                                       const double& current_time,
                                       const double& init_time,
                                       const double& duration,
                                       const std::string& link_name,
                                       VectorXd& opt_qddot_mobile,
                                       VectorXd& opt_torque_manipulator);
   
                
            protected:
                double dt_;
                int dof_;
                int mani_dof_;
                int mobi_dof_;
                int actuator_dof_;
                std::shared_ptr<RobotData::MobileManipulator::MobileManipulatorBase> robot_data_;

                VectorXd Kp_task_;
                VectorXd Kv_task_;

                VectorXd Kp_mani_joint_;
                VectorXd Kv_mani_joint_;

                std::unique_ptr<QP::MobileManipulatorIK> QP_moma_IK_;
                std::unique_ptr<QP::MobileManipulatorID> QP_moma_ID_;
        };
    } // namespace MobileManipulator
} // namespace RobotController