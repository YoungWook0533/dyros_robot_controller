#pragma once
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "dyros_robot_controller/manipulator/QP_IK.h"
#include "dyros_robot_controller/manipulator/QP_ID.h"

namespace drc
{
    namespace Manipulator
    {
        class RobotController
        {
            public:
                RobotController(const double& dt,
                                std::shared_ptr<Manipulator::RobotData> robot_data);
                
                virtual void setJointGain(const VectorXd& Kp, 
                                          const VectorXd& Kv);
                virtual void setTaskGain(const VectorXd& Kp, 
                                         const VectorXd& Kv);
                
                // ================================ Joint space Functions ================================
                virtual VectorXd moveJointPositionCubic(const VectorXd& q_target,
                                                        const VectorXd& qdot_target,
                                                        const VectorXd& q_init,
                                                        const VectorXd& qdot_init,
                                                        const double& current_time,
                                                        const double& init_time,
                                                        const double& duration);
                virtual VectorXd moveJointVelocityCubic(const VectorXd& q_target,
                                                        const VectorXd& qdot_target,
                                                        const VectorXd& q_init,
                                                        const VectorXd& qdot_init,
                                                        const double& current_time,
                                                        const double& init_time,
                                                        const double& duration);

                virtual VectorXd moveJointTorqueStep(const VectorXd& qddot_target);
                virtual VectorXd moveJointTorqueStep(const VectorXd& q_target,
                                                     const VectorXd& qdot_target);

                virtual VectorXd moveJointTorqueCubic(const VectorXd& q_target,
                                                      const VectorXd& qdot_target,
                                                      const VectorXd& q_init,
                                                      const VectorXd& qdot_init,
                                                      const double& current_time,
                                                      const double& init_time,
                                                      const double& duration);
                                                
                // ================================ Task space Functions ================================
                virtual VectorXd CLIKStep(const Affine3d& x_target, 
                                          const VectorXd& xdot_target,
                                          const VectorXd& null_qdot,
                                          const std::string& link_name);
                virtual VectorXd CLIKStep(const Affine3d& x_target, 
                                          const VectorXd& xdot_target,
                                          const std::string& link_name);
                                  
                virtual VectorXd CLIKCubic(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const Affine3d& x_init,
                                           const VectorXd& xdot_init,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const VectorXd& null_qdot,
                                           const std::string& link_name);
                virtual VectorXd CLIKCubic(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const Affine3d& x_init,
                                           const VectorXd& xdot_init,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const std::string& link_name);

                virtual VectorXd OSF(const VectorXd& xddot_target, 
                                     const VectorXd& null_torque,
                                     const std::string& link_name);
                virtual VectorXd OSF(const VectorXd& xddot_target,
                                     const std::string& link_name);
                
                virtual VectorXd OSFStep(const Affine3d& x_target, 
                                         const VectorXd& xdot_target,
                                         const VectorXd& null_torque,
                                         const std::string& link_name);
                virtual VectorXd OSFStep(const Affine3d& x_target, 
                                         const VectorXd& xdot_target,
                                         const std::string& link_name);

                virtual VectorXd OSFCubic(const Affine3d& x_target,
                                          const VectorXd& xdot_target,
                                          const Affine3d& x_init,
                                          const VectorXd& xdot_init,
                                          const double& current_time,
                                          const double& init_time,
                                          const double& duration,
                                          const VectorXd& null_torque,
                                          const std::string& link_name);
                virtual VectorXd OSFCubic(const Affine3d& x_target,
                                          const VectorXd& xdot_target,
                                          const Affine3d& x_init,
                                          const VectorXd& xdot_init,
                                          const double& current_time,
                                          const double& init_time,
                                          const double& duration,
                                          const std::string& link_name);

                virtual VectorXd QPIK(const VectorXd& xdot_target,
                                      const std::string& link_name);

                virtual VectorXd QPIKStep(const Affine3d& x_target, 
                                          const VectorXd& xdot_target,
                                          const std::string& link_name);

                virtual VectorXd QPIKCubic(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const Affine3d& x_init,
                                           const VectorXd& xdot_init,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const std::string& link_name);

                virtual VectorXd QPID(const VectorXd& xddot_target,
                                      const std::string& link_name);

                virtual VectorXd QPIDStep(const Affine3d& x_target, 
                                          const VectorXd& xdot_target,
                                          const std::string& link_name);

                virtual VectorXd QPIDCubic(const Affine3d& x_target,
                                           const VectorXd& xdot_target,
                                           const Affine3d& x_init,
                                           const VectorXd& xdot_init,
                                           const double& current_time,
                                           const double& init_time,
                                           const double& duration,
                                           const std::string& link_name);


            protected:
                double dt_;
                int dof_;
                std::shared_ptr<Manipulator::RobotData> robot_data_;

                VectorXd Kp_task_;
                VectorXd Kv_task_;

                VectorXd Kp_joint_;
                VectorXd Kv_joint_;

                std::unique_ptr<Manipulator::QPIK> QP_mani_IK_;
                std::unique_ptr<Manipulator::QPID> QP_mani_ID_;

        };
    } // namespace Manipulator
} // namespace drc