#pragma once
#include <string>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Dense>
#include <math.h>

// #include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include "math_type_define.h"

#include "dyros_robot_controller/type_define.h"
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "dyros_robot_controller/mobile/robot_data.h"

using namespace Eigen;

namespace drc
{
    namespace MobileManipulator
    {
        class RobotData : public Manipulator::RobotData, public Mobile::RobotData
        {
            public:
                RobotData(const Mobile::KinematicParam& mobile_param,
                          const std::string& urdf_path,
                          const std::string& srdf_path, 
                          const std::string& packages_path, 
                          const JointIndex& joint_idx,
                          const ActuatorIndex& actuator_idx);

                using Manipulator::RobotData::getVerbose;
                using Mobile::RobotData::getVerbose;
                std::string getVerbose() const;
                                               
                using Manipulator::RobotData::updateState; 
                bool updateState(const VectorXd& q_virtual,
                                 const VectorXd& q_mobile,
                                 const VectorXd& q_mani,
                                 const VectorXd& qdot_virtual,
                                 const VectorXd& qdot_mobile,
                                 const VectorXd& qdot_mani);

                // ================================ Compute Functions ================================
                // Wholebody joint space
                using Manipulator::RobotData::computeMassMatrix; 
                using Manipulator::RobotData::computeGravity; 
                using Manipulator::RobotData::computeCoriolis; 
                using Manipulator::RobotData::computeNonlinearEffects; 
                
                virtual MatrixXd computeMassMatrix(const VectorXd& q_virtual,
                                                   const VectorXd& q_mobile,
                                                   const VectorXd& q_mani);
                virtual VectorXd computeGravity(const VectorXd& q_virtual,
                                                const VectorXd& q_mobile,
                                                const VectorXd& q_mani);
                virtual VectorXd computeCoriolis(const VectorXd& q_virtual,
                                                 const VectorXd& q_mobile,
                                                 const VectorXd& q_mani,
                                                 const VectorXd& qdot_virtual,
                                                 const VectorXd& qdot_mobile,
                                                 const VectorXd& qdot_mani);
                virtual VectorXd computeNonlinearEffects(const VectorXd& q_virtual,
                                                         const VectorXd& q_mobile,
                                                         const VectorXd& q_mani,
                                                         const VectorXd& qdot_virtual,
                                                         const VectorXd& qdot_mobile,
                                                         const VectorXd& qdot_mani);

                virtual MatrixXd computeMassMatrixActuated(const VectorXd& q_virtual,
                                                           const VectorXd& q_mobile,
                                                           const VectorXd& q_mani);
                virtual VectorXd computeGravityActuated(const VectorXd& q_virtual,
                                                        const VectorXd& q_mobile,
                                                        const VectorXd& q_mani);
                virtual VectorXd computeCoriolisActuated(const VectorXd& q_virtual,
                                                         const VectorXd& q_mobile,
                                                         const VectorXd& q_mani,
                                                         const VectorXd& qdot_mobile,
                                                         const VectorXd& qdot_mani);
                virtual VectorXd computeNonlinearEffectsActuated(const VectorXd& q_virtual,
                                                                 const VectorXd& q_mobile,
                                                                 const VectorXd& q_mani,
                                                                 const VectorXd& qdot_mobile,
                                                                 const VectorXd& qdot_mani);

                // Wholebody task space
                using Manipulator::RobotData::computePose;
                using Manipulator::RobotData::computeJacobian;
                using Manipulator::RobotData::computeJacobianTimeVariation;
                using Manipulator::RobotData::computeVelocity;
                using Manipulator::RobotData::computeMinDistance;

                virtual Affine3d computePose(const VectorXd& q_virtual,
                                             const VectorXd& q_mobile,
                                             const VectorXd& q_mani, 
                                             const std::string& link_name);
                virtual MatrixXd computeJacobian(const VectorXd& q_virtual,
                                                 const VectorXd& q_mobile,
                                                 const VectorXd& q_mani, 
                                                 const std::string& link_name);
                virtual MatrixXd computeJacobianTimeVariation(const VectorXd& q_virtual,
                                                              const VectorXd& q_mobile,
                                                              const VectorXd& q_mani,
                                                              const VectorXd& qdot_virtual,
                                                              const VectorXd& qdot_mobile,
                                                              const VectorXd& qdot_mani, 
                                                              const std::string& link_name);
                virtual VectorXd computeVelocity(const VectorXd& q_virtual,
                                                 const VectorXd& q_mobile,
                                                 const VectorXd& q_mani,
                                                 const VectorXd& qdot_virtual,
                                                 const VectorXd& qdot_mobile,
                                                 const VectorXd& qdot_mani, 
                                                 const std::string& link_name);
                virtual Manipulator::MinDistResult computeMinDistance(const VectorXd& q_virtual,
                                                                      const VectorXd& q_mobile,
                                                                      const VectorXd& q_mani,
                                                                      const VectorXd& qdot_virtual,
                                                                      const VectorXd& qdot_mobile,
                                                                      const VectorXd& qdot_mani, 
                                                                      const bool& with_grad, 
                                                                      const bool& with_graddot, 
                                                                      const bool verbose);
                virtual MatrixXd computeSelectionMatrix(const VectorXd& q_virtual,
                                                        const VectorXd& q_mobile);

                virtual MatrixXd computeJacobianActuated(const VectorXd& q_virtual,
                                                         const VectorXd& q_mobile,
                                                         const VectorXd& q_mani, 
                                                         const std::string& link_name);
                virtual MatrixXd computeJacobianTimeVariationActuated(const VectorXd& q_virtual,
                                                                      const VectorXd& q_mobile,
                                                                      const VectorXd& q_mani,
                                                                      const VectorXd& qdot_virtual,
                                                                      const VectorXd& qdot_mobile,
                                                                      const VectorXd& qdot_mani, 
                                                                      const std::string& link_name);
                                                        
                                                              
                // Manipulator taskspace
                using Manipulator::RobotData::computeManipulability;
                virtual Manipulator::ManipulabilityResult computeManipulability(const VectorXd& q_mani, 
                                                                                const VectorXd& qdot_mani, 
                                                                                const bool& with_grad, 
                                                                                const bool& with_graddot, 
                                                                                const std::string& link_name);
                                                                        
                // Mobile
                virtual MatrixXd computeMobileFKJacobian(const VectorXd& q_mobile);
                virtual VectorXd computeMobileBaseVel(const VectorXd& q_mobile, const VectorXd& qdot_mobile);

                // ================================ Get Functions ================================
                virtual int getActuatordDof() const {return actuated_dof_;}
                virtual int getManipulatorDof() const {return mani_dof_;}
                virtual int getMobileDof() const {return mobi_dof_;}
                virtual JointIndex getJointIndex() const {return joint_idx_;}
                virtual ActuatorIndex getActuatorIndex() const {return actuator_idx_;}
                virtual VectorXd getMobileJointPosition() const {return q_mobile_;}
                virtual VectorXd getVirtualJointPosition() const {return q_virtual_;}
                virtual VectorXd getManiJointPosition() const {return q_mani_;}
                virtual VectorXd getJointVelocityActuated() const {return qdot_actuated_;}
                virtual VectorXd getMobileJointVelocity() const {return qdot_mobile_;}
                virtual VectorXd getVirtualJointVelocity() const {return qdot_virtual_;}
                virtual VectorXd getManiJointVelocity() const {return qdot_mani_;}
                virtual VectorXd getJointPositionActuated() const {return q_actuated_;}
                
                // Wholebody joint space
                virtual MatrixXd getMassMatrixActuated() const {return M_actuated_;}
                virtual MatrixXd getMassMatrixActuatedInv() const {return M_inv_actuated_;}
                virtual VectorXd getGravityActuated() const {return g_actuated_;}
                virtual VectorXd getCoriolisActuated() const {return c_actuated_;}
                virtual VectorXd getNonlinearEffectsActuated() const {return NLE_actuated_;}

                // Wholebody task space
                virtual MatrixXd getJacobianActuated(const std::string& link_name);
                virtual MatrixXd getJacobianActuatedTimeVariation(const std::string& link_name);
                virtual MatrixXd getSelectionMatrix(){return S_;}

                // Manipulator taskspace
                using Manipulator::RobotData::getManipulability;
                virtual Manipulator::ManipulabilityResult getManipulability(const bool& with_grad, 
                                                                            const bool& with_graddot, 
                                                                            const std::string& link_name);

                // Mobile
                virtual MatrixXd getMobileFKJacobian() const {return J_mobile_;}
                virtual VectorXd getMobileBaseVel() const {return Mobile::RobotData::getBaseVel();}
        

            protected:
                using Manipulator::RobotData::updateKinematics;
                using Manipulator::RobotData::updateDynamics;
                virtual bool updateKinematics(const VectorXd& q_virtual,
                                              const VectorXd& q_mobile,
                                              const VectorXd& q_mani,
                                              const VectorXd& qdot_virtual,
                                              const VectorXd& qdot_mobile,
                                              const VectorXd& qdot_mani);
                virtual bool updateDynamics(const VectorXd& q_virtual,
                                            const VectorXd& q_mobile,
                                            const VectorXd& q_mani,
                                            const VectorXd& qdot_virtual,
                                            const VectorXd& qdot_mobile,
                                            const VectorXd& qdot_mani);

                VectorXd getJointVector(const VectorXd& q_virtual,
                                        const VectorXd& q_mobile,
                                        const VectorXd& q_mani);
                VectorXd getActuatorVector(const VectorXd& q_mobile,
                                           const VectorXd& q_mani);

                int mani_dof_;
                int mobi_dof_;
                int virtual_dof_{3};
                int actuated_dof_;
        
                JointIndex joint_idx_;
                ActuatorIndex actuator_idx_;
                        
                // Selection Matrix for virtual joints (Actuated Joint Velocity -> Total Joint Velocity)
                MatrixXd S_;
                // MatrixXd Sdot_;
                
                VectorXd q_virtual_;
                VectorXd q_mobile_;
                VectorXd q_mani_;
                VectorXd qdot_virtual_;
                VectorXd qdot_mobile_;
                VectorXd qdot_mani_;

                // Actuated Joint state
                VectorXd q_actuated_;        // joint angle
                VectorXd qdot_actuated_;     // joint velocity
                
                // ActuatedJoint space Dynamics
                MatrixXd M_actuated_;     // inertia matrix
                MatrixXd M_inv_actuated_; // inverse of inertia matrix
                VectorXd g_actuated_;     // gravity forces
                VectorXd c_actuated_;     // centrifugal and coriolis forces
                VectorXd NLE_actuated_;   // nonlinear effects ( g_ + c_ )
        };
    } // namespace MobileManipulator
} // namespace drc