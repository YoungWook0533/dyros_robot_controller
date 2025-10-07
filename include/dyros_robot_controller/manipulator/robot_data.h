#pragma once
#include <string>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Dense>
#include <math.h>

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

using namespace Eigen;

namespace drc
{
    namespace Manipulator
    {
        class RobotData
        {
            public:
                EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                RobotData(const std::string& urdf_path, 
                          const std::string& srdf_path, 
                          const std::string& packages_path);

                virtual bool updateState(const VectorXd& q, const VectorXd& qdot);
                virtual std::string getVerbose() const;
                // ================================ Compute Functions ================================
                // Joint space 
                virtual MatrixXd computeMassMatrix(const VectorXd& q);
                virtual VectorXd computeGravity(const VectorXd& q);
                virtual VectorXd computeCoriolis(const VectorXd& q, const VectorXd& qdot);
                virtual VectorXd computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot);
                
                // Task space
                virtual Affine3d computePose(const VectorXd& q, const std::string& link_name);
                virtual MatrixXd computeJacobian(const VectorXd& q, const std::string& link_name);
                virtual MatrixXd computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name);
                virtual VectorXd computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name);
                virtual MinDistResult computeMinDistance(const VectorXd& q, const VectorXd& qdot, const bool& with_grad, const bool& with_graddot, const bool verbose);
                virtual ManipulabilityResult computeManipulability(const VectorXd& q, const VectorXd& qdot, const bool& with_grad, const bool& with_graddot, const std::string& link_name);
                
                // ================================ Get Functions ================================
                virtual int getDof() const {return dof_;}

                // Joint space
                virtual VectorXd getJointPosition() const {return q_;}
                virtual VectorXd getJointVelocity() const {return qdot_;}
                virtual std::pair<VectorXd,VectorXd> getJointPositionLimit() const {return std::make_pair(q_lb_, q_ub_);}
                virtual std::pair<VectorXd,VectorXd> getJointVelocityLimit() const {return std::make_pair(qdot_lb_, qdot_ub_);}
                virtual MatrixXd getMassMatrix() const {return M_;}
                virtual MatrixXd getMassMatrixInv() const {return M_inv_;}
                virtual VectorXd getCoriolis() const {return c_;}
                virtual VectorXd getGravity() const {return g_;}
                virtual VectorXd getNonlinearEffects() const {return NLE_;}

                // Task space
                virtual Affine3d getPose(const std::string& link_name) const;
                virtual MatrixXd getJacobian(const std::string& link_name);
                virtual MatrixXd getJacobianTimeVariation(const std::string& link_name); 
                virtual VectorXd getVelocity(const std::string& link_name);
                virtual MinDistResult getMinDistance(const bool& with_grad, const bool& with_graddot, const bool verbose);
                virtual ManipulabilityResult getManipulability(const bool& with_grad, const bool& with_graddot, const std::string& link_name);

            protected:
                virtual bool updateKinematics(const VectorXd& q, const VectorXd& qdot);
                virtual bool updateDynamics(const VectorXd& q, const VectorXd& qdot);
        
                // pinocchio data
                pinocchio::Model model_;
                pinocchio::Data data_;
                pinocchio::GeometryModel geom_model_;
                pinocchio::GeometryData geom_data_;

                int dof_;
                
                // Joint space state
                VectorXd q_;        // joint angle
                VectorXd qdot_;     // joint velocity
                VectorXd q_lb_;     // lower limit of joint angle
                VectorXd q_ub_;     // upper limit of joint angle
                VectorXd qdot_lb_;  // lower limit of joint velocity
                VectorXd qdot_ub_;  // upper limit of joint velocity

                // Joint space Dynamics
                MatrixXd M_;        // inertia matrix
                MatrixXd M_inv_;    // inverse of inertia matrix
                VectorXd g_;        // gravity forces
                VectorXd c_;        // centrifugal and coriolis forces
                VectorXd NLE_;      // nonlinear effects ( g_ + c_ )
        };
    } //namespce Manipulator
} // namespace drc
