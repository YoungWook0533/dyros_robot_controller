#pragma once
#include "robot_data/robot_data_base.h"

using namespace Eigen;

class RobotDataManipulator : public RobotDataBase
{
    public:
        static inline std::string ee_name_;      // end-effector link name

        RobotDataManipulator(const std::string& urdf_path, 
                             const int& dof, 
                             const std::string& ee_name, 
                             const bool verbose=false);

        Affine3d computePose(const VectorXd& q, const std::string& link_name=ee_name_);
        MatrixXd computeJacobian(const VectorXd& q, const std::string& link_name=ee_name_);
        MatrixXd computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);
        VectorXd computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name=ee_name_);
        MatrixXd computeMassMatrix(const VectorXd& q);
        VectorXd computeCoriolis(const VectorXd& q, const VectorXd& qdot);
        VectorXd computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot);
        VectorXd computeGravity(const VectorXd& q);

        double getDof() const {return dof_;}
        Affine3d getPose(const std::string& link_name=ee_name_) const;
        MatrixXd getJacobian(const std::string& link_name=ee_name_);
        MatrixXd getJacobianTimeVariation(const std::string& link_name=ee_name_); 
        VectorXd getVelocity(const std::string& link_name=ee_name_);
        MatrixXd getMassMatrix() const {return M_;}
        MatrixXd getMassMatrixInv() const {return M_inv_;}
        VectorXd getCoriolis() const {return c_;}
        VectorXd getGravity() const {return g_;}
        VectorXd getNonlinearEffects() const {return NLE_;}

    protected:
        bool updateKinematics(const VectorXd& q, const VectorXd& qdot);
        bool updateDynamics(const VectorXd& q, const VectorXd& qdot);

        int dof_;

        // Task space state
        pinocchio::FrameIndex ee_index_;  // end-effector link index
        Affine3d x_;                      // pose of EE
        VectorXd xdot_;                   // velocity of EE
        MatrixXd J_;                      // jacobian of EE
        MatrixXd Jdot_;                   // time derivative of jacobian of EE

        // Joint space Dynamics
        MatrixXd M_;                 // inertia matrix
        MatrixXd M_inv_;             // inverse of inertia matrix
        VectorXd g_;                 // gravity forces
        VectorXd c_;                 // centrifugal and coriolis forces
        VectorXd NLE_;               // nonlinear effects ( g_ + c_ )
};