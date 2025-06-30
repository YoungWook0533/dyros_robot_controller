#pragma once
#include <string>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Dense>
#include <math.h>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "math_type_define.h"

using namespace Eigen;

namespace HuskyFR3Controller
{
    /*
    Joint Information
    Total nq = 12
    Total nv = 12
    
     id | name              | nq | nv | idx_q | idx_v
    ----+-------------------+----+----+-------+------
      1 |         v_x_joint |  1 |  1 |     0 |    0
      2 |         v_y_joint |  1 |  1 |     1 |    1
      3 |         v_t_joint |  1 |  1 |     2 |    2
      4 |        fr3_joint1 |  1 |  1 |     3 |    3
      5 |        fr3_joint2 |  1 |  1 |     4 |    4
      6 |        fr3_joint3 |  1 |  1 |     5 |    5
      7 |        fr3_joint4 |  1 |  1 |     6 |    6
      8 |        fr3_joint5 |  1 |  1 |     7 |    7
      9 |        fr3_joint6 |  1 |  1 |     8 |    8
     10 |        fr3_joint7 |  1 |  1 |     9 |    9
     11 |        left_wheel |  1 |  1 |    10 |   10
     12 |       right_wheel |  1 |  1 |    11 |   11
    */
    inline constexpr int TASK_DOF     = 6;
    inline constexpr int VIRTUAL_DOF  = 3;
    inline constexpr int MANI_DOF     = 7;
    inline constexpr int MOBI_DOF     = 2;
    inline constexpr int ACTUATOR_DOF = MANI_DOF + MOBI_DOF;
    inline constexpr int JOINT_DOF    = ACTUATOR_DOF + VIRTUAL_DOF;

    struct JointIndex
    {
        const int virtual_start = 0;
        const int mani_start = VIRTUAL_DOF;
        const int mobi_start = VIRTUAL_DOF+MANI_DOF;
    };
    inline const JointIndex joint_idx{};

    struct ActuatorIndex
    {
        const int mani_start = 0;
        const int mobi_start = MANI_DOF;
    };
    inline const ActuatorIndex actuator_idx{};

    typedef Eigen::Matrix<double,TASK_DOF,1>                TaskVec;
    typedef Eigen::Matrix<double,TASK_DOF,TASK_DOF>         TaskMat;
    typedef Eigen::Matrix<double,VIRTUAL_DOF,1>             VirtualVec;
    typedef Eigen::Matrix<double,MANI_DOF,1>                ManiVec;
    typedef Eigen::Matrix<double,MOBI_DOF,1>                MobiVec;
    typedef Eigen::Matrix<double,ACTUATOR_DOF,1>            ActuatorVec;
    typedef Eigen::Matrix<double,ACTUATOR_DOF,ACTUATOR_DOF> ActuatorMat;
    typedef Eigen::Matrix<double,JOINT_DOF,1>               JointVec;
    typedef Eigen::Matrix<double,JOINT_DOF,JOINT_DOF>       JointMat;

    class RobotData
    {
        public:
            RobotData(const std::string& urdf_path, const bool verbose=false);
            ~RobotData();
            bool updateState(const JointVec& q, const JointVec& qdot);

            Affine3d computePose(const JointVec& q, const std::string& link_name=ee_name_);
            Matrix<double,TASK_DOF,JOINT_DOF> computeJacobian(const JointVec& q, const std::string& link_name=ee_name_);
            Matrix<double,TASK_DOF,JOINT_DOF> computeJacobianTimeVariation(const JointVec& q, const JointVec& qdot, const std::string& link_name=ee_name_);
            TaskVec computeVelocity(const JointVec& q, const JointVec& qdot, const std::string& link_name=ee_name_);
            JointMat computeMassMatrix(const JointVec& q);
            JointVec computeCoriolis(const JointVec& q, const JointVec& qdot);
            JointVec computeNonlinearEffects(const JointVec& q, const JointVec& qdot);
            JointVec computeGravity(const JointVec& q);
            Matrix<double,VIRTUAL_DOF,MOBI_DOF> computeMobileFKJacobian(const MobiVec& q_mobile);
            Matrix<double,JOINT_DOF,ACTUATOR_DOF> computeSelectionMatrix(const JointVec& q);
            Matrix<double,JOINT_DOF,ACTUATOR_DOF> computeSelectionMatrixTimeVariation(const JointVec& q, const JointVec& qdot);
            Matrix<double,TASK_DOF,ACTUATOR_DOF> computeJacobianActuated(const ActuatorVec& q_act, const std::string& link_name);
            ActuatorMat computeMassMatrixActuated(const ActuatorVec& q_act);
            ActuatorVec computeCoriolisActuated(const ActuatorVec& q_act, const ActuatorVec& qdot_act);
            ActuatorVec computeNonlinearEffectsActuated(const ActuatorVec& q_act, const ActuatorVec& qdot_act);
            ActuatorVec computeGravityActuated(const ActuatorVec& q_act);
            
            JointVec getJointPosition() const {return q_;}
            JointVec getJointVelocity() const {return qdot_;}
            Affine3d getPose(const std::string& link_name=ee_name_) const;
            Matrix<double,TASK_DOF,JOINT_DOF>  getJacobian(const std::string& link_name=ee_name_);
            Matrix<double,TASK_DOF,JOINT_DOF> getJacobianTimeVariation(const std::string& link_name=ee_name_); 
            Matrix<double,TASK_DOF,ACTUATOR_DOF> getJacobianActuatedTimeVariation(const std::string& link_name=ee_name_);
            TaskVec getVelocity(const std::string& link_name=ee_name_);
            JointMat getMassMatrix() const {return M_;}
            JointMat getMassMatrixInv() const {return M_inv_;}
            ActuatorMat getMassMatrixActuatedInv() const {return M_inv_actuated_;}
            JointVec getCoriolis() const {return c_;}
            JointVec getGravity() const {return g_;}
            JointVec getNonlinearEffects() const {return NLE_;}
            ActuatorVec getJointVelocityActuated() const {return qdot_actuated_;}
            ActuatorVec getJointPositionActuated() const {return q_actuated_;}
            Matrix<double,3,MOBI_DOF> getMobileFKJacobian() const {return J_mobile_;}
            Matrix<double,JOINT_DOF,ACTUATOR_DOF> getSelectionMatrix(){return S_;}
            Matrix<double,TASK_DOF,ACTUATOR_DOF> getJacobianActuated(const std::string& link_name=ee_name_);
            ActuatorMat getMassMatrixActuated() const {return M_actuated_;}
            ActuatorVec getCoriolisActuated() const {return c_actuated_;}
            ActuatorVec getGravityActuated() const {return g_actuated_;}
            ActuatorVec getNonlinearEffectsActuated() const {return NLE_actuated_;}

        private:
            bool updateKinematics(const JointVec& q, const JointVec& qdot);
            bool updateDynamics(const JointVec& q, const JointVec& qdot);

            // pinocchio data
            pinocchio::Model model_;
            pinocchio::Data data_;
            
            // Selection Matrix for virtual joints
            Matrix<double,JOINT_DOF,ACTUATOR_DOF> S_;
            Matrix<double,JOINT_DOF,ACTUATOR_DOF> Sdot_;
            const double wheel_radius_ = 0.1651;
            const double mobile_width_ = 0.2854*2.;
            const double mobile2mani_x = 0.36;
            const double mobile2mani_y = 0.;
            Matrix<double,3,MOBI_DOF> J_mobile_; // wheel_vel -> mobile_base_vel (wrt base frame) 

            // Joint space state
            JointVec q_;                    // joint angle
            JointVec qdot_;                 // joint velocity
            ActuatorVec q_actuated_;        // joint angle
            ActuatorVec qdot_actuated_;     // joint velocity

            // Task space state
            static constexpr const char* ee_name_ = "fr3_link8"; // end-effector link name
            pinocchio::FrameIndex ee_index_;                     // end-effector link index
            Affine3d x_;                                         // pose of EE
            TaskVec xdot_;                                       // velocity of EE
            Matrix<double,TASK_DOF,JOINT_DOF> J_;                // jacobian of EE
            Matrix<double,TASK_DOF,JOINT_DOF> Jdot_;             // time derivative of jacobian of EE
            Matrix<double,TASK_DOF,ACTUATOR_DOF> J_actuated_;    // jacobian of EE
            Matrix<double,TASK_DOF,ACTUATOR_DOF> J_actuateddot_; // jacobian of EE

            // Joint space Dynamics
            JointMat M_;                 // inertia matrix
            JointMat M_inv_;             // inverse of inertia matrix
            JointVec g_;                 // gravity forces
            JointVec c_;                 // centrifugal and coriolis forces
            JointVec NLE_;               // nonlinear effects ( g_ + c_ )
            ActuatorMat M_actuated_;     // inertia matrix
            ActuatorMat M_inv_actuated_; // inverse of inertia matrix
            ActuatorVec g_actuated_;     // gravity forces
            ActuatorVec c_actuated_;     // centrifugal and coriolis forces
            ActuatorVec NLE_actuated_;   // nonlinear effects ( g_ + c_ )
    };
}