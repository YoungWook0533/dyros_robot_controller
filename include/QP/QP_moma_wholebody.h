#ifndef QP_MOMA_WHOLEBODY_H
#define QP_MOMA_WHOLEBODY_H

#include "QP/QP_base.h"
#include "math_type_define.h"
#include "robot_data/mobile_manipulator.h"

namespace QP
{
    class MOMAWholebody : public Base
    {
        public:
            MOMAWholebody(std::shared_ptr<RobotDataMobileManipulator> robot_data);
            void setDesiredEEAcc(const VectorXd &xddot_desired);
            bool getOptJoint(VectorXd &opt_etadot, VectorXd &opt_tau, TimeDuration &time_status);

        private:
            struct QPIndex
            {
                int eta_dot_start; // qddot_actuated 
                int tau_start;     // torque_actuated
                int slack_start;   // slack for task
                
                // equality
                int con_dyn_start; // dynamics constraint
                
                // inequality
                int con_q_mani_min_start; // min q_manipulator
                int con_q_mani_max_start; // max q_manipulator
                int con_sing; // singularity
                int con_sel_col; // self collision
            }si_index_;

            std::shared_ptr<RobotDataMobileManipulator> robot_data_;
            
            int joint_dof_;
            int mani_dof_;

            VectorXd xddot_desired_;
            
            void setCost() override;
            void setBoundConstraint() override;
            void setIneqConstraint() override;
            void setEqConstraint() override;

            void getCBFConstraint(const VectorXd& h, const MatrixXd& dh_dq, const VectorXd& qdot, MatrixXd& A, VectorXd& b);
            void getCBFConstraint(const double& h, const VectorXd& dh_dq, const VectorXd& qdot, MatrixXd& A, double& b);
    };
} // namespace QP
#endif