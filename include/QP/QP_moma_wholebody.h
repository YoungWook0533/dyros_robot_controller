#ifndef QP_MOMA_WHOLEBODY_H
#define QP_MOMA_WHOLEBODY_H

#include "QP/QP_base.h"
#include "math_type_define.h"
#include "husky_fr3_controller/robot_data.h"

#define MANI_DOF 7 // 7 for FR3
#define MOBI_DOF 2 // 2 for Mobile
#define JOINT_DOF 9 // 7 for FR3, 2 for Mobile
#define TASK_DOF 6  // 6 for end-effector

namespace QP
{
    class MOMAWholebody : public Base
    {
        public:
            MOMAWholebody();
            void setCurrentState(std::shared_ptr<HuskyFR3Controller::RobotData> robot_data);
            void setDesiredEEAcc(const Matrix<double, TASK_DOF, 1> &xddot_desired);
            bool getOptJoint(Matrix<double, JOINT_DOF, 1> &opt_etadot, Matrix<double, JOINT_DOF, 1> &opt_tau, TimeDuration &time_status);

        private:
            struct QPIndex
            {
                int eta_dot = 0; // 0-8
                int tau = JOINT_DOF; // 9-17
                
                // equality
                int con_dyn = 0;  // 0-8
                
                // inequality
                int con_q_mani_min = 0;  // 0-6
                int con_q_mani_max = MANI_DOF;  // 7-13
                int con_mani_sing = 2*MANI_DOF; // 14
            }si_index_;

            std::shared_ptr<HuskyFR3Controller::RobotData> robot_data_;
            Matrix<double, TASK_DOF, 1> xddot_desired_;
            
            void setCost() override;
            void setBoundConstraint() override;
            void setIneqConstraint() override;
            void setEqConstraint() override;

            void getCBFConstraint(const VectorXd& h, const MatrixXd& dh_dq, const VectorXd& qdot, MatrixXd& A, VectorXd& b);
            void getCBFConstraint(const double& h, const VectorXd& dh_dq, const VectorXd& qdot, MatrixXd& A, double& b);
    };
} // namespace QP
#endif