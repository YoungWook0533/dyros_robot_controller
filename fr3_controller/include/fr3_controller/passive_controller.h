#ifndef PASSIVE_CONTROLLER_HPP
#define PASSIVE_CONTROLLER_HPP

#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h" 

#include "fr3_controller/robot_data.h"
#include "fr3_controller/math_type_define.h"

namespace FR3Controller
{
    class PassiveController
    {
        public:
            PassiveController(RobotData* robot_data);
            ~PassiveController();

            bool solveController(const Matrix4d& target_pose);
            VectorXd getOptTorque() {return torque_opt_; }

            
        private:
            void setDSParameters();
            void setJointLimits();

            void setDS(const Matrix4d& target_pose);

            void setCost();
            void setConstraints();

            double getRBF(const double &delta, const double &h);
            VectorXd getRBF(const VectorXd& delta, const VectorXd &h);
            double getDRBF(const double &delta, const double &h);
            VectorXd getDRBF(const VectorXd &delta, const VectorXd &h);


            RobotData* rb_;

            OsqpEigen::Solver solver_;

            Matrix6d P_; // Potential function
            Matrix6d Lambda_; // Eigen matrix for damping function
            VectorXd q_lower_, q_upper_; // Joint limits
            VectorXd q_tol_lower_, q_tol_upper_; // Tolerence for joint limits
            VectorXd qddot_lower_, qddot_upper_; // Joint limits
            VectorXd tau_lower_, tau_upper_; // Joint limits
            
            Vector6d Fc_;

            MatrixXd Hess_cost_;
            VectorXd grad_cost_;
            MatrixXd Jac_const_eq_, Jac_const_ineq_, Jac_const_;
            VectorXd const_eq_, l_const_ineq_, u_const_ineq_, l_const_, u_const_;

            bool is_solved_;
            bool is_first_{true};

            VectorXd torque_opt_;
    };
}


#endif // PASSIVE_CONTROLLER_HPP