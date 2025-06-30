#ifndef QP_BASE_H
#define QP_BASE_H

#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>

#include "OsqpEigen/OsqpEigen.h"
#include "suhan_benchmark.h"

using namespace Eigen;

namespace QP
{
    struct TimeDuration
    {
        double set_qp;
        double set_cost;
        double set_bound;
        double set_ineq;
        double set_eq;
        double set_constraint;
        double set_solver;
        double solve_qp;
        void setZero()
        {
            set_qp = 0;
            set_cost = 0;
            set_bound = 0;
            set_ineq = 0;
            set_eq = 0;
            set_constraint = 0;
            set_solver = 0;
            solve_qp = 0;
        }
    };
    
    class Base
    {
        public:
            Base();
            void setQPsize(const int &nx, const int&nbc, const int &nineqc, const int &neqc);
            bool solveQP(MatrixXd &sol, TimeDuration &time_status);
        
        private:
            virtual void setCost() = 0;
            virtual void setBoundConstraint() = 0;
            virtual void setIneqConstraint() = 0;
            virtual void setEqConstraint() = 0;
            void setConstraint();
        
        protected:
            int nx_;     // number of decision variables
            int nc_;     // number of constraints (ineq + bound + eq)
            int nbc_;    // number of bound constraints
            int nineqc_; // number of inequality constraints
            int neqc_;   // number of equality constraints

            MatrixXd P_ds_;
            VectorXd q_ds_;

            MatrixXd A_ds_;
            VectorXd l_ds_;
            VectorXd u_ds_;

            MatrixXd A_ineq_ds_;
            VectorXd l_ineq_ds_;
            VectorXd u_ineq_ds_;

            VectorXd l_bound_ds_;
            VectorXd u_bound_ds_;

            MatrixXd A_eq_ds_;
            VectorXd b_eq_ds_;


            OsqpEigen::Status qp_status_;
            SuhanBenchmark timer_;
            TimeDuration time_status_;
    };
} // namespace QP
#endif