#include "QP/base.h"

namespace QP
{
    QPBase::QPBase()
    {

    }

    void QPBase::setQPsize(const int &nx,  const int&nbc, const int &nineqc, const int &neqc)
    {
        assert(nbc == nx || nbc == 0);

        nx_ = nx;
        nbc_ = nbc;
        nineqc_ = nineqc;
        neqc_ = neqc;
        nc_ = nineqc_ + neqc_+ nbc_ ;
        
        P_ds_.setZero(nx_, nx_);
        q_ds_.setZero(nx_);
        
        A_ineq_ds_.setZero(nineqc_, nx_);
        l_ineq_ds_.setConstant(nineqc_,-std::numeric_limits<double>::infinity());
        u_ineq_ds_.setConstant(nineqc_,std::numeric_limits<double>::infinity());
        
        l_bound_ds_.setConstant(nbc_,-std::numeric_limits<double>::infinity());
        u_bound_ds_.setConstant(nbc_,std::numeric_limits<double>::infinity());
        
        A_eq_ds_.setZero(neqc_, nx_);
        b_eq_ds_.setZero(neqc_);
        
        A_ds_.setZero(nc_, nx_);
        l_ds_.setConstant(nc_,-std::numeric_limits<double>::infinity());
        u_ds_.setConstant(nc_,std::numeric_limits<double>::infinity());

        time_status_.setZero();
    }

    void QPBase::setConstraint()
    {
        // Bound Constraint
        if(nbc_ != 0)
        {
            A_ds_.block(0,0,nbc_,nx_).setIdentity();
            l_ds_.segment(0,nbc_) = l_bound_ds_;
            u_ds_.segment(0,nbc_) = u_bound_ds_;
        }

        // Ineqaulity Constraint
        if(nineqc_ != 0)
        {
            A_ds_.block(nbc_,0,nineqc_,nx_) = A_ineq_ds_;
            l_ds_.segment(nbc_,nineqc_) = l_ineq_ds_;
            u_ds_.segment(nbc_,nineqc_) = u_ineq_ds_;
        }

        // Eqaulity Constraint
        if(neqc_ != 0)
        {
            A_ds_.block(nbc_+nineqc_,0,neqc_,nx_) = A_eq_ds_;
            l_ds_.segment(nbc_+nineqc_,neqc_) = b_eq_ds_;
            u_ds_.segment(nbc_+nineqc_,neqc_) = b_eq_ds_;
        }
    }

    bool QPBase::solveQP(MatrixXd &sol, TimeDuration &time_status)
    {
        timer_.reset();
        setCost();
        time_status.set_cost = timer_.elapsedAndReset();
        if(nbc_ != 0) setBoundConstraint();
        time_status.set_bound = timer_.elapsedAndReset();
        if(nineqc_ != 0) setIneqConstraint();
        time_status.set_ineq = timer_.elapsedAndReset();
        if(neqc_ != 0) setEqConstraint();
        time_status.set_eq = timer_.elapsedAndReset();
        setConstraint();
        time_status.set_constraint = timer_.elapsedAndReset();
        time_status.set_qp = time_status.set_cost + time_status.set_bound + time_status.set_ineq + time_status.set_eq + time_status.set_constraint; 
        // std::cout << "P:\n" << P_ds_<<std::endl;
        // std::cout << "q: " << q_ds_.transpose()<<std::endl;
        // std::cout << "l: " << l_ds_.transpose()<<std::endl;
        // std::cout << "u: " << u_ds_.transpose()<<std::endl;
        // std::cout << "A:\n " << A_ds_ << std::endl;
        /* 
        min   1/2 x' P x + q' x
         x

        subject to
        l <= A x <= u

        with :
        P sparse (nx x nx) positive definite
        q dense  (nx x 1)
        A sparse (nc x n)
        l dense (nc x 1)
        u dense (nc x 1)
        */
        SparseMatrix<double> P(nx_, nx_);
        VectorXd q;
        SparseMatrix<double> A(nc_, nx_);
        VectorXd l, u;
        P = P_ds_.sparseView();
        A = A_ds_.sparseView();
        q = q_ds_;
        l = l_ds_;
        u = u_ds_;

        OsqpEigen::Solver solver;

        // settings
        solver.settings()->setWarmStart(false);
        solver.settings()->getSettings()->eps_abs = 1e-4;
        solver.settings()->getSettings()->eps_rel = 1e-5;
        solver.settings()->getSettings()->verbose = false;

        // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(nx_);
        solver.data()->setNumberOfConstraints(nc_);
        if (!solver.data()->setHessianMatrix(P))           return false;
        if (!solver.data()->setGradient(q))                return false;
        if (!solver.data()->setLinearConstraintsMatrix(A)) return false;
        if (!solver.data()->setLowerBound(l))              return false;
        if (!solver.data()->setUpperBound(u))              return false;


        // instantiate the solver
        if (!solver.initSolver()) return false;

        // solve the QP problem
        if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
        qp_status_ = solver.getStatus();
        if (solver.getStatus() != OsqpEigen::Status::Solved) return false;
        // if (solver.getStatus() != OsqpStatus::Solved && solver.getStatus() != OsqpStatus::SolvedInaccurate) return false;

        time_status.set_solver = timer_.elapsedAndReset();

        sol = solver.getSolution();

        time_status.solve_qp = timer_.elapsedAndReset();

        solver.clearSolverVariables();
        solver.clearSolver();

        return true;
    }
} // namespace QP