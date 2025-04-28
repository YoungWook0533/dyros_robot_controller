#include "fr3_controller/passive_controller.h"

namespace FR3Controller
{
    PassiveController::PassiveController(RobotData* robot_data):
    rb_(robot_data)
    {
        setDSParameters();
        setJointLimits();
    };

    PassiveController::~PassiveController()
    {
        solver_.clearSolverVariables();
        solver_.clearSolver();
    }

    void PassiveController::setDSParameters()
    {
        Vector6d P_diag;
        P_diag << 25., 25., 25., 25., 25., 25.;
        P_ = P_diag.asDiagonal();

        Vector6d Lambda_diag;
        Lambda_diag << 20., 200., 200., 200., 200., 200.;
        Lambda_ = Lambda_diag.asDiagonal(); 
    }

    void PassiveController::setJointLimits()
    {
        q_upper_.setZero(7);
        q_lower_.setZero(7);
        q_tol_upper_.setZero(7);
        q_tol_lower_.setZero(7);
        qddot_upper_.setZero(7);
        qddot_lower_.setZero(7);
        tau_upper_.setZero(7);
        tau_lower_.setZero(7);

        q_upper_ << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973;
        q_lower_ << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973;

        q_tol_upper_.setConstant(0.1);
        q_tol_lower_.setConstant(0.1);

        qddot_upper_ << 15, 7.5, 10, 12.5, 15, 20, 20;
        qddot_lower_ = -qddot_upper_;

        tau_upper_ << 87, 87, 87, 87, 12, 12, 12;
        tau_lower_ = -tau_upper_;
    }

    void PassiveController::setDS(const Matrix4d& target_pose)
    {
        Vector6d x_error;
        x_error.head(3) = rb_->getPose().block(0,3,3,1) - target_pose.block(0,3,3,1);
        x_error.tail(3) = DyrosMath::getPhi(rb_->getPose().block(0,0,3,3), target_pose.block(0,0,3,3));
        
        Vector6d f = -2. * P_ * x_error;
        Vector6d v0 = f.normalized();

        Matrix6d D = Matrix6d::Identity();

        if (v0.norm() > 1E-6)
        {
            Matrix6d V_tmp;
            V_tmp.setRandom();
            V_tmp.col(0) = v0;
            
            // // HouseholderQR 분해를 통해 A를 정규 직교화하여 Q 행렬을 구함
            // HouseholderQR<Matrix6d> qr(V_tmp);
            // Matrix6d V = qr.householderQ();
            
            // // Q의 첫 번째 열이 v0와 같은 방향이 되도록 부호 조정
            // if ((V.col(0) - v0).norm() > (V.col(0) + v0).norm()) 
            // {
            //     V.col(0) *= -1.;
            // }

            for(uint i=1;i<6;i++){
                for(uint j=0;j<i;j++)
                V_tmp.col(i) -= V_tmp.col(j).dot(V_tmp.col(i))*V_tmp.col(j);
                V_tmp.col(i).normalize();
            }
            Matrix6d V = V_tmp;

            D = V * Lambda_ * V.transpose();
        }

        // Fc_ = rb_->getTaskGravity() - D*(rb_->getVelocity() - f);
        Fc_ = - D*rb_->getVelocity() + Lambda_(0,0)* f;
    }

    void PassiveController::setCost()
    {
        // decision variable = [tau, qddot]
        Hess_cost_.setZero(14, 14);
        grad_cost_.setZero(14);

        MatrixXd J_inv_T(6, 7);
        J_inv_T = rb_->getTaskMassMatrix() * rb_->getJacobian() * rb_->getMassMatrix().inverse();
        
        Hess_cost_.block(0,0,7,7) = 2. * J_inv_T.transpose() * J_inv_T;
        grad_cost_.head(7) = -2. * J_inv_T.transpose() * Fc_;
    }

    void PassiveController::setConstraints()
    {
        // decision variable = [tau, qddot]
        // Constraints: tau limit, qddot limit, q upper limit, q limit
        // equality constraint
        Jac_const_eq_.setZero(7, 14);
        const_eq_.setZero(7);

        Jac_const_eq_.block(0,0,7,7) = -MatrixXd::Identity(7,7);
        Jac_const_eq_.block(0,7,7,7) = rb_->getMassMatrix();
        const_eq_ = rb_->getExtTorque() - rb_->getCoriolis();

        // inequality constraint
        Jac_const_ineq_.setZero(14,14);
        l_const_ineq_.setZero(14);
        u_const_ineq_.setZero(14);

        VectorXd h_l = rb_->getJointPosition() - q_lower_ - q_tol_lower_;
        VectorXd h_u = -rb_->getJointPosition() + q_upper_ - q_tol_upper_;
        VectorXd RBF_delta = -0.5*VectorXd::Ones(7);
        VectorXd c_l = -getDRBF(RBF_delta,h_l).cwiseProduct(rb_->getJointVelocity()) - getRBF(RBF_delta, rb_->getJointVelocity() + getRBF(RBF_delta, h_l));
        VectorXd c_u = -getDRBF(RBF_delta,h_u).cwiseProduct(rb_->getJointVelocity()) - getRBF(RBF_delta, rb_->getJointVelocity() - getRBF(RBF_delta, h_l));

        Jac_const_ineq_.setIdentity();
        l_const_ineq_.head(7) = tau_lower_;
        l_const_ineq_.tail(7) = qddot_lower_.cwiseMax(c_l);
        u_const_ineq_.head(7) = tau_upper_;
        u_const_ineq_.tail(7) = qddot_upper_.cwiseMin(c_u);

        // total constraint
        Jac_const_.setZero(Jac_const_eq_.rows()+Jac_const_ineq_.rows() , 14);
        l_const_.setZero(const_eq_.size()+l_const_ineq_.size());
        u_const_.setZero(const_eq_.size()+u_const_ineq_.size());

        Jac_const_.block(0,0,Jac_const_eq_.rows(),14) = Jac_const_eq_;
        Jac_const_.block(Jac_const_eq_.rows(),0,Jac_const_ineq_.rows(),14) = Jac_const_ineq_;
        l_const_.head(const_eq_.size()) = const_eq_;
        l_const_.tail(l_const_ineq_.size()) = l_const_ineq_;
        u_const_.head(const_eq_.size()) = const_eq_;
        u_const_.tail(u_const_ineq_.size()) = u_const_ineq_;
    }

    double PassiveController::getRBF(const double &delta, const double &h)
    {
        // Grandia, Ruben, et al. 
        // "Feedback mpc for torque-controlled legged robots." 
        // 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2019.
        double result;
        if (h >= delta) result = -log(h+1);
        else            result = -log(delta+1) - 1/(delta+1) * (h-delta) + 1/(2*pow(delta+1,2)) * pow(h-delta,2);
        return result;
    }

    VectorXd PassiveController::getRBF(const VectorXd& delta, const VectorXd &h)
    {
        VectorXd result(h.size());
        for(size_t i=0; i<result.size(); i++) result(i) = getRBF(delta(i), h(i));
        return result;
    }

    double PassiveController::getDRBF(const double &delta, const double &h)
    {
        // Grandia, Ruben, et al. 
        // "Feedback mpc for torque-controlled legged robots." 
        // 2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). IEEE, 2019.
        double result;
        if (h >= delta) result = -1/(h+1);
        else            result = -1/(delta+1) + 1/(pow(delta+1,2)) * (h-delta);
        return result;
    }

    VectorXd PassiveController::getDRBF(const VectorXd &delta, const VectorXd &h)
    {
        VectorXd result(h.size());
        for(size_t i=0; i<result.size(); i++) result(i) = getDRBF(delta(i), h(i));
        return result;
    }

    bool PassiveController::solveController(const Matrix4d& target_pose)
    {
        setDS(target_pose);
        setCost();
        setConstraints();
        // std::cout << "Hess_cost_\n"  << Hess_cost_ << std::endl;
        // std::cout << "grad_cost_\n"  << grad_cost_.transpose() << std::endl;
        // std::cout << "Jac_const_\n"  << Jac_const_ << std::endl;
        // std::cout << "l_const_\n"  << l_const_.transpose() << std::endl;
        // std::cout << "u_const_\n"  << u_const_.transpose() << std::endl;
        
        /*
        min   1/2 x' P x + q' x
        x

        subject to
        l <= A x <= u

        with :
        P sparse (n x n) positive definite
        q dense  (n x 1)
        A sparse (nc x n)
        l dense  (nc x 1)
        u dense  (nc x 1)
        */
        SparseMatrix<double> P_sp(Hess_cost_.rows(), Hess_cost_.cols());
        SparseMatrix<double> A_sp(Jac_const_.rows(), Jac_const_.cols());
        VectorXd q_ds;
        VectorXd l_ds;
        VectorXd u_ds;
        P_sp = Hess_cost_.sparseView();
        A_sp = Jac_const_.sparseView();
        q_ds = grad_cost_;
        l_ds = l_const_;
        u_ds = u_const_;
        
        // settings
        if (is_first_)
        {
            solver_.settings()->setWarmStart(false);
            // solver_.settings()->getSettings()->eps_abs = 1e-4;
            // solver_.settings()->getSettings()->eps_rel = 1e-5;
            // solver_.settings()->getSettings()->time_limit = (Ts_ / 5.);
            solver_.settings()->getSettings()->verbose = false;
        }
        
        // set the initial data of the QP solver_
        if(is_first_)
        {
            solver_.data()->setNumberOfVariables(Hess_cost_.rows());
            solver_.data()->setNumberOfConstraints(Jac_const_.rows());
            if (!solver_.data()->setHessianMatrix(P_sp))           return false;
            if (!solver_.data()->setGradient(q_ds))                return false;
            if (!solver_.data()->setLinearConstraintsMatrix(A_sp)) return false;
            if (!solver_.data()->setLowerBound(l_ds))              return false;
            if (!solver_.data()->setUpperBound(u_ds))              return false;

            // instantiate the solver_
            if (!solver_.initSolver()) return false;
        }
        else
        {
            if (!solver_.updateHessianMatrix(P_sp))           return false;
            if (!solver_.updateGradient(q_ds))                return false;
            if (!solver_.updateLinearConstraintsMatrix(A_sp)) return false;
            if (!solver_.updateBounds(l_ds, u_ds))            return false;
        }
        
        // solve the QP problem
        if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
        if (solver_.getStatus() != OsqpEigen::Status::Solved)            return false;
        
        // get the controller input
        VectorXd opt = solver_.getSolution();
        torque_opt_ = opt.head(7);

        if (is_first_) is_first_ = false;

        return true;
    }
}