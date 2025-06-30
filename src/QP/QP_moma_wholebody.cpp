#include "QP/QP_moma_wholebody.h"

namespace QP
{
    MOMAWholebody::MOMAWholebody()
    :Base()
    {
        Base::setQPsize(JOINT_DOF*2, 0, MANI_DOF*2, JOINT_DOF);
    }

    void MOMAWholebody::setCurrentState(std::shared_ptr<HuskyFR3Controller::RobotData> robot_data)
    {
        robot_data_ = robot_data;
    }

    void MOMAWholebody::setDesiredEEAcc(const Matrix<double, TASK_DOF, 1> &xddot_desired)
    {
        xddot_desired_ = xddot_desired;
    }

    bool MOMAWholebody::getOptJoint(Matrix<double, JOINT_DOF, 1> &opt_etadot, Matrix<double, JOINT_DOF, 1> &opt_tau, TimeDuration &time_status)
    {
        MatrixXd sol;
        if(!solveQP(sol, time_status))
        {
            opt_etadot.setZero();
            opt_tau.setZero();
            time_status.setZero();
            return false;
        }
        else
        {
            opt_etadot = sol.block(si_index_.eta_dot,0,JOINT_DOF,1);
            opt_tau = sol.block(si_index_.tau,0,JOINT_DOF,1);
            return true;
        }
    }

    void MOMAWholebody::setCost()
    {
        /*
              min     || x_ddot_des - J*eta_dot - J_tilda_dot * eta ||_2^2
        eta_dot, tau

        =>      min        1/2 * [eta_dot]^T * [2*J^T*J  0] * [eta_dot] + [J_tilda.T * -2(x_ddot_des - J_tilda_dot * eta)]^T * [eta_dot]
            [eta_dot, tau]       [tau    ]     [   0     0]   [tau    ]   [                    0                         ]     [tau    ]
        */
       MatrixXd J_tilda = robot_data_->getJacobianActuated();
       MatrixXd J_tilda_dot = robot_data_->getJacobianActuatedTimeVariation();
       VectorXd eta = robot_data_->getJointVelocityActuated();

       P_ds_.block(si_index_.eta_dot,si_index_.eta_dot,JOINT_DOF,JOINT_DOF) = 2.0 * J_tilda.transpose() * J_tilda;
       q_ds_.segment(0,JOINT_DOF) = -2.0 * J_tilda.transpose() * (xddot_desired_ - J_tilda_dot * eta);
    }

    void MOMAWholebody::setBoundConstraint()    
    {
        // Not yet implemented
    }

    void MOMAWholebody::setIneqConstraint()    
    {
        // Manipulator Joint Angle Limit
        VectorXd q_mani_min(MANI_DOF), q_mani_max(MANI_DOF);
        q_mani_min << -2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445,-3.0159;
        q_mani_max <<  2.7437,  1.7837,  2.9007, -0.1518,  2.8065, 4.5169, 3.0159;
    
        VectorXd q_mani = robot_data_->getJointPositionActuated().segment(0,MANI_DOF);
        VectorXd qdot_mani = robot_data_->getJointVelocityActuated().segment(0,MANI_DOF);
    
        VectorXd h_qmin(MANI_DOF), h_qmax(MANI_DOF);
        MatrixXd dh_qmin_dq(MANI_DOF, MANI_DOF), dh_qmax_dq(MANI_DOF, MANI_DOF);
        h_qmin = q_mani - q_mani_min - Vector7d::Constant(0.05);
        h_qmax = q_mani_max - q_mani - Vector7d::Constant(0.05);
        dh_qmin_dq = Matrix7d::Identity();
        dh_qmax_dq = -Matrix7d::Identity();
    
        MatrixXd A_qmin(MANI_DOF,MANI_DOF), A_qmax(MANI_DOF,MANI_DOF); 
        VectorXd l_qmin(MANI_DOF), l_qmax(MANI_DOF);
        
        getCBFConstraint(h_qmin, dh_qmin_dq, qdot_mani, A_qmin, l_qmin);
        getCBFConstraint(h_qmax, dh_qmax_dq, qdot_mani, A_qmax, l_qmax);
    
    
        A_ineq_ds_.block(si_index_.con_q_mani_min, si_index_.eta_dot, MANI_DOF, MANI_DOF) = A_qmin;
        A_ineq_ds_.block(si_index_.con_q_mani_max, si_index_.eta_dot, MANI_DOF, MANI_DOF) = A_qmax;
        l_ineq_ds_.segment(si_index_.con_q_mani_min, MANI_DOF) = l_qmin;
        l_ineq_ds_.segment(si_index_.con_q_mani_max, MANI_DOF) = l_qmax;
        u_ineq_ds_.segment(si_index_.con_q_mani_min, MANI_DOF).setConstant(std::numeric_limits<double>::infinity());
        u_ineq_ds_.segment(si_index_.con_q_mani_max, MANI_DOF).setConstant(std::numeric_limits<double>::infinity());

        // Manipulator Singularity
        // double mu = robot_data_->getManipulatorManipulability();
        // Vector7d dmu_dq = robot_data_->getDManipulatorManipulability();

        // double h_sing = mu - 0.02;
        // VectorXd dh_sing_dq = dmu_dq;

        // MatrixXd A_sing(1,MANI_DOF);
        // double l_sing;
        
        // getCBFConstraint(h_sing, dh_sing_dq, qdot_mani, A_sing, l_sing);

        // A_ineq_ds_.block(si_index_.con_mani_sing, si_index_.eta_dot, 1, MANI_DOF) = A_sing;
        // l_ineq_ds_(si_index_.con_mani_sing) = l_sing;
        // u_ineq_ds_(si_index_.con_mani_sing) = std::numeric_limits<double>::infinity();

        // A_ineq_ds_.block(0,0,1,7) = d_mu.transpose();
        // l_ineq_ds_(0) = -(d_rbf * d_mu).dot(qdot_mani) - DyrosMath::getRBF(-0.5, d_mu.dot(qdot_mani) + DyrosMath::getRBF(-0.5, mu));
        // u_ineq_ds_.setConstant(std::numeric_limits<double>::infinity());

    }

    void MOMAWholebody::setEqConstraint()    
    {
        /*
        subject to M * eta_dot + g = tau

        => subject to [M -I][eta_dot] = [-g]
                            [tau    ]
        */
        MatrixXd M_tilda  = robot_data_->getMassMatrixActuated();
        MatrixXd g_tilda = robot_data_->getGravityActuated();

        A_eq_ds_.block(si_index_.con_dyn,si_index_.eta_dot,JOINT_DOF,JOINT_DOF) = M_tilda;
        A_eq_ds_.block(si_index_.con_dyn,si_index_.tau,JOINT_DOF,JOINT_DOF) = -MatrixXd::Identity(JOINT_DOF,JOINT_DOF);

        b_eq_ds_.segment(si_index_.con_dyn, JOINT_DOF) = -g_tilda;
    }

    void MOMAWholebody::getCBFConstraint(const VectorXd& h, const MatrixXd& dh_dq, const VectorXd& qdot, MatrixXd& A, VectorXd& b)
    {
        const size_t nh = h.size();
        const size_t nq = qdot.size();

        assert(dh_dq.rows() == nh && dh_dq.cols() == nq);

        A.setZero(nh, nq);
        b.setZero(nh);

        A = dh_dq;
        // b = -(10*DyrosMath::getDRBF(h).asDiagonal() * dh_dq) * qdot
        //     - 10*DyrosMath::getRBF(dh_dq * qdot + 10*DyrosMath::getRBF(h));
        b = -(1.0 * dh_dq) * qdot
            -1.0*(dh_dq * qdot + 1.0*(h));
    }

    void MOMAWholebody::getCBFConstraint(const double& h, const VectorXd& dh_dq, const VectorXd& qdot, MatrixXd& A, double& b)
    {
        const size_t nq = qdot.size();

        assert(dh_dq.size() == nq);

        A.setZero(1,nq);
        b = 0;

        A = dh_dq.transpose();
        // b = -(DyrosMath::getDRBF(h) * dh_dq).dot(qdot) 
        //     - DyrosMath::getRBF(dh_dq.dot(qdot) + DyrosMath::getRBF(h));
        b = -(1.0 * dh_dq).dot(qdot) 
            - 1.0*(dh_dq.dot(qdot) + 1.0*(h));
    }
}