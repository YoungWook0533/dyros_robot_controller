#include "QP/QP_moma_wholebody.h"

namespace QP
{
    MOMAWholebody::MOMAWholebody(std::shared_ptr<RobotDataMobileManipulator> robot_data)
    :Base(), robot_data_(robot_data)
    {
        joint_dof_ = robot_data_->getActuatordDof();
        mani_dof_ = robot_data_->getManipulatorDof();

        const int nx = joint_dof_*2;
        const int nbound = 0;
        const int nineq = mani_dof_*2 + 2;
        const int neq = joint_dof_;

        Base::setQPsize(nx, nbound, nineq, neq);

        si_index_.eta_dot_start = 0;
        si_index_.tau_start     = joint_dof_;

        si_index_.con_dyn_start = 0;

        si_index_.con_q_mani_min_start = 0;
        si_index_.con_q_mani_max_start = mani_dof_;
        si_index_.con_sing             = mani_dof_ * 2;
        si_index_.con_sel_col          = mani_dof_ * 2 + 1;

    }

    void MOMAWholebody::setDesiredEEAcc(const VectorXd &xddot_desired)
    {
        xddot_desired_ = xddot_desired;
    }

    bool MOMAWholebody::getOptJoint(VectorXd &opt_etadot, VectorXd &opt_tau, TimeDuration &time_status)
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
            opt_etadot = sol.block(si_index_.eta_dot_start,0,joint_dof_,1);
            opt_tau = sol.block(si_index_.tau_start,0,joint_dof_,1);
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

       P_ds_.block(si_index_.eta_dot_start,si_index_.eta_dot_start,joint_dof_,joint_dof_) = 2.0 * J_tilda.transpose() * J_tilda;
       q_ds_.segment(0,joint_dof_) = -2.0 * J_tilda.transpose() * (xddot_desired_ - J_tilda_dot * eta);
       
       P_ds_.block(si_index_.eta_dot_start,si_index_.eta_dot_start,joint_dof_,joint_dof_) += 0.1*MatrixXd::Identity(joint_dof_,joint_dof_);
    }

    void MOMAWholebody::setBoundConstraint()    
    {
        // Not yet implemented
    }

    void MOMAWholebody::setIneqConstraint()    
    {

        double alpha = 50.;

        // Manipulator Joint Angle Limit
        VectorXd q_mani_min(mani_dof_), q_mani_max(mani_dof_);
        q_mani_min << -2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445,-3.0159;
        q_mani_max <<  2.7437,  1.7837,  2.9007, -0.1518,  2.8065, 4.5169, 3.0159;

        VectorXd q_actuated = robot_data_->getJointPositionActuated();
        VectorXd qdot_actuated = robot_data_->getJointVelocityActuated();
    
        VectorXd q_mani = q_actuated.segment(robot_data_->getActuatorIndex().mani_start,mani_dof_);
        VectorXd qdot_mani = qdot_actuated.segment(robot_data_->getActuatorIndex().mani_start,mani_dof_);
    
        // VectorXd h_qmin(mani_dof_), h_qmax(mani_dof_);
        // MatrixXd dh_qmin_dq(mani_dof_, mani_dof_), dh_qmax_dq(mani_dof_, mani_dof_);
        // h_qmin = q_mani - q_mani_min - Vector7d::Constant(0.01);
        // h_qmax = q_mani_max - q_mani - Vector7d::Constant(0.01);
        // dh_qmin_dq = MatrixXd::Identity(mani_dof_, mani_dof_);
        // dh_qmax_dq = -MatrixXd::Identity(mani_dof_, mani_dof_);
    
        // MatrixXd A_qmin(mani_dof_,mani_dof_), A_qmax(mani_dof_,mani_dof_); 
        // VectorXd l_qmin(mani_dof_), l_qmax(mani_dof_);
        
        // getCBFConstraint(h_qmin, dh_qmin_dq, qdot_mani, A_qmin, l_qmin);
        // getCBFConstraint(h_qmax, dh_qmax_dq, qdot_mani, A_qmax, l_qmax);
    
        // A_ineq_ds_.block(si_index_.con_q_mani_min_start, si_index_.eta_dot_start, mani_dof_, mani_dof_) = A_qmin;
        // A_ineq_ds_.block(si_index_.con_q_mani_max_start, si_index_.eta_dot_start, mani_dof_, mani_dof_) = A_qmax;
        // l_ineq_ds_.segment(si_index_.con_q_mani_min_start, mani_dof_) = l_qmin;
        // l_ineq_ds_.segment(si_index_.con_q_mani_max_start, mani_dof_) = l_qmax;
        
        A_ineq_ds_.block(si_index_.con_q_mani_min_start, si_index_.eta_dot_start, mani_dof_, mani_dof_).setIdentity();
        l_ineq_ds_.segment(si_index_.con_q_mani_min_start, mani_dof_) = -(alpha+alpha)*qdot_mani - alpha*alpha*(q_mani - q_mani_min);

        A_ineq_ds_.block(si_index_.con_q_mani_max_start, si_index_.eta_dot_start, mani_dof_, mani_dof_) = -MatrixXd::Identity(mani_dof_,mani_dof_);
        l_ineq_ds_.segment(si_index_.con_q_mani_max_start, mani_dof_) = +(alpha+alpha)*qdot_mani - alpha*alpha*(q_mani_max - q_mani);

        // singularity avoidance
        VectorXd mani_grad, mani_grad_dot;
        double mani = robot_data_->getManipulability(&mani_grad, &mani_grad_dot);

        A_ineq_ds_.block(si_index_.con_sing, si_index_.eta_dot_start, 1, mani_dof_) = mani_grad.transpose();
        l_ineq_ds_(si_index_.con_sing) = -mani_grad_dot.dot(qdot_mani) - (alpha + alpha)*mani_grad.dot(qdot_mani) - alpha*alpha*(mani -0.01);

        // self collision avoidance
        VectorXd min_dist_grad, min_dist_grad_dot;
        // double min_dist = robot_data_->getMinDistance(&min_dist_grad, &min_dist_grad_dot); TODO: fix getMinDistance
        double min_dist = robot_data_->computeMinDistance(robot_data_->getJointPosition(), robot_data_->getJointVelocity(), &min_dist_grad, &min_dist_grad_dot);

        min_dist_grad = min_dist_grad.segment(robot_data_->getJointIndex().mani_start, mani_dof_);
        min_dist_grad_dot = min_dist_grad_dot.segment(robot_data_->getJointIndex().mani_start, mani_dof_);

        A_ineq_ds_.block(si_index_.con_sel_col, si_index_.eta_dot_start, 1, mani_dof_) = min_dist_grad.transpose();
        l_ineq_ds_(si_index_.con_sel_col) = -min_dist_grad_dot.dot(qdot_mani) - (alpha + alpha)*min_dist_grad.dot(qdot_mani) - alpha*alpha*(min_dist -0.05);
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

        A_eq_ds_.block(si_index_.con_dyn_start,si_index_.eta_dot_start,joint_dof_,joint_dof_) = M_tilda;
        A_eq_ds_.block(si_index_.con_dyn_start,si_index_.tau_start,joint_dof_,joint_dof_) = -MatrixXd::Identity(joint_dof_,joint_dof_);

        b_eq_ds_.segment(si_index_.con_dyn_start, joint_dof_) = -g_tilda;
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
        b = -(10.0 * dh_dq) * qdot
            -10.0*(dh_dq * qdot + 10.0*(h));
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
        b = -(10.0 * dh_dq).dot(qdot) 
            - 10.0*(dh_dq.dot(qdot) + 10.0*(h));
    }
}