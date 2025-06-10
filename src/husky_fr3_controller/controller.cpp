#include "husky_fr3_controller/controller.h"

namespace HuskyFR3Controller
{
    Controller::Controller(const double& dt,
                           const std::shared_ptr<RobotData>& robot_data)
    : dt_(dt), robot_data_(robot_data)
    {
        q_virtual_.setZero(3);
        q_virtual_init_.setZero(3);
        q_virtual_desired_.setZero(3);
        qdot_virtual_.setZero(3);
        qdot_virtual_init_.setZero(3);
        qdot_virtual_desired_.setZero(3);
        
        q_mani_.setZero(7);
        q_mani_init_.setZero(7);
        q_mani_desired_.setZero(7);
        qdot_mani_.setZero(7);
        qdot_mani_init_.setZero(7);
        qdot_mani_desired_.setZero(7);
        
        q_mobile_.setZero(2);
        q_mobile_init_.setZero(2);
        q_mobile_desired_.setZero(2);
        qdot_mobile_.setZero(2);
        qdot_mobile_init_.setZero(2);
        qdot_mobile_desired_.setZero(2);
        
        x_.setIdentity();
        x_init_.setIdentity();
        x_desired_.setIdentity();
        xdot_.setZero();
        xdot_init_.setZero();
        xdot_desired_.setZero();
        
        torque_mani_desired_.setZero(7);
        qdot_mobile_desired_.setZero(2);
    }

    Controller::~Controller()
    {

    }

    void Controller::starting()
    {
        q_virtual_init_ = q_virtual_;
        qdot_virtual_init_ = qdot_virtual_;
        q_virtual_desired_.setZero();

        q_mani_init_ = q_mani_;
        qdot_mani_init_ = qdot_mani_;
        qdot_mani_desired_.setZero();

        q_mobile_init_ = q_mobile_;
        qdot_mobile_init_ = qdot_mobile_;
        qdot_mobile_desired_.setZero();

        control_start_time_ = current_time_;
        mobile_timer_.reset();
    }

    void Controller::updateState(const double current_time)
    {
        //// robot_data_ are automatically updated in controller.py
        current_time_ = current_time;

        VectorXd tmp_q = robot_data_->getJointPosition();
        VectorXd tmp_qdot = robot_data_->getJointVelocity();

        q_virtual_.head(2) = tmp_q.head(2);
        q_virtual_(2) = std::atan2(tmp_q(3), tmp_q(2)); // tmp_q(2) = cos(θ), tmp_q(3) = sin(θ)
        qdot_virtual_ = tmp_qdot.head(3);

        q_mani_ = tmp_q.segment(4,7);
        qdot_mani_ = tmp_qdot.segment(3,7);

        q_mobile_ = tmp_q.segment(4+7,2);
        qdot_mobile_ = tmp_qdot.segment(3+7,2);

        x_.matrix() = robot_data_->getPose();
        xdot_ = robot_data_->getVelocity();
    }

    void Controller::computeFast()
    {
        if(is_mode_changed_)
        {
            is_mode_changed_ = false;

            q_virtual_init_ = q_virtual_;
            qdot_virtual_init_ = qdot_virtual_;
            q_virtual_desired_.setZero();

            q_mani_init_ = q_mani_;
            qdot_mani_init_ = qdot_mani_;
            qdot_mani_desired_.setZero();

            q_mobile_init_ = q_mobile_;
            qdot_mobile_init_ = qdot_mobile_;
            qdot_mobile_desired_.setZero();

            control_start_time_ = current_time_;
        }

        if(mode_ == "home")
        {
            VectorXd q_mani_target;
            q_mani_target.setZero(7);
            q_mani_target << 0, 0, 0, -M_PI/2, 0, M_PI/2, M_PI/4;

            q_mani_desired_ = DyrosMath::cubicVector<7>(current_time_,
                                                        control_start_time_,
                                                        control_start_time_ + 2.0,
                                                        q_mani_init_,
                                                        q_mani_target,
                                                        qdot_mani_init_,
                                                        VectorXd::Zero(7));
            qdot_mani_desired_ = DyrosMath::cubicDotVector<7>(current_time_,
                                                              control_start_time_,
                                                              control_start_time_ + 2.0,
                                                              q_mani_init_,
                                                              q_mani_target,
                                                              qdot_mani_init_,
                                                              VectorXd::Zero(7));

            torque_mani_desired_ = ManiPDControl(q_mani_desired_, qdot_mani_desired_);
            qdot_mobile_desired_.setZero();
        }
        else if(mode_ == "wholebody_grav_comp")
        {
            auto tmp_torque_desired = robot_data_->getGravityActuated();
            torque_mani_desired_ = tmp_torque_desired.head(7);
            if(mobile_timer_.elapsed() > dt_mobile_)
            {
                mobile_timer_.reset();   
                qdot_mobile_desired_ = MobileAdmControl(tmp_torque_desired.tail(2));
            }
        }
        else if(mode_ == "wholebody_impedence")
        {
            Affine3d target_x = x_init_;
            Vector6d x_error;
            x_error.head(3) = target_x.translation() - x_.translation();
            x_error.tail(3) = DyrosMath::getPhi(target_x.rotation(), x_.rotation());

            Vector6d Kp, Kv;
            Kp << 1,1,1,0,0,0;
            Kv << 0,0,0,0,0,0;

            Vector6d f = Kp.asDiagonal() * x_error - Kv.asDiagonal() * xdot_;
            MatrixXd J_tilda = robot_data_->getJacobianActuated();
            VectorXd etadot = (J_tilda.transpose() * J_tilda).inverse() * J_tilda.transpose() * (f);
            VectorXd tmp_torque_desired = robot_data_->getMassMatrixActuated() * etadot + robot_data_->getGravityActuated();

            torque_mani_desired_ = tmp_torque_desired.head(7);
            qdot_mobile_desired_ += qdot_mobile_desired_.tail(2) * dt_;

        }
        else
        {
            torque_mani_desired_ = (robot_data_->getGravityActuated()).head(7);
            qdot_mobile_desired_.setZero();
        }
    }

    void Controller::computeSlow()
    {

    }

    VectorXd Controller::getCtrlInput()
    {
        VectorXd ctrl_input;
        ctrl_input.setZero(torque_mani_desired_.size() + qdot_mobile_desired_.size());
        ctrl_input << torque_mani_desired_, qdot_mobile_desired_;
        return ctrl_input;
    }

    void Controller::setMode(const std::string mode)
    {
        mode_ = mode;
        is_mode_changed_ = true;
        std::cout << "[HuskyFR3Controller] Mode changed: " << mode << std::endl;
    }

    VectorXd Controller::ManiPDControl(const VectorXd q_mani_desired, const VectorXd qdot_mani_desired)
    {
        VectorXd Kp, Kv;
        Kp.setZero(7);
        Kv.setZero(7);
        Kp.setConstant(400);
        Kv.setConstant(40);
        
        VectorXd f = Kp.asDiagonal() * (q_mani_desired - q_mani_) + Kv.asDiagonal() * (qdot_mani_desired - qdot_mani_);
        return (robot_data_->getMassMatrixActuated()).block(0,0,7,7) * f + (robot_data_->getGravityActuated()).segment(0,7);
    }

    VectorXd Controller::MobileAdmControl(const VectorXd torque_mobile_desired)
    {
        double J = 0.02;   // wheel inertia [kg·m²]
        double B = 0.1;    // wheel damping [N·m·s/rad]

        VectorXd alpha = (torque_mobile_desired - B * qdot_mobile_) / J;
        VectorXd omega = qdot_mobile_ + alpha * dt_;

        return omega;
    }
}