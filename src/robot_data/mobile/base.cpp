#pragma once 
#include "robot_data/mobile/base.h"
#include <iostream>
namespace RobotData
{
    namespace Mobile
    {
        MobileBase::MobileBase(const KinematicParam& param)
        : param_(param)
        {
            std::cout << "Mobile Type: ";
            if(param_.type == DriveType::Differential)
            {
                std::cout << "Differential " << std::endl;
                std::cout << "base_width: " << param_.base_width << std::endl;
                wheel_num_ = 2;
            }
            else if(param_.type == DriveType::Mecanum)
            {
                std::cout << "Mecanum " << std::endl;
                assert(param_.roller_angles.size() == param_.base2wheel_positions.size() &&
                param_.roller_angles.size() == param_.base2wheel_angles.size() &&
                param_.base2wheel_positions.size() == param_.base2wheel_angles.size());
                wheel_num_ = param_.roller_angles.size();
                std::cout << "roller_angles: ";
                for(size_t i=0; i<param_.roller_angles.size(); i++) std::cout << param_.roller_angles[i] << " / ";
                std::cout << "\nbase2wheel_positions: ";
                for(size_t i=0; i<param_.base2wheel_positions.size(); i++) std::cout << param_.base2wheel_positions[i].transpose() << " / ";
                std::cout << "\nbase2wheel_angles: ";
                for(size_t i=0; i<param_.base2wheel_angles.size(); i++) std::cout << param_.base2wheel_angles[i] << " / ";
            }
            else if(param_.type == DriveType::Caster)
            {
                std::cout << "Caster " << std::endl;
                std::cout << "offset: " << param_.wheel_offset << std::endl;
                std::cout << "base2wheel_positions: ";
                for(size_t i=0; i<param_.base2wheel_positions.size(); i++) std::cout << param_.base2wheel_positions[i].transpose() << " / ";
                wheel_num_ = int(param_.base2wheel_positions.size()*2);
            }

            std::cout << "\nWheel num: " << wheel_num_ << std::endl;
            wheel_pos_.setZero(wheel_num_);
            wheel_vel_.setZero(wheel_num_);

            J_mobile_.setZero(3, wheel_num_);
            base_vel_.setZero(3);
        }

        bool MobileBase::updateState(const VectorXd& wheel_pos, const VectorXd& wheel_vel)
        {
            assert(wheel_pos.size() == wheel_num_ && wheel_vel.size() == wheel_num_);
            
            wheel_pos_ = wheel_pos;
            wheel_vel_ = wheel_vel;
            
            J_mobile_ = computeFKJacobian(wheel_pos);
            base_vel_ = J_mobile_ * wheel_vel_;

            return true;
        }

        VectorXd MobileBase::computeBaseVel(const VectorXd& wheel_pos, const VectorXd& wheel_vel)
        {
            assert(wheel_pos.size() == wheel_num_ && wheel_vel.size() == wheel_num_);
            return computeFKJacobian(wheel_pos) * wheel_vel; // Return the forward kinematics result as base velocity
        }

        MatrixXd MobileBase::computeFKJacobian(const VectorXd& wheel_pos)
        {
            switch (param_.type) 
            {
                case DriveType::Differential:
                    return DifferentialFKJacobian();
                case DriveType::Mecanum:
                    return MecanumFKJacobian();
                case DriveType::Caster:
                    return CasterFKJacobian(wheel_pos);
                default:
                    throw std::runtime_error("Unknown DriveType");
            }
        }

        MatrixXd MobileBase::DifferentialFKJacobian()
        {
            MatrixXd J(3, wheel_num_);
            J.setZero();
            J << param_.wheel_radius/2.,                  param_.wheel_radius/2.,
                 0.,                                      0.,
                -param_.wheel_radius/(param_.base_width), param_.wheel_radius/(param_.base_width);
            
            return J;
        }

        MatrixXd MobileBase::MecanumFKJacobian()
        {
            MatrixXd J_inv(wheel_num_, 3);
            J_inv.setZero();

            for(size_t i=0; i<wheel_num_; i++)
            {
                double r = param_.wheel_radius;
                double gamma = param_.roller_angles[i];
                double p_x = param_.base2wheel_positions[i](0);
                double p_y = param_.base2wheel_positions[i](1);
                double p_t = param_.base2wheel_angles[i];

                MatrixXd A1, A2, A3;
                A1.setZero(2,3);
                A2.setZero(2,2);
                A3.setZero(1,2);

                A1 << 1, 0, -p_y,
                      0, 1, p_x;
                A2 <<  cos(p_t), sin(p_t),
                      -sin(p_t), cos(p_t);
                A3 << 1, tan(gamma);
                J_inv.row(i) = (1.0 / r) * A3 * A2 * A1;
            }
            
            return DyrosMath::PinvCOD(J_inv);
        }

        // Holmberg, Robert, and Oussama Khatib. "A POWERED-CASTER HOLONOMIC ROBOTIC VEHICLE FOR MOBILE MANIPULATION TASKS."    
        MatrixXd MobileBase::CasterFKJacobian(const VectorXd& wheel_pos)
        {        
            VectorXd steer_angle(int(wheel_num_/2));
            for(size_t i=0; i<steer_angle.size(); i++) steer_angle(i) = wheel_pos(2*i);

            MatrixXd J_p_tilda(wheel_num_, 3), J_q_inv(wheel_num_, wheel_num_);
            J_p_tilda.setZero();
            J_q_inv.setZero();
            
            for(size_t i=0; i<steer_angle.size(); i++)
            {
                double r = param_.wheel_radius;
                double b = param_.wheel_offset;
                double px = param_.base2wheel_positions[i](0);
                double py = param_.base2wheel_positions[i](1);
                double phi = steer_angle(i);

                J_p_tilda.block(2*i,0,2,3) << 1, 0, -(py + b*sin(phi)),
                                              0, 1,  (px + b*cos(phi));
                J_q_inv.block(2*i,2*i,2,2) <<  b*sin(phi), r*cos(phi),
                                              -b*cos(phi), r*sin(phi);

            }

            return DyrosMath::PinvCOD(J_p_tilda.transpose() * J_p_tilda) * J_p_tilda.transpose() * J_q_inv;
        }
    }
}