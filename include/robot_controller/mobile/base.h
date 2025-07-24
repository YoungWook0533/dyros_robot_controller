#pragma once
#include "robot_data/mobile/base.h"
#include <memory>
#include <algorithm>

namespace RobotController
{
    namespace Mobile
    {
        class MobileBase
        {
            public:
                MobileBase(const double& dt, std::shared_ptr<RobotData::Mobile::MobileBase> robot_data);

                virtual VectorXd computeWheelVel(const VectorXd& base_vel);
                virtual MatrixXd computeIKJacobian();
                virtual VectorXd VelocityCommand(const VectorXd& desired_base_vel);
            
            protected:
                double dt_;
                int wheel_num_;
                std::shared_ptr<RobotData::Mobile::MobileBase> robot_data_;
                RobotData::Mobile::KinematicParam param_;

            private:
                MatrixXd DifferentialIKJacobian();
                MatrixXd MecanumIKJacobian();
                MatrixXd CasterIKJacobian();
        };
    } // namespace Mobile
} // namespace RobotController