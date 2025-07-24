#pragma once
#include "robot_data/type_define.h"
#include <memory>

using namespace Eigen;

namespace RobotData
{
    namespace Mobile
    {
        class MobileBase
        {
            public:
                MobileBase(const KinematicParam& param);

                virtual bool updateState(const VectorXd& wheel_pos, const VectorXd& wheel_vel);

                // ================================ Compute Functions ================================
                virtual VectorXd computeBaseVel(const VectorXd& wheel_pos, const VectorXd& wheel_vel);
                virtual MatrixXd computeFKJacobian(const VectorXd& wheel_pos);

                // ================================ Get Functions ================================
                virtual const KinematicParam& getKineParam() const {return param_;}
                virtual const int& getWheelNum() const {return wheel_num_;}
                virtual const VectorXd& getWheelPosition() const {return wheel_pos_;} 
                virtual const VectorXd& getWheelVelocity() const {return wheel_vel_;} 
                virtual const VectorXd& getBaseVel() const {return base_vel_;}
                virtual const MatrixXd& getFKJacobian() const {return J_mobile_;}
                
            protected:
                KinematicParam param_;
                int wheel_num_;

                VectorXd wheel_pos_;
                VectorXd wheel_vel_;

                MatrixXd J_mobile_;
                VectorXd base_vel_;

            private:
                MatrixXd DifferentialFKJacobian();
                MatrixXd MecanumFKJacobian();
                MatrixXd CasterFKJacobian(const VectorXd& wheel_pos);

        };
    } // namespace Mobile
} // namespace RobotData