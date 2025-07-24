#pragma once
#include <vector>
#include <Eigen/Dense>
#include "math_type_define.h"

using namespace Eigen;

namespace RobotData
{
    namespace Mobile
    {
        enum class DriveType {
            Differential,
            Mecanum,
            Caster
        };

        struct KinematicParam
        {
            DriveType type;

            double wheel_radius;       // Radius of the wheels (m)
            double max_lin_speed = 2;  // Maximum linear speed of the robot (m/s)
            double max_ang_speed = 2;  // Maximum angular speed of the robot (rad/s)
            double max_lin_acc   = 2;  // Maximum linear acceleration of the robot (m/s^2)
            double max_ang_acc   = 2;  // Maximum angular acceleration of the robot (rad/s^2)

            // ---- Differential ----
            double base_width; // Distance between the left and right wheels (m)

            // ---- Mecanum ----
            std::vector<double> roller_angles;          // Angles of the rollers wrt the wheels (rad)
            std::vector<Vector2d> base2wheel_positions; // Positions of the wheels in the robot frame [x,y] (m)
            std::vector<double> base2wheel_angles;      // Angles of the wheels in the robot frame (rad)

            // ---- Caster ----
            double wheel_offset;                        // Offset from the rotaiting axis to the wheel (m)
            // std::vector<Vector2d> base2wheel_positions; // Positions of the wheels in the robot frame [x,y] (m)
        };
    }

    namespace Manipulator
    {
        struct MinDistResult
        {
            double   distance;
            VectorXd grad;
            VectorXd grad_dot;
    
            void setZero(const int size){distance=0; grad.setZero(size); grad_dot.setZero(size);}
        };
    
        struct ManipulabilityResult
        {
            double   manipulability;
            VectorXd grad;
            VectorXd grad_dot;
    
            void setZero(const int size){manipulability=0; grad.setZero(size); grad_dot.setZero(size);}
        };
    }

    namespace MobileManipulator
    {
        struct JointIndex
        {
            int virtual_start;
            int mani_start;
            int mobi_start;
        };

        struct ActuatorIndex
        {
            int mani_start;
            int mobi_start;
        };
    }
}