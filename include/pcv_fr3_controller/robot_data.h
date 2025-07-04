#pragma once
#include "robot_data/mobile_manipulator.h"

using namespace Eigen;

namespace PCVFR3Controller
{
    /*
    Joint Information

    Total nq = 18
    Total nv = 18

     id | name                | nq | nv | idx_q | idx_v
    ----+---------------------+----+----+-------+------
      1 |           v_x_joint |  1 |  1 |     0 |    0
      2 |           v_y_joint |  1 |  1 |     1 |    1
      3 |           v_t_joint |  1 |  1 |     2 |    2
      4 |          fr3_joint1 |  1 |  1 |     3 |    3
      5 |          fr3_joint2 |  1 |  1 |     4 |    4
      6 |          fr3_joint3 |  1 |  1 |     5 |    5
      7 |          fr3_joint4 |  1 |  1 |     6 |    6
      8 |          fr3_joint5 |  1 |  1 |     7 |    7
      9 |          fr3_joint6 |  1 |  1 |     8 |    8
     10 |          fr3_joint7 |  1 |  1 |     9 |    9
     11 |    front_left_steer |  1 |  1 |    10 |   10
     12 |   front_left_rotate |  1 |  1 |    11 |   11
     13 |   front_right_steer |  1 |  1 |    12 |   12
     14 |  front_right_rotate |  1 |  1 |    13 |   13
     15 |     rear_left_steer |  1 |  1 |    14 |   14
     16 |    rear_left_rotate |  1 |  1 |    15 |   15
     17 |    rear_right_steer |  1 |  1 |    16 |   16
     18 |   rear_right_rotate |  1 |  1 |    17 |   17
    */
    inline constexpr int TASK_DOF     = 6;
    inline constexpr int VIRTUAL_DOF  = 3;
    inline constexpr int MANI_DOF     = 7;
    inline constexpr int MOBI_DOF     = 8;
    inline constexpr int ACTUATOR_DOF = MANI_DOF + MOBI_DOF;
    inline constexpr int JOINT_DOF    = ACTUATOR_DOF + VIRTUAL_DOF;

    inline const RobotDataMobileManipulator::JointIndex joint_idx{0,VIRTUAL_DOF,VIRTUAL_DOF+MANI_DOF};
    inline const RobotDataMobileManipulator::ActuatorIndex actuator_idx{0,MANI_DOF};

    typedef Eigen::Matrix<double,TASK_DOF,1>                TaskVec;
    typedef Eigen::Matrix<double,TASK_DOF,TASK_DOF>         TaskMat;
    typedef Eigen::Matrix<double,VIRTUAL_DOF,1>             VirtualVec;
    typedef Eigen::Matrix<double,MANI_DOF,1>                ManiVec;
    typedef Eigen::Matrix<double,MOBI_DOF,1>                MobiVec;
    typedef Eigen::Matrix<double,ACTUATOR_DOF,1>            ActuatorVec;
    typedef Eigen::Matrix<double,ACTUATOR_DOF,ACTUATOR_DOF> ActuatorMat;
    typedef Eigen::Matrix<double,JOINT_DOF,1>               JointVec;
    typedef Eigen::Matrix<double,JOINT_DOF,JOINT_DOF>       JointMat;

    class PCVFR3RobotData: public RobotDataMobileManipulator
    {
        public: 
            PCVFR3RobotData(const std::string& urdf_path, 
                            const bool verbose=false);
            PCVFR3RobotData(const std::string& urdf_path,
                              const std::string& srdf_path, 
                              const bool verbose=false);

            MatrixXd computeMobileFKJacobian(const VectorXd& q_mobile);
            MatrixXd computeSelectionMatrix(const VectorXd& q);
            MatrixXd computeSelectionMatrixTimeVariation(const VectorXd& q, const VectorXd& qdot);
        
        private:
            const double wheel_radius_ = 0.055;
            const double wheel_offset_ = 0.020;
            const double mobile_width_ = 0.125*2.;
            const double mobile_height_ = 0.215*2.;
            const double mobile2mani_x = 0.1;
            const double mobile2mani_y = 0.;
    };
}