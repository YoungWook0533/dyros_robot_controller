#pragma once
#include "robot_data/mobile_manipulator.h"

using namespace Eigen;

namespace XLSFR3Controller
{
    /*
    Joint Information
    Total nq = 14
    Total nv = 14
    
     id | name              | nq | nv | idx_q | idx_v
    ----+-------------------+----+----+-------+------
      1 |         v_x_joint |  1 |  1 |     0 |    0
      2 |         v_y_joint |  1 |  1 |     1 |    1
      3 |         v_t_joint |  1 |  1 |     2 |    2
      4 |        fr3_joint1 |  1 |  1 |     3 |    3
      5 |        fr3_joint2 |  1 |  1 |     4 |    4
      6 |        fr3_joint3 |  1 |  1 |     5 |    5
      7 |        fr3_joint4 |  1 |  1 |     6 |    6
      8 |        fr3_joint5 |  1 |  1 |     7 |    7
      9 |        fr3_joint6 |  1 |  1 |     8 |    8
     10 |        fr3_joint7 |  1 |  1 |     9 |    9
     11 |  front_left_wheel |  1 |  1 |    10 |   10
     12 | front_right_wheel |  1 |  1 |    11 |   11
     13 |   rear_left_wheel |  1 |  1 |    12 |   12
     14 |  rear_right_wheel |  1 |  1 |    13 |   13
    */
    inline constexpr int TASK_DOF     = 6;
    inline constexpr int VIRTUAL_DOF  = 3;
    inline constexpr int MANI_DOF     = 7;
    inline constexpr int MOBI_DOF     = 4;
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

    class XLSFR3RobotData: public RobotDataMobileManipulator
    {
        public: 
            XLSFR3RobotData(const std::string& urdf_path, 
                              const bool verbose=false);
            XLSFR3RobotData(const std::string& urdf_path,
                              const std::string& srdf_path, 
                              const bool verbose=false);

            MatrixXd computeMobileFKJacobian(const VectorXd& q_mobile);
            MatrixXd computeSelectionMatrix(const VectorXd& q);
            MatrixXd computeSelectionMatrixTimeVariation(const VectorXd& q, const VectorXd& qdot);
        
        private:
            const double wheel_radius_ = 0.127;
            const double mobile_width_ = 0.2045*2.;
            const double mobile_height_ = 0.2225*2.;
            const double mobile2mani_x = 0.2;
            const double mobile2mani_y = 0.;
    };
}