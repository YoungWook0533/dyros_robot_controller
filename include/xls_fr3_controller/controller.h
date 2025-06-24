#pragma once
#include "mujoco_ros_sim/ControllerInterface.hpp"
#include "mujoco_ros_sim/ControllerRegistry.hpp"

#include "xls_fr3_controller/robot_data.h"

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcutils/logging_macros.h>

#include "math_type_define.h"
#include "suhan_benchmark.h"

using namespace Eigen;

/*
XLS FR3 Joint/Sensor Information
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  1 | front_right_wheel_rolling_joint | _Hinge |  1 |  1 |     7 |    6
  2 | front_right_slipping_0_joint | _Hinge |  1 |  1 |     8 |    7
  3 | front_right_slipping_1_joint | _Hinge |  1 |  1 |     9 |    8
  4 | front_right_slipping_2_joint | _Hinge |  1 |  1 |    10 |    9
  5 | front_right_slipping_3_joint | _Hinge |  1 |  1 |    11 |   10
  6 | front_right_slipping_4_joint | _Hinge |  1 |  1 |    12 |   11
  7 | front_right_slipping_5_joint | _Hinge |  1 |  1 |    13 |   12
  8 | front_right_slipping_6_joint | _Hinge |  1 |  1 |    14 |   13
  9 | front_right_slipping_7_joint | _Hinge |  1 |  1 |    15 |   14
 10 | front_right_slipping_8_joint | _Hinge |  1 |  1 |    16 |   15
 11 | front_right_slipping_9_joint | _Hinge |  1 |  1 |    17 |   16
 12 | front_right_slipping_10_joint | _Hinge |  1 |  1 |    18 |   17
 13 | front_right_slipping_11_joint | _Hinge |  1 |  1 |    19 |   18
 14 | front_left_wheel_rolling_joint | _Hinge |  1 |  1 |    20 |   19
 15 | front_left_slipping_0_joint | _Hinge |  1 |  1 |    21 |   20
 16 | front_left_slipping_1_joint | _Hinge |  1 |  1 |    22 |   21
 17 | front_left_slipping_2_joint | _Hinge |  1 |  1 |    23 |   22
 18 | front_left_slipping_3_joint | _Hinge |  1 |  1 |    24 |   23
 19 | front_left_slipping_4_joint | _Hinge |  1 |  1 |    25 |   24
 20 | front_left_slipping_5_joint | _Hinge |  1 |  1 |    26 |   25
 21 | front_left_slipping_6_joint | _Hinge |  1 |  1 |    27 |   26
 22 | front_left_slipping_7_joint | _Hinge |  1 |  1 |    28 |   27
 23 | front_left_slipping_8_joint | _Hinge |  1 |  1 |    29 |   28
 24 | front_left_slipping_9_joint | _Hinge |  1 |  1 |    30 |   29
 25 | front_left_slipping_10_joint | _Hinge |  1 |  1 |    31 |   30
 26 | front_left_slipping_11_joint | _Hinge |  1 |  1 |    32 |   31
 27 | rear_right_wheel_rolling_joint | _Hinge |  1 |  1 |    33 |   32
 28 | rear_right_slipping_0_joint | _Hinge |  1 |  1 |    34 |   33
 29 | rear_right_slipping_1_joint | _Hinge |  1 |  1 |    35 |   34
 30 | rear_right_slipping_2_joint | _Hinge |  1 |  1 |    36 |   35
 31 | rear_right_slipping_3_joint | _Hinge |  1 |  1 |    37 |   36
 32 | rear_right_slipping_4_joint | _Hinge |  1 |  1 |    38 |   37
 33 | rear_right_slipping_5_joint | _Hinge |  1 |  1 |    39 |   38
 34 | rear_right_slipping_6_joint | _Hinge |  1 |  1 |    40 |   39
 35 | rear_right_slipping_7_joint | _Hinge |  1 |  1 |    41 |   40
 36 | rear_right_slipping_8_joint | _Hinge |  1 |  1 |    42 |   41
 37 | rear_right_slipping_9_joint | _Hinge |  1 |  1 |    43 |   42
 38 | rear_right_slipping_10_joint | _Hinge |  1 |  1 |    44 |   43
 39 | rear_right_slipping_11_joint | _Hinge |  1 |  1 |    45 |   44
 40 | rear_left_wheel_rolling_joint | _Hinge |  1 |  1 |    46 |   45
 41 | rear_left_slipping_0_joint | _Hinge |  1 |  1 |    47 |   46
 42 | rear_left_slipping_1_joint | _Hinge |  1 |  1 |    48 |   47
 43 | rear_left_slipping_2_joint | _Hinge |  1 |  1 |    49 |   48
 44 | rear_left_slipping_3_joint | _Hinge |  1 |  1 |    50 |   49
 45 | rear_left_slipping_4_joint | _Hinge |  1 |  1 |    51 |   50
 46 | rear_left_slipping_5_joint | _Hinge |  1 |  1 |    52 |   51
 47 | rear_left_slipping_6_joint | _Hinge |  1 |  1 |    53 |   52
 48 | rear_left_slipping_7_joint | _Hinge |  1 |  1 |    54 |   53
 49 | rear_left_slipping_8_joint | _Hinge |  1 |  1 |    55 |   54
 50 | rear_left_slipping_9_joint | _Hinge |  1 |  1 |    56 |   55
 51 | rear_left_slipping_10_joint | _Hinge |  1 |  1 |    57 |   56
 52 | rear_left_slipping_11_joint | _Hinge |  1 |  1 |    58 |   57
 53 | fr3_joint1           | _Hinge |  1 |  1 |    59 |   58
 54 | fr3_joint2           | _Hinge |  1 |  1 |    60 |   59
 55 | fr3_joint3           | _Hinge |  1 |  1 |    61 |   60
 56 | fr3_joint4           | _Hinge |  1 |  1 |    62 |   61
 57 | fr3_joint5           | _Hinge |  1 |  1 |    63 |   62
 58 | fr3_joint6           | _Hinge |  1 |  1 |    64 |   63
 59 | fr3_joint7           | _Hinge |  1 |  1 |    65 |   64

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | front_right_wheel    | _Joint  | front_right_wheel_rolling_joint
  1 | front_left_wheel     | _Joint  | front_left_wheel_rolling_joint
  2 | rear_right_wheel     | _Joint  | rear_right_wheel_rolling_joint
  3 | rear_left_wheel      | _Joint  | rear_left_wheel_rolling_joint
  4 | fr3_joint1           | _Joint  | fr3_joint1
  5 | fr3_joint2           | _Joint  | fr3_joint2
  6 | fr3_joint3           | _Joint  | fr3_joint3
  7 | fr3_joint4           | _Joint  | fr3_joint4
  8 | fr3_joint5           | _Joint  | fr3_joint5
  9 | fr3_joint6           | _Joint  | fr3_joint6
 10 | fr3_joint7           | _Joint  | fr3_joint7

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
  0 | position_sensor             | Framepos         |   3 |   0 | Site:xls_site
  1 | orientation_sensor          | Framequat        |   4 |   3 | Site:xls_site
  2 | linear_velocity_sensor      | Framelinvel      |   3 |   7 | Site:xls_site
  3 | angular_velocity_sensor     | Frameangvel      |   3 |  10 | Site:xls_site
*/

namespace XLSFR3Controller
{
    // Manipulator
    typedef Eigen::Matrix<double,7,7> Matrix7d;
    typedef Eigen::Matrix<double,7,1> Vector7d;
    // Manipulator + Mobile wheel
    typedef Eigen::Matrix<double,11,11> Matrix11d;
    typedef Eigen::Matrix<double,11,1> Vector11d;
    // Manipulator + Mobile wheel + Virtual
    typedef Eigen::Matrix<double,14,14> Matrix14d;
    typedef Eigen::Matrix<double,14,1> Vector14d;

    class Controller : public ControllerInterface
    {
    public:
        // ====================================================================================
        // ================================== Core Functions ================================== 
        // ====================================================================================
        Controller(const rclcpp::Node::SharedPtr& node, double dt, JointDict jd);
        ~Controller() override;
        void starting() override;
        void updateState(const VecMap&,const VecMap&, const VecMap&, const VecMap&, double) override;
        void compute() override;
        CtrlInputMap getCtrlInput() const override;

        // ====================================================================================
        // ===================== Helper / CB / Background Thread Functions ==================== 
        // ====================================================================================
        void setMode(const std::string& mode);
        void keyCallback(const std_msgs::msg::Int32::SharedPtr);
        void subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
        void subtargetBaseVelCallback(const geometry_msgs::msg::Twist::SharedPtr);
        void pubEEPoseCallback();
        void pubTMP1BaseVelCallback();
        void pubTMP2BaseVelCallback();
        void computeSlowLoop();

        // ====================================================================================
        // ================================= Control Functions ================================
        // ====================================================================================
        VectorXd ManiPDControl(const VectorXd& q_mani_desired, const VectorXd& qdot_mani_desired);
        VectorXd MobileAdmControl(const VectorXd& torque_mobile_desired);
        VectorXd MobileIK(const VectorXd& desired_base_vel);

    private :
        std::unique_ptr<RobotData> robot_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr       base_vel_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    ee_pose_pub_;
        
        rclcpp::TimerBase::SharedPtr ee_pose_pub_timer_;
        
        std::vector<std::string> rd_joint_names_;

        SuhanBenchmark mobile_timer_;

        bool is_mode_changed_{false};
        bool is_goal_pose_changed_{false};
        std::string mode_{"home"};
        double control_start_time_;
        double current_time_;

        Vector3d base_vel_; // [lin_x, lin_y, ang]
        Vector3d base_vel_desired_;
        
        //// joint space state
        Vector3d q_virtual_;
        Vector4d q_virtual_tmp_;
        Vector3d q_virtual_init_;
        Vector3d q_virtual_desired_;
        Vector3d qdot_virtual_;
        Vector3d qdot_virtual_init_;
        Vector3d qdot_virtual_desired_;

        Vector7d q_mani_;
        Vector7d q_mani_init_;
        Vector7d q_mani_desired_;
        Vector7d qdot_mani_;
        Vector7d qdot_mani_init_;
        Vector7d qdot_mani_desired_;

        Vector4d q_mobile_;
        Vector4d q_mobile_init_;
        Vector4d q_mobile_desired_;
        Vector4d qdot_mobile_;
        Vector4d qdot_mobile_init_;

        //// operation space state
        Affine3d x_;
        Affine3d x_goal_;
        Affine3d x_init_;
        Affine3d x_desired_;
        Vector6d xdot_;
        Vector6d xdot_init_;
        Vector6d xdot_desired_;

        //// control input
        Vector7d torque_mani_desired_;
        Vector4d qdot_mobile_desired_;
    };
} // namespace XLSFR3Controller