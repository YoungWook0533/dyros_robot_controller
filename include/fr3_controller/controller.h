#pragma once
#include "mujoco_ros_sim/ControllerInterface.hpp"
#include "mujoco_ros_sim/ControllerRegistry.hpp"

#include "fr3_controller/robot_data.h"

#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "math_type_define.h"

/*
FR3 MuJoCo Joint/Sensor Information
 id | name                 | type   | nq | nv | idx_q | idx_v
----+----------------------+--------+----+----+-------+------
  0 | fr3_joint1           | _Hinge |  1 |  1 |     0 |    0
  1 | fr3_joint2           | _Hinge |  1 |  1 |     1 |    1
  2 | fr3_joint3           | _Hinge |  1 |  1 |     2 |    2
  3 | fr3_joint4           | _Hinge |  1 |  1 |     3 |    3
  4 | fr3_joint5           | _Hinge |  1 |  1 |     4 |    4
  5 | fr3_joint6           | _Hinge |  1 |  1 |     5 |    5
  6 | fr3_joint7           | _Hinge |  1 |  1 |     6 |    6

 id | name                 | trn     | target_joint
----+----------------------+---------+-------------
  0 | fr3_joint1           | _Joint  | fr3_joint1
  1 | fr3_joint2           | _Joint  | fr3_joint2
  2 | fr3_joint3           | _Joint  | fr3_joint3
  3 | fr3_joint4           | _Joint  | fr3_joint4
  4 | fr3_joint5           | _Joint  | fr3_joint5
  5 | fr3_joint6           | _Joint  | fr3_joint6
  6 | fr3_joint7           | _Joint  | fr3_joint7

 id | name                        | type             | dim | adr | target (obj)
----+-----------------------------+------------------+-----+-----+----------------
*/
using namespace Eigen;

namespace FR3Controller
{
    typedef Eigen::Matrix<double,7,7> Matrix7d;
    typedef Eigen::Matrix<double,7,1> Vector7d;

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
    
    private:
        // ====================================================================================
        // ===================== Helper / CB / Background Thread Functions ==================== 
        // ====================================================================================
        void setMode(const std::string& mode);
        void keyCallback(const std_msgs::msg::Int32::SharedPtr);
        void subtargetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr);
        void pubEEPoseCallback();
        void computeSlowLoop();

        // ====================================================================================
        // ================================= Control Functions ================================
        // ====================================================================================
        Vector7d PDControl(const Vector7d& q_desired, 
                           const Vector7d& qdot_desired, 
                           const Vector7d& kp,
                           const Vector7d& kv);

        Vector7d CLIK(const Affine3d& target_x, const double& durarion);
    
        /* === data === */
        std::unique_ptr<RobotData> robot_;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr            key_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr    ee_pose_pub_;
    
        std::thread slow_thread_;
    
        bool is_mode_changed_{false};
        bool is_goal_pose_changed_{false};
        std::string mode_{"None"};

        double current_time_;
        double control_start_time_;

        std::vector<std::string> rd_joint_names_;
        
        Vector7d q_;       // joint angle
        Vector7d qdot_;    // joint velocity
        Vector7d tau_ext_;

        Vector7d q_desired_;
        Vector7d qdot_desired_;
        Vector7d tau_desired_;

        Vector7d q_init_;
        Vector7d qdot_init_;

        Affine3d x_;
        Vector6d xdot_;
        
        Affine3d x_init_;
        Vector6d xdot_init_;
        
        Affine3d x_desired_;
        Vector6d xdot_desired_;
        Affine3d x_goal_;

        rclcpp::TimerBase::SharedPtr ee_pose_pub_timer_;
    };
}