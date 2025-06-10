#ifndef HUSKY_FR3_CONTROLLER_HPP
#define HUSKY_FR3_CONTROLLER_HPP

#include <Eigen/Dense>
#include "husky_fr3_controller/robot_data.h"
#include "math_type_define.h"
#include "suhan_benchmark.h"

using namespace Eigen;

namespace HuskyFR3Controller
{
    class Controller
    {
    public:
        Controller(const double& dt,
                   const std::shared_ptr<RobotData>& robot_data);
        ~Controller();

        void starting();
        void updateState(const double current_time);
        void computeFast();
        void computeSlow();
        VectorXd getCtrlInput();
        void setMode(const std::string mode);

        VectorXd ManiPDControl(const VectorXd q_mani_desired, const VectorXd qdot_mani_desired);
        VectorXd MobileAdmControl(const VectorXd torque_mobile_desired);


    private :
        double dt_;
        double dt_mobile_{0.02};
        SuhanBenchmark mobile_timer_;

        std::shared_ptr<RobotData> robot_data_;
        bool is_mode_changed_{false};
        std::string mode_{"home"};
        double control_start_time_;
        double current_time_;
        
        //// joint space state
        VectorXd q_virtual_;
        VectorXd q_virtual_init_;
        VectorXd q_virtual_desired_;
        VectorXd qdot_virtual_;
        VectorXd qdot_virtual_init_;
        VectorXd qdot_virtual_desired_;

        VectorXd q_mani_;
        VectorXd q_mani_init_;
        VectorXd q_mani_desired_;
        VectorXd qdot_mani_;
        VectorXd qdot_mani_init_;
        VectorXd qdot_mani_desired_;

        VectorXd q_mobile_;
        VectorXd q_mobile_init_;
        VectorXd q_mobile_desired_;
        VectorXd qdot_mobile_;
        VectorXd qdot_mobile_init_;

        //// operation space state
        Affine3d x_;
        Affine3d x_init_;
        Affine3d x_desired_;
        Vector6d xdot_;
        Vector6d xdot_init_;
        Vector6d xdot_desired_;

        //// control input
        VectorXd torque_mani_desired_;
        VectorXd qdot_mobile_desired_;
    };
} // namespace HuskyFR3Controller

#endif // HUSKY_FR3_CONTROLLER_HPP