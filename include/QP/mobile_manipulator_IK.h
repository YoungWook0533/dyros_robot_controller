#pragma once

#include "QP/base.h"
#include "math_type_define.h"
#include "robot_data/mobile_manipulator/base.h"

namespace QP
{
    class MobileManipulatorIK : public QPBase
    {
        public:
            MobileManipulatorIK(std::shared_ptr<RobotData::MobileManipulator::MobileManipulatorBase> robot_data);
            void setDesiredTaskVel(const VectorXd &xdot_desired, const std::string &link_name);
            bool getOptJointVel(VectorXd &opt_qdot, TimeDuration &time_status);

        private:
            struct QPIndex
            {
                // decision variables
                int eta_start; // qdot_actuated 

                int eta_size;
                
                // inequality
                int con_q_mani_min_start;    // min q_manipulator
                int con_q_mani_max_start;    // max q_manipulator
                int con_sing_start;          // singularity
                int con_sel_col_start;       // self collision

                int con_q_mani_min_size;    // min q_manipulator size
                int con_q_mani_max_size;    // max q_manipulator size
                int con_sing_size;          // singularity size
                int con_sel_col_size;       // self collision size
            }si_index_;

            std::shared_ptr<RobotData::MobileManipulator::MobileManipulatorBase> robot_data_;
            
            int actuator_dof_;
            int mani_dof_;
            int mobi_dof_;
            VectorXd xdot_desired_;
            std::string link_name_;
            
            void setCost() override;
            void setBoundConstraint() override;
            void setIneqConstraint() override;
            void setEqConstraint() override;
    };
} // namespace QP