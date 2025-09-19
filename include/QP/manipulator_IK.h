#pragma once

#include "QP/base.h"
#include "math_type_define.h"
#include "robot_data/manipulator/base.h"

namespace QP
{
    class ManipulatorIK : public QPBase
    {
        public:
            ManipulatorIK(std::shared_ptr<RobotData::Manipulator::ManipulatorBase> robot_data);
            void setDesiredTaskVel(const VectorXd &xdot_desired, const std::string &link_name);
            bool getOptJointVel(VectorXd &opt_qdot, TimeDuration &time_status);

        private:
            struct QPIndex
            {
                // decision variables
                int qdot_start; 
                int slack_q_min_start;
                int slack_q_max_start;
                int slack_sing_start;
                int slack_sel_col_start;

                int qdot_size;
                int slack_q_min_size;
                int slack_q_max_size;
                int slack_sing_size;
                int slack_sel_col_size;

                // inequality
                int con_q_min_start;
                int con_q_max_start;
                int con_sing_start;    // singularity
                int con_sel_col_start; // self collision

                int con_q_min_size;
                int con_q_max_size;
                int con_sing_size;
                int con_sel_col_size;
            }si_index_;

            std::shared_ptr<RobotData::Manipulator::ManipulatorBase> robot_data_;

            int joint_dof_;
            VectorXd xdot_desired_;
            std::string link_name_;

            void setCost() override;
            void setBoundConstraint() override;
            void setIneqConstraint() override;
            void setEqConstraint() override;
    };
} // namespace QP