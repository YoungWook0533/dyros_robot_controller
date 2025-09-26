#pragma once
#include "dyros_robot_controller/QP_base.h"
#include "dyros_robot_controller/manipulator/robot_data.h"
#include "math_type_define.h"

namespace drc
{
    namespace Manipulator
    {
        class QPID : public QP::QPBase
        {
            public:
                QPID(std::shared_ptr<Manipulator::RobotData> robot_data);
                void setDesiredTaskAcc(const VectorXd &xddot_desired, const std::string &link_name);
                bool getOptJoint(VectorXd &opt_qddot, VectorXd &opt_torque, QP::TimeDuration &time_status);

            private:
                struct QPIndex
                {
                    // decision variables
                    int qddot_start; 
                    int torque_start;
                    int slack_q_min_start;
                    int slack_q_max_start;
                    int slack_qdot_min_start;
                    int slack_qdot_max_start;
                    int slack_sing_start;
                    int slack_sel_col_start;

                    int qddot_size;
                    int torque_size;
                    int slack_q_min_size;
                    int slack_q_max_size;
                    int slack_qdot_min_size;
                    int slack_qdot_max_size;
                    int slack_sing_size;
                    int slack_sel_col_size;
                    
                    // equality
                    int con_dyn_start; // dynamics constraint

                    int con_dyn_size;
                    
                    // inequality
                    int con_q_min_start;    // min joint angle
                    int con_q_max_start;    // max joint angle
                    int con_qdot_min_start; // min joint velocity
                    int con_qdot_max_start; // max joint velocity
                    int con_sing_start;     // singularity
                    int con_sel_col_start;  // self collision


                    int con_q_min_size;    // min joint angle size
                    int con_q_max_size;    // max joint angle size
                    int con_qdot_min_size; // min joint velocity size
                    int con_qdot_max_size; // max joint velocity size
                    int con_sing_size;     // singularity size
                    int con_sel_col_size;  // self collision size

                }si_index_;

                std::shared_ptr<Manipulator::RobotData> robot_data_;
                
                int joint_dof_;
                VectorXd xddot_desired_;
                std::string link_name_;
                
                void setCost() override;
                void setBoundConstraint() override;
                void setIneqConstraint() override;
                void setEqConstraint() override;
        };
    } // namespace QP
} // namespacce drc