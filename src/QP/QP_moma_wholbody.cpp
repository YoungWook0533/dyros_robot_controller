#include "QP/QP_moma_wholebody.h"

namespace QP
{
    MOMAWholebody::MOMAWholebody()
    :Base()
    {
        Base::setQPsize(JOINT_DOF*2, JOINT_DOF);
    }

    void MOMAWholebody::setCurrentState(std::shared_ptr<const HuskyFR3Controller::RobotData> robot_data)
    {
        robot_data_ = std::move(robot_data);
    }

    void MOMAWholebody::setDesiredEEAcc(const Matrix<double, TASK_DOF, 1> &xddot_desired)
    {
        xddot_desired_ = xddot_desired;
    }

    bool MOMAWholebody::getOptJoint(Matrix<double, JOINT_DOF, 1> &opt_qdot, Matrix<double, JOINT_DOF, 1> &opt_tau, TimeDuration &time_status)
    {

    }

    void MOMAWholebody::setCost()
    {

    }

    void MOMAWholebody::setConstraint()    
    {

    }
}