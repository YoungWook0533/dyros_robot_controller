#include "xls_fr3_controller/robot_data.h"


namespace XLSFR3Controller
{

XLSFR3RobotData::XLSFR3RobotData(const std::string& urdf_path, 
                                 const bool verbose)
: RobotDataMobileManipulator(urdf_path,
                             MANI_DOF,
                             MOBI_DOF,
                             joint_idx,
                             actuator_idx,
                             "fr3_link8",
                             verbose)
{

}

XLSFR3RobotData::XLSFR3RobotData(const std::string& urdf_path, 
                                 const std::string& srdf_path,
                                 const bool verbose)
: RobotDataMobileManipulator(urdf_path,
                             srdf_path,
                             MANI_DOF,
                             MOBI_DOF,
                             joint_idx,
                             actuator_idx,
                             "fr3_link8",
                             verbose)
{

}
   
MatrixXd XLSFR3RobotData::computeMobileFKJacobian(const VectorXd& q_mobile)
{
    Matrix<double,VIRTUAL_DOF,MOBI_DOF> J;
        double tmp = mobile_height_ + mobile_width_;
        J.setZero();
        J << 1.0,     1.0,     1.0,     1.0,      
            -1.0,     1.0,     1.0,    -1.0,      
            -2.0/tmp, 2.0/tmp,-2.0/tmp, 2.0/tmp;
        J *= wheel_radius_ / 4.0;

        return J;

    return J;
}

MatrixXd XLSFR3RobotData::computeSelectionMatrix(const VectorXd& q)
{
    MatrixXd S;
    S.setZero(JOINT_DOF,ACTUATOR_DOF);
    S.block(joint_idx.mani_start,actuator_idx.mani_start,MANI_DOF,MANI_DOF).setIdentity();
    S.block(joint_idx.mobi_start,actuator_idx.mobi_start,MOBI_DOF,MOBI_DOF).setIdentity();
    double mobile_yaw = q(joint_idx.virtual_start + 2);
    Matrix3d R_world2Base, v_mobile2mani;
    R_world2Base << cos(mobile_yaw), -sin(mobile_yaw), 0,
                    sin(mobile_yaw),  cos(mobile_yaw), 0,
                    0,                0,               1;
    v_mobile2mani << 1, 0, -mobile2mani_y,
                     0, 1,  mobile2mani_x,
                     0, 0,  1;
    S.block(joint_idx.virtual_start,actuator_idx.mobi_start,VIRTUAL_DOF,MOBI_DOF) = R_world2Base * v_mobile2mani * computeMobileFKJacobian(q.segment(joint_idx.mobi_start,MOBI_DOF));
    return S;
}

MatrixXd XLSFR3RobotData::computeSelectionMatrixTimeVariation(const VectorXd& q, const VectorXd& qdot)
{
    MatrixXd S;
    S.setZero(JOINT_DOF,ACTUATOR_DOF);
    S.block(joint_idx.mani_start,actuator_idx.mani_start,MANI_DOF,MANI_DOF).setIdentity();
    S.block(joint_idx.mobi_start,actuator_idx.mobi_start,MOBI_DOF,MOBI_DOF).setIdentity();
    double mobile_yaw = q(joint_idx.virtual_start + 2);
    Matrix3d R_world2Basedot, v_mobile2mani;
    R_world2Basedot << -sin(mobile_yaw),  -cos(mobile_yaw), 0,
                        cos(mobile_yaw),  -sin(mobile_yaw), 0,
                        0,                 0,               0;
    v_mobile2mani << 1, 0, -mobile2mani_y,
                     0, 1,  mobile2mani_x,
                     0, 0,  1;
    R_world2Basedot *= qdot(joint_idx.virtual_start + 2); // qdot(2) = θdot
    S.block(joint_idx.virtual_start,actuator_idx.mobi_start,VIRTUAL_DOF,MOBI_DOF) = R_world2Basedot * v_mobile2mani * computeMobileFKJacobian(q.segment(joint_idx.mobi_start,MOBI_DOF)); // J_tmp_ is constant wrt time
    return S;
}

}