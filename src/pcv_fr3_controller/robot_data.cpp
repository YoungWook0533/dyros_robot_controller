#include "pcv_fr3_controller/robot_data.h"


namespace PCVFR3Controller
{

PCVFR3RobotData::PCVFR3RobotData(const std::string& urdf_path, 
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

PCVFR3RobotData::PCVFR3RobotData(const std::string& urdf_path, 
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
   
MatrixXd PCVFR3RobotData::computeMobileFKJacobian(const VectorXd& q_mobile)
{
    Matrix<double,MOBI_DOF,3> J_p_tilda;
    Matrix<double,MOBI_DOF,MOBI_DOF> J_q_inv;
    J_p_tilda.setZero();
    J_q_inv.setZero();

    for(size_t i=0; i<4; i++)
    {
        double h = sqrt(pow(mobile_width_/2.0, 2) + pow(mobile_height_/2.0, 2));
        double phi = q_mobile(2*i); // steering angle
        
        double beta;
        if(i == 0)      beta = atan2(mobile_width_/2.0, mobile_height_/2.0);   // front left
        else if(i == 1) beta = atan2(-mobile_width_/2.0, mobile_height_/2.0);  // front right
        else if(i == 2) beta = atan2(mobile_width_/2.0, -mobile_height_/2.0);  // rear left
        else if(i == 3) beta = atan2(-mobile_width_/2.0, -mobile_height_/2.0); // rear right

        J_p_tilda.block(2*i, 0, 2, 3) << 1.0, 0.0, -(h*sin(beta) + wheel_offset_*sin(phi)),
                                         0.0, 1.0,  (h*cos(beta) + wheel_offset_*cos(phi));
        J_q_inv.block(2*i, 2*i, 2, 2) <<  wheel_offset_*sin(phi), wheel_radius_*cos(phi),
                                         -wheel_offset_*cos(phi), wheel_radius_*sin(phi);
    }
    return DyrosMath::PinvCOD(J_p_tilda.transpose() * J_p_tilda) * J_p_tilda.transpose() * J_q_inv;
}

MatrixXd PCVFR3RobotData::computeSelectionMatrix(const VectorXd& q)
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

MatrixXd PCVFR3RobotData::computeSelectionMatrixTimeVariation(const VectorXd& q, const VectorXd& qdot)
{
    // TODO
    return MatrixXd::Zero(JOINT_DOF,ACTUATOR_DOF);
}

}