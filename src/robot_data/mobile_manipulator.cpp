#include "robot_data/mobile_manipulator.h"

RobotDataMobileManipulator::RobotDataMobileManipulator(const std::string& urdf_path, 
                                                       const std::string& srdf_path, 
                                                       const int& mani_dof, 
                                                       const int& mobi_dof,
                                                       const JointIndex& joint_idx,
                                                       const ActuatorIndex& actuator_idx,
                                                       const std::string& ee_name, 
                                                       const bool verbose)
: RobotDataBase(urdf_path, srdf_path, verbose),
  mani_dof_(mani_dof),
  mobi_dof_(mobi_dof),
  joint_idx_(joint_idx),
  actuator_idx_(actuator_idx)
{
    ee_name_ = ee_name;
    actuated_dof_ = mobi_dof_ + mani_dof_;
    total_dof_ = actuated_dof_ + virtual_dof_;

    ee_index_ = model_.getFrameId(ee_name_);
    if (ee_index_ == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << ee_name_ << " not found in URDF." << "\033[0m" << std::endl;
    }

    // Initialize selection matrix
    S_.setZero(total_dof_,actuated_dof_);
    Sdot_.setZero(total_dof_,actuated_dof_);
    J_mobile_.setZero(virtual_dof_,mobi_dof_);
    
    // Initialize joint space state
    q_.setZero(total_dof_);
    qdot_.setZero(total_dof_);

    // Initialize actuated joint space state
    q_actuated_.setZero(actuated_dof_);
    qdot_actuated_.setZero(actuated_dof_);

    // Initialize task space state
    x_.setIdentity();
    xdot_.setZero(6);
    J_.setZero(6,total_dof_);
    Jdot_.setZero(6,total_dof_);
    
    // Initialize actuated task space state
    J_actuated_.setZero(6,actuated_dof_);
    J_actuated_dot_.setZero(6,actuated_dof_);

    // Initialize joint space dynamics
    M_.setZero(total_dof_,total_dof_);
    M_inv_.setZero(total_dof_,total_dof_);
    g_.setZero(total_dof_);       
    c_.setZero(total_dof_);       
    NLE_.setZero(total_dof_);   

    // Initialize actuated joint space dynamics
    M_actuated_.setZero(actuated_dof_,actuated_dof_);
    M_inv_actuated_.setZero(actuated_dof_,actuated_dof_);
    g_actuated_.setZero(actuated_dof_);       
    c_actuated_.setZero(actuated_dof_);       
    NLE_actuated_.setZero(actuated_dof_);   
}

RobotDataMobileManipulator::RobotDataMobileManipulator(const std::string& urdf_path, 
                                                       const int& mani_dof, 
                                                       const int& mobi_dof,
                                                       const JointIndex& joint_idx,
                                                       const ActuatorIndex& actuator_idx,
                                                       const std::string& ee_name, 
                                                       const bool verbose)
: RobotDataBase(urdf_path, verbose),
  mani_dof_(mani_dof),
  mobi_dof_(mobi_dof),
  joint_idx_(joint_idx),
  actuator_idx_(actuator_idx)
{
    ee_name_ = ee_name;
    actuated_dof_ = mobi_dof_ + mani_dof_;
    total_dof_ = actuated_dof_ + virtual_dof_;

    ee_index_ = model_.getFrameId(ee_name_);
    if (ee_index_ == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << ee_name_ << " not found in URDF." << "\033[0m" << std::endl;
    }

    // Initialize selection matrix
    S_.setZero(total_dof_,actuated_dof_);
    Sdot_.setZero(total_dof_,actuated_dof_);
    J_mobile_.setZero(virtual_dof_,mobi_dof_);
    
    // Initialize joint space state
    q_.setZero(total_dof_);
    qdot_.setZero(total_dof_);

    // Initialize actuated joint space state
    q_actuated_.setZero(actuated_dof_);
    qdot_actuated_.setZero(actuated_dof_);

    // Initialize task space state
    x_.setIdentity();
    xdot_.setZero(6);
    J_.setZero(6,total_dof_);
    Jdot_.setZero(6,total_dof_);
    
    // Initialize actuated task space state
    J_actuated_.setZero(6,actuated_dof_);
    J_actuated_dot_.setZero(6,actuated_dof_);

    // Initialize joint space dynamics
    M_.setZero(total_dof_,total_dof_);
    M_inv_.setZero(total_dof_,total_dof_);
    g_.setZero(total_dof_);       
    c_.setZero(total_dof_);       
    NLE_.setZero(total_dof_);   

    // Initialize actuated joint space dynamics
    M_actuated_.setZero(actuated_dof_,actuated_dof_);
    M_inv_actuated_.setZero(actuated_dof_,actuated_dof_);
    g_actuated_.setZero(actuated_dof_);       
    c_actuated_.setZero(actuated_dof_);       
    NLE_actuated_.setZero(actuated_dof_);   
}
  

bool RobotDataMobileManipulator::updateKinematics(const VectorXd& q, const VectorXd& qdot)
{
    q_actuated_.segment(actuator_idx_.mani_start,mani_dof_) = q.segment(joint_idx_.mani_start,mani_dof_);
    q_actuated_.segment(actuator_idx_.mobi_start,mobi_dof_) = q.segment(joint_idx_.mobi_start,mobi_dof_);
    qdot_actuated_.segment(actuator_idx_.mani_start,mani_dof_) = qdot.segment(joint_idx_.mani_start,mani_dof_);
    qdot_actuated_.segment(actuator_idx_.mobi_start,mobi_dof_) = qdot.segment(joint_idx_.mobi_start,mobi_dof_);

    J_mobile_ = computeMobileFKJacobian(q.segment(joint_idx_.mobi_start,mobi_dof_));
    S_ = computeSelectionMatrix(q_);
    Sdot_ = computeSelectionMatrixTimeVariation(q_, qdot_);

    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, qdot);
    x_.matrix() = data_.oMf[ee_index_].toHomogeneousMatrix();
    J_ = pinocchio::getFrameJacobian(model_, data_, ee_index_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    xdot_ = J_ * qdot_;
    pinocchio::getFrameJacobianTimeVariation(model_, data_, ee_index_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot_);

    J_actuated_ = J_ * S_;
    J_actuated_dot_ = Jdot_ * S_ + J_ * Sdot_;

    return true;
}

bool RobotDataMobileManipulator::updateDynamics(const VectorXd& q, const VectorXd& qdot)
{
    pinocchio::crba(model_, data_, q);
    pinocchio::computeGeneralizedGravity(model_, data_, q);
    pinocchio::nonLinearEffects(model_, data_, q, qdot);

    // update joint space dynamics
    M_ = data_.M;
    M_ = M_.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba
    M_inv_ = DyrosMath::PinvCOD(M_);
    g_ = data_.g;
    NLE_ = data_.nle;
    c_ = NLE_ - g_;

    M_actuated_ = S_.transpose() * M_ * S_;
    M_inv_actuated_ = DyrosMath::PinvCOD(M_actuated_);
    g_actuated_ = S_.transpose() * g_;
    NLE_actuated_ = S_.transpose() * NLE_;
    c_actuated_ = NLE_actuated_ - g_actuated_;

    return true;
}

Affine3d RobotDataMobileManipulator::computePose(const VectorXd& q, const std::string& link_name)
{
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return Affine3d::Identity();
    }
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::framesForwardKinematics(model_, data, q);
    Affine3d link_pose;
    link_pose.matrix() = data.oMf[link_index].toHomogeneousMatrix();

    return link_pose;
}

MatrixXd RobotDataMobileManipulator::computeJacobian(const VectorXd& q, const std::string& link_name)
{
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return MatrixXd::Zero(6,total_dof_);
    }
    MatrixXd J;
    J.setZero(6,total_dof_);
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::computeJointJacobians(model_, data, q);
    pinocchio::computeFrameJacobian(model_, data, q, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    return J;
}

MatrixXd RobotDataMobileManipulator::computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
{
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return MatrixXd::Zero(6,total_dof_);
    }
    MatrixXd Jdot;
    Jdot.setZero(6,total_dof_);
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::computeJointJacobiansTimeVariation(model_, data, q, qdot);
    pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);

    return Jdot;
}

VectorXd RobotDataMobileManipulator::computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
{
    MatrixXd J = computeJacobian(q, link_name);
    
    return J * qdot;
}

MatrixXd RobotDataMobileManipulator::computeMassMatrix(const VectorXd& q)
{      
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::crba(model_, data, q);
    data.M = data.M.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba

    return data.M;
}

VectorXd RobotDataMobileManipulator::computeGravity(const VectorXd& q)
{      
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::computeGeneralizedGravity(model_, data, q);
    
    return data.g;
}

VectorXd RobotDataMobileManipulator::computeCoriolis(const VectorXd& q, const VectorXd& qdot)
{ 
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::computeCoriolisMatrix(model_, data, q, qdot);

    return data.C * qdot;
}

VectorXd RobotDataMobileManipulator::computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot)
{
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::nonLinearEffects(model_, data, q, qdot);

    return data.nle;
}


MatrixXd RobotDataMobileManipulator::computeJacobianActuated(const VectorXd& q_act, const std::string& link_name)
{
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return MatrixXd::Zero(6,actuated_dof_);
    }
    
    VectorXd q;
    q.setZero(total_dof_);
    q.segment(joint_idx_.mani_start,mani_dof_) = q_act.segment(actuator_idx_.mani_start, mani_dof_);
    q.segment(joint_idx_.mobi_start,mobi_dof_) = q_act.segment(actuator_idx_.mobi_start, mobi_dof_);
    MatrixXd S = computeSelectionMatrix(q);
    MatrixXd J = computeJacobian(q, link_name);

    return J * S;
}

MatrixXd RobotDataMobileManipulator::computeMassMatrixActuated(const VectorXd& q_act)
{
    VectorXd q;
    q.setZero(total_dof_);
    q.segment(joint_idx_.mani_start,mani_dof_) = q_act.segment(actuator_idx_.mani_start,mani_dof_);
    q.segment(joint_idx_.mobi_start,mobi_dof_) = q_act.segment(actuator_idx_.mobi_start,mobi_dof_);

    MatrixXd S = computeSelectionMatrix(q);

    return S.transpose() * computeMassMatrix(q) * S;
}

VectorXd RobotDataMobileManipulator::computeGravityActuated(const VectorXd& q_act)
{
    VectorXd q;
    q.setZero(total_dof_);
    q.segment(joint_idx_.mani_start,mani_dof_) = q_act.segment(actuator_idx_.mani_start,mani_dof_);
    q.segment(joint_idx_.mobi_start,mobi_dof_) = q_act.segment(actuator_idx_.mobi_start,mobi_dof_);
    
    MatrixXd S = computeSelectionMatrix(q);

    return S.transpose() * computeGravity(q);
}

VectorXd RobotDataMobileManipulator::computeCoriolisActuated(const VectorXd& q_act, const VectorXd& qdot_act)
{
    VectorXd q, qdot;
    q.setZero(total_dof_);
    qdot.setZero(total_dof_);
    q.segment(joint_idx_.mani_start,mani_dof_) = q_act.segment(actuator_idx_.mani_start,mani_dof_);
    q.segment(joint_idx_.mobi_start,mobi_dof_) = q_act.segment(actuator_idx_.mobi_start,mobi_dof_);
    qdot.segment(joint_idx_.mani_start,mani_dof_) = qdot_act.segment(actuator_idx_.mani_start,mani_dof_);
    qdot.segment(joint_idx_.mobi_start,mobi_dof_) = qdot_act.segment(actuator_idx_.mobi_start,mobi_dof_);

    MatrixXd S = computeSelectionMatrix(q);
    
    return S.transpose() * (computeNonlinearEffects(q, qdot) - computeGravity(q));
}

VectorXd RobotDataMobileManipulator::computeNonlinearEffectsActuated(const VectorXd& q_act, const VectorXd& qdot_act)
{
    VectorXd q, qdot;
    q.setZero(total_dof_);
    qdot.setZero(total_dof_);
    q.segment(joint_idx_.mani_start,mani_dof_) = q_act.segment(actuator_idx_.mani_start,mani_dof_);
    q.segment(joint_idx_.mobi_start,mobi_dof_) = q_act.segment(actuator_idx_.mobi_start,mobi_dof_);
    qdot.segment(joint_idx_.mani_start,mani_dof_) = qdot_act.segment(actuator_idx_.mani_start,mani_dof_);
    qdot.segment(joint_idx_.mobi_start,mobi_dof_) = qdot_act.segment(actuator_idx_.mobi_start,mobi_dof_);

    MatrixXd S = computeSelectionMatrix(q);

    return S.transpose() * computeNonlinearEffects(q, qdot);
}


double RobotDataMobileManipulator::computeManipulability(const VectorXd& q_mani, const VectorXd& qdot_mani, VectorXd* grad, VectorXd* grad_dot, const std::string& link_name)
{
    VectorXd q = VectorXd::Zero(total_dof_);
    q.segment(joint_idx_.mani_start,mani_dof_) = q_mani;

    MatrixXd J = computeJacobian(q, link_name);
    J = J.block(0,joint_idx_.mani_start,6,mani_dof_); // singularity does not depends on base frame, so J can be computed on world frame
    double mani = sqrt((J*J.transpose()).determinant());

    if(grad || grad_dot)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            grad->setZero(mani_dof_);
            grad_dot->setZero(mani_dof_);
            return mani;
        }
        
        grad->setZero(mani_dof_);

        MatrixXd JJt = J*J.transpose();
        MatrixXd JJt_inv = DyrosMath::PinvCOD(JJt);
        std::vector<MatrixXd> dJ_dq;
        dJ_dq.resize(mani_dof_);
        pinocchio::Data data = pinocchio::Data(model_);
        
        for(size_t i=0; i<mani_dof_; ++i)
        {
            VectorXd qdot_i = VectorXd::Zero(total_dof_);
            qdot_i[joint_idx_.mani_start + i] = 1.0;
            pinocchio::computeJointJacobiansTimeVariation(model_, data, q, qdot_i);
            dJ_dq[i].setZero(6,total_dof_);
            pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_dq[i]);
            dJ_dq[i] = dJ_dq[i].block(0,joint_idx_.mani_start,6,mani_dof_);

            (*grad)(i) = mani * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
        }

        if(grad_dot)
        {
            grad_dot->setZero(mani_dof_);

            VectorXd qdot = VectorXd::Zero(total_dof_);
            qdot.segment(joint_idx_.mani_start,mani_dof_) = qdot_mani;

            pinocchio::computeJointJacobiansTimeVariation(model_, data, q, qdot);
            MatrixXd Jdot;
            pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);
            Jdot = Jdot.block(0,joint_idx_.mani_start,6,mani_dof_);
            double mani_dot = mani * (Jdot * J.transpose() * JJt_inv).trace();

            MatrixXd JJt_dot = 2 * Jdot* J.transpose();
            MatrixXd JJt_inv_dot = -(JJt_inv * JJt_dot * JJt_inv);

            for(size_t i=0; i<mani_dof_; ++i)
            {
                (*grad_dot)[i] = mani_dot * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
                (*grad_dot)[i] += mani * (dJ_dq[i] * Jdot.transpose() * JJt_inv + 
                                          dJ_dq[i] * J.transpose() * JJt_inv_dot).trace(); // negelect dJ_dq_dot term
            }
        }
    }

    return mani;
}


Affine3d RobotDataMobileManipulator::getPose(const std::string& link_name) const
{
    if(link_name == ee_name_) return x_;
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return Affine3d::Identity();
    }
    Affine3d link_pose;
    link_pose.matrix() = data_.oMf[link_index].toHomogeneousMatrix();

    return link_pose;
}

MatrixXd RobotDataMobileManipulator::getJacobian(const std::string& link_name)
{
    if(link_name == ee_name_) return J_;
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return MatrixXd::Zero(6,total_dof_);
    }

    return pinocchio::getFrameJacobian(model_, data_, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
}

MatrixXd RobotDataMobileManipulator::getJacobianTimeVariation(const std::string& link_name)
{
    if(link_name == ee_name_) return Jdot_;
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return MatrixXd::Zero(6,total_dof_);
    }
    MatrixXd Jdot;
    Jdot.setZero(6,total_dof_);
    pinocchio::getFrameJacobianTimeVariation(model_, data_, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);

    return Jdot;
}

VectorXd RobotDataMobileManipulator::getVelocity(const std::string& link_name)
{
    if(link_name == ee_name_) return xdot_;
    return getJacobian(link_name) * qdot_;
}


MatrixXd RobotDataMobileManipulator::getJacobianActuated(const std::string& link_name)
{
    if(link_name == ee_name_) return J_actuated_;
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return MatrixXd::Zero(6,actuated_dof_);
    }

    return getJacobian(link_name)*S_;
}

MatrixXd RobotDataMobileManipulator::getJacobianActuatedTimeVariation(const std::string& link_name)
{
    if(link_name == ee_name_) return J_actuated_dot_;
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return MatrixXd ::Zero(6,actuated_dof_);
    }

    return getJacobianTimeVariation(link_name) * S_ + getJacobian(link_name) * Sdot_;
}


double RobotDataMobileManipulator::getManipulability(VectorXd* grad, VectorXd* grad_dot, const std::string& link_name)
{
    MatrixXd J = getJacobian(link_name); // singularity does not depends on base frame, so J can be computed on world frame
    J = J.block(0,joint_idx_.mani_start,6,mani_dof_);
    MatrixXd JJt = J*J.transpose();
    double mani = sqrt((JJt).determinant());


    if(grad || grad_dot)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            grad->setZero(mani_dof_);
            grad_dot->setZero(mani_dof_);
            return mani;
        }

        grad->setZero(mani_dof_);
        
        MatrixXd JJt_inv = DyrosMath::PinvCOD(JJt);
        std::vector<MatrixXd> dJ_dq;
        dJ_dq.resize(mani_dof_);
        pinocchio::Data data = pinocchio::Data(model_);
        
        for(size_t i=0; i<mani_dof_; ++i)
        {
            VectorXd qdot_i = VectorXd::Zero(total_dof_);
            qdot_i[joint_idx_.mani_start + i] = 1.0;
            pinocchio::computeJointJacobiansTimeVariation(model_, data, q_, qdot_i);
            dJ_dq[i].setZero(6,total_dof_);
            pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, dJ_dq[i]);
            dJ_dq[i] = dJ_dq[i].block(0,joint_idx_.mani_start,6,mani_dof_);
            
            (*grad)(i) = mani * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
        }


        if(grad_dot)
        {
            grad_dot->setZero(mani_dof_);

            MatrixXd Jdot = getJacobianTimeVariation(link_name);
            Jdot = Jdot.block(0,joint_idx_.mani_start,6,mani_dof_);
            double mani_dot = mani * (Jdot * J.transpose() * JJt_inv).trace();

            MatrixXd JJt_dot = 2 * Jdot* J.transpose();
            MatrixXd JJt_inv_dot = -(JJt_inv * JJt_dot * JJt_inv);

            for(size_t i=0; i<mani_dof_; ++i)
            {
                (*grad_dot)[i] = mani_dot * (dJ_dq[i] * J.transpose() * JJt_inv).trace();
                (*grad_dot)[i] += mani * (dJ_dq[i] * Jdot.transpose() * JJt_inv + 
                                          dJ_dq[i] * J.transpose() * JJt_inv_dot).trace(); // negelect dJ_dq_dot term
            }
        }
    }

    return mani;
}