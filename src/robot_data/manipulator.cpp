#include "robot_data/manipulator.h"

RobotDataManipulator::RobotDataManipulator(const std::string& urdf_path, 
                                           const int& dof, 
                                           const std::string& ee_name, 
                                           const bool verbose)
: RobotDataBase(urdf_path, verbose),
  dof_(dof)
{
    ee_name_ = ee_name;

    ee_index_ = model_.getFrameId(ee_name_);
    if (ee_index_ == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << ee_name_ << " not found in URDF." << "\033[0m" << std::endl;
    }

    // Initialize joint space state
    q_.setZero(dof);
    qdot_.setZero(dof);

    // Initialize task space state
    x_.setIdentity();
    xdot_.setZero(6);
    J_.setZero(6,dof);
    Jdot_.setZero(6,dof);

    // Initialize joint space dynamics
    M_.setZero(dof,dof);
    M_inv_.setZero(dof,dof);
    g_.setZero(dof);       
    c_.setZero(dof);       
    NLE_.setZero(dof);   
}

bool RobotDataManipulator::updateKinematics(const VectorXd& q, const VectorXd& qdot)
{
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, qdot);
    x_.matrix() = data_.oMf[ee_index_].toHomogeneousMatrix();
    J_ = pinocchio::getFrameJacobian(model_, data_, ee_index_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    xdot_ = J_ * qdot_;
    pinocchio::getFrameJacobianTimeVariation(model_, data_, ee_index_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot_);

    return true;
}

bool RobotDataManipulator::updateDynamics(const VectorXd& q, const VectorXd& qdot)
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

    return true;
}

Affine3d RobotDataManipulator::computePose(const VectorXd& q, const std::string& link_name)
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

MatrixXd RobotDataManipulator::computeJacobian(const VectorXd& q, const std::string& link_name)
{
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return MatrixXd::Zero(6,dof_);
    }
    MatrixXd J;
    J.setZero(6,dof_);
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::computeJointJacobians(model_, data, q);
    pinocchio::computeFrameJacobian(model_, data, q, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

    return J;
}

MatrixXd RobotDataManipulator::computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
{
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return MatrixXd::Zero(6,dof_);
    }
    MatrixXd Jdot;
    Jdot.setZero(6,dof_);
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::computeJointJacobiansTimeVariation(model_, data, q, qdot);
    pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);

    return Jdot;
}

VectorXd RobotDataManipulator::computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
{
    MatrixXd J = computeJacobian(q, link_name);
    
    return J * qdot;
}

MatrixXd RobotDataManipulator::computeMassMatrix(const VectorXd& q)
{      
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::crba(model_, data, q);
    data.M = data.M.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba

    return data.M;
}

VectorXd RobotDataManipulator::computeGravity(const VectorXd& q)
{      
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::computeGeneralizedGravity(model_, data, q);
    
    return data.g;
}

VectorXd RobotDataManipulator::computeCoriolis(const VectorXd& q, const VectorXd& qdot)
{ 
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::computeCoriolisMatrix(model_, data, q, qdot);

    return data.C * qdot;
}

VectorXd RobotDataManipulator::computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot)
{
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::nonLinearEffects(model_, data, q, qdot);

    return data.nle;
}


Affine3d RobotDataManipulator::getPose(const std::string& link_name) const
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

MatrixXd RobotDataManipulator::getJacobian(const std::string& link_name)
{
    if(link_name == ee_name_) return J_;
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return MatrixXd::Zero(6,dof_);
    }

    return pinocchio::getFrameJacobian(model_, data_, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
}

MatrixXd RobotDataManipulator::getJacobianTimeVariation(const std::string& link_name)
{
    if(link_name == ee_name_) return Jdot_;
    pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
    if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
    {
        std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
        return MatrixXd::Zero(6,dof_);
    }
    MatrixXd Jdot;
    Jdot.setZero(6,dof_);
    pinocchio::getFrameJacobianTimeVariation(model_, data_, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);

    return Jdot;
}

VectorXd RobotDataManipulator::getVelocity(const std::string& link_name)
{
    if(link_name == ee_name_) return xdot_;
    return getJacobian(link_name) * qdot_;
}