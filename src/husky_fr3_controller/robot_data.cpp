#include "husky_fr3_controller/robot_data.h"


namespace HuskyFR3Controller
{

    RobotData::RobotData(const std::string& urdf_path, const bool verbose)
    {
        std::ifstream file(urdf_path);
        if (!file.good()) std::cout << "\033[1;31m" << "URDF file does not exist! : " << "\033[0m" << urdf_path << "\033[0m" << std::endl;
        
        pinocchio::urdf::buildModel(urdf_path, model_, false);
        data_ = pinocchio::Data(model_);
                
        if(verbose)
        {
            std::cout << "Total nq = " << model_.nq << '\n' << "Total nv = " << model_.nv << "\n\n";
            std::cout << " id | name              | nq | nv | idx_q | idx_v\n";
            std::cout << "----+-------------------+----+----+-------+------\n";
            for(pinocchio::JointIndex id = 1; id < model_.joints.size(); ++id)
            {
                std::cout << std::setw(3)  << id << " | "
                          << std::setw(17) << model_.names[id] << " | "
                          << std::setw(2)  << model_.nqs[id]   << " | "
                          << std::setw(2)  << model_.nvs[id]   << " | "
                          << std::setw(5)  << model_.idx_qs[id]<< " | "
                          << std::setw(4)  << model_.idx_vs[id]<< '\n';
            }
        }

        ee_index_ = model_.getFrameId(ee_name_);

        // Initialize joint space state
        q_.setZero();
        qdot_.setZero();

        q_actuated_.setZero();
        qdot_actuated_.setZero();

        // Initialize task space state
        x_.setIdentity();
        xdot_.setZero();
        J_.setZero();
        Jdot_.setZero();
        J_actuated_.setZero();
        J_actuateddot_.setZero();

        // Initialize joint space dynamics
        M_.setZero();
        M_inv_.setZero();
        g_.setZero();       
        c_.setZero();       
        NLE_.setZero();   
        
        M_actuated_.setZero();
        M_inv_actuated_.setZero();
        g_actuated_.setZero();       
        c_actuated_.setZero();       
        NLE_actuated_.setZero();

        // Initialize selection matrix for virtual joints
        S_.setZero();
        Sdot_.setZero();
        J_mobile_.setZero();
    }

    RobotData::~RobotData()
    {
    }

    bool RobotData::updateState(const JointVec& q, const JointVec& qdot)
    {
        q_ = q;
        qdot_ = qdot;
        q_actuated_.segment<MANI_DOF>(actuator_idx.mani_start) = q_.segment<MANI_DOF>(joint_idx.mani_start);
        q_actuated_.segment<MOBI_DOF>(actuator_idx.mobi_start) = q_.segment<MOBI_DOF>(joint_idx.mobi_start);
        qdot_actuated_.segment<MANI_DOF>(actuator_idx.mani_start) = qdot_.segment<MANI_DOF>(joint_idx.mani_start);
        qdot_actuated_.segment<MOBI_DOF>(actuator_idx.mobi_start) = qdot_.segment<MOBI_DOF>(joint_idx.mobi_start);

        if(!updateKinematics(q_, qdot_)) return false;
        if(!updateDynamics(q_, qdot_)) return false;
        return true;
    }

    bool RobotData::updateKinematics(const JointVec& q, const JointVec& qdot)
    {             
        J_mobile_ = computeMobileFKJacobian(q.segment<MOBI_DOF>(joint_idx.mobi_start));
        S_ = computeSelectionMatrix(q_);
        Sdot_ = computeSelectionMatrixTimeVariation(q_, qdot_);
        
        pinocchio::computeJointJacobians(model_, data_, q);
        pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, qdot);
        x_.matrix() = data_.oMf[ee_index_].toHomogeneousMatrix();
        J_ = pinocchio::getFrameJacobian(model_, data_, ee_index_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
        J_actuated_ = J_ * S_;
        xdot_ = J_actuated_ * qdot_actuated_;
        pinocchio::getFrameJacobianTimeVariation(model_, data_, ee_index_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot_);
        J_actuateddot_ = Jdot_ * S_ + J_ * Sdot_;

        return true;
    }

    bool RobotData::updateDynamics(const JointVec& q, const JointVec& qdot)
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

    Affine3d RobotData::computePose(const JointVec& q, const std::string& link_name)
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

    Affine3d RobotData::getPose(const std::string& link_name) const
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

    Matrix<double,JOINT_DOF,ACTUATOR_DOF> RobotData::computeSelectionMatrix(const JointVec& q)
    {
        Matrix<double,JOINT_DOF,ACTUATOR_DOF> S;
        S.setZero();
        S.block<MANI_DOF,MANI_DOF>(joint_idx.mani_start,actuator_idx.mani_start).setIdentity();
        S.block<MOBI_DOF,MOBI_DOF>(joint_idx.mobi_start,actuator_idx.mobi_start).setIdentity();
        double mobile_yaw = q(joint_idx.virtual_start + 2);
        Matrix3d R_world2Base, v_mobile2mani;
        R_world2Base << cos(mobile_yaw), -sin(mobile_yaw), 0,
                        sin(mobile_yaw),  cos(mobile_yaw), 0,
                        0,                0,               1;
        v_mobile2mani << 1, 0, -mobile2mani_y,
                         0, 1,  mobile2mani_x,
                         0, 0,  1;
        S.block<VIRTUAL_DOF,MOBI_DOF>(joint_idx.virtual_start,actuator_idx.mobi_start) = R_world2Base * v_mobile2mani * computeMobileFKJacobian(q.segment<MOBI_DOF>(joint_idx.mobi_start));
        return S;
    }

    Matrix<double,JOINT_DOF,ACTUATOR_DOF> RobotData::computeSelectionMatrixTimeVariation(const JointVec& q, const JointVec& qdot)
    {
        Matrix<double,JOINT_DOF,ACTUATOR_DOF> S;
        S.setZero();
        S.block<MANI_DOF,MANI_DOF>(joint_idx.mani_start,actuator_idx.mani_start).setIdentity();
        S.block<MOBI_DOF,MOBI_DOF>(joint_idx.mobi_start,actuator_idx.mobi_start).setIdentity();
        double mobile_yaw = q(joint_idx.virtual_start + 2);
        Matrix3d R_world2Basedot, v_mobile2mani;
        R_world2Basedot << -sin(mobile_yaw),  -cos(mobile_yaw), 0,
                            cos(mobile_yaw),  -sin(mobile_yaw), 0,
                            0,                 0,               0;
        v_mobile2mani << 1, 0, -mobile2mani_y,
                         0, 1,  mobile2mani_x,
                         0, 0,  1;
        R_world2Basedot *= qdot(joint_idx.virtual_start + 2); // qdot(2) = θdot
        S.block<VIRTUAL_DOF,MOBI_DOF>(joint_idx.virtual_start,actuator_idx.mobi_start) = R_world2Basedot * v_mobile2mani * computeMobileFKJacobian(q.segment<MOBI_DOF>(joint_idx.mobi_start)); // J_tmp_ is constant wrt time
        return S;
    }

    Matrix<double,TASK_DOF,JOINT_DOF> RobotData::computeJacobian(const JointVec& q, const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return Matrix<double,TASK_DOF,JOINT_DOF>::Zero();
        }
        Matrix<double,TASK_DOF,JOINT_DOF> J;
        J.setZero();
        pinocchio::Data data = pinocchio::Data(model_);
        pinocchio::computeJointJacobians(model_, data, q);
        pinocchio::computeFrameJacobian(model_, data, q, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J);

        return J;
    }

    Matrix<double,TASK_DOF,ACTUATOR_DOF> RobotData::computeJacobianActuated(const ActuatorVec& q_act, const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return Matrix<double,TASK_DOF,ACTUATOR_DOF>::Zero();
        }
        
        JointVec q;
        q.setZero();
        q.segment<MANI_DOF>(joint_idx.mani_start) = q_act.segment<MANI_DOF>(actuator_idx.mani_start);
        q.segment<MOBI_DOF>(joint_idx.mobi_start) = q_act.segment<MOBI_DOF>(actuator_idx.mobi_start);
        Matrix<double,JOINT_DOF,ACTUATOR_DOF> S = computeSelectionMatrix(q);
        Matrix<double,TASK_DOF,JOINT_DOF> J = computeJacobian(q, link_name);

        return J * S;
    }

    Matrix<double,TASK_DOF,JOINT_DOF> RobotData::getJacobian(const std::string& link_name)
    {
        if(link_name == ee_name_) return J_;
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return Matrix<double,TASK_DOF,JOINT_DOF>::Zero();
        }

        return pinocchio::getFrameJacobian(model_, data_, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
    }

    Matrix<double,TASK_DOF,ACTUATOR_DOF> RobotData::getJacobianActuated(const std::string& link_name)
    {
        if(link_name == ee_name_) return J_actuated_;
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return Matrix<double,TASK_DOF,ACTUATOR_DOF>::Zero();
        }

        return getJacobian(link_name)*S_;
    }

    Matrix<double,TASK_DOF,ACTUATOR_DOF> RobotData::getJacobianActuatedTimeVariation(const std::string& link_name)
    {
        if(link_name == ee_name_) return J_actuateddot_;
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return Matrix<double,TASK_DOF,ACTUATOR_DOF> ::Zero();
        }

        return getJacobianTimeVariation(link_name) * S_ + getJacobian(link_name) * Sdot_;
    }

    Matrix<double,TASK_DOF,JOINT_DOF> RobotData::computeJacobianTimeVariation(const JointVec& q, const JointVec& qdot, const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return Matrix<double,TASK_DOF,JOINT_DOF>::Zero();
        }
        Matrix<double,TASK_DOF,JOINT_DOF> Jdot;
        Jdot.setZero();
        pinocchio::Data data = pinocchio::Data(model_);
        pinocchio::computeJointJacobiansTimeVariation(model_, data, q, qdot);
        pinocchio::getFrameJacobianTimeVariation(model_, data, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);

        return Jdot;
    }

    Matrix<double,TASK_DOF,JOINT_DOF> RobotData::getJacobianTimeVariation(const std::string& link_name)
    {
        if(link_name == ee_name_) return Jdot_;
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "\033[1;31m" << "Error: Link name " << link_name << " not found in URDF." << "\033[0m" << std::endl;
            return Matrix<double,TASK_DOF,JOINT_DOF>::Zero();
        }
        Matrix<double,TASK_DOF,JOINT_DOF> Jdot;
        Jdot.setZero();
        pinocchio::getFrameJacobianTimeVariation(model_, data_, link_index, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, Jdot);

        return Jdot;
    }

    TaskVec RobotData::computeVelocity(const JointVec& q, const JointVec& qdot, const std::string& link_name)
    {
        Matrix<double,TASK_DOF,JOINT_DOF> J = computeJacobian(q, link_name);
        
        return J * qdot;
    }

    TaskVec RobotData::getVelocity(const std::string& link_name)
    {
        if(link_name == ee_name_) return xdot_;
        return getJacobian(link_name) * qdot_;
    }

    JointMat RobotData::computeMassMatrix(const JointVec& q)
    {      
        pinocchio::Data data = pinocchio::Data(model_);
        pinocchio::crba(model_, data, q);
        data.M = data.M.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba

        return data.M;
    }

    ActuatorMat RobotData::computeMassMatrixActuated(const ActuatorVec& q_act)
    {
        JointVec q;
        q.setZero();
        q.segment<MANI_DOF>(joint_idx.mani_start) = q_act.segment<MANI_DOF>(actuator_idx.mani_start);
        q.segment<MOBI_DOF>(joint_idx.mobi_start) = q_act.segment<MOBI_DOF>(actuator_idx.mobi_start);

        Matrix<double,JOINT_DOF,ACTUATOR_DOF> S = computeSelectionMatrix(q);

        return S.transpose() * computeMassMatrix(q) * S;
    }

    JointVec RobotData::computeCoriolis(const JointVec& q, const JointVec& qdot)
    { 
        pinocchio::Data data = pinocchio::Data(model_);
        pinocchio::computeCoriolisMatrix(model_, data, q, qdot);

        return data.C * qdot;
    }

    ActuatorVec RobotData::computeCoriolisActuated(const ActuatorVec& q_act, const ActuatorVec& qdot_act)
    {
        JointVec q,qdot;
        q.setZero();
        q.segment<MANI_DOF>(joint_idx.mani_start) = q_act.segment<MANI_DOF>(actuator_idx.mani_start);
        q.segment<MOBI_DOF>(joint_idx.mobi_start) = q_act.segment<MOBI_DOF>(actuator_idx.mobi_start);
        qdot.setZero();
        qdot.segment<MANI_DOF>(joint_idx.mani_start) = qdot_act.segment<MANI_DOF>(actuator_idx.mani_start);
        qdot.segment<MOBI_DOF>(joint_idx.mobi_start) = qdot_act.segment<MOBI_DOF>(actuator_idx.mobi_start);

        Matrix<double,JOINT_DOF,ACTUATOR_DOF> S = computeSelectionMatrix(q);
        
        return S.transpose() * (computeNonlinearEffects(q, qdot) - computeGravity(q));
    }

    JointVec RobotData::computeGravity(const JointVec& q)
    {      
        pinocchio::Data data = pinocchio::Data(model_);
        pinocchio::computeGeneralizedGravity(model_, data, q);

        return data.g;
    }

    ActuatorVec RobotData::computeGravityActuated(const ActuatorVec& q_act)
    {
        JointVec q;
        q.setZero();
        q.segment<MANI_DOF>(joint_idx.mani_start) = q_act.segment<MANI_DOF>(actuator_idx.mani_start);
        q.segment<MOBI_DOF>(joint_idx.mobi_start) = q_act.segment<MOBI_DOF>(actuator_idx.mobi_start);
        
        Matrix<double,JOINT_DOF,ACTUATOR_DOF> S = computeSelectionMatrix(q);

        return S.transpose() * computeGravity(q);
    }

    JointVec RobotData::computeNonlinearEffects(const JointVec& q, const JointVec& qdot)
    {
        pinocchio::Data data = pinocchio::Data(model_);
        pinocchio::nonLinearEffects(model_, data, q, qdot);

        return data.nle;
    }

    ActuatorVec RobotData::computeNonlinearEffectsActuated(const ActuatorVec& q_act, const ActuatorVec& qdot_act)
    {
        JointVec q,qdot;
        q.setZero();
        q.segment<MANI_DOF>(joint_idx.mani_start) = q_act.segment<MANI_DOF>(actuator_idx.mani_start);
        q.segment<MOBI_DOF>(joint_idx.mobi_start) = q_act.segment<MOBI_DOF>(actuator_idx.mobi_start);
        qdot.setZero();
        qdot.segment<MANI_DOF>(joint_idx.mani_start) = qdot_act.segment<MANI_DOF>(actuator_idx.mani_start);
        qdot.segment<MOBI_DOF>(joint_idx.mobi_start) = qdot_act.segment<MOBI_DOF>(actuator_idx.mobi_start);

        Matrix<double,JOINT_DOF,ACTUATOR_DOF> S = computeSelectionMatrix(q);

        return S.transpose() * computeNonlinearEffects(q, qdot);
    }

    Matrix<double,VIRTUAL_DOF,MOBI_DOF> RobotData::computeMobileFKJacobian(const MobiVec& q_mobile)
    {
        Matrix<double,VIRTUAL_DOF,MOBI_DOF> J;
        J.setZero();
        J << wheel_radius_/2.,               wheel_radius_/2.,
             0.,                             0.,
             -wheel_radius_/(mobile_width_), wheel_radius_/(mobile_width_);

        return J;
    }
}