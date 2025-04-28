#include "fr3_controller/robot_data.h"


namespace FR3Controller
{
    RobotData::RobotData(const std::string& urdf_path)
    {
        std::ifstream file(urdf_path);
        if (!file.good()) std::cout << "URDF file does not exist! : " << urdf_path << std::endl;
    
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
                
        joint_names_ = model_.names;
        joint_names_.erase(joint_names_.begin());  // Remove the first element "universe_joint"

        // Initialize joint space state
        q_.setZero(model_.nq);
        qdot_.setZero(model_.nq);
        tau_ext_.setZero(model_.nq);

        // Initialize task space state
        x_ .setIdentity();
        xdot_.setZero();
        J_.setZero(6, model_.nv);
        Jdot_.setZero(6, model_.nv);

        // Initialize joint space dynamics
        M_.setZero(model_.nq, model_.nv);
        M_inv_.setZero(model_.nq, model_.nv);
        g_.setZero(model_.nq);       
        c_.setZero(model_.nq);       
        NLE_.setZero(model_.nq);       

        // Initialize task space dynamics
        M_ee_.setZero();
        M_ee_inv_.setZero();
        g_ee_.setZero();       
        c_ee_.setZero();       
        NLE_ee_.setZero();  
    }

    RobotData::~RobotData()
    {
    }

    bool RobotData::updateState(const VectorXd& q, const VectorXd& qdot, const VectorXd& tau_ext)
    {
        q_ = q;
        qdot_ = qdot;
        tau_ext_ = tau_ext;
        if(!updateKinematics(q_, qdot_)) return false;
        if(!updateDynamics(q_, qdot_)) return false;
        return true;
    }

    bool RobotData::updateKinematics(const VectorXd& q, const VectorXd& qdot)
    {
        if(q.size() != model_.nq)
        {
            std::cerr << "updateDynamics Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
            return false;
        }
        if(qdot.size() != model_.nv)
        {
            std::cerr << "updateDynamics Error: size of qdot " << qdot.size() << " is not equal to model.nv size: " << model_.nv << std::endl;
            return false;
        }
        
        pinocchio::FrameIndex link_index = model_.getFrameId(ee_name_);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << ee_name_ << " not found in URDF." << std::endl;
            return false;
        }

        pinocchio::computeJointJacobians(model_, data_, q);
        pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, qdot);
        x_ = getPose(ee_name_);
        J_ = getJacobian(ee_name_);
        xdot_ = J_ * qdot;
        Jdot_ = getJacobianTimeVariation();

        return true;
    }

    bool RobotData::updateDynamics(const VectorXd& q, const VectorXd& qdot)
    {
        if(q.size() != model_.nq)
        {
            std::cerr << "updateDynamics Error: size of q " << q.size() << " is not equal to model.nq size: " << model_.nq << std::endl;
            return false;
        }
        if(qdot.size() != model_.nv)
        {
            std::cerr << "updateDynamics Error: size of qdot " << qdot.size() << " is not equal to model.nv size: " << model_.nv << std::endl;
            return false;
        }
        pinocchio::crba(model_, data_, q);
        pinocchio::computeGeneralizedGravity(model_, data_, q);
        pinocchio::nonLinearEffects(model_, data_, q, qdot);

        // update joint space dynamics
        M_ = data_.M;
        M_ = M_.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba
        M_inv_ = M_.inverse();
        g_ = data_.g;
        NLE_ = data_.nle;
        c_ = NLE_ - g_;

        // update task space dynamic
        M_ee_inv_ = J_ * M_inv_ * J_.transpose();
        M_ee_ = M_ee_inv_.inverse();
        c_ee_ = M_ee_ * (J_ * M_ * c_ - Jdot_ * qdot_);
        g_ee_ = M_ee_ * J_ * M_inv_ * g_;
        NLE_ee_ = c_ee_ + g_ee_;

        return true;
    }

    Matrix4d RobotData::computePose(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == model_.nq);

        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Matrix4d::Identity();
        }
        pinocchio::Data tmp_data;
        pinocchio::framesForwardKinematics(model_, tmp_data, q);
        Matrix4d link_pose = tmp_data.oMf[link_index].toHomogeneousMatrix();

        return link_pose;
    }

    Matrix4d RobotData::getPose(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Matrix4d::Identity();
        }
        Matrix4d link_pose = data_.oMf[link_index].toHomogeneousMatrix();

        return link_pose;
    }

    MatrixXd RobotData::computeJacobian(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == model_.nq);

        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd J;
        J.setZero(6, model_.nv);
        pinocchio::Data tmp_data;
        pinocchio::computeFrameJacobian(model_, tmp_data, q, link_index, J);

        return J;
    }

    MatrixXd RobotData::getJacobian(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd J = pinocchio::getFrameJacobian(model_, data_, link_index, pinocchio::ReferenceFrame::WORLD);

        return J;
    }

    MatrixXd RobotData::computeJacobianTimeVariation(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd Jdot;
        Jdot.setZero(6, model_.nv);
        pinocchio::Data tmp_data;
        pinocchio::computeJointJacobiansTimeVariation(model_, tmp_data, q, qdot_);
        pinocchio::getFrameJacobianTimeVariation(model_, tmp_data, link_index, pinocchio::ReferenceFrame::WORLD, Jdot);

        return Jdot;
    }

    MatrixXd RobotData::getJacobianTimeVariation(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return MatrixXd::Zero(6, model_.nv);
        }
        MatrixXd Jdot;
        Jdot.setZero(6, model_.nv);
        pinocchio::getFrameJacobianTimeVariation(model_, data_, link_index, pinocchio::ReferenceFrame::WORLD, Jdot);

        return Jdot;
    }

    Vector6d RobotData::computeVelocity(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

        MatrixXd J = computeJacobian(q, link_name);
        
        return J * qdot;
    }

    Vector6d RobotData::getVelocity(const std::string& link_name)
    {
        MatrixXd J = getJacobian(link_name);

        return J * qdot_;
    }

    MatrixXd RobotData::computeMassMatrix(const VectorXd& q)
    {
        assert(q.size() == model_.nq);
        
        pinocchio::Data tmp_data;
        pinocchio::crba(model_, tmp_data, q);
        tmp_data.M = tmp_data.M.selfadjointView<Upper>();  // Only upper triangular part of M_ is computed by pinocchio::crba

        return tmp_data.M;
    }

    MatrixXd RobotData::getMassMatrix()
    {
        return M_;
    }

    VectorXd RobotData::computeCoriolis(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);
        
        pinocchio::Data tmp_data;
        pinocchio::computeCoriolisMatrix(model_, tmp_data, q, qdot);

        return tmp_data.C * qdot;
    }

    VectorXd RobotData::getCoriolis()
    {
        return c_;
    }

    VectorXd RobotData::computeGravity(const VectorXd& q)
    {
        assert(q.size() == model_.nq);
        
        pinocchio::Data tmp_data;
        pinocchio::computeGeneralizedGravity(model_, tmp_data, q);

        return tmp_data.g;
    }

    VectorXd RobotData::getGravity()
    {
        return g_;
    }

    VectorXd RobotData::computeNonlinearEffects(const VectorXd& q, const VectorXd& qdot)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);

        pinocchio::Data tmp_data;
        pinocchio::nonLinearEffects(model_, tmp_data, q, qdot);

        return tmp_data.nle;
    }

    VectorXd RobotData::getNonlinearEffects()
    {
        return NLE_;
    }

    Matrix6d RobotData::computeTaskMassMatrix(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == model_.nq);
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Matrix6d::Zero();
        }

        MatrixXd M = computeMassMatrix(q);
        MatrixXd J = computeJacobian(q, link_name);

        return (J * M.inverse() * J.transpose()).inverse();
    }

    Matrix6d RobotData::getTaskMassMatrix(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Matrix6d::Zero();
        }

        MatrixXd J = getJacobian(link_name);

        return (J * M_.inverse() * J.transpose()).inverse();
    }

    Vector6d RobotData::computeTaskCoriolis(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Vector6d::Zero();
        }

        MatrixXd M = computeMassMatrix(q);
        MatrixXd J = computeJacobian(q, link_name);
        MatrixXd M_task = computeTaskMassMatrix(q, link_name);
        VectorXd c = computeCoriolis(q, qdot);
        MatrixXd Jdot = computeJacobianTimeVariation(q, qdot, link_name);

        return M_task * J * M * c - M_task * Jdot * qdot;
    }

    Vector6d RobotData::getTaskCoriolis(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Vector6d::Zero();
        }

        MatrixXd J = getJacobian(link_name);
        MatrixXd M_task = getTaskMassMatrix(link_name);
        MatrixXd Jdot = getJacobianTimeVariation(link_name);

        return M_task * J * M_ * c_ - M_task * Jdot * qdot_;
    }

    Vector6d RobotData::computeTaskGravity(const VectorXd& q, const std::string& link_name)
    {
        assert(q.size() == model_.nq);
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Vector6d::Zero();
        }

        MatrixXd M = computeMassMatrix(q);
        MatrixXd J = computeJacobian(q, link_name);
        MatrixXd M_task = computeTaskMassMatrix(q, link_name);
        VectorXd g = computeGravity(q);

        return M_task * J * M.inverse() * g;
    }

    Vector6d RobotData::getTaskGravity(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Vector6d::Zero();
        }

        MatrixXd J = getJacobian(link_name);
        MatrixXd M_task = getTaskMassMatrix(link_name);

        return M_task * J * M_.inverse() * g_;
    }

    Vector6d RobotData::computeTaskNonlinearEffects(const VectorXd& q, const VectorXd& qdot, const std::string& link_name)
    {
        assert(q.size() == model_.nq);
        assert(qdot.size() == model_.nv);
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Vector6d::Zero();
        }

        VectorXd c_task = computeTaskCoriolis(q, qdot, link_name);
        VectorXd g_task = computeTaskGravity(q, link_name);

        return c_task + g_task;
    }

    Vector6d RobotData::getTaskNonlinearEffects(const std::string& link_name)
    {
        pinocchio::FrameIndex link_index = model_.getFrameId(link_name);
        if (link_index == static_cast<pinocchio::FrameIndex>(-1))  
        {
            std::cerr << "Error: Link name " << link_name << " not found in URDF." << std::endl;
            return Vector6d::Zero();
        }

        VectorXd c_task = getTaskCoriolis(link_name);
        VectorXd g_task = getTaskGravity(link_name);

        return c_task + g_task;
    }
}