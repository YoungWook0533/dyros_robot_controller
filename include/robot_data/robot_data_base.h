#pragma once
#include <string>
#include <mutex>
#include <shared_mutex>
#include <Eigen/Dense>
#include <math.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea-derivatives.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/spatial/fcl-pinocchio-conversions.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include "math_type_define.h"

using namespace Eigen;

class RobotDataBase
{
    public:
        RobotDataBase(const std::string& urdf_path, const bool verbose=false);
        RobotDataBase(const std::string& urdf_path, const std::string& srdf_path, const bool verbose=false);
        // RobotDataBase(const std::string& urdf_path, const std::string& model_pkg_path, const bool verbose=false);
        // RobotDataBase(const std::string& urdf_path, const std::string& srdf_path, const std::string& model_pkg_path, const bool verbose=false);
        ~RobotDataBase();
        bool updateState(const VectorXd& q, const VectorXd& qdot);

        double computeMinDistance(const VectorXd& q, const VectorXd& qdot, VectorXd* grad, VectorXd* grad_dot, const bool verbose=false);
        double getMinDistance(VectorXd* grad, VectorXd* grad_dot, const bool verbose=false);

        VectorXd getJointPosition() const {return q_;}
        VectorXd getJointVelocity() const {return qdot_;}

    protected:
        virtual bool updateKinematics(const VectorXd& q, const VectorXd& qdot) = 0;
        virtual bool updateDynamics(const VectorXd& q, const VectorXd& qdot) = 0;

        // pinocchio data
        pinocchio::Model model_;
        pinocchio::Data data_;
        pinocchio::GeometryModel geom_model_;
        pinocchio::GeometryData geom_data_;
        
        // Joint space state
        VectorXd q_;     // joint angle
        VectorXd qdot_;  // joint velocity
        VectorXd q_lb_;  // lower limit of joint angle
        VectorXd q_ub_;  // upper limit of joint angle
};