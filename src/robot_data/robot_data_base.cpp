#include "robot_data/robot_data_base.h"

RobotDataBase::RobotDataBase(const std::string& urdf_path, const bool verbose)
{
    std::ifstream file(urdf_path);
    if (!file.good()) std::cout << "\033[1;31m" << "URDF file does not exist! : " << "\033[0m" << urdf_path << "\033[0m" << std::endl;
    
    
    pinocchio::urdf::buildModel(urdf_path, model_, false);
    data_ = pinocchio::Data(model_);
    
    pinocchio::urdf::buildGeom(model_, urdf_path, pinocchio::COLLISION, geom_model_, ament_index_cpp::get_package_share_directory("mujoco_ros_sim"));
    geom_model_.addAllCollisionPairs();
    
    geom_data_ = pinocchio::GeometryData(geom_model_);
    
    q_lb_ = model_.lowerPositionLimit;
    q_ub_ = model_.upperPositionLimit;
    
    if(verbose)
    {
        std::cout << "Total nq = " << model_.nq << '\n' << "Total nv = " << model_.nv << "\n\n";
        std::cout << " id | name                 | nq | nv | idx_q | idx_v\n";
        std::cout << "----+----------------------+----+----+-------+------\n";
        for(pinocchio::JointIndex id = 1; id < model_.joints.size(); ++id)
        {
            std::cout << std::setw(3)  << id << " | "
            << std::setw(20) << model_.names[id] << " | "
            << std::setw(2)  << model_.nqs[id]   << " | "
            << std::setw(2)  << model_.nvs[id]   << " | "
            << std::setw(5)  << model_.idx_qs[id]<< " | "
            << std::setw(4)  << model_.idx_vs[id]<< '\n';
        }
    }
}

RobotDataBase::RobotDataBase(const std::string& urdf_path, const std::string& srdf_path, const bool verbose)
: RobotDataBase(urdf_path,verbose)
{
    pinocchio::srdf::removeCollisionPairs(model_, geom_model_, srdf_path);
}


RobotDataBase::~RobotDataBase()
{

}

bool RobotDataBase::updateState(const VectorXd& q, const VectorXd& qdot)
{
    q_ = q;
    qdot_ = qdot;

    if(!updateKinematics(q_, qdot_)) return false;
    if(!updateDynamics(q_, qdot_)) return false;
    return true;
}

double RobotDataBase::getMinDistance(VectorXd* grad, VectorXd* grad_dot, const bool verbose)
{
    pinocchio::computeDistances(model_, data_, geom_model_, geom_data_, q_);

    double minDistance = std::numeric_limits<double>::max();
    int    minPairIdx  = -1;

    for (std::size_t idx = 0; idx < geom_data_.distanceResults.size(); ++idx)
    {
        const auto &res = geom_data_.distanceResults[idx];
        if (res.min_distance < minDistance)
        {
            minDistance = res.min_distance;
            minPairIdx  = static_cast<int>(idx);
        }
    }

    if (minPairIdx >= 0 && verbose)
    {
        const auto &pair   = geom_model_.collisionPairs[minPairIdx];
        const std::string &link1 =
            geom_model_.geometryObjects[pair.first].name;
        const std::string &link2 =
            geom_model_.geometryObjects[pair.second].name;

        std::cout << "[RobotDataBase] Closest links: " << link1
                  << "  <->  " << link2
                  << "   |  distance = " << minDistance << " [m]\n";
    }

    if(grad || grad_dot)
    {
        pinocchio::updateGeometryPlacements(model_, data_, geom_model_, geom_data_, q_);

        const auto &pair  = geom_model_.collisionPairs[minPairIdx];
        const int   geomA = pair.first,  geomB = pair.second;
        const int   jointA = geom_model_.geometryObjects[geomA].parentJoint;
        const int   jointB = geom_model_.geometryObjects[geomB].parentJoint;

        // Witness points & normal (world frame)
        const auto &res = geom_data_.distanceResults[minPairIdx];
        const Vector3d pA = res.nearest_points[0];
        const Vector3d pB = res.nearest_points[1];
        const Vector3d n  = (pB - pA).normalized();

        //---------------- 3. Jacobian of each point ----------------
        // Joint-space 6 × total_dof Jacobians for the two parent joints
        MatrixXd J_jointA = MatrixXd::Zero(6, q_.size());
        MatrixXd J_jointB = MatrixXd::Zero(6, q_.size());
        pinocchio::getJointJacobian(model_, data_, jointA, pinocchio::LOCAL_WORLD_ALIGNED, J_jointA);
        pinocchio::getJointJacobian(model_, data_, jointB, pinocchio::LOCAL_WORLD_ALIGNED, J_jointB);

        // r = point - joint-origin (world)
        const Vector3d rA = pA - data_.oMi[jointA].translation();
        const Vector3d rB = pB - data_.oMi[jointB].translation();

        // Linear part of point Jacobian: Jp = Jv + ω×r
        auto skew = [](const Vector3d &v)->Matrix3d{
                    return (Matrix3d() <<   0, -v.z(),  v.y(),
                                        v.z(),      0, -v.x(),
                                       -v.y(),  v.x(),     0).finished(); };

        const MatrixXd JA = J_jointA.topRows<3>() - skew(rA) * J_jointA.bottomRows<3>();
        const MatrixXd JB = J_jointB.topRows<3>() - skew(rB) * J_jointB.bottomRows<3>();

        //---------------- 4. Gradient  ------------------------------
        // d/dq (‖pB - pA‖) = nᵀ (J_B - J_A)
        *grad =( n.transpose() * (JB - JA)).transpose();   // total_dof × 1
        if(minDistance < 0) *grad *= -1.;

        if(grad_dot)
        {
            MatrixXd J_jointA_dot = MatrixXd::Zero(6, q_.size());
            MatrixXd J_jointB_dot = MatrixXd::Zero(6, q_.size());
            pinocchio::getJointJacobianTimeVariation(model_, data_, jointA, pinocchio::LOCAL_WORLD_ALIGNED, J_jointA_dot);
            pinocchio::getJointJacobianTimeVariation(model_, data_, jointB, pinocchio::LOCAL_WORLD_ALIGNED, J_jointB_dot);

            const Vector3d pA_dot = JA * qdot_;
            const Vector3d pB_dot = JB * qdot_;

            const Vector3d rA_dot = pA_dot - J_jointA.topRows<3>() * qdot_;
            const Vector3d rB_dot = pB_dot - J_jointB.topRows<3>() * qdot_;

            const MatrixXd JA_dot = J_jointA_dot.topRows<3>() - (skew(rA_dot) * J_jointA.bottomRows<3>() + skew(rA) * J_jointA_dot.bottomRows<3>());
            const MatrixXd JB_dot = J_jointB_dot.topRows<3>() - (skew(rB_dot) * J_jointB.bottomRows<3>() + skew(rB) * J_jointB_dot.bottomRows<3>());

            *grad_dot = (n.transpose() * (JB_dot - JA_dot)).transpose(); // neglect n_dot term
        }
    }

    return minDistance;     
}

double RobotDataBase::computeMinDistance(const VectorXd& q, const VectorXd& qdot, VectorXd* grad, VectorXd* grad_dot, const bool verbose)
{
    pinocchio::Data data = pinocchio::Data(model_);
    pinocchio::GeometryData geom_data = pinocchio::GeometryData(geom_model_);

    pinocchio::computeDistances(model_, data, geom_model_, geom_data, q);

    double minDistance = std::numeric_limits<double>::max();
    int    minPairIdx  = -1;

    for (std::size_t idx = 0; idx < geom_data.distanceResults.size(); ++idx)
    {
        const auto &res = geom_data.distanceResults[idx];
        if (res.min_distance < minDistance)
        {
            minDistance = res.min_distance;
            minPairIdx  = static_cast<int>(idx);
        }
    }

    if (minPairIdx >= 0 && verbose)
    {
        const auto &pair   = geom_model_.collisionPairs[minPairIdx];
        const std::string &link1 =
            geom_model_.geometryObjects[pair.first].name;
        const std::string &link2 =
            geom_model_.geometryObjects[pair.second].name;

        std::cout << "[RobotDataBase] Closest links: " << link1
                  << "  <->  " << link2
                  << "   |  distance = " << minDistance << " [m]\n";
    }

    if(grad || grad_dot)
    {
        pinocchio::computeJointJacobians(model_, data, q);
        pinocchio::updateGeometryPlacements(model_, data, geom_model_, geom_data, q);

        const auto &pair  = geom_model_.collisionPairs[minPairIdx];
        const int   geomA = pair.first,  geomB = pair.second;
        const int   jointA = geom_model_.geometryObjects[geomA].parentJoint;
        const int   jointB = geom_model_.geometryObjects[geomB].parentJoint;

        // Witness points & normal (world frame)
        const auto &res = geom_data.distanceResults[minPairIdx];
        const Vector3d pA = res.nearest_points[0];
        const Vector3d pB = res.nearest_points[1];
        const Vector3d n  = (pB - pA).normalized();

        //---------------- 3. Jacobian of each point ----------------
        // Joint-space 6 × total_dof Jacobians for the two parent joints
        MatrixXd J_jointA = MatrixXd::Zero(6, q.size());
        MatrixXd J_jointB = MatrixXd::Zero(6, q.size());
        pinocchio::getJointJacobian(model_, data, jointA, pinocchio::LOCAL_WORLD_ALIGNED, J_jointA);
        pinocchio::getJointJacobian(model_, data, jointB, pinocchio::LOCAL_WORLD_ALIGNED, J_jointB);

        // r = point - joint-origin (world)
        const Vector3d rA = pA - data.oMi[jointA].translation();
        const Vector3d rB = pB - data.oMi[jointB].translation();

        // Linear part of point Jacobian: Jp = Jv + ω×r
        auto skew = [](const Vector3d &v)->Matrix3d{
                    return (Matrix3d() <<   0, -v.z(),  v.y(),
                                        v.z(),      0, -v.x(),
                                       -v.y(),  v.x(),     0).finished(); };

        const MatrixXd JA = J_jointA.topRows<3>() - skew(rA) * J_jointA.bottomRows<3>();
        const MatrixXd JB = J_jointB.topRows<3>() - skew(rB) * J_jointB.bottomRows<3>();

        //---------------- 4. Gradient  ------------------------------
        // d/dq (‖pB - pA‖) = nᵀ (J_B - J_A)
        *grad =( n.transpose() * (JB - JA)).transpose();   // total_dof × 1
        if(minDistance < 0) *grad *= -1.;

        if(grad_dot)
        {
            pinocchio::computeJointJacobiansTimeVariation(model_, data_, q, qdot);

            MatrixXd J_jointA_dot = MatrixXd::Zero(6, q.size());
            MatrixXd J_jointB_dot = MatrixXd::Zero(6, q.size());
            pinocchio::getJointJacobianTimeVariation(model_, data, jointA, pinocchio::LOCAL_WORLD_ALIGNED, J_jointA_dot);
            pinocchio::getJointJacobianTimeVariation(model_, data, jointB, pinocchio::LOCAL_WORLD_ALIGNED, J_jointB_dot);

            const Vector3d pA_dot = JA * qdot;
            const Vector3d pB_dot = JB * qdot;

            const Vector3d rA_dot = pA_dot - J_jointA.topRows<3>() * qdot;
            const Vector3d rB_dot = pB_dot - J_jointB.topRows<3>() * qdot;

            const MatrixXd JA_dot = J_jointA_dot.topRows<3>() - (skew(rA_dot) * J_jointA.bottomRows<3>() + skew(rA) * J_jointA_dot.bottomRows<3>());
            const MatrixXd JB_dot = J_jointB_dot.topRows<3>() - (skew(rB_dot) * J_jointB.bottomRows<3>() + skew(rB) * J_jointB_dot.bottomRows<3>());

            *grad_dot = (n.transpose() * (JB_dot - JA_dot)).transpose(); // neglect n_dot term
        }
    }

    return minDistance;     
}