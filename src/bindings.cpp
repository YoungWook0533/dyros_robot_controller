#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 40

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/geometry.hpp>
#include <Eigen/Dense>
#include <numpy/ndarrayobject.h>
#include <numpy/arrayobject.h>

#include "robot_data/type_define.h"
#include "robot_data/mobile/base.h"
#include "robot_data/manipulator/base.h"
#include "robot_data/mobile_manipulator/base.h"

#include "robot_controller/mobile/base.h"
#include "robot_controller/manipulator/base.h"
#include "robot_controller/mobile_manipulator/base.h"

namespace bp    = boost::python;
namespace RD_MO = RobotData::Mobile;
namespace RD_MN = RobotData::Manipulator;
namespace RD_MM = RobotData::MobileManipulator;
namespace RC_MO = RobotController::Mobile;
namespace RC_MN = RobotController::Manipulator;
namespace RC_MM = RobotController::MobileManipulator;

typedef RD_MO::MobileBase            RDMOBase;
typedef RD_MN::ManipulatorBase       RDMNBase;
typedef RD_MM::MobileManipulatorBase RDMMBase;
typedef RC_MO::MobileBase            RCMOBase;
typedef RC_MN::ManipulatorBase       RCMNBase;
typedef RC_MM::MobileManipulatorBase RCMMBase;

namespace
{
    bp::tuple RCMMBase_QPIK_tuple(RCMMBase& self, const VectorXd& xdot_target, const std::string& link_name)
    {
        VectorXd qdot_mobi, qdot_mani;
        self.QPIK(xdot_target, link_name, qdot_mobi, qdot_mani);
        return bp::make_tuple( qdot_mobi, qdot_mani );
    }
    
    bp::tuple RCMMBase_QPIKStep_tuple(RCMMBase& self, const Affine3d& x_target, const VectorXd& xdot_target, const std::string& link_name)
    {
        VectorXd qdot_mobi, qdot_mani;
        self.QPIKStep(x_target, xdot_target, link_name, qdot_mobi, qdot_mani);
        return bp::make_tuple( qdot_mobi, qdot_mani );
    }
    
    bp::tuple RCMMBase_QPIKCubic_tuple(RCMMBase& self, 
                                       const Affine3d& x_target,
                                       const VectorXd& xdot_target,
                                       const Affine3d& x_init,
                                       const VectorXd& xdot_init,
                                       const double& current_time,
                                       const double& init_time,
                                       const double& duration,
                                       const std::string& link_name)
    {
        VectorXd qdot_mobi, qdot_mani;
        self.QPIKCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, link_name, qdot_mobi, qdot_mani);
        return bp::make_tuple( qdot_mobi, qdot_mani );
    }

    bp::tuple RCMMBase_QPID_tuple(RCMMBase& self, const VectorXd& xddot_target, const std::string& link_name)
    {
        VectorXd qddot_mobi, torque_mani;
        self.QPID(xddot_target, link_name, qddot_mobi, torque_mani);
        return bp::make_tuple( qddot_mobi, torque_mani );
    }
    
    bp::tuple RCMMBase_QPIDStep_tuple(RCMMBase& self, const Affine3d& x_target, const VectorXd& xdot_target, const std::string& link_name)
    {
        VectorXd qddot_mobi, torque_mani;
        self.QPIDStep(x_target, xdot_target, link_name, qddot_mobi, torque_mani);
        return bp::make_tuple( qddot_mobi, torque_mani );
    }
    
    bp::tuple RCMMBase_QPIDCubic_tuple(RCMMBase& self, 
                                       const Affine3d& x_target,
                                       const VectorXd& xdot_target,
                                       const Affine3d& x_init,
                                       const VectorXd& xdot_init,
                                       const double& current_time,
                                       const double& init_time,
                                       const double& duration,
                                       const std::string& link_name)
    {
        VectorXd qddot_mobi, torque_mani;
        self.QPIDCubic(x_target, xdot_target, x_init, xdot_init, current_time, init_time, duration, link_name, qddot_mobi, torque_mani);
        return bp::make_tuple( qddot_mobi, torque_mani );
    }
}

struct PairVectorXdToPython
{
    static PyObject* convert(const std::pair<VectorXd, VectorXd>& p)
    {
        bp::object first_obj(p.first);
        bp::object second_obj(p.second);
        bp::tuple t = bp::make_tuple(first_obj, second_obj);
        return bp::incref(t.ptr());
    }
};

struct Affine3dToPython
{
  static PyObject* convert(const Affine3d& T)
  {
    const Matrix4d& M = T.matrix();
    bp::object mat(M);
    return bp::incref(mat.ptr());
  }
};

struct VecDoubleToPython
{
    static PyObject *convert(const std::vector<double> &v)
    {
        bp::list l;
        for (double d : v) l.append(d);
        return bp::incref(l.ptr());
    }
};

struct VecDoubleFromPython
{
    VecDoubleFromPython()
    {
        bp::converter::registry::push_back(&convertible, &construct, bp::type_id<std::vector<double>>());
    }

    static void *convertible(PyObject *obj_ptr)
    {
        return PySequence_Check(obj_ptr) ? obj_ptr : nullptr;
    }

    static void construct(PyObject *obj_ptr, bp::converter::rvalue_from_python_stage1_data *data)
    {
        void *storage = ((bp::converter::rvalue_from_python_storage<std::vector<double>> *)data)->storage.bytes;
        new (storage) std::vector<double>();

        auto *vec = static_cast<std::vector<double> *>(storage);
        const Py_ssize_t len = PySequence_Size(obj_ptr);
        vec->reserve(len);

        for (Py_ssize_t i = 0; i < len; ++i)
        {
            bp::object item(bp::handle<>(PySequence_GetItem(obj_ptr, i)));
            vec->push_back(bp::extract<double>(item));
        }
        data->convertible = storage;
    }
};

struct Vec2dToPython
{
    static PyObject *convert(const std::vector<Vector2d> &v)
    {
        bp::list l;
        for (const auto &e : v) l.append(bp::object(e));
        return bp::incref(l.ptr());
    }
};

struct Vec2dFromPython
{
    Vec2dFromPython()
    {
        bp::converter::registry::push_back(&convertible, &construct, bp::type_id<std::vector<Vector2d>>());
    }

    static void *convertible(PyObject *obj_ptr)
    { 
        return PySequence_Check(obj_ptr) ? obj_ptr : nullptr; 
    }

    static void construct(PyObject *obj_ptr, bp::converter::rvalue_from_python_stage1_data *data)
    {
        void *storage = ((bp::converter::rvalue_from_python_storage<std::vector<Vector2d>> *)data)->storage.bytes;
        new (storage) std::vector<Vector2d>();
        auto *vec = static_cast<std::vector<Vector2d> *>(storage);
        const Py_ssize_t len = PySequence_Size(obj_ptr);
        vec->reserve(len);
        for (Py_ssize_t i = 0; i < len; ++i)
        {
            bp::object item(bp::handle<>(PySequence_GetItem(obj_ptr, i)));
            vec->emplace_back(bp::extract<Vector2d>(item));
        }
        data->convertible = storage;
    }
};

struct Affine3dFromNumpy
{
    Affine3dFromNumpy()
    {
        bp::converter::registry::push_back(&convertible, &construct, bp::type_id<Affine3d>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        if (!PyArray_Check(obj_ptr))                            return nullptr;
        auto* arr = reinterpret_cast<PyArrayObject*>(obj_ptr);
        if (PyArray_NDIM(arr) != 2)                             return nullptr;
        if (PyArray_DIM(arr,0) != 4 || PyArray_DIM(arr,1) != 4) return nullptr;
        if (PyArray_TYPE(arr) != NPY_DOUBLE)                    return nullptr;
        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, bp::converter::rvalue_from_python_stage1_data* data)
    {
        void* storage = ((bp::converter::rvalue_from_python_storage<Affine3d>*)data)->storage.bytes;
        double* buf = reinterpret_cast<double*>(PyArray_DATA((PyArrayObject*)obj_ptr));
        Map<Matrix<double,4,4,RowMajor>> M(buf);
        new (storage) Affine3d(M);
        data->convertible = storage;
    }
};

BOOST_PYTHON_MODULE(dyros_robot_controller_cpp_wrapper)
{
    eigenpy::enableEigenPy();
    eigenpy::enableEigenPySpecific<Matrix<double, Dynamic, Dynamic>>();
    eigenpy::enableEigenPySpecific<Matrix<double, Dynamic, 1>>();
    eigenpy::enableEigenPySpecific<Matrix4d>();

    bp::to_python_converter<std::pair<VectorXd, VectorXd>, PairVectorXdToPython>();
    bp::to_python_converter<Affine3d, Affine3dToPython>();
    bp::to_python_converter<std::vector<double>, VecDoubleToPython>();
    bp::to_python_converter<std::vector<Vector2d>, Vec2dToPython>();
    
    static VecDoubleFromPython _reg_vecdouble_from_python;
    static Vec2dFromPython     _reg_vec2d_from_python;
    static Affine3dFromNumpy   _reg_affine3d_from_numpy;

    bp::enum_<RD_MO::DriveType>("DriveType")
        .value("Differential", RD_MO::DriveType::Differential)
        .value("Mecanum",      RD_MO::DriveType::Mecanum)
        .value("Caster",       RD_MO::DriveType::Caster)
        .export_values();

    bp::class_<RD_MO::KinematicParam>("KinematicParam")
        .def(bp::init<>())
        .def_readwrite("type",                  &RD_MO::KinematicParam::type)
        .def_readwrite("wheel_radius",          &RD_MO::KinematicParam::wheel_radius)
        .def_readwrite("max_lin_speed",         &RD_MO::KinematicParam::max_lin_speed)
        .def_readwrite("max_ang_speed",         &RD_MO::KinematicParam::max_ang_speed)
        .def_readwrite("max_lin_acc",           &RD_MO::KinematicParam::max_lin_acc)
        .def_readwrite("max_ang_acc",           &RD_MO::KinematicParam::max_ang_acc)
        .def_readwrite("base_width",            &RD_MO::KinematicParam::base_width)
        .def_readwrite("roller_angles",         &RD_MO::KinematicParam::roller_angles)
        .def_readwrite("base2wheel_positions",  &RD_MO::KinematicParam::base2wheel_positions)
        .def_readwrite("base2wheel_angles",     &RD_MO::KinematicParam::base2wheel_angles)
        .def_readwrite("wheel_offset",          &RD_MO::KinematicParam::wheel_offset);

    bp::class_<RD_MN::MinDistResult>("MinDistResult")
        .def(bp::init<>())
        .def_readwrite("distance", &RD_MN::MinDistResult::distance)
        .def_readwrite("grad",     &RD_MN::MinDistResult::grad)
        .def_readwrite("grad_dot", &RD_MN::MinDistResult::grad_dot)
        .def("setZero",            &RD_MN::MinDistResult::setZero);

    bp::class_<RD_MN::ManipulabilityResult>("ManipulabilityResult")
        .def(bp::init<>())
        .def_readwrite("manipulability", &RD_MN::ManipulabilityResult::manipulability)
        .def_readwrite("grad",           &RD_MN::ManipulabilityResult::grad)
        .def_readwrite("grad_dot",       &RD_MN::ManipulabilityResult::grad_dot)
        .def("setZero",                  &RD_MN::ManipulabilityResult::setZero);

    bp::class_<RD_MM::JointIndex>("JointIndex")
        .def(bp::init<>())
        .def_readwrite("virtual_start", &RD_MM::JointIndex::virtual_start)
        .def_readwrite("mani_start",    &RD_MM::JointIndex::mani_start)
        .def_readwrite("mobi_start",    &RD_MM::JointIndex::mobi_start);

    bp::class_<RD_MM::ActuatorIndex>("ActuatorIndex")
        .def(bp::init<>())
        .def_readwrite("mani_start", &RD_MM::ActuatorIndex::mani_start)
        .def_readwrite("mobi_start", &RD_MM::ActuatorIndex::mobi_start);

    bp::class_<RDMOBase, boost::noncopyable>("MobileBase", bp::init<const RD_MO::KinematicParam&>())
        .def("getVerbose",        &RDMOBase::getVerbose)
        .def("updateState",       &RDMOBase::updateState)
        .def("computeBaseVel",    &RDMOBase::computeBaseVel)
        .def("computeFKJacobian", &RDMOBase::computeFKJacobian)
        .def("getWheelNum",       &RDMOBase::getWheelNum, bp::return_value_policy<bp::return_by_value>())
        .def("getKineParam",      &RDMOBase::getKineParam,     bp::return_internal_reference<>())
        .def("getWheelPosition",  &RDMOBase::getWheelPosition, bp::return_internal_reference<>())
        .def("getWheelVelocity",  &RDMOBase::getWheelVelocity, bp::return_internal_reference<>())
        .def("getBaseVel",        &RDMOBase::getBaseVel,       bp::return_internal_reference<>())
        .def("getFKJacobian",     &RDMOBase::getFKJacobian,    bp::return_internal_reference<>())
        ;

    bp::class_<RDMNBase, boost::noncopyable>("ManipulatorBase", bp::init<const std::string&, const std::string&, const std::string&>())
        .def("getVerbose",                   &RDMNBase::getVerbose)
        .def("updateState",                  &RDMNBase::updateState)
        .def("computeMassMatrix",            &RDMNBase::computeMassMatrix)
        .def("computeGravity",               &RDMNBase::computeGravity)
        .def("computeCoriolis",              &RDMNBase::computeCoriolis)
        .def("computeNonlinearEffects",      &RDMNBase::computeNonlinearEffects)
        .def("computePose",                  &RDMNBase::computePose)
        .def("computeJacobian",              &RDMNBase::computeJacobian)
        .def("computeJacobianTimeVariation", &RDMNBase::computeJacobianTimeVariation)
        .def("computeVelocity",              &RDMNBase::computeVelocity)
        .def("computeMinDistance",           &RDMNBase::computeMinDistance)
        .def("computeManipulability",        &RDMNBase::computeManipulability)
        .def("getDof",                       &RDMNBase::getDof)
        .def("getJointPosition",             &RDMNBase::getJointPosition)
        .def("getJointVelocity",             &RDMNBase::getJointVelocity)
        .def("getJointPositionLimit",        &RDMNBase::getJointPositionLimit)
        .def("getJointVelocityLimit",        &RDMNBase::getJointVelocityLimit)
        .def("getMassMatrix",                &RDMNBase::getMassMatrix)
        .def("getMassMatrixInv",             &RDMNBase::getMassMatrixInv)
        .def("getCoriolis",                  &RDMNBase::getCoriolis)
        .def("getGravity",                   &RDMNBase::getGravity)
        .def("getNonlinearEffects",          &RDMNBase::getNonlinearEffects)
        .def("getPose",                      &RDMNBase::getPose)
        .def("getJacobian",                  &RDMNBase::getJacobian)
        .def("getJacobianTimeVariation",     &RDMNBase::getJacobianTimeVariation)
        .def("getVelocity",                  &RDMNBase::getVelocity)
        .def("getMinDistance",               &RDMNBase::getMinDistance)
        .def("getManipulability",            &RDMNBase::getManipulability);

    typedef bool (RDMMBase::*Upd6)(const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&);
    typedef MatrixXd (RDMMBase::*Mat3)(const VectorXd&, const VectorXd&, const VectorXd&);
    typedef VectorXd (RDMMBase::*Vec3)(const VectorXd&, const VectorXd&, const VectorXd&);
    typedef VectorXd (RDMMBase::*Vec6)(const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&);
    typedef Affine3d (RDMMBase::*Aff4)(const VectorXd&, const VectorXd&, const VectorXd&, const std::string&);
    typedef MatrixXd (RDMMBase::*Mat4)(const VectorXd&, const VectorXd&, const VectorXd&, const std::string&);
    typedef MatrixXd (RDMMBase::*Mat7)(const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const std::string&);
    typedef VectorXd (RDMMBase::*Vec7)(const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const std::string&);
    typedef RobotData::Manipulator::MinDistResult (RDMMBase::*Min9)(const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const VectorXd&, const bool&, const bool&, const bool);
    typedef RobotData::Manipulator::ManipulabilityResult (RDMMBase::*Man5)(const VectorXd&, const VectorXd&, const bool&, const bool&, const std::string&);

    bp::class_<RDMMBase, bp::bases<RDMNBase, RDMOBase>, boost::noncopyable>("MobileManipulatorBase", bp::init<const RD_MO::KinematicParam&, const std::string&, const std::string&, const std::string&, const RD_MM::JointIndex&, const RD_MM::ActuatorIndex&>())
        .def("getVerbose",                                     &RDMMBase::getVerbose)
        .def("updateState",                  static_cast<Upd6>(&RDMMBase::updateState))
        .def("computeMassMatrix",            static_cast<Mat3>(&RDMMBase::computeMassMatrix))
        .def("computeGravity",               static_cast<Vec3>(&RDMMBase::computeGravity))
        .def("computeCoriolis",              static_cast<Vec6>(&RDMMBase::computeCoriolis))
        .def("computeNonlinearEffects",      static_cast<Vec6>(&RDMMBase::computeNonlinearEffects))
        .def("computeMassMatrixActuated",                      &RDMMBase::computeMassMatrixActuated)
        .def("computeGravityActuated",                         &RDMMBase::computeGravityActuated)
        .def("computeCoriolisActuated",                        &RDMMBase::computeCoriolisActuated)
        .def("computeNonlinearEffectsActuated",                &RDMMBase::computeNonlinearEffectsActuated)
        .def("computePose",                  static_cast<Aff4>(&RDMMBase::computePose))
        .def("computeJacobian",              static_cast<Mat4>(&RDMMBase::computeJacobian))
        .def("computeJacobianTimeVariation", static_cast<Mat7>(&RDMMBase::computeJacobianTimeVariation))
        .def("computeVelocity",              static_cast<Vec7>(&RDMMBase::computeVelocity))
        .def("computeMinDistance",                        Min9(&RDMMBase::computeMinDistance))
        .def("computeSelectionMatrix",                         &RDMMBase::computeSelectionMatrix)
        .def("computeJacobianActuated",                        &RDMMBase::computeJacobianActuated)
        .def("computeJacobianTimeVariationActuated",           &RDMMBase::computeJacobianTimeVariationActuated)
        .def("computeManipulability",                     Man5(&RDMMBase::computeManipulability))
        .def("computeMobileFKJacobian",                        &RDMMBase::computeMobileFKJacobian)
        .def("computeMobileBaseVel",                           &RDMMBase::computeMobileBaseVel)
        .def("getActuatordDof",                                &RDMMBase::getActuatordDof)
        .def("getManipulatorDof",                              &RDMMBase::getManipulatorDof)
        .def("getMobileDof",                                   &RDMMBase::getMobileDof)
        .def("getJointIndex",                                  &RDMMBase::getJointIndex)
        .def("getActuatorIndex",                               &RDMMBase::getActuatorIndex)
        .def("getMobileJointPosition",                         &RDMMBase::getMobileJointPosition)
        .def("getVirtualJointPosition",                        &RDMMBase::getVirtualJointPosition)
        .def("getManiJointPosition",                           &RDMMBase::getManiJointPosition)
        .def("getJointVelocityActuated",                       &RDMMBase::getJointVelocityActuated)
        .def("getMobileJointVelocity",                         &RDMMBase::getMobileJointVelocity)
        .def("getVirtualJointVelocity",                        &RDMMBase::getVirtualJointVelocity)
        .def("getManiJointVelocity",                           &RDMMBase::getManiJointVelocity)
        .def("getJointPositionActuated",                       &RDMMBase::getJointPositionActuated)
        .def("getMassMatrixActuated",                          &RDMMBase::getMassMatrixActuated)
        .def("getMassMatrixActuatedInv",                       &RDMMBase::getMassMatrixActuatedInv)
        .def("getGravityActuated",                             &RDMMBase::getGravityActuated)
        .def("getCoriolisActuated",                            &RDMMBase::getCoriolisActuated)
        .def("getNonlinearEffectsActuated",                    &RDMMBase::getNonlinearEffectsActuated)
        .def("getJacobianActuated",                            &RDMMBase::getJacobianActuated)
        .def("getJacobianActuatedTimeVariation",               &RDMMBase::getJacobianActuatedTimeVariation)
        .def("getSelectionMatrix",                             &RDMMBase::getSelectionMatrix)
        .def("getManipulability",                              &RDMMBase::getManipulability)
        .def("getMobileFKJacobian",                            &RDMMBase::getMobileFKJacobian)
        .def("getMobileBaseVel",                               &RDMMBase::getMobileBaseVel)
        ;

    bp::class_<RCMOBase, boost::noncopyable >("MobileControllerBase", bp::init<const double&, std::shared_ptr< RD_MO::MobileBase > >())
        .def("computeWheelVel",   &RCMOBase::computeWheelVel)
        .def("computeIKJacobian", &RCMOBase::computeIKJacobian)
        .def("VelocityCommand",   &RCMOBase::VelocityCommand);
    
    typedef VectorXd (RCMNBase::*CLIKStep1)(const Affine3d&, const VectorXd&, const VectorXd&, const std::string&);
    typedef VectorXd (RCMNBase::*CLIKStep2)(const Affine3d&, const VectorXd&, const std::string&);
    typedef VectorXd (RCMNBase::*CLIKCub1)(const Affine3d&, const VectorXd&, const Affine3d&, const VectorXd&, const double&, const double&, const double&, const VectorXd&, const std::string&);
    typedef VectorXd (RCMNBase::*CLIKCub2)(const Affine3d&, const VectorXd&, const Affine3d&, const VectorXd&, const double&, const double&, const double&, const std::string&);
    typedef VectorXd (RCMNBase::*OSF1)(const VectorXd&, const VectorXd&, const std::string&);
    typedef VectorXd (RCMNBase::*OSF2)(const VectorXd&, const std::string&);
    typedef VectorXd (RCMNBase::*OSFStep1)(const Affine3d&, const VectorXd&, const VectorXd&, const std::string&);
    typedef VectorXd (RCMNBase::*OSFStep2)(const Affine3d&, const VectorXd&, const std::string&);
    typedef VectorXd (RCMNBase::*OSFCub1)(const Affine3d&, const VectorXd&, const Affine3d&, const VectorXd&, const double&, const double&, const double&, const VectorXd&, const std::string&);
    typedef VectorXd (RCMNBase::*OSFCub2)(const Affine3d&, const VectorXd&, const Affine3d&, const VectorXd&, const double&, const double&, const double&, const std::string&);

    bp::class_<RCMNBase, boost::noncopyable >("ManipulatorControllerBase", bp::init<const double&, std::shared_ptr<RD_MN::ManipulatorBase>>())
        .def("setJointGain",                                                                               &RCMNBase::setJointGain)
        .def("setTaskGain",                                                                                &RCMNBase::setTaskGain)
        .def("moveJointPositionCubic",                                                                     &RCMNBase::moveJointPositionCubic)
        .def("moveJointTorqueStep",                   static_cast<VectorXd (RCMNBase::*)(const VectorXd&)>(&RCMNBase::moveJointTorqueStep))
        .def("moveJointTorqueStep",  static_cast<VectorXd (RCMNBase::*)(const VectorXd&, const VectorXd&)>(&RCMNBase::moveJointTorqueStep))
        .def("moveJointTorqueCubic",                                                                       &RCMNBase::moveJointTorqueCubic)
        .def("CLIKStep",                                                            static_cast<CLIKStep1>(&RCMNBase::CLIKStep))
        .def("CLIKStep",                                                            static_cast<CLIKStep2>(&RCMNBase::CLIKStep))
        .def("CLIKCubic",                                                            static_cast<CLIKCub1>(&RCMNBase::CLIKCubic))
        .def("CLIKCubic",                                                            static_cast<CLIKCub2>(&RCMNBase::CLIKCubic))
        .def("OSF",                                                                      static_cast<OSF1>(&RCMNBase::OSF))
        .def("OSF",                                                                      static_cast<OSF2>(&RCMNBase::OSF))
        .def("OSFStep",                                                              static_cast<OSFStep1>(&RCMNBase::OSFStep))
        .def("OSFStep",                                                              static_cast<OSFStep2>(&RCMNBase::OSFStep))
        .def("OSFCubic",                                                              static_cast<OSFCub1>(&RCMNBase::OSFCubic))
        .def("OSFCubic",                                                              static_cast<OSFCub2>(&RCMNBase::OSFCubic))
        .def("QPIK",                                                                                       &RCMNBase::QPIK)
        .def("QPIKStep",                                                                                   &RCMNBase::QPIKStep)
        .def("QPIKCubic",                                                                                  &RCMNBase::QPIKCubic)
        .def("QPID",                                                                                       &RCMNBase::QPID)
        .def("QPIDStep",                                                                                   &RCMNBase::QPIDStep)
        .def("QPIDCubic",                                                                                  &RCMNBase::QPIDCubic)
        ;

    typedef VectorXd (RCMNBase::*CLIKStep1)(const Affine3d&, const VectorXd&, const VectorXd&, const std::string&);

    bp::class_<RCMMBase, boost::noncopyable>("MobileManipulatorControllerBase", bp::init<const double&, std::shared_ptr<RD_MM::MobileManipulatorBase>>())
        .def("setManipulatorJointGain",                                                                                &RCMMBase::setManipulatorJointGain)
        .def("setTaskGain",                                                                                            &RCMMBase::setTaskGain)
        .def("moveManipulatorJointPositionCubic",                                                                      &RCMMBase::moveManipulatorJointPositionCubic)
        .def("moveManipulatorJointTorqueStep",                     static_cast<VectorXd(RCMMBase::*)(const VectorXd&)>(&RCMMBase::moveManipulatorJointTorqueStep))
        .def("moveManipulatorJointTorqueStep",    static_cast<VectorXd(RCMMBase::*)(const VectorXd&, const VectorXd&)>(&RCMMBase::moveManipulatorJointTorqueStep))
        .def("moveManipulatorJointTorqueCubic",                                                                        &RCMMBase::moveManipulatorJointTorqueCubic)
        .def("QPIK",                                                                                                   &RCMMBase_QPIK_tuple)
        .def("QPIKStep",                                                                                               &RCMMBase_QPIKStep_tuple)
        .def("QPIKCubic",                                                                                              &RCMMBase_QPIKCubic_tuple)
        .def("QPID",                                                                                                   &RCMMBase_QPID_tuple)
        .def("QPIDStep",                                                                                               &RCMMBase_QPIDStep_tuple)
        .def("QPIDCubic",                                                                                              &RCMMBase_QPIDCubic_tuple)
        ;
}