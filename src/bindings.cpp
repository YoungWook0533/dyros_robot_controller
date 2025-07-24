// #define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
// #define BOOST_MPL_LIMIT_LIST_SIZE 40

// #include <boost/python.hpp>
// #include <eigenpy/eigenpy.hpp>
// #include <Eigen/Dense>

// #include "robot_data/manipulator.h"
// #include "robot_data/mobile_manipulator.h"

// #include "robot_controller/mobile_controller/differential.h"
// #include "robot_controller/mobile_controller/mecanum.h"
// #include "robot_controller/mobile_controller/powered_caster.h"
// #include "robot_controller/manipulator_controller/position.h"
// #include "robot_controller/manipulator_controller/torque.h"

// namespace bp = boost::python;

// // std::pair<VectorXd,VectorXd> → Python tuple
// struct PairVectorXdToPython
// {
//   static PyObject* convert(const std::pair<Eigen::VectorXd, Eigen::VectorXd>& p)
//   {
//     bp::object first_obj(p.first);
//     bp::object second_obj(p.second);
//     bp::tuple t = make_tuple(first_obj, second_obj);
//     return bp::incref(t.ptr());
//   }
// };

// BOOST_PYTHON_MODULE(dyros_robot_controller_wrapper_cpp) 
// {
//   eigenpy::enableEigenPy();
//   eigenpy::enableEigenPySpecific<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>();
//   eigenpy::enableEigenPySpecific<Eigen::Matrix<double, Eigen::Dynamic, 1>>();
//   eigenpy::enableEigenPySpecific<Eigen::Matrix4d>();
//   eigenpy::enableEigenPySpecific<Eigen::Affine3d>();

//   bp::to_python_converter<std::pair<Eigen::VectorXd, Eigen::VectorXd>,PairVectorXdToPython>();
  
  
//   // ==================================== RobotData ====================================
//   // ============ Base ============
//   bp::class_<RobotData::MinDistResult>("MinDistResult")
//     .def_readonly("distance", &RobotData::MinDistResult::distance)
//     .def_readonly("grad"    , &RobotData::MinDistResult::grad)
//     .def_readonly("grad_dot", &RobotData::MinDistResult::grad_dot)
//   ;

//   bp::class_<RobotData::ManipulabilityResult>("ManipulabilityResult")
//     .def_readonly("manipulability", &RobotData::ManipulabilityResult::manipulability)
//     .def_readonly("grad"    ,       &RobotData::ManipulabilityResult::grad)
//     .def_readonly("grad_dot",       &RobotData::ManipulabilityResult::grad_dot)
//   ;

//   // ============ Manipulator ============
//   bp::class_<RobotData::RobotDataManipulator, boost::noncopyable>("RobotDataManipulator", bp::init<std::string, 
//                                                                                           std::string, 
//                                                                                           int, 
//                                                                                           std::string, 
//                                                                                           bool>())
//     .def("updateState",                  &RobotData::RobotDataManipulator::updateState)
//     .def("computeMinDistance",           &RobotData::RobotDataManipulator::computeMinDistance)
//     .def("getMinDistance",               &RobotData::RobotDataManipulator::getMinDistance)
//     .def("getJointPosition",             &RobotData::RobotDataManipulator::getJointPosition)
//     .def("getJointVelocity",             &RobotData::RobotDataManipulator::getJointVelocity)
//     .def("getJointPositionLimit",        &RobotData::RobotDataManipulator::getJointPositionLimit)
//     .def("getJointVelocityLimit",        &RobotData::RobotDataManipulator::getJointVelocityLimit)
//     .def("computePose",                  &RobotData::RobotDataManipulator::computePose)
//     .def("computeJacobian",              &RobotData::RobotDataManipulator::computeJacobian)
//     .def("computeJacobianTimeVariation", &RobotData::RobotDataManipulator::computeJacobianTimeVariation)
//     .def("computeVelocity",              &RobotData::RobotDataManipulator::computeVelocity)
//     .def("computeMassMatrix",            &RobotData::RobotDataManipulator::computeMassMatrix)
//     .def("computeCoriolis",              &RobotData::RobotDataManipulator::computeCoriolis)
//     .def("computeNonlinearEffects",      &RobotData::RobotDataManipulator::computeNonlinearEffects)
//     .def("computeGravity",               &RobotData::RobotDataManipulator::computeGravity)
//     .def("computeManipulability",        &RobotData::RobotDataManipulator::computeManipulability)
//     .def("getDof",                       &RobotData::RobotDataManipulator::getDof)
//     .def("getEEName",                    &RobotData::RobotDataManipulator::getEEName)
//     .def("getPose",                      &RobotData::RobotDataManipulator::getPose)
//     .def("getJacobian",                  &RobotData::RobotDataManipulator::getJacobian)
//     .def("getJacobianTimeVariation",     &RobotData::RobotDataManipulator::getJacobianTimeVariation)
//     .def("getVelocity",                  &RobotData::RobotDataManipulator::getVelocity)
//     .def("getMassMatrix",                &RobotData::RobotDataManipulator::getMassMatrix)
//     .def("getMassMatrixInv",             &RobotData::RobotDataManipulator::getMassMatrixInv)
//     .def("getCoriolis",                  &RobotData::RobotDataManipulator::getCoriolis)
//     .def("getGravity",                   &RobotData::RobotDataManipulator::getGravity)
//     .def("getNonlinearEffects",          &RobotData::RobotDataManipulator::getNonlinearEffects)
//     .def("getManipulability",            &RobotData::RobotDataManipulator::getManipulability)
//   ;

//   // ============ Mobile Manipulator ============
//   bp::class_<RobotData::RobotDataMobileManipulator::JointIndex>("JointIndex")
//     .def_readonly("virtual_start",  &RobotData::RobotDataMobileManipulator::JointIndex::virtual_start)
//     .def_readonly("mani_start"    , &RobotData::RobotDataMobileManipulator::JointIndex::mani_start)
//     .def_readonly("mobi_start",     &RobotData::RobotDataMobileManipulator::JointIndex::mobi_start)
//   ;

//   bp::class_<RobotData::RobotDataMobileManipulator::ActuatorIndex>("ActuatorIndex")
//     .def_readonly("mani_start"    , &RobotData::RobotDataMobileManipulator::ActuatorIndex::mani_start)
//     .def_readonly("mobi_start",     &RobotData::RobotDataMobileManipulator::ActuatorIndex::mobi_start)
//   ;

//   bp::class_<RobotData::RobotDataMobileManipulator, boost::noncopyable>("RobotDataMobileManipulator", bp::init<std::string, 
//                                                                                                       std::string, 
//                                                                                                       int, 
//                                                                                                       int,
//                                                                                                       RobotData::RobotDataMobileManipulator::JointIndex,
//                                                                                                       RobotData::RobotDataMobileManipulator::ActuatorIndex,
//                                                                                                       std::string, 
//                                                                                                       bool>())
//     .def("updateState",                  &RobotData::RobotDataManipulator::updateState)
//     .def("computeMinDistance",           &RobotData::RobotDataManipulator::computeMinDistance)
//     .def("getMinDistance",               &RobotData::RobotDataManipulator::getMinDistance)
//     .def("getJointPosition",             &RobotData::RobotDataManipulator::getJointPosition)
//     .def("getJointVelocity",             &RobotData::RobotDataManipulator::getJointVelocity)
//     .def("getJointPositionLimit",        &RobotData::RobotDataManipulator::getJointPositionLimit)
//     .def("getJointVelocityLimit",        &RobotData::RobotDataManipulator::getJointVelocityLimit)
//     .def("computePose",                  &RobotData::RobotDataManipulator::computePose)
//     .def("computeJacobian",              &RobotData::RobotDataManipulator::computeJacobian)
//     .def("computeJacobianTimeVariation", &RobotData::RobotDataManipulator::computeJacobianTimeVariation)
//     .def("computeVelocity",              &RobotData::RobotDataManipulator::computeVelocity)
//     .def("computeMassMatrix",            &RobotData::RobotDataManipulator::computeMassMatrix)
//     .def("computeCoriolis",              &RobotData::RobotDataManipulator::computeCoriolis)
//     .def("computeNonlinearEffects",      &RobotData::RobotDataManipulator::computeNonlinearEffects)
//     .def("computeGravity",               &RobotData::RobotDataManipulator::computeGravity)
//     .def("computeManipulability",        &RobotData::RobotDataManipulator::computeManipulability)
//     .def("getDof",                       &RobotData::RobotDataManipulator::getDof)
//     .def("getEEName",                    &RobotData::RobotDataManipulator::getEEName)
//     .def("getPose",                      &RobotData::RobotDataManipulator::getPose)
//     .def("getJacobian",                  &RobotData::RobotDataManipulator::getJacobian)
//     .def("getJacobianTimeVariation",     &RobotData::RobotDataManipulator::getJacobianTimeVariation)
//     .def("getVelocity",                  &RobotData::RobotDataManipulator::getVelocity)
//     .def("getMassMatrix",                &RobotData::RobotDataManipulator::getMassMatrix)
//     .def("getMassMatrixInv",             &RobotData::RobotDataManipulator::getMassMatrixInv)
//     .def("getCoriolis",                  &RobotData::RobotDataManipulator::getCoriolis)
//     .def("getGravity",                   &RobotData::RobotDataManipulator::getGravity)
//     .def("getNonlinearEffects",          &RobotData::RobotDataManipulator::getNonlinearEffects)
//     .def("getManipulability",            &RobotData::RobotDataManipulator::getManipulability)
//   ;
  
//   // ==================================== RobotData ====================================

// }