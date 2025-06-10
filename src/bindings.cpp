#define BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#define BOOST_MPL_LIMIT_LIST_SIZE 40

#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <Eigen/Dense>

#include "fr3_controller/robot_data.h"
#include "fr3_controller/controller.h"

#include "husky_fr3_controller/robot_data.h"
#include "husky_fr3_controller/controller.h"

namespace bp = boost::python;
using namespace Eigen;

// Converter for std::vector<std::string>
struct VectorString_to_python
{
    static PyObject* convert(const std::vector<std::string>& vec)
    {
        boost::python::list py_list;
        for (const auto& str : vec)
        {
            py_list.append(str);
        }   
        return bp::incref(py_list.ptr());
    }
};

struct VectorString_from_python
{
    VectorString_from_python()
    {
        bp::converter::registry::push_back(&convertible, &construct, boost::python::type_id<std::vector<std::string>>());
    }

    static void* convertible(PyObject* obj_ptr)
    {
        if (!PySequence_Check(obj_ptr)) return nullptr;
        return obj_ptr;
    }

    static void construct(PyObject* obj_ptr, bp::converter::rvalue_from_python_stage1_data* data)
    {
        void* storage = ((bp::converter::rvalue_from_python_storage<std::vector<std::string>>*)data)->storage.bytes;
        new (storage) std::vector<std::string>();
        std::vector<std::string>& vec = *(std::vector<std::string>*)(storage);

        int len = PySequence_Size(obj_ptr);
        if (len < 0) bp::throw_error_already_set();
        vec.reserve(len);

        for (int i = 0; i < len; ++i)
        {
            vec.push_back(bp::extract<std::string>(PySequence_GetItem(obj_ptr, i)));
        }

        data->convertible = storage;
    }
};


BOOST_PYTHON_MODULE(dyros_robot_controller_wrapper_cpp) 
{
   
    eigenpy::enableEigenPy();
    eigenpy::enableEigenPySpecific<Eigen::Matrix<double, Eigen::Dynamic, 1>>();
    eigenpy::enableEigenPySpecific<Eigen::Matrix<double, 4, 4>>();
    eigenpy::enableEigenPySpecific<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>>();

    bp::to_python_converter<std::vector<std::string>, VectorString_to_python>();
    VectorString_from_python();

    // ===================================================================================
    // ============================= Bindings for FR3Controller ==========================
    // ===================================================================================
    {
        using namespace FR3Controller;

        // Bind RobotData class
        bp::class_<RobotData, boost::noncopyable>("FR3RobotData", bp::init<std::string, bool>())
        .def("updateState", &RobotData::updateState)
        .def("getJointNames", &RobotData::getJointNames)
        .def("computePose", &RobotData::computePose)
        .def("getPose", &RobotData::getPose)
        .def("computeJacobian", &RobotData::computeJacobian)
        .def("getJacobian", &RobotData::getJacobian)
        .def("computeJacobianTimeVariation", &RobotData::computeJacobianTimeVariation)
        .def("getJacobianTimeVariation", &RobotData::getJacobianTimeVariation)
        .def("computeVelocity", &RobotData::computeVelocity)
        .def("getVelocity", &RobotData::getVelocity)
        .def("computeMassMatrix", &RobotData::computeMassMatrix)
        .def("getMassMatrix", &RobotData::getMassMatrix)
        .def("computeCoriolis", &RobotData::computeCoriolis)
        .def("getCoriolis", &RobotData::getCoriolis)
        .def("computeGravity", &RobotData::computeGravity)
        .def("getGravity", &RobotData::getGravity)
        .def("computeNonlinearEffects", &RobotData::computeNonlinearEffects)
        .def("getNonlinearEffects", &RobotData::getNonlinearEffects)
        ;
        
        // Bind Controller class
        bp::class_<Controller, boost::noncopyable>("FR3Controller", bp::init<double>())
        .def("tmpControl", &Controller::tmpControl)
        ;  
    }

    // ===================================================================================
    // =========================== Bindings for HuskyFR3Controller =======================
    // ===================================================================================
    {
        using namespace HuskyFR3Controller;

        // Bind RobotData class
        bp::class_<RobotData, boost::noncopyable>("HuskyFR3RobotData", bp::init<std::string, bool>())
            .def("updateState", &RobotData::updateState)
            .def("getJointNames", &RobotData::getJointNames)
            .def("computePose", &RobotData::computePose)
            .def("getPose", &RobotData::getPose)
            .def("computeJacobian", &RobotData::computeJacobian)
            .def("computeJacobianActuated", &RobotData::computeJacobianActuated)
            .def("getJacobian", &RobotData::getJacobian)
            .def("getJacobianActuated", &RobotData::getJacobianActuated)
            .def("computeJacobianTimeVariation", &RobotData::computeJacobianTimeVariation)
            .def("getJacobianTimeVariation", &RobotData::getJacobianTimeVariation)
            .def("computeVelocity", &RobotData::computeVelocity)
            .def("getVelocity", &RobotData::getVelocity)
            .def("computeMassMatrix", &RobotData::computeMassMatrix)
            .def("getMassMatrix", &RobotData::getMassMatrix)
            .def("computeCoriolis", &RobotData::computeCoriolis)
            .def("getCoriolis", &RobotData::getCoriolis)
            .def("computeGravity", &RobotData::computeGravity)
            .def("getGravity", &RobotData::getGravity)
            .def("computeNonlinearEffects", &RobotData::computeNonlinearEffects)
            .def("getNonlinearEffects", &RobotData::getNonlinearEffects)
            .def("computeMassMatrixActuated", &RobotData::computeMassMatrixActuated)
            .def("getMassMatrixActuated", &RobotData::getMassMatrixActuated)
            .def("computeCoriolisActuated", &RobotData::computeCoriolisActuated)
            .def("getCoriolisActuated", &RobotData::getCoriolisActuated)
            .def("computeGravityActuated", &RobotData::computeGravityActuated)
            .def("getGravityActuated", &RobotData::getGravityActuated)
            .def("computeNonlinearEffectsActuated", &RobotData::computeNonlinearEffectsActuated)
            .def("getNonlinearEffectsActuated", &RobotData::getNonlinearEffectsActuated)
            ;
        
        // Bind Controller class
        bp::class_<Controller, boost::noncopyable>("HuskyFR3Controller", bp::init<double, std::shared_ptr<RobotData>>())
            .def("starting", &Controller::starting)
            .def("updateState", &Controller::updateState)
            .def("computeFast", &Controller::computeFast)
            .def("computeSlow", &Controller::computeSlow)
            .def("getCtrlInput", &Controller::getCtrlInput)
            .def("setMode", &Controller::setMode)
            ;  
    }
}
