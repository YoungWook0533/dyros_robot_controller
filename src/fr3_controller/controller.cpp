#include "fr3_controller/controller.h"

namespace FR3Controller
{
    Controller::Controller(double dt)
    {
        dt_ = dt;
    }

    Controller::~Controller()
    {
    }

    VectorXd Controller::tmpControl()
    {
        return VectorXd::Zero(7);
    }
}