#pragma once

#include "PBDSolverParameters.hpp"

namespace Eloy {

class IBehavior {
private:
public:
    virtual ~IBehavior() {}
    virtual void start(InjectionParameters&) {}
    virtual void step(InjectionParameters&) {}
    virtual void reset(InjectionParameters&) {}
    virtual void imgui(InjectionParameters&) {}
};

};