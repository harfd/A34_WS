#pragma once
#include "Solver/solver_base.h"
#include "Basic/mpc.h"

namespace ns_control{
    class Solver_mpc : public Solver{
        public:
        void solve();
    };
}