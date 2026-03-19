#include "Solver/solver_base.h"
#include "ros/ros.h"

namespace ns_control{
    class Solver_purePursuit : public Solver{
        public:
        void solve();

        private:
        geometry_msgs::Point next_point;
    };
}