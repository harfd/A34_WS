#include "Solver/solver_mpc.h"
#include "ros/ros.h"
#include <cmath>
namespace ns_control {

    void Solver_mpc::solve() {
        using CppAD::AD;
        typedef CPPAD_TESTVECTOR(double) Dvector;
        const int N = param_.N;
        const int predicition_domain = N;
        const int control_domain = N - 1;
        const double dt = param_.dt;
        const double car_length = param_.car_length;

        const int x_range_begin              = 0;
        const int y_range_begin              = x_range_begin + predicition_domain;
        const int yaw_range_begin            = y_range_begin + predicition_domain;
        const int v_range_begin              = yaw_range_begin + predicition_domain;
        const int lateral_error_range_begin  = v_range_begin + predicition_domain;
        const int yaw_error_range_begin      = lateral_error_range_begin + predicition_domain;
        const int steering_angle_range_begin = yaw_error_range_begin  + predicition_domain;
        const int throttle_range_begin       = steering_angle_range_begin + control_domain;

        double vehicle_v       = state_.vehicle_v;
        double steering_angle  = state_.steering_angle;
        double throttle        = state_.throttle;
        int    state_size      = 6;
        int    control_size    = 2;
        int    n_vars          = state_size*predicition_domain + control_size*control_domain;
        int    n_constraints   = state_size * predicition_domain;

        Dvector vars(n_vars);
        for (int i = 0; i < n_vars; i++) { vars[i] = 0; }


        vars[x_range_begin]                  = 0.0;
        vars[y_range_begin]                  = 0.0;
        vars[yaw_range_begin]                = 0.0;
        vars[v_range_begin]                  = vehicle_v;
        vars[lateral_error_range_begin]      = 0.0;
        vars[yaw_error_range_begin]          = 0.0;
        vars[steering_angle_range_begin]     = 0.0;
        vars[throttle_range_begin]           = 0.0;

        Dvector vars_lowerbound(n_vars);
        Dvector vars_upperbound(n_vars);

        for (int i = 0; i < steering_angle_range_begin; i++)
        {
            vars_lowerbound[i] = -1.0e19;
            vars_upperbound[i] = 1.0e19;
        }

        for (int i = steering_angle_range_begin; i < throttle_range_begin; i++)
        {
            vars_lowerbound[i] = -0.5;
            vars_upperbound[i] = 0.5;
        }

        for (int i = throttle_range_begin; i < n_vars; i++)
        {
            vars_lowerbound[i] = -1.0;
            vars_upperbound[i] = +1.0;
        }

        Dvector constraints_lowerbound(n_constraints);
        Dvector constraints_upperbound(n_constraints);
        
        for (int i = 0; i < n_constraints; i++)
        {
            constraints_lowerbound[i] = 0.0;
            constraints_upperbound[i] = 0.0;
        }

        constraints_lowerbound[x_range_begin]    = 0.0;
        constraints_upperbound[x_range_begin]    = 0.0;

        constraints_lowerbound[y_range_begin]    = 0.0;
        constraints_upperbound[y_range_begin]    = 0.0;

        constraints_lowerbound[yaw_range_begin]  = 0.0;
        constraints_upperbound[yaw_range_begin]  = 0.0;

        constraints_lowerbound[v_range_begin]    = vehicle_v;
        constraints_upperbound[v_range_begin]    = vehicle_v;

        constraints_lowerbound[lateral_error_range_begin]  = 0.0;
        constraints_upperbound[lateral_error_range_begin]  = 0.0;

        constraints_lowerbound[yaw_error_range_begin] = 0.0;
        constraints_upperbound[yaw_error_range_begin] = 0.0;

        std::string options;
        bool ok = true;
        options += "Integer print_level  0\n";
        
        options += "Sparse  true        forward\n";
        options += "Sparse  true        reverse\n";

        options += "Numeric max_cpu_time          0.5\n";

        CppAD::ipopt::solve_result<Dvector> solution;

        FG_eval fg_eval(trajectory_);
        
        CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound, constraints_upperbound, fg_eval, solution);

        ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

        double cost = solution.obj_value;

          for (int i = 0; i < N; i++)
        {
            geometry_msgs::Point p;
            p.x = solution.x[x_range_begin + i];
            p.y = solution.x[y_range_begin + i];
        }
        controlCommand_.steering_angle.data = solution.x[steering_angle_range_begin];
        controlCommand_.throttle.data = solution.x[throttle_range_begin];

        std::cout << " steering angle : " << controlCommand_.steering_angle.data << std::endl;
        std::cout << " throttle angle : " << controlCommand_.throttle.data << std::endl;

        predictive_path.clear();
        TrajectoryPoint p_tmp;
        for (int i = 0; i < N; i++)
        {
            geometry_msgs::Point p;
            p_tmp.track_x = solution.x[x_range_begin + i];
            p_tmp.track_y = solution.x[y_range_begin + i];
            predictive_path.push_back(p_tmp);
        }
    }
}