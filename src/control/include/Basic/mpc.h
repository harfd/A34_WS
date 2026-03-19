#pragma once
#include "cppad/cppad.hpp"
#include "cppad/ipopt/solve.hpp"
#include "Basic/types.h"
#include "Basic/param.h"

namespace ns_control
{
    using CppAD::AD;
    class FG_eval {
    public:

    Trajectory relativePath_;
    FG_eval(Trajectory relativePath)
    {
        this->relativePath_ = relativePath;
    }
    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

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

    void operator()(ADvector& fg, const ADvector& vars){
        const double desired_lateral_error = 0.0;
        const double desired_yaw_error = 0.0;

        const double cost_func_x_weight = param_.weight.x;
        const double cost_func_y_weight = param_.weight.y;
        const double cost_func_yaw_weight = param_.weight.yaw;
        const double cost_func_lateral_error_weight = param_.weight.lateral_error;
        const double cost_func_yaw_error_weight = param_.weight.yaw_error;
        const double cost_func_v_weight = param_.weight.v;
        const double cost_func_steering_angle_weight = param_.weight.steering_angle;
        const double cost_func_throttle_weight = param_.weight.throttle;
        const double cost_func_steering_rate_weight = param_.weight.steering_rate;
        const double cost_func_throttle_rate_weight = param_.weight.throttle_rate;
    

        fg[0] = 0.0;
        for (int t = 0; t < predicition_domain; t++)
        {
        fg[0] += cost_func_x_weight * pow(vars[x_range_begin + t] - relativePath_[t].track_x, 2);
        fg[0] += cost_func_y_weight * pow(vars[y_range_begin + t] - relativePath_[t].track_y, 2);
        fg[0] += cost_func_yaw_weight * pow(vars[yaw_range_begin + t] - relativePath_[t].track_yaw, 2);
        fg[0] += cost_func_lateral_error_weight * pow(vars[lateral_error_range_begin + t]  - desired_lateral_error,  2);
        //fg[0] += cost_func_yaw_error_weight * pow(vars[yaw_error_range_begin + t] - desired_yaw_error, 2);
        fg[0] += cost_func_v_weight * pow(vars[v_range_begin + t] - relativePath_[t].track_v, 2);
        }

        for (int t = 0; t < control_domain; t++)
        {
        fg[0] += cost_func_steering_angle_weight * pow(vars[steering_angle_range_begin + t], 2);
        fg[0] += cost_func_throttle_weight * pow(vars[throttle_range_begin + t], 2);
        }

        for (int t = 0; t < control_domain - 1; t++)
        {
        fg[0] += cost_func_steering_rate_weight  * pow(vars[steering_angle_range_begin + t + 1] - vars[steering_angle_range_begin + t], 2);
        fg[0] += cost_func_throttle_rate_weight * pow(vars[throttle_range_begin + t + 1] - vars[throttle_range_begin + t], 2);
        }

        fg[1 + x_range_begin] = vars[x_range_begin];
        fg[1 + y_range_begin] = vars[y_range_begin];
        fg[1 + yaw_range_begin] = vars[yaw_range_begin];
        fg[1 + v_range_begin] = vars[v_range_begin];
        fg[1 + lateral_error_range_begin] = vars[lateral_error_range_begin];
        fg[1 + yaw_error_range_begin] = vars[yaw_error_range_begin];

        for (int i = 0; i < N - 1; i++) {
        AD<double> x1 = vars[x_range_begin + i + 1];
        AD<double> y1 = vars[y_range_begin + i + 1];
        AD<double> yaw1 = vars[yaw_range_begin + i + 1];
        AD<double> v1 = vars[v_range_begin + i + 1];
        AD<double> lateral_error1 = vars[lateral_error_range_begin + i + 1];
        AD<double> yaw_error1 = vars[yaw_error_range_begin + i + 1];

        AD<double> x0 = vars[x_range_begin + i];
        AD<double> y0 = vars[y_range_begin + i];
        AD<double> yaw0 = vars[yaw_range_begin + i];
        AD<double> v0 = vars[v_range_begin + i];
        AD<double> lateral_error0 = vars[lateral_error_range_begin + i];
        AD<double> yaw_error0 = vars[yaw_error_range_begin + i];

        AD<double> steering_angle0 = vars[steering_angle_range_begin + i];
        AD<double> throttle0 = vars[throttle_range_begin + i];
        AD<double> y_relative0 = relativePath_[i].track_y;
        AD<double> yaw_relative0 = relativePath_[i].track_yaw;
        
        fg[2 + x_range_begin + i] = x1 - (x0 + v0 * CppAD::cos(yaw0) * dt);
        fg[2 + y_range_begin + i] = y1 - (y0 + v0 * CppAD::sin(yaw0) * dt);
        fg[2 + yaw_range_begin + i] = yaw1 - (yaw0 + v0 * steering_angle0 / car_length * dt);
        fg[2 + v_range_begin + i] = v1 - (v0 + throttle0 * dt);
        fg[2 + lateral_error_range_begin + i] = lateral_error1 - ((y_relative0 - y0) + (v0 * CppAD::sin(yaw_error0) * dt));
        fg[2 + yaw_error_range_begin + i] = yaw_error1 - ((yaw0 - yaw_relative0) + v0 * steering_angle0 / car_length * dt);
        }
     }
  };
}
