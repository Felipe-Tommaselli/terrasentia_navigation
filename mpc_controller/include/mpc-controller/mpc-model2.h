#pragma once

#include <vector>
#include <Eigen/Dense>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using namespace std;

struct MpcParams{
    double gain_ctrack_error;
    double gain_heading_error;
    //double gain_velocity_error;
    double gain_kappa_effort;
    //double gain_accel_effor;
    double gain_deriv_kappa;
    //double gain_deriv_accel;
    double ref_cte;
    double ref_epsi;
    //double ref_v;
    double Rmin;
    double sigma;
    double latency;
    //double timestep_dt;
    double ds;
    size_t timestep_N;
    double bounds_vars_limit;
    //double bounds_accel;
    double options_max_cpu_time_seconds;
    string solver;
    bool use_warm_start;
};
struct Indexes{
    size_t x_start;
    size_t y_start;
    size_t psi_start;
    size_t v_start;
    size_t cte_start;
    size_t epsi_start;
    size_t kappa_start;
    size_t a_start;
};

class MPC
{
    public:
        MPC();
        MPC(MpcParams);
        virtual ~MPC();

        // Solve the model given an initial state and polynomial coefficients.
        // Return the first actuations.
        double Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& mpc_ptsx, const Eigen::VectorXd& mpc_ptsy);

        std::vector<double> x_pred_vals;
        std::vector<double> y_pred_vals;
        std::vector<double> kappa_pred_vals;

    private:
        MpcParams   _p;
        Indexes     _idx;
        CppAD::vector<double> _last_solution;
};
