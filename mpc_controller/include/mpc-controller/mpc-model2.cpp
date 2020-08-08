#include "mpc-model2.h"
#include <Eigen/Dense>

using CppAD::AD;

class FG_eval 
{
    public:
        FG_eval(MpcParams params, Indexes idx, const Eigen::VectorXd& mpc_ptsx, const Eigen::VectorXd& mpc_ptsy)
        { 
            _p = params;
            _idx = idx;
            _ptsx = mpc_ptsx;
            _ptsy = mpc_ptsy;
        }

        typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

        void operator()(ADvector& fg, const ADvector& vars) 
        {
            // --- Cost Function
            fg[0] = 0;
            // minimize:
            // 1. cte,
            // 2. epsi (orientation diff between vehicle and ref trajectory)
            for (int t = 0; t < _p.timestep_N - 1; t++)
            {
                const auto cte = vars[_idx.cte_start + t] - _p.ref_cte;
                const auto epsi = vars[_idx.epsi_start + t] - _p.ref_epsi;

                fg[0] += (_p.gain_ctrack_error * cte*cte + _p.gain_heading_error * epsi*epsi);
            }

            const auto cte = vars[_idx.cte_start + _p.timestep_N - 1] - _p.ref_cte;
            const auto epsi = vars[_idx.epsi_start + _p.timestep_N - 1] - _p.ref_epsi;

            fg[0] += 10*_p.gain_ctrack_error * cte*cte + 10*_p.gain_heading_error * epsi*epsi;

            // Minimize the use of actuators
            for (int t = 0; t < _p.timestep_N - 1; t++)
            {
                const auto delta = vars[_idx.kappa_start + t];

                fg[0] += (_p.gain_kappa_effort * delta * delta);
            }
            
            // Minimize control derivative (more smooth)
            for (int t = 0; t < _p.timestep_N - 2; t++)
            {
                const auto ddelta = vars[_idx.kappa_start + t + 1] - vars[_idx.kappa_start + t];

                fg[0] += (_p.gain_deriv_kappa * ddelta*ddelta);
            }

            // -----------------------  Model Constraints
            // initial constraints
            fg[1 + _idx.x_start] = vars[_idx.x_start];
            fg[1 + _idx.y_start] = vars[_idx.y_start];
            fg[1 + _idx.psi_start] = vars[_idx.psi_start];
            fg[1 + _idx.cte_start] = vars[_idx.cte_start];
            fg[1 + _idx.epsi_start] = vars[_idx.epsi_start];

            // rest of the constraints
            for (int t = 1; t < _p.timestep_N; t++) 
            {
                // The state at time t+1
                AD<double> x1 = vars[_idx.x_start + t];
                AD<double> y1 = vars[_idx.y_start + t];
                AD<double> psi1 = vars[_idx.psi_start + t];
                AD<double> cte1 = vars[_idx.cte_start + t];
                AD<double> epsi1 = vars[_idx.epsi_start + t];

                // The state at time t
                AD<double> x0 = vars[_idx.x_start + t - 1];
                AD<double> y0 = vars[_idx.y_start + t - 1];
                AD<double> psi0 = vars[_idx.psi_start + t - 1];
                AD<double> cte0 = vars[_idx.cte_start + t - 1];
                AD<double> epsi0 = vars[_idx.epsi_start + t - 1];

                // Only consider the actuation at time t.
                AD<double> kappa0 = vars[_idx.kappa_start + t - 1];

                //AD<double> f0 = _coeffs[0] + _coeffs[1] * x0 + _coeffs[2]*x0*x0 + _coeffs[3]*x0*x0*x0;
                //AD<double> f1 = coeffs[0] + coeffs[1] * x1 + coeffs[2]*x1*x1 + coeffs[3]*x1*x1*x1;
                //cout << "F0: " << f0 << ", F1: " << f1 << endl;
                //AD<double> f0prime = _coeffs[1] + 2*_coeffs[2]*x0 + 3*_coeffs[3]*x0*x0;
                //AD<double> psi_desired = CppAD::atan(f0prime);

                double psi_wp = atan2(_ptsy(t)-_ptsy(t-1), _ptsx(t)-_ptsx(t-1));
                // Cross-track error calculation
                AD<double> Ru = CppAD::sqrt(CppAD::pow(_ptsx(t-1)-x0, 2) + CppAD::pow(_ptsy(t-1)-y0, 2));
                AD<double> psi_U = CppAD::atan2(y0-_ptsy(t-1), x0-_ptsx(t-1));
                AD<double> cte_d = Ru*CppAD::sin(psi_wp - psi_U);
                //AD<double> cte_d = Ru;

                /*
                AD<double> R = CppAD::sqrt(Ru*Ru - CppAD::pow(Ru*CppAD::sin(psi_wp - psi_U), 2));
                AD<double> xt = (R + _p.sigma)*cos(psi_wp) + _ptsx(t-1);
                AD<double> yt = (R + _p.sigma)*sin(psi_wp) + _ptsy(t-1);
                AD<double> psi_desired = CppAD::atan2(yt-y0, xt-x0);
                */

                //AD<double> psi_desired = CppAD::atan2(_ptsy(t)-y0, _ptsx(t)-x0);
                AD<double> psi_desired = CppAD::atan2(_ptsy(t)-_ptsy(t-1), _ptsx(t)-_ptsx(t-1));

                // equations for the model:
                /*
                AD<double> px1_f = x0 + _speed * CppAD::cos(psi0) * _p.timestep_dt;
                AD<double> py1_f = y0 + _speed * CppAD::sin(psi0) * _p.timestep_dt;
                AD<double> psi1_f = psi0 + _speed * kappa0 * _p.timestep_dt;
                AD<double> cte1_f = f0 - y0 + _speed * CppAD::sin(epsi0) * _p.timestep_dt;
                AD<double> epsi1_f = psi_desired - (psi0 + _speed * kappa0 * _p.timestep_dt);
                */
                
                AD<double> px1_f = x0 + CppAD::cos(psi0 + kappa0/2 * _p.ds) * _p.ds;
                AD<double> py1_f = y0 + CppAD::sin(psi0 + kappa0/2 * _p.ds) * _p.ds;
                AD<double> psi1_f = psi0 + kappa0 * _p.ds;
                AD<double> cte1_f = cte_d + CppAD::sin(epsi0 - kappa0/2 * _p.ds) * _p.ds;
                AD<double> epsi1_f = psi_desired - (psi0 + kappa0 * _p.ds);

                /*
                fg[1 + _idx.x_start + t] = x1 - px1_f;
                fg[1 + _idx.y_start + t] = y1 - py1_f;
                fg[1 + _idx.psi_start + t] = psi1 - psi1_f;
                fg[1 + _idx.cte_start + t] = cte1 - cte1_f;
                fg[1 + _idx.epsi_start + t] = epsi1 - epsi1_f;
                */

                AD<double> pi(M_PI);
                AD<double> fg_psi = psi1 - psi1_f;
                AD<double> fg_epsi = epsi1 - epsi1_f;

                fg[1 + _idx.x_start + t] = x1 - px1_f;
                fg[1 + _idx.y_start + t] = y1 - py1_f;
                fg[1 + _idx.psi_start + t] = CppAD::CondExpGt(CppAD::abs(fg_psi), pi, fg_psi-CppAD::sign(fg_psi)*2*M_PI, fg_psi);
                fg[1 + _idx.cte_start + t] = cte1 - cte1_f;
                fg[1 + _idx.epsi_start + t] = CppAD::CondExpGt(CppAD::abs(fg_epsi), pi, fg_epsi-CppAD::sign(fg_epsi)*2*M_PI, fg_epsi);
            }
        }
    private:
        MpcParams   _p;
        Indexes     _idx;
        //AD<double> _speed;
        //int _direction;
        // Fitted polynomial coefficients
        //Eigen::VectorXd _coeffs;
        // Waypoints
        Eigen::VectorXd _ptsx;
        Eigen::VectorXd _ptsy;

};

//
// MPC class definition implementation.
//
MPC::MPC(){}
MPC::MPC(MpcParams params)
{
    _p = params;

    _idx.x_start = 0;                                   // Has N values
    _idx.y_start = _idx.x_start + _p.timestep_N;        // Has N values
    _idx.psi_start = _idx.y_start + _p.timestep_N;      // Has N values
    _idx.cte_start = _idx.psi_start + _p.timestep_N;    // Has N values
    _idx.epsi_start = _idx.cte_start + _p.timestep_N;   // Has N values
    _idx.kappa_start = _idx.epsi_start + _p.timestep_N; // Has N-1 values

    size_t N = _p.timestep_N;
    const int NUMBER_OF_STATES = 5;     // px, py, psi, cte, epsi
    const int NUMBER_OF_ACTUATIONS = 1; // steering curvature
    size_t n_vars = NUMBER_OF_STATES*N + NUMBER_OF_ACTUATIONS*(N-1);

    _last_solution.resize(n_vars);
    for (int i = 0; i < n_vars; i++)
        _last_solution[i] = 0;
}

MPC::~MPC() {}

double MPC::Solve(const Eigen::VectorXd& state, const Eigen::VectorXd& mpc_ptsx, const Eigen::VectorXd& mpc_ptsy) 
{
    size_t N = _p.timestep_N;
    bool ok = true;
    // size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    // If the state is a 4 element vector, the actuators is a 2
    // element vector and there are 10 timesteps. The number of variables is:
    // 4 * 10 + 2 * 9
    const int NUMBER_OF_STATES = 5;     // px, py, psi, cte, epsi
    const int NUMBER_OF_ACTUATIONS = 1; // steering curvature
    size_t n_vars = NUMBER_OF_STATES*N + NUMBER_OF_ACTUATIONS*(N-1);
    
    // Set the number of constraints
    size_t n_constraints = NUMBER_OF_STATES*N;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);

    if(_p.use_warm_start && _last_solution.size() == n_vars)
        vars = _last_solution;
    else
    {
        for (int i = 0; i < n_vars; i++)
            vars[i] = 0;
    }
    

    // set initial state
    vars[_idx.x_start] = state[0];
    vars[_idx.y_start] = state[1];
    vars[_idx.psi_start] = state[2];
    vars[_idx.cte_start] = state[4];
    vars[_idx.epsi_start] = state[5];

    //AD<double> speed = state[3];

    // Set lower and upper limits for variables.
    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // Set all non-actuators upper and lowerlimits
    // to the max negative and positive values.
    for (int i = 0; i < _idx.kappa_start; i++) 
    {
        vars_lowerbound[i] = -_p.bounds_vars_limit;
        vars_upperbound[i] = _p.bounds_vars_limit;
    }

    // The upper and lower limits of kappa (curvature) are set to -1/Rmin and 1/Rmin
    for (int i = _idx.kappa_start; i < n_vars; i++)
    {
        vars_lowerbound[i] = -1.0/_p.Rmin;
        vars_upperbound[i] = 1.0/_p.Rmin;
    }

    // Lower and upper limits for the constraints
    // Should be 0 besides initial state.
    Dvector constraints_lowerbound(n_constraints);
    Dvector constraints_upperbound(n_constraints);
    for (int i = 0; i < n_constraints; i++)
    {
        constraints_lowerbound[i] = 0;
        constraints_upperbound[i] = 0;
    }
    constraints_lowerbound[_idx.x_start] = state[0];
    constraints_lowerbound[_idx.y_start] = state[1];
    constraints_lowerbound[_idx.psi_start] = state[2];
    constraints_lowerbound[_idx.cte_start] = state[4];
    constraints_lowerbound[_idx.epsi_start] = state[5];

    constraints_upperbound[_idx.x_start] = state[0];
    constraints_upperbound[_idx.y_start] = state[1];
    constraints_upperbound[_idx.psi_start] = state[2];
    constraints_upperbound[_idx.cte_start] = state[4];
    constraints_upperbound[_idx.epsi_start] = state[5];

    // object that computes objective and constraints
    FG_eval fg_eval(_p, _idx, mpc_ptsx, mpc_ptsy);

    // options for IPOPT solver
    std::string options;
    options += "Integer print_level  0\n";
    //options += "String linear_solver ma27\n";
    options += "String linear_solver " + _p.solver + "\n";
    options += "String print_timing_statistics yes\n";
    options += "Sparse  true    forward\n";
    options += "Sparse  true    reverse\n";
    options += "Numeric max_cpu_time " + to_string(_p.options_max_cpu_time_seconds) + "\n";
    //options += "Numeric max_cpu_time          0.5\n";
    //options += "Numeric tol                 0.01\n";
    //options += "Numeric acceptable_tol      0.05\n";
    /*
    options += "String warm_start_init_point       yes\n";
    options += "Numeric warm_start_bound_push       1e-9\n";
    options += "Numeric warm_start_bound_frac       1e-9\n";
    options += "Numeric warm_start_slack_bound_frac 1e-9\n";
    options += "Numeric warm_start_slack_bound_push 1e-9\n";
    options += "Numeric warm_start_mult_bound_push  1e-9\n";
    */
    std::cout << "\noptions " << options << std::endl;

    // place to return solution
    CppAD::ipopt::solve_result<Dvector> solution;

    // solve the problem
    CppAD::ipopt::solve<Dvector, FG_eval>(
        options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
        constraints_upperbound, fg_eval, solution);

    // Check some of the solution values
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

    // Cost
    auto cost = solution.obj_value;
    std::cout << "Cost " << cost << std::endl;
    std::cout << "OK? " << ok << std::endl;

    // set x_pred_vals and y_pred_vals for plotting
    x_pred_vals.clear();
    y_pred_vals.clear();
    kappa_pred_vals.clear();
    for (int i = 1; i<N; ++i)
    {
        x_pred_vals.push_back(solution.x[_idx.x_start+i]);
        y_pred_vals.push_back(solution.x[_idx.y_start+i]);
        kappa_pred_vals.push_back(solution.x[_idx.kappa_start+i-1]);
    }

    _last_solution = solution.x;
    return solution.x[_idx.kappa_start];
}