#include "run-mpc-controller.h"
//#include "algorithms/ekf/utils.h"
#include "utils.h"

#include <chrono> 

using namespace std;

// Constructor for the main class
RunMpcController::RunMpcController(string path)
{
	cout << "RunMpc object is being created with config file" << endl;
	MpcParams mpc_params;
    try
    {
		mpc_params = RunMpcController::loadParams(path);
    }
    catch (const ifstream::failure& e)
    {
        cout << "Exception opening/reading parameter file" << endl;
    }
	printParams(mpc_params);
	_p = mpc_params;
	_mpc = new MPC(mpc_params);
}
void RunMpcController::printParams(MpcParams params)
{
	std::cout << "gain_ctrack_error " << params.gain_ctrack_error << std::endl;
	std::cout << "gain_heading_error " << params.gain_heading_error << std::endl;
	//std::cout << "gain_velocity_error " << params.gain_velocity_error << std::endl;
	std::cout << "gain_kappa_effort " << params.gain_kappa_effort << std::endl;
	//std::cout << "gain_accel_effort " << params.gain_accel_effor << std::endl;
	std::cout << "gain_deriv_kappa " << params.gain_deriv_kappa << std::endl;
	//std::cout << "gain_deriv_accel " << params.gain_deriv_accel << std::endl;
	std::cout << "ref_cte " << params.ref_cte << std::endl;
	std::cout << "ref_epsi " << params.ref_epsi << std::endl;
	//std::cout << "ref_v " << params.ref_v << std::endl;
	std::cout << "Rmin " << params.Rmin << std::endl;
	std::cout << "sigma " << params.sigma << std::endl;
	//std::cout << "timestep_dt " << params.timestep_dt << std::endl;
	std::cout << "timestep_N " << params.timestep_N << std::endl;
	std::cout << "ds " << params.ds << std::endl;
	std::cout << "bounds_vars_limit " << params.bounds_vars_limit << std::endl;
	//std::cout << "bounds_accel " << params.bounds_accel << std::endl;
	std::cout << "options_max_cpu_time_seconds " << params.options_max_cpu_time_seconds << std::endl;
}

MpcParams RunMpcController::getParams() { return _p; }

MpcParams RunMpcController::loadParams(string dir)
{
	std::cout << "getParams from " << dir << std::endl;
    ParameterReader pd(dir);
    MpcParams params;

    params.gain_ctrack_error = atof(pd.getData( "gain_ctrack_error" ).c_str());
    params.gain_heading_error = atof(pd.getData( "gain_heading_error" ).c_str());
    //params.gain_velocity_error = atof(pd.getData( "gain_velocity_error" ).c_str());
    params.gain_kappa_effort = atof(pd.getData( "gain_kappa_effort" ).c_str());
    //params.gain_accel_effort = atof(pd.getData( "gain_accel_effort" ).c_str());
    params.gain_deriv_kappa = atof(pd.getData( "gain_deriv_kappa" ).c_str());
    //params.gain_deriv_accel = atof(pd.getData( "gain_deriv_accel" ).c_str());
    params.ref_cte = atof(pd.getData( "ref_cte" ).c_str());
    params.ref_epsi = atof(pd.getData( "ref_epsi" ).c_str());
    //params.ref_v = atof(pd.getData( "ref_v" ).c_str());
    params.Rmin = atof(pd.getData( "Rmin" ).c_str());
	params.sigma = atof(pd.getData( "sigma" ).c_str());
	params.latency = atof(pd.getData( "latency" ).c_str());
    //params.timestep_dt = atof(pd.getData( "timestep_dt" ).c_str());
	params.ds = atof(pd.getData( "ds" ).c_str());
    params.timestep_N = atoi(pd.getData( "timestep_N" ).c_str());
    params.bounds_vars_limit = atof(pd.getData( "bounds_vars_limit" ).c_str());
    //params.bounds_accel = atof(pd.getData( "bounds_accel" ).c_str());
    params.options_max_cpu_time_seconds = atof(pd.getData( "options_max_cpu_time_seconds" ).c_str());
	params.solver = pd.getData("solver");
	string use_warm_start = pd.getData("use_warm_start");
	params.use_warm_start = use_warm_start == "1" || use_warm_start == "true";

    return params;
}
RunMpcController::~RunMpcController(){}

MpcOutput RunMpcController::mpcController(MpcInput input)
{	
	double px = input.x;
	double py = input.y;
	double currentYaw = constrainAngle(input.theta);
	double dt = input.dt + _p.latency;
	vector<double> ptsx = input.wp_x;
	vector<double> ptsy = input.wp_y;

	/*
	Eigen::VectorXd ptsx_car;
	Eigen::VectorXd ptsy_car;
	ptsx_car.resize(ptsx.size());
	ptsy_car.resize(ptsy.size());

	// Given the desired path was provided in map coordinates, first we need to convert the path into coordinates relative to the car.
	// Transform the points to the vehicle's orientation
	for (int i = 0; i < ptsx.size(); i++)
	{
		double x = ptsx[i] - px;
		double y = ptsy[i] - py;
		ptsx_car(i) = x * cos(-currentYaw) - y * sin(-currentYaw);
		ptsy_car(i) = x * sin(-currentYaw) + y * cos(-currentYaw);
	}
	*/

	Eigen::Map<Eigen::VectorXd>  ptsx_car(ptsx.data(), ptsx.size());
	Eigen::Map<Eigen::VectorXd>  ptsy_car(ptsy.data(), ptsy.size());

	cout << "ptsx_car:\n" << ptsx_car << endl;
	cout << "ptsy_car:\n" << ptsy_car << endl;

	/*
	 * Error Calculation
	 */
	Eigen::VectorXd mpc_ptsx(_p.timestep_N);
	Eigen::VectorXd mpc_ptsy(_p.timestep_N);

	// The input waypoints are interpolated to have the same distance between points as the distance step used in the MPC
	interpolatePath(ptsx_car, ptsy_car, mpc_ptsx, mpc_ptsy, _p.ds);
	
	cout << "mpc_ptsx:\n" << mpc_ptsx << endl;
	cout << "mpc_ptsy:\n" << mpc_ptsy << endl;

	double theta_wp = atan2(mpc_ptsy(1)-mpc_ptsy(0), mpc_ptsx(1)-mpc_ptsy(0));
	double Ru = sqrt(mpc_ptsx(0)*mpc_ptsx(0) + mpc_ptsy(0)*mpc_ptsy(0));
	double thetaU = atan2(-mpc_ptsy(0), -mpc_ptsx(0));
	double beta = theta_wp - thetaU;
	double cte = Ru*sin(beta);
	/*
	double R = sqrt(Ru*Ru - pow(Ru*sin(beta), 2));
	double xt = (R + _p.sigma)*cos(theta_wp) + mpc_ptsx(0);
	double yt = (R + _p.sigma)*sin(theta_wp) + mpc_ptsy(0);
	double epsi = atan2(yt, xt);
	*/
	double epsi = atan2(mpc_ptsy(1), mpc_ptsx(1));

	cout << "--> MPC heading error: " << epsi << endl;

	// Prediction step: vehicle's future position, orientation, speed and error are calculated for the following timestep.
	double a = 0;

	// Predict state after latency
	double pred_px = 0.0 + input.vx * cos(input.wz/2*dt) * dt;		// Since psi is zero, cos(0) = 1, can leave out
	const double pred_py = 0.0 * sin(input.wz/2*dt);				// Since sin(0) = 0, y stays as 0 (y + v * 0 * dt)
	double pred_psi = 0.0 + input.wz * dt;
	double pred_v = input.vx + a * dt;
	double pred_cte = cte + input.vx * sin(epsi - input.wz/2*dt) * dt;
	double pred_epsi = epsi - input.wz * dt;

	Eigen::VectorXd state(6);
	state << pred_px, pred_py, pred_psi, pred_v, pred_cte, pred_epsi;

	/*
	 * Steering and Throttle Calculation
	 */
	// This step invokes the solver used to optimise the cost function outlined in mpc-controller.cpp.
	// The values returned from the solver are sent to the simulator for actuation.
	auto start = chrono::high_resolution_clock::now();
	// Solve for new actuations (and to show predicted x and y in the future)
	double kappa = _mpc->Solve(state, mpc_ptsx, mpc_ptsy);

	auto end = chrono::high_resolution_clock::now(); 
	double time_taken = chrono::duration_cast<chrono::nanoseconds>(end - start).count()*1e-9;
	cout << "---> Time taken to solve IPT: " << time_taken << endl;

	cout << "_mpc predvals size: " << _mpc->x_pred_vals.size() << "/" << _mpc->y_pred_vals.size() << endl;
	cout << "_mpc kappa predvals size: " << _mpc->kappa_pred_vals.size() << endl;

	MpcOutput output;
	output.kappa = kappa;
	output.vx = pred_v;
	output.wz = pred_v*kappa;
	output.x_pred_vals = _mpc->x_pred_vals;
	output.y_pred_vals = _mpc->y_pred_vals;
	output.kappa_pred_vals = _mpc->kappa_pred_vals;

	output.cte = cte;
	output.heading_diff = epsi;
	output.solver_time = time_taken;

	return output;
}