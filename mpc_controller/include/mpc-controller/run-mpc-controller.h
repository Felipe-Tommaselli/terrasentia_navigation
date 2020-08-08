#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>
#include "mpc-model2.h"
#include "paramReader.h"

struct MpcOutput{
	double  kappa;
	double 	vx;
	double 	wz;
	double 	cte;
	double 	heading_diff;
	double	solver_time;
	std::vector<double> x_pred_vals;
	std::vector<double> y_pred_vals;
	std::vector<double> kappa_pred_vals;
};

struct MpcInput{
	double dt;
	double x;
	double y;
	double theta;
	double vx;
	double wz;
	std::vector<double> wp_x;
	std::vector<double>	wp_y;
};

class RunMpcController
{

	public:
		RunMpcController(string path="mpc.config");
		~RunMpcController();

		MpcOutput mpcController(MpcInput);
		MpcParams getParams();
	private:
        MpcParams loadParams(std::string dir);
		void printParams(MpcParams params);
		MPC* _mpc;
		MpcParams _p;
};