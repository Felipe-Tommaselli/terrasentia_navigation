#pragma once

#include <Eigen/Dense>

using namespace Eigen;

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
static Eigen::VectorXd polyfit(const Eigen::VectorXd& xvals, const Eigen::VectorXd& yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    auto Q = A.householderQr();
    auto result = Q.solve(yvals);
    return result;
}

// Evaluate a polynomial.
static double polyeval(const Eigen::VectorXd& coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

static void interpolatePath(const VectorXd& xvals, const VectorXd& yvals, Ref<VectorXd> xouts, Ref<VectorXd> youts, float ds)
{
    assert(xvals.size() == yvals.size());
    assert(xouts.size() == youts.size());
    assert(xvals.size() > 1);
    assert(xouts.size() > 1);

    int N = xouts.size();
    xouts(0) = xvals(0);
    youts(0) = yvals(0);

    int j = 1;
    std::cout << "\ninterpolatePath ds " << ds << endl;
    for(int i = 1; i < N; i++)
    {
        double dist = sqrt((xvals(j)-xvals(j-1))*(xvals(j)-xvals(j-1)) + (yvals(j)-yvals(j-1))*(yvals(j)-yvals(j-1)));
        if(dist < 0.01 && j < xvals.size()-1 )
            j++;
        else if(dist < 0.01 && j == xvals.size()-1 )
            j--;

        if(dist > 0.01)
        {
            xouts(i) = xvals(j-1) + ds/dist*(xvals(j)-xvals(j-1));
            youts(i) = yvals(j-1) + ds/dist*(yvals(j)-yvals(j-1));

            if( j < xvals.size()-1)
            {
                j++;
            }
        }
        else
            i--;
    }
}

static double constrainAngle(double x)
{
    x = fmod(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}

/*
static void interpolatePath(const VectorXd& xvals, const VectorXd& yvals, Ref<VectorXd> xouts, Ref<VectorXd> youts, float ds)
{
    assert(xvals.size() == yvals.size());
    assert(xouts.size() == youts.size());
    assert(xvals.size() > 1);
    assert(xouts.size() > 1);

    vector<double> new_xvals;
    vector<double> new_yvals;

    new_xvals.clear();
    new_yvals.clear();

    new_xvals.push_back(xvals(0));
    new_yvals.push_back(yvals(0));

    //cout << "new_xvals: " << new_xvals.back() << endl;
    //cout << "new_yvals: " << new_yvals.back() << endl;

    int N = xouts.size();

    for(int i=1; i<xvals.size(); i++)
    {
        if(xvals(i) != xvals(i-1) && yvals(i) != yvals(i-1))
        {
            new_xvals.push_back(xvals(i));
            new_yvals.push_back(yvals(i));
            //cout << "new_xvals: " << new_xvals.back() << endl;
            //cout << "new_yvals: " << new_yvals.back() << endl;
        }

        if(new_xvals.size() == N)
            break;
    }

    Map<VectorXd> v1(new_xvals.data(), N);
    Map<VectorXd> v2(new_yvals.data(), N);

    xouts = v1;
    youts = v2;
}
*/