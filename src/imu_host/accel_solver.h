#pragma once
#include <armadillo>
#include <iostream>

using namespace std;

class AccelSolver
{
    const double delta = 0.0001;
    const double gamma = 0.005;

    vector<arma::Col<double>> meas;

    double term(double x, double m, double d) {
        return ((m - x) / d) * ((m - x) / d);
    }
    double r(arma::Col<double> point) {
        arma::Col<double> ret(meas.size());
        for(size_t i = 0; i < meas.size(); i++) {
            ret[i] =  term(meas[i][0], point[0], point[3])
                    + term(meas[i][1], point[1], point[4])
                    + term(meas[i][2], point[2], point[5])
                    - 1;
        }
        return arma::dot(ret, ret);
    }

    arma::Col<double> gradient(arma::Col<double> point){
        arma::Col<double> ret(6), dPoint(6);
        double val = r(point);
        for(size_t i = 0; i < 6; i++) {
            (dPoint = point)[i] += delta;
            ret[i] = (r(dPoint) - val) / delta;
        }
        return ret;
    }
    arma::Col<double> step(arma::Col<double> b_k) {
        return b_k - 0.005 * gradient(b_k);
    }
public:
    AccelSolver(vector<arma::Col<double>> meas) : meas(meas) {}

    arma::Col<double> solve(double* error){
        arma::Col<double> b;
        b << 0 << 0 << 0 << 1 << 1 << 1;
        for(int i = 0; i < 5000; i++) b = step(b);
        if(error) *error = r(b);
        return b;
    }
};
