#ifndef QUATERNION_H
#define QUATERNION_H
#include <tuple>
#include <math.h>
#include <armadillo>

struct Quaternion
{
    double a, b, c, d;
public:
    Quaternion(double a = 1.0, double b = 0.0, double c = 0.0, double d = 0.0);
    Quaternion(const arma::vec& f);
    Quaternion(double angle, const arma::vec& axis);
    Quaternion operator*(const Quaternion& other) const;
    Quaternion conjugate() const;
    Quaternion inverse() const;
    arma::vec rotate(const arma::vec & vec) const;
    double length() const;
    friend class Vector3d;
};

#endif // QUATERNION_H
