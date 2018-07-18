#include "quaternion.h"

Quaternion::Quaternion(double a, double b, double c, double d) : a(a), b(b), c(c), d(d)
{

}

Quaternion::Quaternion(const arma::vec& f) : a(0.0), b(f[0]), c(f[1]), d(f[2]){

}

Quaternion::Quaternion(double angle, const arma::vec& axis) {
    arma::vec norm = arma::normalise(axis);
    a = cos(angle/2.0);
    b = c = d = sin(angle/2.0);
    b *= norm[0];
    c *= norm[1];
    d *= norm[2];
}

Quaternion Quaternion::conjugate() const {
    return Quaternion(a, -b, -c, -d);
}

Quaternion Quaternion::inverse() const {
    return conjugate() * Quaternion(1.0 / length(), 0, 0, 0);
}

double Quaternion::length() const {
    return sqrt(a*a + b*b + c*c + d*d);
}

arma::vec Quaternion::rotate(const arma::vec &vec) const {
    auto q = ((*this) * Quaternion(vec) * conjugate());
    return arma::vec {q.b, q.c, q.d};
}

Quaternion Quaternion::operator *(const Quaternion& other) const {
    auto [e,f,g,h] = std::tie(other.a, other.b, other.c, other.d);
    return Quaternion(
        a*e - b*f - c*g - d*h,
        a*f + b*e + c*h - d*g,
        a*g - b*h + c*e + d*f,
        a*h + b*g - c*f + d*e
    );
}
