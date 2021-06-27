#include <math.h>
#include "quaternion.h"

Quaternion::Quaternion(Scalar a, Scalar b, Scalar c, Scalar d) : x(a), y(b), z(c), w(0) {}

Quaternion::Quaternion() : x(0), y(0), z(0), w(1) {}

Quaternion::Quaternion(const Point3& p) : x(p.x), y(p.y), z(p.z), w(0) {}

Quaternion Quaternion::operator*(Quaternion r) const {
    return Quaternion();
}

Vector3 Quaternion::apply(Vector3  v) const {
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

Versor3 Quaternion::apply(Versor3 dir) const {
    return apply(dir.asVector()).asVersor();
}

Point3 Quaternion::apply(Point3 p) const {
    return apply(p.asVector()).asPoint();
}

Quaternion Quaternion::inverse(Quaternion r) const 
{
    Scalar n = norm(r);
    return conjugated(r) / (n * n);
}

void Quaternion::invert() 
{
    conjugate();
    Scalar n = norm(*this);
    x = x / (n * n);
    y = y / (n * n);
    z = z / (n * n);
    w = w / (n * n);
}

Quaternion Quaternion::conjugated() const 
{
    Quaternion r(-x, -y, -z, w);
    return r;
}

Quaternion Quaternion::conjugated(Quaternion q) const 
{
    q.x = -q.x;
    q.y = -q.y;
    q.z = -q.z;

    return q;
}

void Quaternion::conjugate() 
{
    x = -x;
    y = -y;
    z = -z;
}

Scalar Quaternion::dot(const Quaternion& l, const Quaternion& r)
{
    return l.x * r.x + l.y * r.y + l.z * r.z + l.w * r.w;
}

Scalar Quaternion::angle(const Quaternion& a, const Quaternion& b)
{
    Scalar d = dot(a, b);
    return acos(fmin(fabs(d), 1)) * 2;
}