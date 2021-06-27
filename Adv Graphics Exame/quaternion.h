#pragma once

#include <iostream>
#include <math.h>
#include "vector3.h"
#include "point3.h"
#include "versor3.h"
#include "axis_angle.h"
#include "euler.h"
#include "matrix3.h"

using Scalar = double;

/* Quaternion class */
/* this class is a candidate to store a rotation! */
/* as such, it implements all expected methods    */

class Matrix3;
class AxisAngle;
class Euler;

class Quaternion {
public:

    /* fields */
    // TODO Q-Fields: which fields to store? (also add a constuctor taking these fields).
    Scalar x;
    Scalar y; 
    Scalar z;
    Scalar w;

    Quaternion(Scalar a, Scalar b, Scalar c, Scalar d); // DONE Q-Constr

    // DONE Q-Ide: this constructor construct the identity rotation
    Quaternion();

    // DONE Q-FromPoint
    // returns a quaternion encoding a point
    Quaternion(const Point3& p);

    // DONE Q-App: how to apply a rotation of this type?
    Vector3 apply(Vector3  v) const;

    // Rotations can be applied to versors or vectors just as well
    Versor3 apply(Versor3 dir) const;

    Point3 apply(Point3 p) const;

    // syntactic sugar: "R( p )" as a synomim of "R.apply( p )"
    Versor3 operator() (Versor3 p) { return apply(p); }
    Point3  operator() (Point3  p) { return apply(p); }
    Vector3 operator() (Vector3 p) { return apply(p); }

    Versor3 axisX() const;  // TODO Q-Ax a
    Versor3 axisY() const;  // TODO Q-Ax b
    Versor3 axisZ() const;  // TODO Q-Ax c

    Quaternion operator * (Quaternion r) const;

    // DONE Q-Inv a
    Quaternion inverse(Quaternion r) const;

    // TODO Q-Inv b
    void invert();

    // specific methods for quaternions...
    // DONE Q-Conj a
    Quaternion conjugated() const;

    // DONE Q-Conj a
    Quaternion conjugated(Quaternion q) const;

    // TODO Q-Conj b
    void conjugate();

    Scalar dot(const Quaternion& l, const Quaternion& r);

    Scalar angle(const Quaternion& a, const Quaternion& b);

    // returns a rotation to look toward target, if you are in eye, and the up-vector is up
    static Quaternion lookAt( Point3 eye, Point3 target, Versor3 up = Versor3::up() ){
        // TODO Q-LookAt
        return Quaternion();
    }

    // returns a rotation
    static Quaternion toFrom( Versor3 to, Versor3 from ){
        // TODO Q-ToFrom
    }

    static Quaternion toFrom( Vector3 to, Vector3 from ){
        return toFrom( normalize(to) , normalize(from) );
    }

    // conversions to this representation
    static Quaternion from( Matrix3 m );   // TODO M2Q
    static Quaternion from( Euler e );     // DONE E2Q
    static Quaternion from( AxisAngle e ); // DONE A2Q

    // does this quaternion encode a rotation?
    bool isRot() const{
        // DONE Q-isR
        bool isARot = false;
        Scalar n = squaredNormQ(); // ??
        isARot = (n - 1 <= std::numeric_limits<Scalar>::epsilon() * std::numeric_limits<Scalar>::epsilon());
        return isARot;
    }


    // does this quaternion encode a poont?
    bool isPoint() const{
        // DONE Q-isP
        bool isAPoint = false;
        isAPoint = w <= std::numeric_limits<Scalar>::epsilon();
        return isAPoint;
    }

    void printf() const 
    {
        std::cout << x << y << z << w << std::endl;
    } // DONE Print
};

// from functions
inline Quaternion from(Matrix3 m)
{

}

inline Quaternion from(Euler e)
{
    Scalar cosx = cos(e.pitch * 0.5);
    Scalar sinx = sin(e.pitch * 0.5);
    Scalar cosy = cos(e.yaw * 0.5);
    Scalar siny = sin(e.yaw * 0.5);
    Scalar cosz = cos(e.roll * 0.5);
    Scalar sinz = sin(e.roll * 0.5);
    Quaternion q;
    q.x = cosx * siny * sinz + cosy * cosz * sinx;
    q.y = cosx * cosz * siny - cosy * sinx * sinz;
    q.z = cosx * cosy * sinz - cosz * sinx * siny;
    q.w = sinx * siny * sinz + cosx * cosy * cosz;
    return q;
}

inline Quaternion from(AxisAngle e)
{
    Quaternion q;
    Scalar m = sqrt(e.axis.x * e.axis.x + e.axis.y * e.axis.y + e.axis.z * e.axis.z);
    Scalar s = sin(e.angle / 2) / m;
    q.x = e.axis.x * s;
    q.y = e.axis.y * s;
    q.z = e.axis.z * s;
    q.w = cos(e.angle / 2);
    return q;
}

// interpolation or roations
inline Quaternion lerp( const Quaternion& a,const Quaternion& b, Scalar t){
    // DONE Q-Lerp: how to interpolate quaternions
    // hints: shortest path! Also, consdider them are 4D unit vectors.
    if (t < 0) return normalized(a);
    else if (t > 1) return normalized(b);
    return lerpUnclamped(a, b, t);
}

inline Quaternion lerpUnclamped(const Quaternion& a, const Quaternion& b, Scalar t)
{
    Quaternion q;
    if (dot(a, b) >= 0)
        q = a * (1 - t) + b * t;
    else
        q = a * (1 - t) - b * t;
    return normalize(q);
}

inline Scalar norm(Quaternion r)
{
    return sqrt(r.x * r.x +
        r.y * r.y +
        r.z * r.z +
        r.w * r.w);
}

inline Quaternion normalized(Quaternion r)
{
    return r / norm(r);
}