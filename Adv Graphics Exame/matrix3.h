#pragma once

#include <iostream>
#include <math.h>
#include "vector3.h"
#include "point3.h"
#include "versor3.h"
#include "euler.h"
#include "axis_angle.h"
#include "quaternion.h"

using Scalar = double;
/* Matrix3 class */
/* this class is a candidate to store a rotation! */
/* as such, it implements all the expected methods    */

class Quaternion;
class AxisAngle;
class Euler;

class Matrix3{
public:

    /* fields */
    // DONE M-Fields: which fields to store? (also add a constuctor taking these fields).
    Vector3 x, y, z; // columns


    // DONE M-Ide: this constructor construct the identity rotation
    Matrix3(){}

    // constructor that takes as input the coefficient (RAW-MAJOR order!)
    Matrix3(Scalar m00, Scalar m01, Scalar m02,
            Scalar m10, Scalar m11, Scalar m12,
            Scalar m20, Scalar m21, Scalar m22){
        // DONE M-Constr
    }

    // DONE M-App: how to apply a rotation of this type?
    Vector3 apply(Vector3  v) const;

    // Rotations can be applied to versors or vectors just as well
    Versor3 apply( Versor3 dir ) const {
        return apply( dir.asVector() ).asVersor();
    }

    Point3 apply( Point3 p ) const {
        return apply( p.asVector() ).asPoint();
    }

    // syntactic sugar: "R( p )" as a synomim of "R.apply( p )"
    Versor3 operator() (Versor3 p) { return apply(p); }
    Point3  operator() (Point3  p) { return apply(p); }
    Vector3 operator() (Vector3 p) { return apply(p); }

    Versor3 axisX() const;  // DONE M-Ax a
    Versor3 axisY() const;  // DONE M-Ax b
    Versor3 axisZ() const;  // DONE M-Ax c

    // combine two rotations (r goes first!)
    Matrix3 operator*(Matrix3 r) const;
    Matrix3 operator*(Scalar b) const;
    Matrix3 operator-(Matrix3 r) const;
    Matrix3 operator+(Matrix3 r) const;

    // DONE M-Inv a
    Matrix3 inverse(Matrix3 m) const;

    // DONE M-Inv b
    void invert() const;

    // returns a rotation to look toward target, if you are in eye, and the up-vector is up
    // DONE M-lookAt
    static Matrix3 lookAt(Point3 eye, Point3 target, Versor3 up = Versor3::up());

    // returns a rotation
    // DONE M-ToFrom
    static Matrix3 toFrom(Versor3 to, Versor3 from);

    static Matrix3 toFrom(Vector3 to, Vector3 from);

    // conversions to this representation
    static Matrix3 from( const Quaternion& m );// DONE Q2M
    static Matrix3 from( const Euler& e );     // DONE E2M
    static Matrix3 from( const AxisAngle& e ); // DONE A2M

    // does this Matrix3 encode a rotation?
    // TODO M-isR
    bool isRot() const;

    // return a rotation matrix around an axis
    static Matrix3 rotationX( Scalar angleDeg );   // DONE M-Rx
    static Matrix3 rotationY( Scalar angleDeg );   // DONE M-Ry
    static Matrix3 rotationZ( Scalar angleDeg );   // DONE M-Rz

    void printf() const; // DONE Print

    Scalar determinate() const;
    Scalar determinate(const Matrix3& m) const;
};

// get Rotation Matrix
inline Matrix3 GetRXMatrix(const Scalar& x)
{
    return Matrix3(1, 0, 0,
        0, cos(x), -sin(x),
        0, sin(x), cos(x));
}

inline Matrix3 GetRYMatrix(const Scalar& y)
{
    return Matrix3(cos(y), 0, sin(y),
        0, 1, 0,
        -sin(y), 0, cos(y));
}

inline Matrix3 GetRZMatrix(const Scalar& z)
{
    return Matrix3(cos(z), -sin(z), 0,
        sin(z), cos(z), 0,
        0, 0, 1);
}

// interpolation of roations
inline Matrix3 directLerp( const Matrix3& a,const Matrix3& b, Scalar t){
    // DONE M-directLerp: how to interpolate Matrix3s
    Matrix3 res;

    res.x = normalize(lerp(a.x, b.x, t));
    res.y = normalize(lerp(a.y, b.y, t));
    res.z = normalize(lerp(a.z, b.z, t));

    return res;
}

inline Matrix3 lerp( const Matrix3& a,const Matrix3& b, Scalar t){
    // DONE M-smartLerp: how to interpolate Matrix3s
    return (a + (b - a) * t);
}