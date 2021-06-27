#pragma once

#include <math.h>
#include "vector3.h"
#include "point3.h"
#include "versor3.h"
#include "quaternion.h"
#include "euler.h"
#include "matrix3.h"

/* AxisAngle class */
/* this class is a candidate to store a rotation! */
/* as such, it implements all expected methods    */

class Matrix3;
class Quaternion;
class Euler;

class AxisAngle{
public:

    /* fields */
    // DONE A-Field: which fields to store? (also add a constuctor taking these fields).
    Scalar angle;
    Versor3 axis;


    // TODO A-Ide: this constructor construct the identity rotation
    AxisAngle(){}

    // TODO A-FromPoint
    // returns a AxisAngle encoding a point
    AxisAngle( const Point3& p){
        // TODO
    }

    Vector3 apply( Vector3  v) const {
        // TODO A-App: how to apply a rotation of this type?
        Scalar alpha = v.x, beta = v.y, gamma = v.z;
        Scalar cosAlpha = cos(alpha), cosBeta = cos(beta), cosGamma = cos(gamma);
        Scalar sinAlpha = sin(alpha), sinBeta = sin(beta), sinGamma = sin(gamma);

        Matrix3 matRot = Matrix3(
            cosGamma * cosBeta, cosGamma * sinBeta * sinAlpha - sinBeta * cosAlpha, cosGamma * sinBeta * cosAlpha + sinGamma * sinAlpha,
            sinGamma * cosBeta, sinGamma * sinBeta * sinAlpha + cosGamma * cosAlpha, sinGamma * sinBeta * cosAlpha - cosGamma * sinAlpha,
            -sinBeta, cosBeta * sinAlpha, cosBeta * cosAlpha
        );
        return matRot * v;
    }

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

    Versor3 axisX() const;  // TODO A-Ax a
    Versor3 axisY() const;  // TODO A-Ax b
    Versor3 axisZ() const;  // TODO A-Ax c

    // conjugate
    AxisAngle operator * (AxisAngle r) const {
        return AxisAngle();
    }

    AxisAngle inverse() const{
        // TODO A-Inv a
        return AxisAngle();
    }

    void invert() const{
        // TODO A-Inv b
    }

    // returns a rotation to look toward target, if you are in eye, and the up-vector is up
    static AxisAngle lookAt( Point3 eye, Point3 target, Versor3 up = Versor3::up() ){
        // TODO A-LookAt
        return AxisAngle();
    }

    // returns a rotation
    static AxisAngle toFrom( Versor3 to, Versor3 from ){
        // TODO A-ToFrom
        return AxisAngle();
    }

    static AxisAngle toFrom( Vector3 to, Vector3 from ){
        return toFrom( normalize(to) , normalize(from) );
    }

    // conversions to this representation
    static AxisAngle from(const Matrix3& m );   // TODO M2A
    static AxisAngle from(const Euler& e );     // TODO E2A
    static AxisAngle from(const Quaternion& q );// DONE Q2A

    // does this AxisAngle encode a poont?
    bool isPoint() const{
        // TODO A-isP
        return false;
    }

    void printf() const {} // TODO Print
};


// interpolation or roations
inline AxisAngle lerp( const AxisAngle& a,const AxisAngle& b, Scalar t){
    // TODO A-Lerp: how to interpolate AxisAngles
    // hints: shortest path! Also, consdider them are 4D unit vectors.
    return AxisAngle();
}