#pragma once

#include "vector3.h"
#include "point3.h"
#include "versor3.h"

#include "quaternion.h"

// The BIG CHOICE:
typedef Quaternion Rotation;
//typedef AxisAngle Rotation;
//typedef Matrix3 Rotation;
//typedef Euler Rotation;

class Transform
{
public:
    Scalar   scale;   // scaling (isotropic)
    Rotation rotation; // rotation
    Vector3  translation;  // translation

    // description of the local frame!
    Point3 origin() const    { return translation.asPoint();}
    Versor3 left() const     { return -rotation.axisX(); }
    Versor3 right()const     { return +rotation.axisX(); }
    Versor3 up() const       { return -rotation.axisY(); }
    Versor3 down()const      { return +rotation.axisY(); }
    Versor3 forward() const  { return -rotation.axisZ(); }
    Versor3 backward()const  { return +rotation.axisZ(); }

    // constructor: is the ide
    Transform():scale(1),rotation(),translation(0,0,0){}

    Versor3 apply(Versor3 dir){
        // apply only r
        return rotation.apply(dir);
    }

    Vector3 apply(Vector3 v){
        // apply s r
        return rotation.apply( v*scale );
    }

    Point3 apply(Point3 p){ // apply s r t
        // apply s r t
        return rotation.apply( p.scaled( scale ) ) + translation;
    }

    // inverse out-of-place
    Transform inverse() const{
        Transform result;
        result.scale = 1/scale;
        result.rotation = rotation.inverse();
        result.translation = result.rotation.apply( (-translation)*result.scale );
        return result;
    }


    void invert(); // TODO T-invert  (in-place)

    // CUMULATE: first b, then *this
    Transform operator * (const Transform &b){
        Transform result;
        result.scale = scale * b.scale;
        result.rotation = rotation * b.rotation;
        result.translation = b.translation * scale + translation;
        return result;
    }

    // fills a column-major Scalar 4x4 zmatrix for DirectX (or OpenGL)
    // vector d is already allocated
    void fillDirectXMatrix( Scalar d[]){
        // TODO
    }

    // places this transform in the given origin, looking (Z-axis) toward target, given the up vector
    void place( Point3 origin, Point3 target, Versor3 up ){
        // TODO: use methods
    }

    void printf() const {} // TODO

};

Transform lerp( const Transform &a, const Transform &b, float t ){
    // TODO: T-lerp

    return Transform();
}

