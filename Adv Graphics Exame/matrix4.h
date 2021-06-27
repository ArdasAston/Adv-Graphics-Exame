#pragma once

#include "vector3.h"
#include "point3.h"
#include "versor3.h"

// INGORE THIS CLASS


// stores a 4x4 transformation matrix
// (as three column vectors)
// the last raw is implicitly 0,0,0,1
class Matrix4{
public:
    Vector3 x,y,z; // the columns 1,2,3 (and the axes of the INPUT SPACE)
    Point3 o; // the column 4 (and origin of the INPUT SPACE)

    Vector3 right()   { return x;}
    Vector3 left()    { return -x;}
    Vector3 up()      { return y;}
    Vector3 down()    { return -y;}
    Vector3 fowrard() { return z;}
    Vector3 backward(){ return -z;}

    Versor3 apply(Versor3 d){
        return normalize(d.x * x + d.y * y + d.z * z);
    }

    Vector3 apply(Vector3 v){
        return v.x * x + v.y * y + v.z * z;
    }

    Point3 apply(Point3 p){
        return o + p.x * x + p.y * y + p.z * z ;
    }

    Matrix4 inverse() const;  // out-of-place
    void invert(); // in-place

    Matrix4 operator * (const Matrix4 &b){
        Matrix4 m;
    }

    Scalar det() const{
        return dot( cross(x,y) , z );
    }

};

Matrix4 lerp( const Matrix4 &a, const Matrix4 &b, float t );

