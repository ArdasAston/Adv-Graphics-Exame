#pragma once

#include "vector3.h"

class Point3{
public:
    Scalar x,y,z;

    // constructors
    Point3():x(0),y(0),z(0){ } // origin
    Point3(Scalar _x, Scalar _y, Scalar _z):x(_x),y(_y),z(_z){ }

    static Point3 origin()   { return Point3(0, 0, 0);}

    // access to the coordinates
    Scalar& operator[] (int i){
        if (i==0) return x;
        if (i==1) return y;
        if (i==2) return z;
        //assert(0);
        return x;
    }

    Scalar operator[] (int i) const{
        if (i==0) return x;
        if (i==1) return y;
        if (i==2) return z;
        //assert(0);
        return x;
    }

    // point + vector = Point
    Point3 operator-(const Vector3 &b) const{
        return Point3( x-b.x, y-b.y, z-b.z );
    }

    // point to point difference
    Vector3 operator-(const Point3 &b) const{
        return Vector3( x-b.x, y-b.y, z-b.z );
    }

    // point plus vector out-of-place
    Point3 operator + (const Vector3& b ) const{
        return Point3( x + b.x , y + b.y, z + b.z );
    }

    // point plus vector in-place
    void operator+=(const Vector3 &b) {
        x += b.x;
        y += b.y;
        z += b.z;
    }

    Point3 scaled( Scalar k ) const{
        return Point3(x*k, y*k, z*k);
    }

    Vector3 asVector() const{
        //return *this - origin();

        return Vector3(x,y,z);
    }

    bool isEqual( const Point3 &b){
        return squaredNorm(*this - b) < EPSILON2;
    }

    void printf() const
    {
        std::cout << Point3(x, y, z) << std::endl;
    } // TODO Print
};


inline Point3 lerp( const Point3& a,const Point3& b, Scalar t){
    return a + t*(b-a);
}

inline Point3 Vector3::asPoint() const{
    return Point3(x,y,z);
}

inline Scalar squaredDist(const Point3 &a,const Point3 &b){
    return  squaredNorm( a-b );
}

inline Scalar dist(const Point3 &a,const Point3 &b){
    return norm(a-b);
}
