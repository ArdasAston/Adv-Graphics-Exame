#pragma once


// problems.h:
// a small set of matematical problems

#include "vector3.h"
#include "point3.h"
#include "versor3.h"

// POD -- Plain Old Data
struct Ray{
    Point3 p;
    Vector3 v;
};

struct Sphere{
    Point3 c;
    Scalar r;
};

struct Plane{
    // TODO-plane: fill
};

const Point3 NO_INTERSECTION(666,666,666);

Point3 intersect(const Ray &r, const Sphere &s){
    Scalar a = dot( r.v , r.v );
    Scalar b = 2*dot( r.p-s.c , r.v );
    Scalar c = dot(r.p-s.c,r.p-s.c) - s.r * s.r;

    // euqation is: a*k^2 + b*k + c == 0

    Scalar delta = b*b-4*a*c;
    if (delta < 0) return NO_INTERSECTION;

    Scalar k = (-b -sqrt(delta)) / (2*a);
    if (k<0) return NO_INTERSECTION;
    return r.p + k * r.v;
}

Point3 intersect(const Ray &r, const Plane &p){
    // TODO: P-inter-plane
    return Point3();
}

// does a eye in a given position sees the target, given the angle cone and maxDist?
bool isSeen( const Point3 &eye, const Point3 &target, Scalar angleDeg , Scalar maxDist){
    // TODO: isSeen
}

// returns the reflected direction of something bouncing in n
Versor3 reflect( const Versor3 &d, const Versor3 &n){
    // TODO: P-reflect
    return Versor3::up();
}

bool areCoplanar(const Versor3 &a, const Versor3 &b, const Versor3 &c){
    // TODO: P-coplanar
}

// normal of a plane, given three points
Versor3 planeNormal(const Versor3 &a, const Versor3 &b, const Versor3 &c){
    // TODO: P-palneNorm
    return Versor3::up();
}

// return a versor as similar as possible to a, but ortogonal to b
Versor3 orthogonalize( const Versor3 &a, const Versor3 &b ){
    // TODO: P-ortogonalize
    return Versor3::up();
}

// a bullet is in position pa and has velocity va
// a target is in position pb and has velocity vb
// returns the position of the target in the moment it is closest to the bullet
Point3 hitPos( const Point3 &pa, const Vector3 &va, const Point3 &pb, const Vector3 &vb ){
    // TODO: P-hitPos
    return Point3();
}



