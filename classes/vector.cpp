#pragma once

#include <math.h>

class Vector {
    public:
        explicit Vector ( double x = 0. , double y = 0. , double z = 0. ) {
            coords[0] = x ;
            coords[1] = y ;
            coords[2] = z ;
        }
        Vector& operator+=(const Vector& b ) {
            coords[0] += b[0];
            coords[1] += b[1];
            coords[2] += b[2];
            return *this;
        }
        const double& operator[](int i) const {
            return coords[i];
        }
        double& operator[](int i) {
            return coords[i];
        }
        double norm() const{
            return sqrt(coords[0]*coords[0]+coords[1]*coords[1]+coords[2]*coords[2]);
        }
    private:
        double coords[3];
};

Vector operator+(const Vector& a, const Vector &b){
    return Vector(a[0]+b[0],a[1]+b[1],a[2]+b[2]);
}
Vector operator-(const Vector& a, const Vector &b){
    return Vector(a[0]-b[0],a[1]-b[1],a[2]-b[2]);
}
Vector operator*(const Vector& a, const double r){
    return Vector(a[0]*r,a[1]*r,a[2]*r);
}
Vector operator*(const Vector& a, const Vector& b){
    return Vector(a[0]*b[0],a[1]*b[1],a[2]*b[2]);
}
Vector normalize(const Vector& v){
    double norm = v.norm();
    return v*(1/norm);
}
Vector cros(const Vector& v1, const Vector& v2){
    return Vector( v1[1]*v2[2] - v1[2]*v2[1],
		   v1[2]*v2[0] - v1[0]*v2[2],
		   v1[0]*v2[1] - v1[1]*v2[0] 
		);
}
double dot(const Vector& a, const Vector& b){
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}
