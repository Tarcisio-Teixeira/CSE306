#pragma once

#include "vector.cpp"

class Ray{
    public:
        Vector origin;
        Vector direction;
        double time;
        Ray(){};
        Ray(const Vector& origin, const Vector& direction, const double& time){
            this->origin = origin;
            this->direction = direction;
            this->time = time;
        }

};