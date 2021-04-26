#pragma once

#include "vector.cpp"

class Ray{
    public:
        Vector origin;
        Vector direction;
        double time;
        Ray(Vector origin, Vector direction, double time){
            this->origin = origin;
            this->direction = direction;
            this->time = time;
        }

};