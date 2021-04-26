#pragma once

#include "vector.cpp"
#include "ray.cpp"
#include <cmath>

class Sphere {
    
    public:
        double radius;
        Vector center;
        Vector albedo;
        bool mirror;
        double refractionIndex;
        bool isLight;
        double lightIntensity;
        Vector speed;
        Sphere(Vector center, Vector albedo, double radius, bool isMirror, double refractInd, bool isLight, double lightIntensity, Vector speed) {
            this->center = center;
            this->albedo = albedo;
            this->radius = radius;
            this->mirror = isMirror;
            this->refractionIndex = refractInd;
            this->isLight = isLight;
            this->lightIntensity = lightIntensity;
            this->speed = speed;

        }
        Sphere(){}

};


