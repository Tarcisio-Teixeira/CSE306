#pragma once

#include <vector>
#include <iostream>
#include "sphere.cpp"

class Scene{
    public:
        std::vector<Sphere> scene;
        double refractionIndex;
        Scene(std::vector<Sphere> scene, double refractionIndex){
            this->scene = scene;
            this->refractionIndex = refractionIndex;
        }
        
};