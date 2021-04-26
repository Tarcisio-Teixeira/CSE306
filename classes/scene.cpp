#pragma once

#include <vector>
#include <iostream>
#include "geometry.cpp"

class Scene{
    public:
        std::vector<Geometry*> scene;
        double refractionIndex;
        Scene(std::vector<Geometry*> scene, double refractionIndex){
            this->scene = scene;
            this->refractionIndex = refractionIndex;
        }
        
};