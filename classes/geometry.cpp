#pragma once
#include "ray.cpp"
struct Intersection {
    int objectId;
    Vector point;
    Vector unitNormal;
    double dist;
    int intersects;
    Vector albedo;
};

class Geometry{
    public:              
        Vector albedo;          
        int type =0 ; 
        int id;
        Vector center;
        
        bool mirror=false;
        double refractionIndex=-1;
        bool isLight=false;
        double lightIntensity =0;
        Vector speed =Vector(0.,0.,0.);
        virtual Intersection intersect(const Ray& ray)=0;

};
