#pragma once
#include "geometry.cpp"

class Sphere : public Geometry{
    
    public:
        double radius;
        Sphere(const Vector& center, const Vector& albedo, const double& radius, const bool& isMirror, const double& refractInd, const bool& isLight, const double& lightIntensity, const Vector& speed) {
            this->center = center;
            this->albedo = albedo;
            this->radius = radius;
            this->mirror = isMirror;
            this->refractionIndex = refractInd;
            this->isLight = isLight;
            this->lightIntensity = lightIntensity;
            this->speed = speed;
            this->type = 0;

        }
        Sphere(){}
        Intersection intersect(const Ray& ray){
            Intersection inter;
            double t;
            Vector center =this->center +this->speed*ray.time;
            double delta = dot(ray.direction,ray.origin-center)*dot(ray.direction,ray.origin-center) - (dot(ray.origin-center,ray.origin-center) -this->radius*this->radius );
            if (delta<0){
                inter.intersects = 0;
                return inter;
            }
            else {
                double t1 = dot(ray.direction,center-ray.origin) - sqrt(delta);
                double t2 = dot(ray.direction,center-ray.origin) + sqrt(delta);
                if (t2<0){
                    inter.intersects = 0;
                    return inter;
                }
                else if (t1>=0){
                    t = t1;
                    
                } else {
                    t = t2;
                }
                inter.point = Vector(ray.origin+ray.direction*t);
                inter.intersects = 1;
                inter.objectId = this->id;
                inter.unitNormal = normalize(inter.point - center);
                inter.albedo = this->albedo;
                return inter;
            }
        };

};


