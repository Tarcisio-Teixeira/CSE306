#pragma once
#include <vector>
#include "vector.cpp"
#include "sphere.cpp"
#include "scene.cpp"


struct Intersection {
    Sphere sphere;
    Vector point;
    Vector unitNormal;
    int intersects;
};

Intersection intersect_sphere(Ray ray,Sphere s){
    struct Intersection inter;
    double t;
    Vector center = s.center + s.speed*ray.time;
    double delta = dot(ray.direction,ray.origin-center)*dot(ray.direction,ray.origin-center) - (dot(ray.origin-center,ray.origin-center) - s.radius*s.radius );
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
        inter.sphere = s;
        inter.unitNormal = normalize(inter.point - center);
        return inter;
    }
};

Intersection intersect_scene(Ray ray,Scene s){
    Intersection res;
    if (s.scene.size() == 0){
        res.intersects=0;
        return res;
    }
    else{
        res = intersect_sphere(ray,s.scene[0]);
        double cur_dist = (res.point - ray.origin).norm();
        for (int i = 1; i<s.scene.size(); i++){
            if (!res.intersects){
                res = intersect_sphere(ray,s.scene[i]);
                cur_dist = (res.point - ray.origin).norm();
            }
            else {
                Intersection local = intersect_sphere(ray,s.scene[i]);
                if (local.intersects){
                    double tmp_dist = (local.point-ray.origin).norm();
                    if (tmp_dist!=0 && tmp_dist < cur_dist){
                        cur_dist = tmp_dist;
                        res = local;
                    }
                }
            }
            
        }
        
        return res;
    }
    
};