#pragma once

#include "geometry.cpp"
class Scene{
    public:
        std::vector<Geometry*> scene;
        double refractionIndex;
        Scene(){}
        Scene( double refractionIndex){
            this->scene = std::vector<Geometry*>{};
            this->refractionIndex = refractionIndex;
        }
        void addGeometry(Geometry* obj){
            obj->id = scene.size();
            scene.push_back(obj);
        }
        Intersection intersect(const Ray& ray){
            Intersection res;
            res.intersects=0;
            if (this->scene.size() == 0){
                return res;
            }
            else{
                double cur_dist = INFINITY;
                Intersection local;
                double tmp_dist;
                for (int i = 0; i<this->scene.size(); i++){
                    local = this->scene[i]->intersect(ray);
                    if (local.intersects==1){
                        tmp_dist = (local.point-ray.origin).norm();
                        if ( tmp_dist < cur_dist){
                            cur_dist = tmp_dist;
                            res = local;
                        }
                    }  
                }
                return res;
            }
        };
};