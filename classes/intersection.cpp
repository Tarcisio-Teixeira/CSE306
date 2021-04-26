#pragma once
#include <vector>
#include "vector.cpp"
#include "sphere.cpp"
#include "scene.cpp"
#include "mesh.cpp"


struct Intersection {
    Geometry* object;
    Vector point;
    Vector unitNormal;
    int intersects;
};
Intersection intersect_mesh(Ray ray, TriangleMesh* mesh){
    Intersection inter;
    std::vector<TriangleIndices> indices = mesh->indices;
	std::vector<Vector> vertices = mesh->vertices;
	std::vector<Vector> normals=mesh->normals;
	std::vector<Vector> uvs=mesh->uvs;
	std::vector<Vector> vertexcolors=mesh->vertexcolors;
    int type = mesh->type;
    double min = INFINITY;
    for (int i = 0; i < indices.size(); i++){
        TriangleIndices triangle = indices[i];
        Vector A = vertices[triangle.vtxi];
        Vector B = vertices[triangle.vtxj];
        Vector C = vertices[triangle.vtxk];
        Vector e1 = B - A;
        Vector e2 = C - A;
        Vector N = cros(e1,e2);
        // Vector u = normalize(ray.u);
        Vector u = ray.direction;
        double Beta = dot(e2, cros(A - ray.origin, u))/dot(u, N);
        double Gamma = -dot(e1, cros(A - ray.origin, u))/dot(u, N);
        double alpha = 1.- Beta - Gamma;
        double t = dot(A-ray.origin, N)/dot(u, N);

        bool in_triangle = ((0 <= alpha) && (alpha <= 1) && (0 <= Beta) && (Beta <= 1) && (0 <= Gamma) && (Gamma <= 1));
        bool closest = t < min;

        if (t >= 0 && in_triangle && closest){
            min = t;
            inter.intersects = 1;
            inter.point = ray.origin + u*t;
            inter.unitNormal = N;
            inter.object = mesh;
            
            // printf("%f%f%f\n", albedo[0], albedo[1], albedo[2]);
            // printf("%f\n", type);
        }

    }
    
    return inter;
}
Intersection intersect_sphere(Ray ray,Sphere* s){
    struct Intersection inter;
    double t;
    Vector center = s->center + s->speed*ray.time;
    double delta = dot(ray.direction,ray.origin-center)*dot(ray.direction,ray.origin-center) - (dot(ray.origin-center,ray.origin-center) - s->radius*s->radius );
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
        inter.object = s;
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
        if (s.scene[0]->type == 0){
            res = intersect_sphere(ray,(Sphere*)s.scene[0]);
        } else if (s.scene[0]->type == 1) {
            res = intersect_mesh(ray,(TriangleMesh*)s.scene[0]);
        }
        double cur_dist = (res.point - ray.origin).norm();
        for (int i = 1; i<s.scene.size(); i++){
            if (!res.intersects){
                if (s.scene[i]->type == 0){
                    res = intersect_sphere(ray,(Sphere*)s.scene[i]);
                } else if (s.scene[i]->type == 1) {
                    res = intersect_mesh(ray,(TriangleMesh*)s.scene[i]);
                }
                cur_dist = (res.point - ray.origin).norm();
            }
            else {
                Intersection local;
                if (s.scene[i]->type == 0){
                    local = intersect_sphere(ray,(Sphere*)s.scene[i]);
                } else if (s.scene[i]->type == 1) {
                    local = intersect_mesh(ray,(TriangleMesh*)s.scene[i]);
                }
                
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