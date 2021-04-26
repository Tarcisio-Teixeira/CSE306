#pragma once
#include "vector.cpp"
#include "mesh.cpp"

class BoundingBox {
    Vector Bmin;
    Vector Bmax;
    BoundingBox(TriangleMesh* mesh){
        std::vector<Vector> vertices = mesh->vertices;
        Vector min = vertices[0];
        Vector max = vertices[0];
        for(long long i =0; i<vertices.size();++i){
            min[0] = std::min(vertices[i][0],min[0]);
            min[1] = std::min(vertices[i][1],min[1]);
            min[2] = std::min(vertices[i][2],min[2]);
            max[0] = std::max(vertices[i][0],max[0]);
            max[1] = std::max(vertices[i][1],max[1]);
            max[2] = std::max(vertices[i][2],max[2]);

        }
        Bmax = max;
        Bmin = min;

    }

}