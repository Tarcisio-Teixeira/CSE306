#include "vector.cpp"
#include <set>
#include <vector>
#include "mesh.cpp"

std::vector<Vector> idBoundary(TriangleMesh mesh){}


double sumBoundary(std::vector<Vector> boundary){
    double res = 0;
    int n = boundary.size();
    for(int i=0;i<n;i++){
        res += (boundary[(i+1)%n]-boundary[i%n]).norm();
    }
    return res;
}

std::vector<Vector> initialiseDisk(int numberOfVertices,std::vector<Vector> boundary){
    int n  = boundary.size(); double cs = 0;
    std::vector<Vector> verticesDisk(numberOfVertices);
    double s = sumBoundary(boundary);
    for(int i=0;i<n;i++){
        double theta = 2 * M_PI * (cs/s);
        verticesDisk[i] = Vector(cos(theta),sin(theta),0.);
        cs += (boundary[(i+1)%n]-boundary[i%n]).norm();
    }
    for(int i=n;i<numberOfVertices;i++){
        double theta = 2*M_PI* rand()/RAND_MAX;
        double r = 1 * sqrt(rand()/RAND_MAX);
        verticesDisk[i] = Vector(r*cos(theta),r*sin(theta),0);
    }
    return verticesDisk;
}

std::vector<std::set<Vector>> findAdjacent(TriangleMesh mesh){
    std::vector<std::set<Vector>> adjacent(mesh.indices.size());
    for(int i=0;i<mesh.indices.size();i++){
        TriangleIndices triangle = mesh.indices[i];
        adjacent[triangle.vtxi].insert(mesh.vertices[triangle.vtxj]);
        adjacent[triangle.vtxi].insert(mesh.vertices[triangle.vtxk]);
        adjacent[triangle.vtxj].insert(mesh.vertices[triangle.vtxi]);
        adjacent[triangle.vtxj].insert(mesh.vertices[triangle.vtxk]);
        adjacent[triangle.vtxk].insert(mesh.vertices[triangle.vtxj]);
        adjacent[triangle.vtxk].insert(mesh.vertices[triangle.vtxi]);
    }
    return adjacent;
}

Vector sum(std::set<Vector> s){
    Vector res(0,0,0);
    for (std::set<Vector>::iterator it = s.begin(); it != s.end(); ++it) {
    res += *it;
    }
    return res;
}

std::vector<Vector>  tutteEmbedding(TriangleMesh mesh,int n_iter = 100){
    std::vector<Vector> boundM = idBoundary(mesh); 
    int n = boundM.size(); 
    std::vector<Vector> v = initialiseDisk(mesh.indices.size(),boundM);
    std::vector<std::set<Vector>> adjacent = findAdjacent(mesh);
    for(int i=0;i<n_iter;i++){
        for(int i=n;i<mesh.indices.size();i++){
            double K = adjacent[i].size();
            v[i] = (1./K) * sum(adjacent[i]);
        }
    }
    return v;
    
}


int main(){
    return 0;
}