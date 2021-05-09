#pragma once
#include "mesh.cpp"
#include <list>

class BoundingBox : public Geometry{
public:
    Vector Bmin;
    Vector Bmax;
    TriangleMesh* mesh;
    
    long startIndex;
    long endIndex;

    BoundingBox(){};
    BoundingBox(TriangleMesh* mesh, const long& startIndex, const long& endIndex){
        
        this->mesh = mesh;
        this->type = 1;
        this->startIndex = startIndex;
        this->endIndex = endIndex;
        TriangleIndices vts;
        std::vector<Vector> vertices = mesh->vertices;
        Vector min = Vector(INFINITY,INFINITY,INFINITY);
        Vector max = Vector(0.,0.,0.);
        
        for(long i =startIndex; i<endIndex;++i){
            vts = mesh->indices[i];
            min[0] = std::min(vertices[vts.vtxi][0],min[0]);
            min[1] = std::min(vertices[vts.vtxi][1],min[1]);
            min[2] = std::min(vertices[vts.vtxi][2],min[2]);
            max[0] = std::max(vertices[vts.vtxi][0],max[0]);
            max[1] = std::max(vertices[vts.vtxi][1],max[1]);
            max[2] = std::max(vertices[vts.vtxi][2],max[2]);

            min[0] = std::min(vertices[vts.vtxj][0],min[0]);
            min[1] = std::min(vertices[vts.vtxj][1],min[1]);
            min[2] = std::min(vertices[vts.vtxj][2],min[2]);
            max[0] = std::max(vertices[vts.vtxj][0],max[0]);
            max[1] = std::max(vertices[vts.vtxj][1],max[1]);
            max[2] = std::max(vertices[vts.vtxj][2],max[2]);

            min[0] = std::min(vertices[vts.vtxk][0],min[0]);
            min[1] = std::min(vertices[vts.vtxk][1],min[1]);
            min[2] = std::min(vertices[vts.vtxk][2],min[2]);
            max[0] = std::max(vertices[vts.vtxk][0],max[0]);
            max[1] = std::max(vertices[vts.vtxk][1],max[1]);
            max[2] = std::max(vertices[vts.vtxk][2],max[2]);

        }
        Bmax = max;
        Bmin = min;
        
    }
    Intersection intersect(const Ray& ray){
        Intersection inter;
        TriangleMesh* mesh = this->mesh;
        double xmin = this->Bmin[0];
        double ymin = this->Bmin[1];
        double zmin = this->Bmin[2];
        double xmax = this->Bmax[0];
        double ymax = this->Bmax[1];
        double zmax = this->Bmax[2];
        double ux = ray.direction[0];
        double uy = ray.direction[1];
        double uz = ray.direction[2];
        double x0 = ray.origin[0];
        double y0 = ray.origin[1];
        double z0 = ray.origin[2];
        double t0x;
        double t0y;
        double t0z;
        double t1x;
        double t1y;
        double t1z;
        // std::cout << ux<< " " << uy<< " " << uz<< std::endl;
        if(ux!=0.){ 
            t1x= std::max((xmax-x0)/(ux),(xmin-x0)/(ux));
            t0x = std::min((xmax-x0)/(ux),(xmin-x0)/(ux));
        } 
        else{ 
            t1x= INFINITY;
            t0x= 0.;
        }
        
        if(uy!=0.){ 
            t1y= std::max((ymax-y0)/(uy),(ymin-y0)/(uy));
            t0y= std::min((ymax-y0)/(uy),(ymin-y0)/(uy));
        } 
        else{ 
            t1y= INFINITY;
            t0y= 0.;
        }
        
        if(uz!=0.){ 
            t1z= std::max((zmax-z0)/(uz),(zmin-z0)/(uz));
            t0z= std::min((zmax-z0)/(uz),(zmin-z0)/(uz));
        } 
        else{
            t1z= INFINITY;
            t0z = 0.;
        }
        double mint1 = std::min(t1x,std::min(t1y,t1z));
        double maxt0 = std::max(t0x,std::max(t0y,t0z));
        // std::cout<< mint1 << " " << maxt0<< std::endl;
        if (mint1 <= maxt0){
            inter.intersects=0;
            return inter;
        }
        else {
            inter.intersects=1;
            inter.dist = maxt0;
            return inter;
        }
    }
};

class BVH : public Geometry {
public:
    BVH* leftChild;
    BVH* rightChild;
    BoundingBox* bbox;
    bool isThereChild;
    TriangleMesh* mesh;
    const unsigned char* texture;
    long startIndex;
    long endIndex;
    
    BVH( TriangleMesh* mesh, const long& startIndex, const long& endIndex, const unsigned char* texture){
        this->leftChild = NULL;
        this->rightChild = NULL;
        this->type = 2;
        this->mesh = mesh;
        this->bbox = new BoundingBox(mesh,startIndex,endIndex);
        this->startIndex = startIndex;
        this->endIndex = endIndex;
        this->albedo = mesh->albedo;
        Vector diag = computeDiag(this->bbox);
        Vector midDiag = (this->bbox)->Bmin +diag*(0.5);
        int longestAxis = getLongestAxis(diag);
        int pivotIndex = startIndex;
        Vector barycenter;
        TriangleIndices t;
        this->texture = texture;
        
        for ( int i = startIndex;i<endIndex;i++){
            t =mesh->indices[i] ;
            barycenter=computeBarycenter(t);
            if (barycenter[longestAxis] < midDiag[longestAxis]){
                std::swap(mesh->indices[i], mesh->indices[pivotIndex]  );
                pivotIndex++;
            }
        }
        
        if (pivotIndex<=startIndex || pivotIndex >=endIndex-1 || endIndex - startIndex < 5){
            this->isThereChild = false;
            return;
        } 
        this->isThereChild = true;
        leftChild = new BVH(mesh, startIndex, pivotIndex,texture);
        rightChild = new BVH(mesh, pivotIndex, endIndex, texture);

    }
    Intersection intersect(const Ray& ray){
        BVH* curNode;
        Intersection inter;
        Intersection linter,rinter;

                
        std::vector<TriangleIndices> indices= this->mesh->indices;
        // std::vector<TriangleIndices> indices(&realindices[curNode->startIndex],&realindices[curNode->endIndex]);
        
        std::vector<Vector> vertices=this->mesh->vertices;
        std::vector<Vector> normals=this->mesh->normals;
        std::vector<Vector> uvs=this->mesh->uvs;
        std::vector<Vector> vertexcolors=this->mesh->vertexcolors;

        int type = this->mesh->type;
        TriangleIndices triangle;
        Vector A;
        Vector B;
        Vector C;
        Vector e1;
        Vector e2;
        Vector N;
        Vector u;
        Vector uv;
        double beta;
        double gamma;
        double alpha;
        double t,x,y;

        inter.intersects =0;
        if ((this->bbox)->intersect(ray).intersects==0){
            return inter;
        }
        std::list<BVH*> nodes_to_visit;
        nodes_to_visit.push_front(this);
        double best_inter_distance = std::numeric_limits<double >::max();
        while(!nodes_to_visit.empty()){
            curNode = nodes_to_visit.back();
            nodes_to_visit.pop_back();
            if (curNode->isThereChild){
                linter = curNode->leftChild->bbox->intersect(ray);
                rinter = curNode->rightChild->bbox->intersect(ray);
                if (linter.intersects==1 ){
                    if (linter.dist <=best_inter_distance){
                        nodes_to_visit.push_back(curNode->leftChild);
                        // best_inter_distance = linter.dist;
                    }
                }
                
                if (rinter.intersects==1 ){
                    if (rinter.dist <= best_inter_distance){
                        nodes_to_visit.push_back(curNode->rightChild);
                        // best_inter_distance = rinter.dist;
                    }
                }
            }
            else {
                // std::cout << "here" << std::endl;
                linter = curNode->bbox->intersect(ray);
                if (linter.dist > best_inter_distance){
                    continue;
                }
                double min = best_inter_distance;
                for (int i = curNode->startIndex; i < curNode->endIndex; i++){
                    triangle = indices[i];
                    A = vertices[triangle.vtxi];
                    B = vertices[triangle.vtxj];
                    C = vertices[triangle.vtxk];
                    e1 = B - A;
                    e2 = C - A;
                    N = cros(e1,e2);
                    u = normalize(ray.direction);
                    if (dot(u, N)!=0.){
                        beta = dot(e2, cros(A - ray.origin, u))/dot(u, N);
                        gamma = -dot(e1, cros(A - ray.origin, u))/dot(u, N);
                        alpha = 1.- beta - gamma;
                        t = dot(A-ray.origin, N)/dot(u, N);
                        if ((t >= 0) && (t < min) ){
                            bool b = (0 <= alpha) && (alpha <= 1) && (0 <= beta) && (beta <= 1) && (0 <= gamma) && (gamma <= 1);
                            if (b) {
                                min = t;
                                inter.intersects = 1;
                                inter.dist = t;
                                inter.point = ray.origin + u*t;
                                inter.unitNormal = normalize(N);
                                // inter.unitNormal = normalize(normals[triangle.ni]*alpha+ normals[triangle.nj]*beta+ normals[triangle.nk]*gamma);
                                inter.objectId = this->id;
                                inter.albedo=mesh->albedo;
                                // uv=  mesh->uvs[triangle.uvi]*alpha+ mesh->uvs[triangle.uvj] * beta+  mesh->uvs[triangle.uvk]*gamma;
                                // x = int(std::max( std::min(uv[0]*512.,512.) ,0.));
                                // y = int(1024 - 1 - std::max(std::min(uv[1]*1024.,1024.),0.));
                                // inter.albedo[0] = pow(texture[int(y*512*3 + x*3) + 0]/255.,2.2);
                                // inter.albedo[1] = pow(texture[int(y*512*3 + x*3 )+ 1]/255.,2.2);
                                // inter.albedo[2] = pow(texture[int(y*512*3 + x*3) + 2]/255.,2.2);
                            }
                        }
                    }
                }
                if (inter.intersects==1 && min < best_inter_distance) best_inter_distance = min;
            }
        }
        return inter;
    }
    Vector computeDiag(const BoundingBox* bbox){
        return bbox->Bmax - bbox->Bmin;
    }
    int getLongestAxis(const Vector& v){
        double m = std::max(std::abs(v[0]),std::max(std::abs(v[1]),std::abs(v[2])));
        if (m == std::abs(v[0])){
            return 0;
        }
        else {
            return (m==std::abs(v[1])) ? 1 : 2;
        }
    }
    Vector computeBarycenter(const TriangleIndices& t){
        Vector A = mesh->vertices[t.vtxi];
        A += mesh->vertices[t.vtxj];
        A += mesh->vertices[t.vtxk];
        return A*(1./3.);
    }
};