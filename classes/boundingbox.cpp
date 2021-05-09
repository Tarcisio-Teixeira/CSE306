#pragma once
#include "mesh.cpp"
#include <list>

class BoundingBox : public Geometry{
public:
    Vector Bmin;
    Vector Bmax;
    TriangleMesh* mesh;
    

    BoundingBox(){};
    BoundingBox(TriangleMesh* mesh,  int startIndex,  int endIndex){
        
        this->mesh = mesh;
        this->type = 1;
        
        TriangleIndices vts;
        std::vector<Vector> vertices = this->mesh->vertices;
        vts = mesh->indices[startIndex];
        Vector min(INFINITY,INFINITY,INFINITY);
        Vector max(0.,0.,0.);
        
        for(int i =startIndex; i<endIndex;++i){
            vts = this->mesh->indices[i];
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
    void setId(int id){
        this->id = id;
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
            ///////////  only one bbox ///////////////
            // return mesh->intersect(ray);
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
    int startIndex;
    int endIndex;
    BVH(){}
    BVH( TriangleMesh* mesh, const int& _startIndex, const int& _endIndex, const unsigned char* texture){
        this->leftChild = NULL;
        this->rightChild = NULL;
        this->type = 2;
        this->mesh = mesh;
        
        this->startIndex = _startIndex;
        this->endIndex = _endIndex;
        this->albedo = mesh->albedo;
        this->refractionIndex = -1;
        this->mirror = false;
        this->texture = texture;
        this->bbox = new BoundingBox(mesh,_startIndex,_endIndex);
        
        Vector diag = this->bbox->Bmax - this->bbox->Bmin;
        Vector midDiag = (this->bbox)->Bmin +diag*(0.5);
        int longestAxis = getLongestAxis(diag);
        int pivotIndex = this->startIndex;
        Vector barycenter;
        TriangleIndices t;
        
        
        for ( int i = this->startIndex;i<this->endIndex;i++){
            t =mesh->indices[i] ;
            barycenter=this->mesh->compute_barycenter(i);
            if (barycenter[longestAxis] < midDiag[longestAxis]){
                std::swap(mesh->indices[i], mesh->indices[pivotIndex]  );
                pivotIndex++;
            }
        }
        
        if (pivotIndex<=this->startIndex || pivotIndex >=this->endIndex-1 || this->endIndex - this->startIndex < 5){
            this->isThereChild = false;
            return;
        } 
        this->isThereChild = true;
        this->leftChild = new BVH(this->mesh, this->startIndex, pivotIndex,this->texture);
        this->rightChild = new BVH(this->mesh, pivotIndex, this->endIndex, this->texture);

    }
    void setId(int id){
        this->id = id;
        if (this->isThereChild){
            leftChild->setId(id);
            rightChild->setId(id);
        }
    }
    
    Intersection intersect(const Ray& ray){
        BVH* curNode;
        Intersection inter;
        Intersection linter,rinter;

                
        
        bool b;

        int type = this->mesh->type;
        
        

        inter.intersects =0;
        Intersection it = (this->bbox)->intersect(ray);
        if (it.intersects==0){
            return inter;
        }
        std::list<BVH*> nodes_to_visit;
        nodes_to_visit.push_front(this);
        double best_inter_distance = std::numeric_limits<double >::max();
        while(!nodes_to_visit.empty()){
            curNode = nodes_to_visit.back();
            nodes_to_visit.pop_back();
            if (curNode->isThereChild){
                linter = (curNode->leftChild)->bbox->intersect(ray);
                
                if (linter.intersects==1 ){
                    if (linter.dist <=best_inter_distance){
                        nodes_to_visit.push_back(curNode->leftChild);
                        // best_inter_distance = linter.dist;
                    }
                }
                rinter = (curNode->rightChild)->bbox->intersect(ray);
                if (rinter.intersects==1 ){
                    if (rinter.dist <= best_inter_distance){
                        nodes_to_visit.push_back(curNode->rightChild);
                        // best_inter_distance = rinter.dist;
                    }
                }
            }
            else {
                
                Intersection in = this->interAux(ray, curNode->startIndex, curNode->endIndex, best_inter_distance);
                if (in.intersects &&in.dist < best_inter_distance){
					best_inter_distance = in.dist;
					inter = in;
				}
            }
        }
        return inter;
    }
    Intersection interAux(const Ray& ray, int startIndex, int endIndex, double previousBest){
      
        Intersection intertmp;
        TriangleIndices triangle;
        Vector A, B, C, e1, e2, N, u, uv;
        double beta, gamma, alpha, t,x,y;
        double min = previousBest;
        intertmp.intersects = 0;
        bool b;
        for (int i = startIndex; i < endIndex; i++){
            triangle = this->mesh->indices[i];
            A = this->mesh->vertices[triangle.vtxi];
            B = this->mesh->vertices[triangle.vtxj];
            C = this->mesh->vertices[triangle.vtxk];
            e1 = B - A;
            e2 = C - A;
            N = cros(e1,e2);
            u = normalize(ray.direction);
            if (dot(u, N)!=0.){
                beta = dot(e2, cros(A - ray.origin, u))/dot(u, N);
                gamma = -dot(e1, cros(A - ray.origin, u))/dot(u, N);
                alpha = 1.- beta - gamma;
                t = dot(A-ray.origin, N)/dot(u, N);
                b = (0 <= alpha) && (alpha <= 1) && (0 <= beta) && (beta <= 1) && (0 <= gamma) && (gamma <= 1);
                if ((t > 0) && (t <min) && b){
                    min = t;
                    intertmp.intersects = 1;
                    intertmp.dist = t;
                    intertmp.point = ray.origin + u*t;
                    // intertmp.unitNormal = normalize(N);
                    intertmp.unitNormal = normalize(this->mesh->normals[triangle.ni]*alpha+ this->mesh->normals[triangle.nj]*beta+ this->mesh->normals[triangle.nk]*gamma);
                    // intertmp.objectId = this->id;
                    intertmp.albedo=this->mesh->albedo;
                    // uv=  this->mesh->uvs[triangle.uvi]*alpha+ this->mesh->uvs[triangle.uvj] * beta+  this->mesh->uvs[triangle.uvk]*gamma;
                    // x = int(std::max( std::min(uv[0]*512.,512.) ,0.));
                    // y = int(1024 - 1 - std::max(std::min(uv[1]*1024.,1024.),0.));
                    // intertmp.albedo[0] = pow(texture[int(y*512*3 + x*3) + 0]/255.,2.2);
                    // intertmp.albedo[1] = pow(texture[int(y*512*3 + x*3 )+ 1]/255.,2.2);
                    // intertmp.albedo[2] = pow(texture[int(y*512*3 + x*3) + 2]/255.,2.2);
                    
                }
            }
        }
        return intertmp;
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