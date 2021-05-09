#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb-master/stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "../stb-master/stb_image.h"
#include <iostream>
// #include <cmath>
#include <chrono>
#include "classes/vector.cpp"
#include "classes/scene.cpp"
#include "classes/ray.cpp"
#include "classes/sphere.cpp"
#include "classes/getcolor.cpp"
#include "classes/geometry.cpp"
#include "classes/boundingbox.cpp"
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif

using namespace std;
    
int main() {
    
    int W = 512;
    int H = 512;
    double I = 2*10000000000;
    double alpha = M_PI/3.;
    double f = (W/2.)/tan(alpha/2.);
    int maxDepth = 4;
    double airRefractionIndex = 1.0;
    double aperture = 2.8;
    double distance = 55.;

    Sphere* centerTestSphere=new Sphere(Vector(0.,0.,0.),Vector(1.,1.,1.),10.,false,-1, false, 0,Vector(0.,0.,0.));//normalrefract
    Sphere* leftTestSphere= new Sphere(Vector(-10.,0.,15),Vector(1.,1.,1.),10.,false,1.5, false, 0,Vector(0.,0.,0.));//refract
    Sphere* rightTestSphere= new Sphere(Vector(9.,0.,-15),Vector(1.,1.,1.),10.,false,-1, false, 0,Vector(0.,0.,0.));//mirror
    Sphere* internalToRightSphere= new Sphere(Vector(20.,0.,0.), Vector(1.,1.,1.), 9.5, false, 1.0 , false, 0,Vector(0.,0.,0.));
    Sphere* topSphere= new Sphere(Vector(0.,1000.,0.),Vector(1.,0.,0.),940.,false,-1., false, 0,Vector(0.,0.,0.));
    Sphere* leftSphere= new Sphere(Vector(-1000.,0.,0.),Vector(0.,1.,1.),940.,false,-1., false, 0,Vector(0.,0.,0.));
    Sphere* bottomSphere= new Sphere(Vector(0.,-1000.,0.),Vector(0.,0.,1.),990.,false,-1., false, 0,Vector(0.,0.,0.));
    Sphere* rightSphere= new Sphere(Vector(1000.,0.,0.),Vector(1.,1.,0.),940.,false,-1., false, 0,Vector(0.,0.,0.));
    Sphere* backSphere= new Sphere(Vector(0.,0.,1000.),Vector(1.,0.,1.),940.,false,-1., false, 0,Vector(0.,0.,0.));
    Sphere* frontSphere= new Sphere(Vector(0.,0.,-1000.),Vector(0.,1.,0.),940.,false,-1., false, 0,Vector(0.,0.,0.));
    TriangleMesh* Cat=new TriangleMesh(Vector(1.,1.,1.));
    Cat->readOBJ("./model/Models/cat.obj");
    for (int i = 0; i < (Cat->vertices).size(); i++) {
        Cat->vertices[i] = Cat->vertices[i]*0.6 + Vector(0, -10, 0);
    }
    const char* catTextureFileName = "./model/Models/cat_diff.png";
    int comp = 1;
    int H_=512,W_=512;
    unsigned char* catTexture= stbi_load(catTextureFileName,&H_,&W_,&comp,3);
    BVH* bvh = new BVH(Cat,0,Cat->indices.size(),catTexture);
    BoundingBox* bbox= new BoundingBox(Cat,0,Cat->indices.size());
    Vector lightOrigin(-10.,20.,40.);
    Sphere* lightSphere = new Sphere(lightOrigin,Vector(1.,1.,1.),5,false,-1., true, I,Vector(0.,0.,0.));
    Vector cameraPosition(0,0,55);
    Scene scene(airRefractionIndex);
    scene.addGeometry(leftSphere);
    scene.addGeometry(rightSphere);
    scene.addGeometry(topSphere);
    scene.addGeometry(bottomSphere);
    scene.addGeometry(frontSphere);
    scene.addGeometry( backSphere);
    scene.addGeometry(lightSphere);
    // scene.addGeometry(internalToRightSphere);
    // scene.addGeometry(rightTestSphere);
    // scene.addGeometry(leftTestSphere);
    // scene.addGeometry(centerTestSphere);

    scene.addGeometry(bvh);
    // scene.addGeometry(Cat);
    // scene.addGeometry(bbox);
    
    GetColor getColor(scene);
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<unsigned char> image(W*H * 3, 0);
    #pragma omp parallel for schedule(dynamic,1)
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            ////////////////////////////////////// Spherical Light + Indirect Light /////////////////
            int new_i = H - 1 - i;
            Vector pix(cameraPosition[0]+j+0.5-W/2.,cameraPosition[1]+new_i+0.5-H/2.,cameraPosition[2]-f);
            Vector pixelColor(0.,0.,0.);
            for (int k = 0; k < 32; ++k){
                Vector randDir = pix + getColor.boxMuller(.8);
                // Ray cameraRay(cameraPosition,normalize(randDir-cameraPosition),0);
                // Ray cameraRay(cameraPosition,normalize(pix-cameraPosition),0);
                double time =  (((double) rand())/(RAND_MAX)); 
                Ray cameraRay = getColor.dephtOfField(randDir, aperture, distance, cameraPosition,time);
                pixelColor+= getColor.getColor(*lightSphere,lightOrigin,cameraRay,maxDepth,airRefractionIndex,false);
                

            }
            Vector L = pixelColor*(1./32.);
            image[(i*W + j) * 3 + 0] = min(int(pow(L[0],1./2.2)),255);
            image[(i*W + j) * 3 + 1] = min(int(pow(L[1],1./2.2)),255);
            image[(i*W + j) * 3 + 2] = min(int(pow(L[2],1./2.2)),255);
            /////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////// Point Light /////////////////
            // int new_i = H - 1 - i;
            // Vector pix(cameraPosition[0]+j+0.5-W/2.,cameraPosition[1]+new_i+0.5-H/2.,cameraPosition[2]-f);
            // Ray cameraRay(cameraPosition,normalize(pix-cameraPosition),0.);
            
            // Vector L = scene.getColorMultipleRays(1000,*lightSphere,lightOrigin,cameraRay,maxDepth,airRefractionIndex);
            // image[(i*W + j) * 3 + 0] = min(int(pow(L[0],1./2.2)),255);
            // image[(i*W + j) * 3 + 1] = min(int(pow(L[1],1./2.2)),255);
            // image[(i*W + j) * 3 + 2] = min(int(pow(L[2],1./2.2)),255);
            // image[(i*W + j) * 3 + 0] = min(int(L[0]),255);
            // image[(i*W + j) * 3 + 1] = min(int(L[1]),255);
            // image[(i*W + j) * 3 + 2] = min(int(L[2]),255);
            /////////////////////////////////////////////////////////////////////////////////////
            
        }
    }
    stbi_write_png("image.png", W, H, 3, &image[0], 0);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  
    cout << "Time taken by function: "
         << duration.count() << " milliseconds" << endl;
    return 0;
}

