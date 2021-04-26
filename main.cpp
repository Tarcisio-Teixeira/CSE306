#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>
 
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../stb-master/stb_image_write.h"
 
#define STB_IMAGE_IMPLEMENTATION
#include "../stb-master/stb_image.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <omp.h>
#include "classes/vector.cpp"
#include "classes/scene.cpp"
#include "classes/ray.cpp"
#include "classes/sphere.cpp"
#include "classes/intersection.cpp"
#include "classes/getcolor.cpp"
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
    int maxDepth = 5;
    double airRefractionIndex = 1.0;
    double aperture = 2.8;
    double distance = 55.;
    GetColor getColor;
    Sphere centerTestSphere(Vector(0.,0.,0.),Vector(1.,1.,1.),10.,false,-1, false, 0,Vector(0.,9.8,0.));//white
    Sphere leftTestSphere(Vector(-10.,0.,15.),Vector(1.,1.,1.),10.,false,1.5, false, 0,Vector(0.,0.,0.));//refract
    Sphere centralSphere(Vector(10.,0.,-15.),Vector(1.,1.,1.),10.,true,-1, false, 0,Vector(0.,0.,0.));//mirror
    Sphere internalToCentralSphere(Vector(20.,0.,0.), Vector(1.,1.,1.), 9.5, false, 1.0 , false, 0,Vector(0.,0.,0.));
    Sphere topSphere(Vector(0.,1000.,0.),Vector(1.,0.,0.),940.,false,-1., false, 0,Vector(0.,0.,0.));
    Sphere leftSphere(Vector(-1000.,0.,0.),Vector(0.,1.,1.),940.,false,-1., false, 0,Vector(0.,0.,0.));
    Sphere bottomSphere(Vector(0.,-1000.,0.),Vector(0.,0.,1.),990.,false,-1., false, 0,Vector(0.,0.,0.));
    Sphere rightSphere(Vector(1000.,0.,0.),Vector(1.,1.,0.),940.,false,-1., false, 0,Vector(0.,0.,0.));
    Sphere backSphere(Vector(0.,0.,1000.),Vector(1.,0.,1.),940.,false,-1., false, 0,Vector(0.,0.,0.));
    Sphere frontSphere(Vector(0.,0.,-1000.),Vector(0.,1.,0.),940.,false,-1., false, 0,Vector(0.,0.,0.));
    
    Vector lightOrigin(-10.,20.,40.);
    Sphere lightSphere(lightOrigin,Vector(1.,1.,1.),5.,false,-1., true, I,Vector(0.,0.,0.));
    Vector cameraPosition(0,0,55);
    vector<Sphere> spheres;
    spheres.push_back(leftSphere);
    spheres.push_back(rightSphere);
    spheres.push_back(centralSphere);
    spheres.push_back(topSphere);
    spheres.push_back(bottomSphere);
    spheres.push_back(frontSphere);
    spheres.push_back( backSphere);
    spheres.push_back(lightSphere);
    // spheres.push_back(internalToCentralSphere);
    spheres.push_back(leftTestSphere);
    spheres.push_back(centerTestSphere);
    Scene scene(spheres, airRefractionIndex);
    
    


    std::vector<unsigned char> image(W*H * 3, 0);
    #pragma omp parallel for schedule(dynamic,1)
    for (int i = 0; i < H; i++) {
        GetColor g;
        for (int j = 0; j < W; j++) {
            int new_i = H - 1 - i;
            Vector pix(cameraPosition[0]+j+0.5-W/2.,cameraPosition[1]+new_i+0.5-H/2.,cameraPosition[2]-f);
            Vector pixelColor(0.,0.,0.);
            for (int k = 0; k < 32; ++k){
                Vector randDir = pix + g.boxMuller(1);
                // Ray cameraRay(cameraPosition,normalize(randDir-cameraPosition));
                double time =  ((double) rand()/(RAND_MAX)); 
                Ray cameraRay = g.dephtOfField(randDir, aperture, distance, cameraPosition,time);
                pixelColor+= g.getColor(lightSphere,lightOrigin,cameraRay,maxDepth,scene,airRefractionIndex,false);
            }
            Vector L = pixelColor*(1./32.);
            image[(i*W + j) * 3 + 0] = min(int(pow(L[0],1./2.2)),255);
            image[(i*W + j) * 3 + 1] = min(int(pow(L[1],1./2.2)),255);
            image[(i*W + j) * 3 + 2] = min(int(pow(L[2],1./2.2)),255);
            
        }
    }
    stbi_write_png("image.png", W, H, 3, &image[0], 0);
    return 0;
}

