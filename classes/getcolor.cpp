#pragma once
#include "geometry.cpp"
#include "vector.cpp"
#include "intersection.cpp"
#include "ray.cpp"
#include "scene.cpp"
#include <cmath>
#include <vector>
#include <random>
#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
using namespace std;
//static uniform_real_distribution<double> uniform(0,1);
//static default_random_engine engine(10);
class GetColor {
    public:
        vector<double> last_refr_index{1};
        int tryLeave = 0;
        GetColor(){}
        Vector boxMuller(double stdev){
            double r1 = ((double) rand()/(RAND_MAX));
            double r2 = ((double) rand()/(RAND_MAX));
            double x = sqrt(-2 * log(r1)) * cos(2 * M_PI * r2) * stdev;
            double y = sqrt(-2 * log(r1)) * sin(2 * M_PI * r2) * stdev;
            return Vector(x,y);

        }
        Vector randomCos(const Vector &N){
            double r1 = ((double) rand()/(RAND_MAX));
            double r2 = ((double) rand()/(RAND_MAX));
            double x = cos(2*M_PI*r1)*sqrt(1-r2);
            double y = sin(2*M_PI*r1)*sqrt(1-r2);
            double z = sqrt(r2);
            double Nx = N[0];
            double Ny = N[1];
            double Nz = N[2];
            Vector T1;
            if (abs(Nx) <= abs(Nz) ){
                if (abs(Nx)<=abs(Ny)){
                    T1 = Vector(0,Nz,-Ny);	
                }
                else {
                    T1 = Vector(Nz,0,-Nx);	
                }
            }
            else{
                if (abs(Nz)<=abs(Ny)){
                    T1 = Vector(Ny,-Nx,0);	
                } 
                else {	
                    T1 = Vector(Nz,0,-Nx);	
                }
            }
            Vector T2 = cros(N,T1);
            return T1*x + T2*y + N*z;
        }
        // Vector lambertian(double I, Vector s, Intersection inter, Scene scene){
        //     Vector rho = inter.sphere.albedo;
        //     Vector p = inter.point;
        //     Vector n = inter.unitNormal;
        //     double d = (s-p).norm();
        //     double vps = 1;
        //     double c = (I/ ( 4 * M_PI*d*M_PI*d ) )* max(dot( n,normalize(s-p) ),0.);
        //     Vector p_ = p+n*0.000001;
        //     Ray testRay(p_, normalize(s-p));
        //     Intersection testInter = intersect_scene(testRay,scene);
        //     if (testInter.intersects){
        //         double dTest = (testInter.point-p).norm();
        //         if (dTest!=0. && dTest < d){
        //             vps = 0;
        //         }
        //     }
        //     return rho*(c*vps);
        // }
        Ray generateReflectedRay(Ray incomingRay, Intersection inter){
            Ray reflectedRay(  inter.point+inter.unitNormal*0.000001, 
                                normalize(incomingRay.direction - inter.unitNormal*(2*dot(incomingRay.direction,inter.unitNormal))),
                                incomingRay.time 
                            );
            return reflectedRay;
        }
        void correctParamsRayDirection(Intersection& inter, double& n2, Ray ray){
            if (dot(ray.direction,inter.unitNormal)>0){
                n2 = last_refr_index.back();
                inter.unitNormal = inter.unitNormal*(-1.);
                tryLeave = 1;
            } else {
                tryLeave = 0;
            }
        }
        Ray generateRefractedRay(Ray incomingRay, Intersection inter, double n1, double& n2){
            Vector n= inter.unitNormal;
            double dir_dot_unit = dot(incomingRay.direction,n);
            double delta = 1 - ((n1/n2)*(n1/n2)*(1-(dir_dot_unit*dir_dot_unit)));
            if (delta<0){
                n2 = n1;
                return generateReflectedRay(incomingRay, inter);
            }
            if (tryLeave==1){
                last_refr_index.pop_back();
            } else {
                last_refr_index.push_back(n1);
            }
            Vector p_ = inter.point - n*0.000001;
            Vector omegaT = (incomingRay.direction - n*dir_dot_unit)*(n1/n2);
            Vector omegaN = n*(-sqrt(delta));
            Ray refractedRay(p_,normalize(omegaT+omegaN),incomingRay.time);
            return refractedRay;
            
        }
        Vector getColor(Sphere lightSphere, Vector s, const Ray& ray, int rayDepth, Scene scene, double curRefractionIndex, bool lastBounceDiffuse){
            
            if (rayDepth<0){
                return Vector(0.,0.,0.);
            }
            Intersection inter = intersect_scene(ray,scene);
            if (inter.intersects){
                Geometry* obj_ = inter.object;
                if (obj_->type == 0){
                    Sphere* obj = (Sphere*) obj_;
                    if (obj->isLight){
                        if (lastBounceDiffuse) return Vector(0.,0.,0.);
                        else return Vector(1.,1.,1.)*(obj->lightIntensity/(4*M_PI*M_PI*obj->radius*obj->radius));
                    }
                    else if (obj->mirror){
                        return getColor(lightSphere, s, generateReflectedRay(ray, inter), rayDepth-1, scene, curRefractionIndex, false);
                    }
                    else if (obj->refractionIndex >0){
                        double n1 = curRefractionIndex;
                        double n2 = obj->refractionIndex;
                        correctParamsRayDirection(inter, n2, ray);
                        Vector n= inter.unitNormal;
                        double k0 = (n1-n2)*(n1-n2)/((n1+n2)*(n1+n2));
                        double dir_dot_unit = dot(ray.direction,n);
                        double var = ( 1 - abs(dir_dot_unit) );
                        double R = k0 + (1-k0)*var*var*var*var*var;
                        double u = ((double) rand()/(RAND_MAX));
                        if (u < R){
                            return getColor(lightSphere, s, generateReflectedRay(ray, inter), rayDepth-1, scene, n1,false);
                        } else {
                            return getColor(lightSphere, s, generateRefractedRay(ray,inter,n1,n2), (rayDepth-1), scene, n2,false);
                        }
                    } 
                    else {
                        Vector Lo(0.,0.,0.);
                        Vector xprime = randomPointOnLightSphere(inter.point,lightSphere);
                        Vector Nprime = normalize( xprime - lightSphere.center);
                        Vector omega_i = normalize( xprime - inter.point );
                        double visibility = 1 ;
                        Vector p_ = inter.point+inter.unitNormal*0.000001;
                        Ray testRay(p_, omega_i, ray.time);
                        Intersection testInter = intersect_scene(testRay,scene);
                        if (testInter.intersects){
                            double dTest = (testInter.point-inter.point).norm();
                            if ( (testInter.object->type==1)||(! ((Sphere*)testInter.object)->isLight) ){
                                visibility = 0;
                            }
                        }
                        double R = obj->radius;
                        double pdf = dot ( Nprime , normalize( inter.point - lightSphere.center) ) / ( M_PI*R*R );
                        Lo = obj->albedo*(lightSphere.lightIntensity / ( 4* M_PI*M_PI*M_PI*R*R )); 
                        Lo = Lo* (visibility * std::max (dot(inter.unitNormal , omega_i ) , 0. ));
                        Lo = Lo*(std::max( dot ( Nprime , omega_i*(-1) ) , 0.) / ( (xprime - inter.point).norm()*(xprime - inter.point).norm()* pdf )) ;
                        Ray randomRay(inter.point, normalize(randomCos(inter.unitNormal)), ray.time );
                        Lo+=obj->albedo*getColor(lightSphere,s,randomRay,rayDepth-1,scene,curRefractionIndex, true);
                        return Lo;
                    }
                }
                else if (obj_->type==1){
                    TriangleMesh* obj = (TriangleMesh*) obj_;
                    Vector Lo(0.,0.,0.);
                    Vector xprime = randomPointOnLightSphere(inter.point,lightSphere);
                    Vector Nprime = normalize( xprime - lightSphere.center);
                    Vector omega_i = normalize( xprime - inter.point );
                    double visibility = 1 ;
                    Vector p_ = inter.point+inter.unitNormal*0.000001;
                    Ray testRay(p_, omega_i, ray.time);
                    Intersection testInter = intersect_scene(testRay,scene);
                    if (testInter.intersects){
                        double dTest = (testInter.point-inter.point).norm();
                        if ( (testInter.object->type==1)||(! ((Sphere*)testInter.object)->isLight) ){
                            visibility = 0;
                        }
                    }
                    Vector rho = obj->albedo;
                    Vector p = inter.point;
                    Vector n = inter.unitNormal;
                    double d = (xprime - inter.point).norm();
                    double c = (lightSphere.lightIntensity/ ( 4 * M_PI*d*M_PI*d ) )* max(dot( n,normalize(s-p) ),0.);
                    // double pdf = dot ( Nprime , normalize( inter.point - lightSphere.center) ) / ( M_PI*R*R );
                    // Lo = obj->albedo*(lightSphere.lightIntensity / ( 4* M_PI*M_PI*M_PI*R*R )); 
                    // Lo = Lo* (visibility * std::max (dot(inter.unitNormal , omega_i ) , 0. ));
                    // Lo = Lo*(std::max( dot ( Nprime , omega_i*(-1) ) , 0.) / ( (xprime - inter.point).norm()*(xprime - inter.point).norm()* pdf )) ;
                    Lo = rho*(c*visibility);
                    Ray randomRay(inter.point, normalize(randomCos(inter.unitNormal)), ray.time );
                    Lo+=obj->albedo*getColor(lightSphere,s,randomRay,rayDepth-1,scene,curRefractionIndex, true);
                    return Lo;
                }
            }
            return Vector(0.,0.,0.);

        }
        Vector randomPointOnLightSphere(Vector x, Sphere lightSphere){
            Vector D = normalize(x-lightSphere.center);
            Vector V = randomCos(D);
            return V*lightSphere.radius + lightSphere.center;
        }
        Ray dephtOfField(Vector randDir, double aperture, double distance, Vector cameraPosition, double time){
            Vector u = randDir - cameraPosition;
            Vector P = cameraPosition + u*(distance/abs(u[2]));
            double r = ((double) rand()/(RAND_MAX)); 
            r = sqrt(r)*aperture;
            double theta = ((double) rand()/(RAND_MAX));
            theta = theta*2*M_PI;
            Vector Qprime(r*cos(theta),r*sin(theta), cameraPosition[2]);
            return Ray(Qprime, normalize(P-Qprime),time);
        }
        // Vector getColorMultipleRays(int numberOfRays, double I, Vector s, const Ray& ray, int rayDepth, Scene scene, double curRefractionIndex){
        //     Vector totalPixelColor;
        //     for (int i = 0; i<numberOfRays; ++i){
        //         totalPixelColor += getColor(I, s, ray, rayDepth, scene, curRefractionIndex);
        //     }
        //     return totalPixelColor*(1./numberOfRays);
        // }
};
