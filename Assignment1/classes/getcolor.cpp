#pragma once
#include "scene.cpp"
// #include <cmath>
#include <vector>
#include <random>

using namespace std;
// static uniform_real_distribution<double> uniform(0,1);
// static default_random_engine engine(10);
class GetColor {
    public:
        // vector<double> last_refr_index{1};
        // int tryLeave = 0;
        Scene scene;
        GetColor(Scene s){scene = s;}
        Vector boxMuller(const double& stdev){
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
        Ray generateReflectedRay(const Ray& incomingRay, const Intersection& inter){
            Vector dir = normalize(incomingRay.direction - inter.unitNormal*(2*dot(incomingRay.direction,inter.unitNormal)));
            Ray reflectedRay(  inter.point+inter.unitNormal*0.000001, dir,incomingRay.time);
            return reflectedRay;
        }
        void correctParamsRayDirection(Intersection& inter, double& n2, const Ray& ray){
            if (dot(ray.direction,inter.unitNormal)>0){            
                if (n2 == 1.){
                    n2 = 1.5;
                }
                else {
                    n2 = 1.;
                }
                inter.unitNormal = inter.unitNormal*(-1.);
            }
        }
        Ray generateRefractedRay(const Ray& incomingRay, Intersection& inter,const double& n1, double& n2){
            Vector n= inter.unitNormal;
            double dir_dot_unit = dot(incomingRay.direction,n);
            double delta = 1. - ((n1/n2)*(n1/n2)*(1.-(dir_dot_unit*dir_dot_unit)));
            if (delta<0){
                // std::cout << "here" << std::endl;
                // n2 = n1;
                return generateReflectedRay(incomingRay, inter);
            }
            // inter.unitNormal = inter.unitNormal*(-1.);
            Vector p_ = inter.point - n*0.000001;
            Vector omegaT = (incomingRay.direction - n*dir_dot_unit)*(n1/n2);
            Vector omegaN = n*(-sqrt(delta));
            Vector dir = normalize(omegaT+omegaN);
            Ray refractedRay(p_,dir,incomingRay.time);
            return refractedRay;
            
        }
        Vector randomPointOnLightSphere(const Vector& x, const Sphere& lightSphere){
            Vector D = normalize(x-lightSphere.center);
            Vector V = randomCos(D);
            return V*lightSphere.radius + lightSphere.center;
        }
        Vector getColor(const Sphere& lightSphere, const Vector& s, const Ray& ray, const int& rayDepth, const double& curRefractionIndex,const bool& lastBounceDiffuse){
            if (rayDepth<0){
                return Vector(0.,0.,0.);
            }
            Intersection inter = scene.intersect(ray);
            if (inter.intersects){
                Geometry* obj = scene.scene[inter.objectId];
                if (obj->isLight){
                    if (lastBounceDiffuse) return Vector(0.,0.,0.);
                    else return Vector(1.,1.,1.)*(obj->lightIntensity/(4*M_PI*M_PI*lightSphere.radius*lightSphere.radius));
                }
                else if (obj->mirror){
                    return getColor(lightSphere, s, generateReflectedRay(ray, inter), rayDepth-1, curRefractionIndex, false);
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
                    // double u = uniform (engine) ;
                    if (u < R){
                        return getColor(lightSphere, s, generateReflectedRay(ray, inter), rayDepth-1, n1, false);
                    } else {
                        Ray r =  generateRefractedRay(ray,inter,n1,n2);
                        return getColor(lightSphere, s, generateRefractedRay(ray,inter,n1,n2), (rayDepth-1),  n2, false);
                    }
                } 
                else {
                    /////////////// Spherical Light + Indirect Light //////////////////////
                    Vector Lo(0.,0.,0.);
                    Vector xprime = randomPointOnLightSphere(inter.point,lightSphere);
                    Vector Nprime = normalize( xprime - lightSphere.center);
                    Vector omega_i = normalize( xprime - inter.point );
                    
                    int visibility = 1 ;
                    Vector p_ = inter.point+inter.unitNormal*0.000001;
                    Ray testRay(p_, omega_i, ray.time);
                    Intersection testInter = scene.intersect(testRay);
                    if (testInter.intersects == 1){
                        double dTest = (testInter.point-inter.point).norm();
                        if ( !(scene.scene[testInter.objectId]->isLight) ){
                            visibility = 0;
                        }
                    }
                    else visibility = 0;
                    double R = lightSphere.radius;
                    double pdf = dot ( Nprime , normalize( p_ - lightSphere.center) ) / ( M_PI*R*R );
                    Lo = inter.albedo*(lightSphere.lightIntensity / ( 4* M_PI*M_PI*M_PI*R*R )); 
                    
                    Lo = (Lo* visibility )* (std::max(dot(inter.unitNormal , omega_i ) , 0. ));
                    
                    Lo = (Lo*std::max( dot ( Nprime , omega_i*(-1) ) , 0.)) *(1/( (xprime - p_).norm()*(xprime - p_).norm()* pdf )) ;
                    
                    Ray randomRay(p_, normalize(randomCos(inter.unitNormal)), ray.time );
                    Lo+=inter.albedo*getColor(lightSphere,s,randomRay,rayDepth-1,curRefractionIndex, true);
                    return Lo;
                    ///////////////////////////////////////////////////////////////////////

                    /////////////////////// point light ///////////////////////////////////
                    // Vector Lo(0.,0.,0.);
                    // Vector p_ = inter.point+inter.unitNormal*0.000001;
                    // Lo += lambertian(lightSphere.lightIntensity,lightSphere.center,inter,ray.time);
                    // Ray randomRay(p_, normalize(randomCos(inter.unitNormal)), ray.time );
                    // Lo+=inter.albedo*getColor(lightSphere,s,randomRay,rayDepth-1,curRefractionIndex, true);
                    // return Lo;
                    ///////////////////////////////////////////////////////////////////////
                }
            }
            
            return Vector(0.,0.,0.);

        }
        Vector lambertian(double I, Vector s, Intersection inter, double time){
            Vector rho = inter.albedo;
            Vector p = inter.point;
            Vector n = inter.unitNormal;
            double d = (s-p).norm();
            double vps = 1;
            double c = (I/ ( 4 * M_PI*d*M_PI*d ) )* max(dot( n,normalize(s-p) ),0.);
            Vector p_ = p+n*0.000001;
            Ray testRay(p_, normalize(s-p),time);
            Intersection testInter = scene.intersect(testRay);
            if (testInter.intersects){
                double dTest = (testInter.point-p).norm();
                if (dTest!=0. && dTest < d){
                    vps = 0;
                }
            }
            return rho*(c*vps);
        }
        Vector getColorMultipleRays(int numberOfRays, const Sphere& lightSphere, Vector s, const Ray& ray, int rayDepth, double curRefractionIndex){
            Vector totalPixelColor;
            for (int i = 0; i<numberOfRays; ++i){
                totalPixelColor += getColor(lightSphere, s, ray, rayDepth,curRefractionIndex, false);
            }
            return totalPixelColor*(1./numberOfRays);
        }
        Ray dephtOfField(const Vector& randDir, const double& aperture, const double& distance, const Vector& cameraPosition, const double& time){
            Vector u = randDir - cameraPosition;
            Vector P = cameraPosition + u*(distance/abs(u[2]));
            double r = ((double) rand()/(RAND_MAX)); 
            r = sqrt(r)*aperture;
            double theta = ((double) rand()/(RAND_MAX));
            theta = theta*2*M_PI;
            Vector Qprime(r*cos(theta),r*sin(theta), cameraPosition[2]);
            return Ray(Qprime, normalize(P-Qprime),time);
        }
};
