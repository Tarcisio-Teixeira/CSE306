#pragma once
#include <stdio.h>
#include "../liblbfgs-master/include/lbfgs.h"
#include <vector>
#include "svjsaver.cpp"
#include "powerdiagram.cpp"

namespace FluidSimulation {
    static double waterCellVolume;
    static double airCellVolume;
    static Polygon polygon;
    static std::vector<Polygon> polygons;
    static double constantDistribution;
    static int numOfWaterCells;
    static int numOfAirCells;
    static double eps = 0.004;
    static double dt = 0.002;
    static Vector g( 0., -9.8, 0.);
    static double mass = 200.;
    std::vector<Vector> velocities;
    static void setCellVolumes(double waterVol, double airVol){
        waterCellVolume = waterVol;
        airCellVolume = airVol;
    }
    static void setNumberOfCells(int numAir, int numWater){
        numOfWaterCells = numWater;
        numOfAirCells = numAir;
        for (int j=0; j<numAir + numWater; ++j){
            Vector v(0.,0.,0.);
            velocities.push_back(v);
        }
    }
    static void setPolygon(Polygon polyg){
        polygon = polyg;
    }
    static void setConstantDistribution(double c){
        constantDistribution = c;
    }
    static void oneStepGMS(){
        for(int i= FluidSimulation::numOfAirCells; i< FluidSimulation::numOfAirCells+FluidSimulation::numOfWaterCells; i++){
            Vector Fi,Fi_spring;
            Polygon cell = FluidSimulation::polygons[i];
            Vector Xi = FluidSimulation::polygon.vertices[i];
            double inv = 1./(eps*eps);
            Fi_spring =  (cell.center()-Xi)*inv;
            Fi_spring[2] = 0.;
            Fi = mass* g + Fi_spring;
            FluidSimulation::polygon.vertices[i] += dt * velocities[i];
            velocities[i] += (dt/mass) * Fi;
        }

    }
    static lbfgsfloatval_t evaluate(void *instance, const lbfgsfloatval_t *weights, lbfgsfloatval_t *gradients, const int n, const lbfgsfloatval_t step){
        lbfgsfloatval_t fx = 0.0;
        gradients[FluidSimulation::numOfWaterCells]=0;
        FluidSimulation::polygon.updateWeights((double*) weights, n, 1);
        std::vector<Polygon> polygons = PowerDiagram::voronoiParallelLinearEnumeration(FluidSimulation::polygon);
        for (int i = 0;i < FluidSimulation::numOfWaterCells + FluidSimulation::numOfAirCells; ++i) {
            Polygon tmpPolyg = polygons[i];
            Vector Pi = FluidSimulation::polygon.vertices[i];
            lbfgsfloatval_t integralTerm = 0.;
            
            for (int j=0; j < tmpPolyg.vertices.size(); ++j){
                Vector vj = tmpPolyg.vertices[j];
                Vector vj1 = tmpPolyg.vertices[(j +1)% tmpPolyg.vertices.size()];
                double xk = vj1[0];
                double yk = vj1[1];
                double xkm1 = vj[0];
                double ykm1 = vj[1];
                integralTerm += ((xkm1*yk - xk*ykm1)*(
                                    xkm1*xkm1 + xkm1*xk + xk*xk + ykm1*ykm1 + ykm1*yk + yk*yk 
                                    - 4*(Pi[0]*(xkm1+xk) + Pi[1]*(ykm1+yk) ) + 6*Pi.normSquared()  
                                    )
                                )/12.;
            }
            integralTerm = std::abs(integralTerm);

            if(i>=FluidSimulation::numOfAirCells){
                int ind = i - FluidSimulation::numOfAirCells; //we are dealing with a water molecule
                fx += constantDistribution*integralTerm - constantDistribution* weights[ind]*tmpPolyg.area() + FluidSimulation::waterCellVolume * weights[ind];
            
                gradients[ind] = constantDistribution * tmpPolyg.area() - FluidSimulation::waterCellVolume ;

            } 
            else{
                int ind = FluidSimulation::numOfWaterCells;
                fx += constantDistribution*integralTerm - constantDistribution* weights[ind]*tmpPolyg.area() + (FluidSimulation::airCellVolume/ FluidSimulation::numOfAirCells)* weights[ind];
                gradients[ind] += constantDistribution * tmpPolyg.area() - (FluidSimulation::airCellVolume/ FluidSimulation::numOfAirCells) ;
            }

        }
        return -fx;
    }
    static int progress(void *instance,const lbfgsfloatval_t *x,const lbfgsfloatval_t *g,const lbfgsfloatval_t fx,const lbfgsfloatval_t xnorm,const lbfgsfloatval_t gnorm,const lbfgsfloatval_t step,int n,int k,int ls) {
        printf("Iteration %d:\n", k);
        printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
        printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
        printf("\n");
        return 0;
    }

    void run(double* weights, int N, int numberOfFrames){
        // int i, ret = 0;
        lbfgsfloatval_t fx;
        lbfgsfloatval_t *x = lbfgs_malloc(N);

        lbfgs_parameter_t param;
        if (x == NULL) {
            printf("ERROR: Failed to allocate a memory block for variables.\n");
            return;
        }
        /* Initialize the variables. */
        for (int i = 0;i < N;i ++) {
            x[i] = weights[i];
        }

        /* Initialize the parameters for the L-BFGS optimization. */
        lbfgs_parameter_init(&param);
        param.max_iterations = 500;
        

        for(int i=0;i<numberOfFrames;i++){
            /*
            Start the L-BFGS optimization; this will invoke the callback functions
            evaluate() and progress() when necessary.
            */
            int ret = lbfgs(N, x, &fx, evaluate, progress, NULL, &param);

            /* Report the result. */

            printf("L-BFGS optimization terminated with status code = %d\n", ret);
            std::cout << lbfgs_strerror(ret) << std::endl;
            printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
            FluidSimulation::polygon.updateWeights(x, N, 1);
            FluidSimulation::polygons = PowerDiagram::voronoiParallelLinearEnumeration(FluidSimulation::polygon);
            // save_voronoi_svg(FluidSimulation::polygons, "frames/fframe_test"+to_string(i)+".svg", FluidSimulation::polygon);
            save_frame(FluidSimulation::polygons,"frames/frame",FluidSimulation::numOfAirCells,i);
            oneStepGMS();
        }

        
        lbfgs_free(x);
    }    

    void fluidSimulation (int numOfAir, int numOfWater, int numberOfFrames){
        Polygon polygon;
        double radius = 0.3;
        double areaWater = radius*radius*M_PI;
        double areaAir = 1. - areaWater;
        double volAir = areaAir;
        double volWater = areaWater/numOfWater;
        double weights[numOfWater+1] = {};
        double x,y,z,w,sq;

        for (int i=0; i<numOfAir; ++i ){
            x = ((double) rand() )/RAND_MAX;
            y = ((double) rand() )/RAND_MAX;
            z =0;
            w = 0.5;
            Vector v(x,y,z,w,0.5);
            polygon.vertices.push_back(v);
        }

        polygon = PowerDiagram::centersOfVoronoiTesselation(polygon);

        for (int i=0; i<numOfWater; ++i ){
            double r = (double) rand()/RAND_MAX; 
            r = radius*sqrt(r);
            double theta = (double) rand()/RAND_MAX * 2*M_PI;
            x = r*cos(theta); 
            y = r*sin(theta);
            z =0;
            w = 0.5;
            weights[i] = 0.5;
            Vector v(0.5 + x,0.5 + y,z,w,0.5);
            polygon.vertices.push_back(v);
        }
        weights[numOfWater] = 0.5;

        FluidSimulation::setConstantDistribution(1.);
        FluidSimulation::setPolygon(polygon);
        FluidSimulation::setNumberOfCells(numOfAir, numOfWater);
        FluidSimulation::setCellVolumes(volWater, volAir);
        FluidSimulation::run(weights, numOfWater +1, numberOfFrames);
    }



}