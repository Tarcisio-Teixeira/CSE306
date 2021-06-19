#pragma once
#include <stdio.h>
#include "../liblbfgs-master/include/lbfgs.h"
// #include "../liblbfgs-master/lib/arithmetic_ansi.h"
// #include "../liblbfgs-master/lib/lbfgs.c"
#include <vector>
#include "svjsaver.cpp"
#include "powerdiagram.cpp"
#include <random>


namespace OptimalTransportSimulation {
    static double* lambdas;
    static Polygon polygon;
    static double constantDistribution;
    static void setLambdas(double* lambds){
        lambdas = lambds;
    }
    static void setPolygon(Polygon polyg){
        polygon = polyg;
    }
    static void setConstantDistribution(double c){
        constantDistribution = c;
    }
    static lbfgsfloatval_t evaluate(void *instance, const lbfgsfloatval_t *weights, lbfgsfloatval_t *gradients, const int n, const lbfgsfloatval_t step){
        // printf("creating power cells\n");
        lbfgsfloatval_t fx = 0.0;

        OptimalTransportSimulation::polygon.updateWeights((double*) weights, n);
        std::vector<Polygon> polygons = PowerDiagram::voronoiParallelLinearEnumeration(OptimalTransportSimulation::polygon);
        int counter=0;
        for (int i = 0;i < n; ++i) {
            Polygon tmpPolyg = polygons[i];
            Vector Pi = OptimalTransportSimulation::polygon.vertices[i];
            // printf("neww cell\n");
            lbfgsfloatval_t integralTerm = 0.;
            if (tmpPolyg.vertices.size() ==0) counter ++;
            
            for (int j=0; j < tmpPolyg.vertices.size(); ++j){
                
                Vector vj = tmpPolyg.vertices[j];
                Vector vj1 = tmpPolyg.vertices[(j +1)% tmpPolyg.vertices.size()];
                // printf("%f,%f\n", vj[0],vj[1]);
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
            integralTerm = abs(integralTerm);
            fx += constantDistribution*integralTerm - constantDistribution* weights[i]*tmpPolyg.area() + lambdas[i]* weights[i];
            // printf("%f",fx);
            
            gradients[i] = constantDistribution * tmpPolyg.area() - lambdas[i];
            // printf("%f\n",gradients[i]);

        }
        printf("Number of empty cells is %d\n", counter);

        return -fx;
    }
    static int progress(
        void *instance,
        const lbfgsfloatval_t *x,
        const lbfgsfloatval_t *g,
        const lbfgsfloatval_t fx,
        const lbfgsfloatval_t xnorm,
        const lbfgsfloatval_t gnorm,
        const lbfgsfloatval_t step,
        int n,
        int k,
        int ls
        )
    {
        printf("Iteration %d:\n", k);
        printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx, x[0], x[1]);
        printf("  xnorm = %f, gnorm = %f, step = %f\n", xnorm, gnorm, step);
        printf("\n");
        return 0;
    }

    void run(double* weights, int N){
        int i, ret = 0;
        lbfgsfloatval_t fx;
        lbfgsfloatval_t *x = lbfgs_malloc(N);

        lbfgs_parameter_t param;
        // if (x == NULL) {
        //     printf("ERROR: Failed to allocate a memory block for variables.\n");
        //     return 1;
        // }
        /* Initialize the variables. */
        for (i = 0;i < N;i ++) {
            x[i] = weights[i];
        }
        /* Initialize the parameters for the L-BFGS optimization. */
        lbfgs_parameter_init(&param);
        // param.linesearch = OptimalTransportSimulation_LINESEARCH_BACKTRACKING;
        /*
            Start the L-BFGS optimization; this will invoke the callback functions
            evaluate() and progress() when necessary.
        */
        param.max_iterations = 500;
        ret = lbfgs(N, x, &fx, evaluate, progress, NULL, &param);
        std::vector<Polygon> polygons = PowerDiagram::voronoiParallelLinearEnumeration(OptimalTransportSimulation::polygon);
        save_voronoi_svg(polygons, "voronois/optimaltransport.svg", OptimalTransportSimulation::polygon);
        /* Report the result. */
        printf("L-BFGS optimization terminated with status code = %d\n", ret);
        printf("  fx = %f, x[0] = %f, x[1] = %f\n", fx,x[0], x[1]);
        lbfgs_free(x);
    }    
    void optimalTransportSimulation(int numOfVertices ){
        std::default_random_engine generator;
        std::normal_distribution<double> distribution(0.,1.0);
        double lambdas[numOfVertices] = {};
        double weights[numOfVertices] = {};
        int counter = 0;
        double x,y,z,w,sq;
        
        while (counter < numOfVertices){
            weights[counter] = std::abs(distribution(generator));
            // weights[counter] = 0.5;
            // lambdas[counter] = 0.5;
            if (weights[counter] < 1.){
                counter++;
            }
        }
        
        
        Polygon polygon;
        double maxEl = *std::max_element(weights, weights+numOfVertices);
        
        double tot=0.;
        for (int i=0; i<numOfVertices; ++i ){
            x = ((double) rand() )/RAND_MAX;
            y = ((double) rand() )/RAND_MAX;
            z =0;
            lambdas[i] = exp(  -((x-0.5)*(x-0.5) +  (y-0.5)*(y-0.5) )  /(0.02) );
            tot += lambdas[i];
            w = weights[i];
            sq = sqrt(maxEl - w);
            Vector v = Vector(x,y,z,0.,sq);
            polygon.vertices.push_back(v);
        }

        for (int i=0; i<numOfVertices; ++i ){
            lambdas[i] = lambdas[i]/tot;
        }

        polygon.updateWeights(weights, numOfVertices);

        // for (int i=0; i<numOfVertices; ++i ){
        //     std::cout << polygon.vertices[i].w<<std::endl;
        // }
        OptimalTransportSimulation::setConstantDistribution(1.);
        OptimalTransportSimulation::setPolygon(polygon);
        OptimalTransportSimulation::setLambdas(lambdas);
        OptimalTransportSimulation::run(weights, numOfVertices);
    }
}