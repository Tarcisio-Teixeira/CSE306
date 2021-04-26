
#include <random>
#include <cmath>
#include <iostream>

using namespace std;

static default_random_engine engine(10);
static uniform_real_distribution<double> uniform(0,1);

void boxMuller(double stdev, double& x, double& y){
    double r1 = uniform(engine);
    double r2 = uniform(engine);
    x = sqrt(-2 * log(r1)) * cos(2 * M_PI * r2) * stdev;
    y = sqrt(-2 * log(r1)) * sin(2 * M_PI * r2) * stdev;
}
double gaussianPdf(double x, double y, double z){
    double cstTerm = pow((1./sqrt(2*M_PI)),3);
    double expTerm = exp(-(x*x+y*y+z*z)/2);
    return cstTerm*expTerm;

}
double f(double x, double y, double z){
    return cos(x*y*z);
}
int main(){
    double x;
    double y;
    double z;
    double w;
    double res=0;
    #pragma omp parallel for
    for (int i=0; i<10000; ++i){
        boxMuller(1., x, y);
        boxMuller(1., z, w);
        res+=f(x,y,z)/gaussianPdf(x,y,z);
    }
    cout << res/10000 << endl;
}