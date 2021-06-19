
#include "voronoidiagram.cpp"
#include "powerdiagram.cpp"
#include "fluidsimulation.cpp"
#include <chrono>
#include "optimaltransportsimulation.cpp"

int main(){
    FluidSimulation::fluidSimulation(250,70,500);
    OptimalTransportSimulation::optimalTransportSimulation(1000);
    return 0;
}
