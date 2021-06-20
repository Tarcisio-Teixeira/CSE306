
#include "voronoidiagram.cpp"
#include "powerdiagram.cpp"
#include "fluidsimulation.cpp"
#include <chrono>
#include "optimaltransportsimulation.cpp"

int main(){
    auto start = std::chrono::high_resolution_clock::now();
    // PowerDiagram::createPowerDiagram(10000, "voronois/powerDiagram.svg" );
    // VoronoiDiagram::createVoronoiDiagram(10000, false, "voronois/voronoiDiagram.svg");
    // VoronoiDiagram::createVoronoiDiagram(1000, false, "voronois/voronoiDiagram1000.svg");
    // VoronoiDiagram::createVoronoiDiagram(1000, true, "voronois/centeredVoronoiDiagram.svg");
    FluidSimulation::fluidSimulation(250,70,1000);
    // OptimalTransportSimulation::optimalTransportSimulation(2000);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Time taken by function: "
         << duration.count() << " ms." << std::endl;
    return 0;
}
