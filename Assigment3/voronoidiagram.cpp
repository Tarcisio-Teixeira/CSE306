#pragma once
#include <map>
#include <string>
#include <iostream>
#include "svjsaver.cpp"
#include <mutex>
#include <algorithm>

namespace VoronoiDiagram {
        Vector intersectWithBisector(Vector vertexA, Vector vertexB, Vector curPoint, Vector cmpPoint){
            // returns the intersection of (vertexA, vertexB) with the bisector of (curPoint,cmpPoint)
            Vector M = (curPoint+cmpPoint)*(0.5);
            double t = dot(M-vertexA,curPoint-cmpPoint)/dot(vertexB-vertexA,curPoint-cmpPoint);
            Vector P = vertexA + t*(vertexB-vertexA);
            return P;
        };
        bool isInsideBisector(Vector point, Vector curPoint, Vector cmpPoint){
            Vector M = (curPoint+cmpPoint)*(0.5);
            return dot(point-M,cmpPoint-curPoint)<0;
        };
        Polygon sutherlandHodgmanAux(Vector curPoint, Vector cmpPoint, Polygon polygon, double& optDistance ){
            Polygon outPolygon;
            optDistance = 0;
            for (int i=0; i<polygon.vertices.size(); ++i){
                Vector curVertex = polygon.vertices[i];
                Vector prevVertex = polygon.vertices[(i > 0)?(i - 1):(polygon.vertices.size()-1)];
                Vector intersection = intersectWithBisector(prevVertex, curVertex, curPoint, cmpPoint);
                if (isInsideBisector(curVertex,curPoint, cmpPoint)){
                    if (!isInsideBisector(prevVertex,curPoint, cmpPoint)){
                        outPolygon.vertices.push_back(intersection);
                        optDistance = std::max(optDistance,(curPoint-intersection).norm() );

                    }
                    outPolygon.vertices.push_back(curVertex);
                    optDistance = std::max(optDistance,(curPoint-curVertex).norm() );
                }
                else if (isInsideBisector(prevVertex,curPoint, cmpPoint)){
                    outPolygon.vertices.push_back(intersection);
                    optDistance = std::max(optDistance,(curPoint-intersection).norm() );
                }
            }
            return outPolygon;
        };
        Polygon computeBoundingBox(Polygon polygon){
            Polygon result;
            // result.vertices = polygon.vertices;
            result.vertices.push_back(Vector(0.,0.,0.));
            result.vertices.push_back(Vector(0.,1.,0.));
            result.vertices.push_back(Vector(1.,1.,0.));
            result.vertices.push_back(Vector(1.,0.,0.));
            return result;
        };
        double computeOptimalDistance(Vector p){
            return std::max(
                            std::max(
                                (p-Vector(0.,0.,0.)).norm(),
                                (p-Vector(0.,1.,0.)).norm()
                            ),
                            std::max(
                                (p-Vector(1.,0.,0.)).norm(),
                                (p-Vector(1.,1.,0.)).norm()
                            )
                    );
        };
        std::vector<Polygon> voronoiParallelLinearEnumeration(Polygon polygon){
            std::vector<Polygon> result(polygon.vertices.size());
            Polygon boundingBox = computeBoundingBox(polygon);
            std::mutex m;
            #pragma omp parallel for 
            for (int i=0; i<polygon.vertices.size(); ++i){
                Vector p = polygon.vertices[i];
                Polygon tmpPolygon = boundingBox;
                // k-nearest
                std::vector<Vector> points = polygon.vertices;
                // std::sort(points.begin(), points.end(), [p](const Vector& v1, const Vector& v2) {return (v1-p).norm() < (v2-p).norm();} );
                double optimalDistance = computeOptimalDistance(p);
                for (int j = 0; j < points.size(); ++j ){
                    Vector q = points[j];
                    // if ((q-p).norm() > 2*optimalDistance){
                    //     break;
                    // }
                    if (p[0]==q[0] && p[1]==q[1] && p[2]==q[2]) continue; 
                    else{
                        tmpPolygon=sutherlandHodgmanAux(p,q,tmpPolygon,std::ref(optimalDistance));
                    }
                }
                // m.lock();
                result[i] = tmpPolygon;
                // m.unlock(); 
            }
            return result;
        };
        Polygon computeCenterOfVoronoiDiagram(std::vector<Polygon> polygons){
            Polygon polygon;
            for (int i=0; i <polygons.size(); ++i){
                polygon.vertices.push_back(polygons[i].center());
            }
            return polygon;
        };

        std::vector<Polygon> centroidalVoronoiTesselation(Polygon polygon, std::string filename, int iterations = 100){
            std::vector<Polygon> result = voronoiParallelLinearEnumeration(polygon);
            Polygon tmpPolygon=computeCenterOfVoronoiDiagram(result);
            for (int i=1; i < iterations; ++i){
                
                result = voronoiParallelLinearEnumeration(tmpPolygon);
                tmpPolygon = computeCenterOfVoronoiDiagram(result);
            }
            save_voronoi_svg(result, filename ,tmpPolygon);
            return result;
        };
        Polygon centersOfVoronoiTesselation(Polygon polygon, int iterations = 50){
                

            std::vector<Polygon> result = voronoiParallelLinearEnumeration(polygon);
            Polygon tmpPolygon=computeCenterOfVoronoiDiagram(result);
            for (int i=1; i < iterations; ++i){
                result = voronoiParallelLinearEnumeration(tmpPolygon);
                tmpPolygon = computeCenterOfVoronoiDiagram(result);
            }
            // save_voronoi_svg(result, "inter.svg",tmpPolygon);
            return tmpPolygon;
        };
        void createVoronoiDiagram (int numOfVertices, bool isCentered, std::string filename){
            double x,y,z,w,sq;
            Polygon polygon;
            for (int i=0; i<numOfVertices; ++i ){
                x = ((double) rand() )/RAND_MAX;
                y = ((double) rand() )/RAND_MAX;
                z =0;
                Vector v = Vector(x,y,z,0.,sq);
                polygon.vertices.push_back(v);
            }
            if (isCentered) centroidalVoronoiTesselation(polygon, filename);
            else {
                std::vector<Polygon> polygons = VoronoiDiagram::voronoiParallelLinearEnumeration(polygon);
                save_voronoi_svg(polygons, filename ,polygon);
            }


        }


};
