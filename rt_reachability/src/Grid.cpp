#include <iostream>
#include "rt_reachability/Grid.hpp"
using namespace rt_reachability;
int Grid::Nx;
int Grid::Ny;
int Grid::Nv;
int Grid::Ntheta;
float Grid::x_max;
float Grid::y_max;
float Grid::v_max;
float Grid::theta_max;
float Grid::x_min;
float Grid::y_min;
float Grid::v_min;
float Grid::theta_min;
void Grid::setSize(int nx, int ny, int nv, int ntheta){
    Grid::Nx = nx;
    Grid::Ny = ny;
    Grid::Nv = nv;
    Grid::Ntheta = ntheta;
}
int Grid::getSizeX(){
    return Grid::Nx;
}
int Grid::getSizeY(){
    return Grid::Ny;
}
int Grid::getSizeV(){
    return Grid::Nv;
}
int Grid::getSizeTheta(){
    return Grid::Ntheta;
}
void Grid::setLowerBounds(float xmin, float ymin, float vmin, float thetamin){
    Grid::x_min = xmin;
    Grid::y_min = ymin;
    Grid::v_min = vmin;
    Grid::theta_min = thetamin;
}
void Grid::setUpperBounds(float xmax, float ymax, float vmax, float thetamax){
    Grid::x_max = xmax;
    Grid::y_max = ymax;
    Grid::v_max = vmax;
    Grid::theta_max = thetamax;
}
float Grid::getMinX(){
    return Grid::x_min;
}
float Grid::getMaxX(){
    return Grid::x_max;
}
float Grid::getMinY(){
    return Grid::y_min;
}
float Grid::getMaxY(){
    return Grid::y_max;
}

float Grid::getMinV(){
    return Grid::v_min;
}
float Grid::getMaxV(){
    return Grid::v_max;
}

float Grid::getMinTheta(){
    return Grid::theta_min;
}
float Grid::getMaxTheta(){
    return Grid::theta_max;
}

