#include<iostream>
#include"Grid.hpp"
using namespace rt_reachability;
int Grid::Nx = Grid::Ny = Grid::Nv = Grid::Ntheta = 0;
float Grid::x_min = Grid::y_min = Grid::v_min = Grid::theta_min = 0.0;
float Grid::x_max = Grid::y_max = Grid::v_max = Grid::theta_max = 0.0;
void Grid::setSize(int nx, int ny, int nv, int ntheta){
    Nx = nx;
    Ny = ny;
    Nv = nv;
    Ntheta = ntheta;
}
int Grid::getSizeX(){
    return Nx;
}
int Grid::getSizeY(){
    return Ny;
}float Grid::x_min = Grid::y_min = Grid::v_min = Grid::theta_min = 0.0;
int Grid::getSizeV(){
    return Nv;
}
int Grid::getSizeTheta(){
    return Ntheta;
}
void Grid::setLowerBounds(float xmin, float ymin, float vmin, float thetamin){
    x_min = xmin;
    y_min = ymin;
    v_min = vmin;
    theta_min = thetamin;
}
void Grid::setUpperBounds(float xmax, float ymax, float vmax, float thetamax){
    x_max = xmax;
    y_max = ymax;
    v_max = vmax;
    theta_max = thetamax;
}
float Grid::getMinX(){
    return x_min;
}
float Grid::getMaxX(){
    return x_max;
}
float Grid::getMinY(){
    return y_min;
}
float Grid::getMaxY(){
    return y_max;
}

float Grid::getMinV(){
    return v_min;
}
float Grid::getMaxV(){
    return v_max;
}

float Grid::getMinTheta(){
    return theta_min;
}
float Grid::getMaxTheta(){
    return theta_max;
}

