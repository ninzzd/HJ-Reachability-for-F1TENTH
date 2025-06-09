#include <iostream>
#include "rt_reachability/Grid.hpp"
#include "Grid.hpp"
#include "SDF.hpp"

#define getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta){\
    grid[l*Nx*Ny*Nv + k*Nx*Ny + i*Ny + j];\
}
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
float* Grid::grid;
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
__global__ void init4DGridKernel(float* cuda_SDF, float* cuda_value_function_4D, int Nx, int Ny, int Nv, int Ntheta){
    int idx = blockDim.x*blockIdx.x + threadIdx.x;
    int l = idx/(Nx*Ny*Nv);
    int k = (idx%(Nx*Ny*Nv))/(Nx*Ny);
    int i = ((idx%(Nx*Ny*Nv))%(Nx*Ny))/Ny;
    int j = ((idx%(Nx*Ny*Nv))%(Nx*Ny))%Ny;
    cuda_value_function_4D[idx] = cuda_SDF[i*Ny + j];
}
__global__ void partialDerivKernel(float* grid, int Nx, int Ny, int Nv, int Ntheta){
    int idx = blockDim.x*blockIdx.x + threadIdx.x;
    int l = idx/(Nx*Ny*Nv);
    int k = (idx%(Nx*Ny*Nv))/(Nx*Ny);
    int i = ((idx%(Nx*Ny*Nv))%(Nx*Ny))/Ny;
    int j = ((idx%(Nx*Ny*Nv))%(Nx*Ny))%Ny;
    getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta);
}
void Grid::initializeGrid(float* r_obstacles,int num_obstacles,float angle_min,float angle_max,float angle_inc){
    float* cuda_SDF = rt_reachability::gridInitSDF(r_obstacles,num_obstacles,angle_min,angle_max,angle_inc);
    cudaMalloc(&Grid::grid,sizeof(float)*Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta);
    init4DGridKernel<<<(Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta/1024),1024>>>(cuda_SDF,Grid::grid,Grid::Nx,Grid::Ny,Grid::Nv,Grid::Ntheta);
    cudaFree(cuda_SDF);
}
void Grid::computeReachability(){
    
}