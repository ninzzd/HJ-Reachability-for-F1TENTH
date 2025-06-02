#include "rt_reachability/SDF.hpp"
#include <algorithm>
using namespace rt_reachability;
__global__ void computeObstacleSetKernel(float* r_obstacles, float angle_min, float angle_max, float angle_inc, float* _2Dvaluefunc, float x_max, float x_min, int Nx, float y_max, float y_min, int Ny){
    // flattenedGridPoints -> even indices: x-coordinate, odd indices: y-coordinate
    int i = blockIdx.x;
    int j = threadIdx.x;
    if(i < Nx && j < Ny){
        float x = x_max - i*(x_max - x_min)/(Nx - 1);
        float y = y_max - i*(2*y_max)/(Ny - 1);
        float theta = acos(x/sqrt(x*x + y*y))*(y >= 0?1:-1);
        int k = (int)floor((theta - angle_min)/angle_inc);
        if(k < (int)sizeof(r_obstacles)/sizeof(float)){
            if(sqrt(x*x + y*y) > std::min(r_obstacles[k],r_obstacles[(k < (int)sizeof(r_obstacles)/sizeof(float))-1?k+1:k]))
                _2Dvaluefunc[i*Nx + j] = -1.0;
            else _2Dvaluefunc[i*Nx + j] = 0.0;
        }
    }
}
__global__ void computeSDF(){
    
}

void rt_reachability::computeObstacleSet(float* r_obstacles, float angle_min, float angle_max, float angle_inc, float* value_func){
    float* cuda_r_obstacles;
    float* cuda_value_function;
    size_t n = sizeof(r_obstacles)/sizeof(float);
    cudaMalloc(&cuda_r_obstacles,sizeof(float)*n);
    cudaMalloc(&cuda_value_function,sizeof(float)*rt_reachability::Grid::getSizeX()*rt_reachability::Grid::getSizeY());
    cudaMemcpy(cuda_r_obstacles,r_obstacles,sizeof(float)*n,cudaMemcpyHostToDevice);
    dim3 blockSize(100);
    dim3 gridSize(100);
    computeObstacleSetKernel<<<gridSize,blockSize>>>(cuda_r_obstacles,angle_min,angle_max,angle_inc,cuda_value_function,rt_reachability::Grid::getMaxX(),rt_reachability::Grid::getMinX(),rt_reachability::Grid::getSizeX(),rt_reachability::Grid::getMaxY(),rt_reachability::Grid::getMinY(),rt_reachability::Grid::getSizeY());
    cudaMemcpy(value_func,cuda_value_function,sizeof(float)*n,cudaMemcpyDeviceToHost);
    cudaFree(cuda_r_obstacles);
    cudaFree(cuda_value_function);
}