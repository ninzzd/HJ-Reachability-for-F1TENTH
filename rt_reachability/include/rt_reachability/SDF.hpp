#pragma once

#include "rt_reachability/Grid.hpp"
#include <sstream>
#include <string>
#include <cmath>
#include <cuda_runtime.h>
#define CUDA_CHECK(cuda_error) rt_reachability::checkCUDAerror((cuda_error),__FILE__,__LINE__)
namespace rt_reachability {
    void firstInitMem();
    void computeObstacleSet(
        float* r_obstacles,
        int num_obstacles,
        float angle_min,
        float angle_max,
        float angle_inc,
        float* value_func
    );
    void cylToCart(float* r_obstacles,int num_obstacles,float angle_min,float angle_max,float angle_inc, float* x_obstacles, float* y_obstacles);
    void computeSDF(
        float* r_obstacles,
        int num_obstacles,
        float angle_min,
        float angle_max,
        float angle_inc,
        float* value_func
    );
    float* computeSDF(float* r_obstacles,int num_obstacles,float angle_min,float angle_max,float angle_inc);
    inline void checkCUDAerror(cudaError_t err,const char* file, int line){
        if(err != cudaSuccess){
            std::cout << "CUDA Error at Line:" << line << " in " << file << ": " << cudaGetErrorString(err) << std::endl;
            exit(EXIT_FAILURE);
        }
    }
}