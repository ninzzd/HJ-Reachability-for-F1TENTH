#include "rt_reachability/SDF.hpp"
#include "stdio.h"
#include "math.h"
#include <algorithm>
#include <chrono>
#include <iostream>
using namespace rt_reachability;
__global__ void computeObstacleSetKernel(float* r_obstacles, int num_obstacles, float angle_min, float angle_max, float angle_inc, float* _2Dvaluefunc, float x_max, float x_min, int Nx, float y_max, float y_min, int Ny){
    // flattenedGridPoints -> even indices: x-coordinate, odd indices: y-coordinate
    int id = blockIdx.x*blockDim.x + threadIdx.x;
    int i = id/Ny;
    int j = id%Ny;
    if(i < Nx && j < Ny && i >= 0 && j >= 0){
        _2Dvaluefunc[i*Ny + j] = -2.0;
        float x = x_max - i*(x_max - x_min)/(Nx - 1);
        float y = y_max - j*(y_max - y_min)/(Ny - 1);
        float theta = acos(fminf(fmaxf(x/sqrt(x*x + y*y),-1),1))*(y > 0?+1.0:-1.0);
        int k = min((int)floorf((theta - angle_min)/angle_inc),num_obstacles-1);
        // printf("(%d,%d) -> Angle and index at (%f,%f) = %f , %d\n",i,j,x,y,theta*180/(float)M_PI,k);
        //printf("Index at (%f,%f) = %d\n",x,y,k);
        if(k < num_obstacles && k >= 0){
            if(sqrt(x*x + y*y) >= fminf(r_obstacles[k],r_obstacles[(k < num_obstacles-1)?k+1:k]))
                _2Dvaluefunc[i*Ny + j] = -1.0;
            else _2Dvaluefunc[i*Ny + j] = 0.0;
        }
    }
}
__global__ void cylToCartKernel(float* r_obstacles, int num_obstacles, float angle_min, float angle_max, float angle_inc, float* x_obstacles, float* y_obstacles){
    int i = blockIdx.x*blockDim.x + threadIdx.x;
    if(i < num_obstacles){
        x_obstacles[i] = r_obstacles[i]*cos(angle_min + i*angle_inc);
        y_obstacles[i] = r_obstacles[i]*sin(angle_min + i*angle_inc);
    }
}
__global__ void computeSDFKernel(float* x_obstacles, float* y_obstacles, int num_obstacles,float* valuefunc2D, float x_max, float x_min, int Nx, float y_max, float y_min, int Ny){
    int id = blockIdx.x*blockDim.x + threadIdx.x;
    int i = id/Ny;
    int j = id%Ny;
    float min_dist = (float)MAXFLOAT;
    float x = x_max - i*(x_max-x_min)/(Nx-1);
    float y = y_max - j*(y_max-y_min)/(Ny-1);
    for(int k = 0;k < num_obstacles;k++){
        float temp = sqrt((x-x_obstacles[k])*(x-x_obstacles[k]) + (y-y_obstacles[k])*(y-y_obstacles[k]));
        if(temp < min_dist) min_dist = temp;
    }
    if(valuefunc2D[i*Ny + j] == 0.0) valuefunc2D[i*Ny + j] = min_dist;
    else valuefunc2D[i*Ny + j] = -min_dist;
}
void rt_reachability::firstInitMem(){
    // float* dummy;
    // cudaMalloc(&dummy,sizeof(float)*1);
    cudaFree(0);
}
void rt_reachability::computeObstacleSet(float* r_obstacles, int num_obstacles, float angle_min, float angle_max, float angle_inc, float* value_func){
    float* cuda_r_obstacles;
    float* cuda_value_function;

    std::cout << "----- Latency (For Obstacle Set Computation) -----" << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    cudaMalloc(&cuda_r_obstacles,sizeof(float)*num_obstacles);
    auto end = std::chrono::high_resolution_clock::now();
    auto gpu_memory_allocation_time1 = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << "GPU Memory Allocation Time: " << gpu_memory_allocation_time1 << " microseconds" << std::endl;
    
    start = std::chrono::high_resolution_clock::now();
    cudaMalloc(&cuda_value_function,sizeof(float)*rt_reachability::Grid::getSizeX()*rt_reachability::Grid::getSizeY());
    end = std::chrono::high_resolution_clock::now();
    auto gpu_memory_allocation_time2 = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << "GPU Memory Allocation Time: " << gpu_memory_allocation_time2 << " microseconds" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    cudaMemcpy(cuda_r_obstacles,r_obstacles,sizeof(float)*num_obstacles,cudaMemcpyHostToDevice);
    end = std::chrono::high_resolution_clock::now();
    auto gpu_h2d_copying_time = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << "GPU Memory Host-To-Device Copying Time: " << gpu_h2d_copying_time << " microseconds" << std::endl;

    // int n = 2;
    // dim3 blockSize(32*n);
    // dim3 gridSize(1+(int)((Grid::getSizeX()*Grid::getSizeY())/(32*n)));

    start = std::chrono::high_resolution_clock::now();
    computeObstacleSetKernel<<<dim3(50),dim3(50)>>>(cuda_r_obstacles,num_obstacles,angle_min,angle_max,angle_inc,cuda_value_function,rt_reachability::Grid::getMaxX(),rt_reachability::Grid::getMinX(),rt_reachability::Grid::getSizeX(),rt_reachability::Grid::getMaxY(),rt_reachability::Grid::getMinY(),rt_reachability::Grid::getSizeY());
    end = std::chrono::high_resolution_clock::now();
    auto gpu_compute_time = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << "GPU Compute Time: " << gpu_compute_time << " microseconds" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    cudaMemcpy(value_func,cuda_value_function,sizeof(float)*rt_reachability::Grid::getSizeX()*rt_reachability::Grid::getSizeY(),cudaMemcpyDeviceToHost);
    end = std::chrono::high_resolution_clock::now();
    auto gpu_d2h_copying_time = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << "GPU Memory Device-To-Host Copying Time: " << gpu_d2h_copying_time << " microseconds" << std::endl;
    cudaFree(cuda_r_obstacles);
    cudaFree(cuda_value_function);
}
void rt_reachability::cylToCart(float* r_obstacles,int num_obstacles,float angle_min,float angle_max,float angle_inc, float* x_obstacles, float* y_obstacles){
    float* cuda_r_obstacles;
    float* cuda_x_obstacles;
    float* cuda_y_obstacles;
    cudaMalloc(&cuda_r_obstacles,sizeof(float)*num_obstacles);
    cudaMalloc(&cuda_x_obstacles,sizeof(float)*num_obstacles);
    cudaMalloc(&cuda_y_obstacles,sizeof(float)*num_obstacles);
    cudaMemcpy(cuda_r_obstacles,r_obstacles,sizeof(float)*num_obstacles,cudaMemcpyHostToDevice);
    cylToCartKernel<<<dim3(50),dim3(50)>>>(cuda_r_obstacles,num_obstacles,angle_min, angle_max, angle_inc, cuda_x_obstacles, cuda_y_obstacles);
    cudaMemcpy(x_obstacles,cuda_x_obstacles, sizeof(float)*num_obstacles,cudaMemcpyDeviceToHost);
    cudaMemcpy(y_obstacles,cuda_y_obstacles, sizeof(float)*num_obstacles,cudaMemcpyDeviceToHost);
    cudaFree(cuda_r_obstacles);
    cudaFree(cuda_x_obstacles);
    cudaFree(cuda_y_obstacles);
}
void checkCUDAerror(cudaError_t err,const char* msg = nullptr){
    if(err != cudaSuccess){
        std::cout << "CUDA Error: "<< cudaGetErrorString(err);
        if(msg) std::cout << ": " << msg << std::endl;
        else std::cout << std::endl;
        throw std::runtime_error("Terminating SDF Computation\n");
    }
}
void rt_reachability::computeSDF(float* r_obstacles,int num_obstacles,float angle_min,float angle_max,float angle_inc,float* value_func){
    float* cuda_r_obstacles;
    float* cuda_value_function;
    float* cuda_x_obstacles;
    float* cuda_y_obstacles;

    cudaStream_t obstacleSet, cylToCart;
    cudaEvent_t event1, event2;
    checkCUDAerror(cudaStreamCreate(&obstacleSet),"Error in Obstacle Set Kernel Stream Creation");
    checkCUDAerror(cudaStreamCreate(&cylToCart),"Error in Cylindrical-to-Cartesian Kernel Stream Creation");
    checkCUDAerror(cudaEventCreate(&event1),"Error in Obstacle Set Kernel Event Creation");
    checkCUDAerror(cudaEventCreate(&event2),"Error in Cylindical-To-Cartesian Kernel Event Creation");

    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization (Before Attempting to Allocate Memory) ");
    checkCUDAerror(cudaMalloc(&cuda_r_obstacles,sizeof(float)*num_obstacles),"Error in Memory Allocation to cuda_r_obstacles");
    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization (After Allocating Memory to cuda_r_obstacles) ");
    checkCUDAerror(cudaMalloc(&cuda_value_function,sizeof(float)*rt_reachability::Grid::getSizeX()*rt_reachability::Grid::getSizeY()),"Error in Memory Allocation to cuda_value_function");
    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization (After Allocating Memory to cuda_value_function) ");
    checkCUDAerror(cudaMalloc(&cuda_x_obstacles,sizeof(float)*num_obstacles),"Error in Memory Allocation to cuda_x_obstacles");
    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization (After Allocating Memory to cuda_x_obstacles) ");
    checkCUDAerror(cudaMalloc(&cuda_y_obstacles,sizeof(float)*num_obstacles),"Error in Memory Allocation to cuda_y_obstacles");
    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization (After Allocating Memory to cuda_y_obstacles) ");
    checkCUDAerror(cudaMemcpy(cuda_r_obstacles,r_obstacles,sizeof(float)*num_obstacles,cudaMemcpyHostToDevice),"Error in Memory Copying from r_obstacles to cuda_r_obstacles");
    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization (After Copying Memory into cuda_r_obstacles) ");

    // dim3 blockSize(32*n);
    // dim3 gridSize(1+(int)((Grid::getSizeX()*Grid::getSizeY())/(32*n)));
    auto start = std::chrono::high_resolution_clock::now();
    computeObstacleSetKernel<<<dim3(50),dim3(50),0,obstacleSet>>>(cuda_r_obstacles,num_obstacles,angle_min,angle_max,angle_inc,cuda_value_function,rt_reachability::Grid::getMaxX(),rt_reachability::Grid::getMinX(),rt_reachability::Grid::getSizeX(),rt_reachability::Grid::getMaxY(),rt_reachability::Grid::getMinY(),rt_reachability::Grid::getSizeY());
    checkCUDAerror(cudaEventRecord(event1,obstacleSet),"Error in Recording the Event for Obstacle Set Kernel");
    cylToCartKernel<<<dim3(2),dim3(1024),0,cylToCart>>>(cuda_r_obstacles, num_obstacles, angle_min, angle_max, angle_inc, cuda_x_obstacles, cuda_y_obstacles);
    checkCUDAerror(cudaEventRecord(event2,cylToCart),"Error in Recording the Event for Cylindircal-to-Cartesian Kernel");
    checkCUDAerror(cudaStreamWaitEvent(0,event1,0),"Error in Waiting for End of Execution of Obstacle Set Kernel");
    checkCUDAerror(cudaStreamWaitEvent(0,event2,0),"Error in Waiting for End of Execution of Cylindrical-to-Cartesian Kernel");
    auto end =   std::chrono::high_resolution_clock::now();
    auto stream_duration = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << "\033[2A";
    std::cout << "\rSimulatneous Obstacle Set and Cartesian Coordinates Processing Time (in microseconds) : " << stream_duration << std::endl;
    checkCUDAerror(cudaStreamDestroy(obstacleSet),"Error in Destroying the Obstacle Set Kernel Stream");
    checkCUDAerror(cudaStreamDestroy(cylToCart),"Error in Destroying the Cylindrical-to-Cartesian Kernel Stream");
    checkCUDAerror(cudaEventDestroy(event1),"Error in Destroying the Obstacle Set Kernel Event");
    checkCUDAerror(cudaEventDestroy(event2),"Error in Destroying the Cylindrical-to-Cartesian Kernel Event");
    start = std::chrono::high_resolution_clock::now();
    computeSDFKernel<<<dim3(50),dim3(50)>>>(cuda_x_obstacles, cuda_y_obstacles,num_obstacles,cuda_value_function,rt_reachability::Grid::getMaxX(),rt_reachability::Grid::getMinX(),rt_reachability::Grid::getSizeX(),rt_reachability::Grid::getMaxY(),rt_reachability::Grid::getMinY(),rt_reachability::Grid::getSizeY());
    end =   std::chrono::high_resolution_clock::now();
    auto sdf_duration = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    std::cout << "\rSDF Processing Time (in microseconds): " << sdf_duration << std::endl;
    
    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization Before Final Memory Copying (Before Attempting to Freeing Memory) ");
    checkCUDAerror(cudaMemcpy(value_func,cuda_value_function,sizeof(float)*rt_reachability::Grid::getSizeX()*rt_reachability::Grid::getSizeY(),cudaMemcpyDeviceToHost),"Error in Memory Copying from cuda_value_function to value_function");
    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization (After Copying Memory into value_function) ");
    checkCUDAerror(cudaFree(cuda_r_obstacles),"Error in Freeing Memory for cuda_r_obstacles");
    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization (After Freeing Memory of cuda_r_obstacles) ");
    checkCUDAerror(cudaFree(cuda_value_function),"Error in Freeing Memory for cuda_value_function");
    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization (After Freeing Memory of cuda_value_function) ");
    checkCUDAerror(cudaFree(cuda_x_obstacles),"Error in Freeing Memory for cuda_x_obstacles");
    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization (After Freeing Memory of cuda_x_obstacles) ");
    checkCUDAerror(cudaFree(cuda_y_obstacles),"Error in Freeing Memory for cuda_y_obstacles");
    checkCUDAerror(cudaDeviceSynchronize(),"Error in Device Synchronization (After Freeing Memory of cuda_y_obstacles) ");
    
}