#include <iostream>
#include "rt_reachability/Grid.hpp"
#include "rt_reachability/SDF.hpp"
#include "float.h"

#define getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta) grid[l*Nx*Ny*Nv + k*Nx*Ny + i*Ny + j].value

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
Grid::Point* Grid::grid = nullptr;
int Grid::Ndelta;
int Grid::Na;
float Grid::delta_min;
float Grid::delta_max;
float Grid::a_min;
float Grid::a_max;
float Grid::length = 1.0f;
float Grid::delta_t;
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
__global__ void initGridKernel(float* cuda_SDF, Grid::Point* cuda_grid, int Nx, int Ny, int Nv, int Ntheta){
    int idx = blockDim.x*blockIdx.x + threadIdx.x;
    if(idx < Nx*Ny*Nv*Ntheta){
        // int l = idx/(Nx*Ny*Nv);
        // int k = (idx%(Nx*Ny*Nv))/(Nx*Ny);
        int i = ((idx%(Nx*Ny*Nv))%(Nx*Ny))/Ny;
        int j = ((idx%(Nx*Ny*Nv))%(Nx*Ny))%Ny;
        cuda_grid[idx].value = cuda_SDF[i*Ny + j];
        for(int t = 0;t < 4;t++){
            cuda_grid[idx].left_deriv[t] = 0.0f;
            cuda_grid[idx].right_deriv[t] = 0.0f;
        }
    }
}
__global__ void partialDerivKernel(Grid::Point* grid, int Nx, int Ny, int Nv, int Ntheta,float delta_x,float delta_y, float delta_v, float delta_theta){
    int idx = blockDim.x*blockIdx.x + threadIdx.x;
    if(idx < Nx*Ny*Nv*Ntheta){
        int l = idx/(Nx*Ny*Nv);
        int k = (idx%(Nx*Ny*Nv))/(Nx*Ny);
        int i = ((idx%(Nx*Ny*Nv))%(Nx*Ny))/Ny;
        int j = ((idx%(Nx*Ny*Nv))%(Nx*Ny))%Ny;
        for(int t = 0;t < 4;t++){
            float delta = delta_x*(~(t/2)&~(t%2)) + delta_y*(~(t/2)&(t%2)) + delta_v*((t/2)&~(t%2)) + delta_theta*((t/2)&(t%2));
            if(i*(~(t/2)&~(t%2)) + j*(~(t/2)&(t%2)) + k*((t/2)&~(t%2)) + l*((t/2)&(t%2)) == (Nx-1)*(~(t/2)&~(t%2)) + (Ny-1)*(~(t/2)&(t%2)) + (Nv-1)*((t/2)&~(t%2)) + (Ntheta-1)*((t/2)&(t%2))){
                float temp = (getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta) - getValue(grid,i - (~(t/2)&~(t%2)),j - (~(t/2)&(t%2)),k - ((t/2)&~(t%2)),l - ((t/2)&(t%2)),Nx,Ny,Nv,Ntheta))/delta;
                grid[idx].left_deriv[t] = temp;
                grid[idx].right_deriv[t] = temp;
            }
            else if(i*(~(t/2)&~(t%2)) + j*(~(t/2)&(t%2)) + k*((t/2)&~(t%2)) + l*((t/2)&(t%2)) == 0){
                float temp = (getValue(grid,i + (~(t/2)&~(t%2)),j + (~(t/2)&(t%2)),k + ((t/2)&~(t%2)),l + ((t/2)&(t%2)),Nx,Ny,Nv,Ntheta) - getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta))/delta;
                grid[idx].left_deriv[t] = temp;
                grid[idx].right_deriv[t] = temp;
            }
            else{
                grid[idx].left_deriv[t] = (getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta) - getValue(grid,i - (~(t/2)&~(t%2)),j - (~(t/2)&(t%2)),k - ((t/2)&~(t%2)),l - ((t/2)&(t%2)),Nx,Ny,Nv,Ntheta))/delta;
                grid[idx].right_deriv[t] = (getValue(grid,i + (~(t/2)&~(t%2)),j + (~(t/2)&(t%2)),k + ((t/2)&~(t%2)),l + ((t/2)&(t%2)),Nx,Ny,Nv,Ntheta) - getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta))/delta;
            }
        }
    }
}
__device__ float approxHamiltonian(Grid::Point point, int i, int j, int k, int l, int Nx, int Ny, int Nv, int Ntheta, float x_min, float x_max, float y_max, float y_min, float v_max, float v_min, float theta_max, float theta_min, float a, float delta){
    float ham= 0.0;
    for(int t = 0;t < 4;t++){
        float v = (v_min + k*(v_max - v_min)/(Nv - 1));
        float theta = (theta_min + l*(theta_max - theta_min)/(Ntheta - 1));
        float dot = 0;
        switch(t){
            case 0:
                dot = v*cosf(theta);
                break;
            case 1:
                dot = v*sinf(theta);
                break;
            case 2:
                dot = a;
                break;
            case 3:
                dot = v*tanf(delta)/1.0; // Find out car length
                break;
        }
        ham+=dot*(point.left_deriv[t] + point.right_deriv[t])/2;
    }
    return ham;
}
__device__ float correctedHamiltonian(float approx_ham, Grid::Point point, int i, int j, int k, int l, int Nx, int Ny, int Nv, int Ntheta, float x_min, float x_max, float y_max, float y_min, float v_max, float v_min, float theta_max, float theta_min, float a, float delta){
    float ham = approx_ham;
    for(int t = 0;t < 4;t++){
        float v = (v_min + k*(v_max - v_min)/(Nv - 1));
        float theta = (theta_min + l*(theta_max - theta_min)/(Ntheta - 1));
        float dot = 0;
        switch(t){
            case 0:
                dot = v*cosf(theta);
                break;
            case 1:
                dot = v*sinf(theta);
                break;
            case 2:
                dot = a;
                break;
            case 3:
                dot = v*tanf(delta)/1.0; // Find out car length
                break;
        }
        ham-=fabs(dot)*(point.right_deriv[t] - point.left_deriv[t])/2;
    }
    return ham;
}
__global__ void updateValueKernel(Grid::Point* grid, int Nx, int Ny, int Nv, int Ntheta, float x_min, float x_max, float y_max, float y_min, float v_max, float v_min, float theta_max, float theta_min, int Na, int Ndelta, float a_min, float a_max, float delta_min, float delta_max, float delta_t){
    int idx = blockDim.x*blockIdx.x + threadIdx.x;
    int l = idx/(Nx*Ny*Nv);
    int k = (idx%(Nx*Ny*Nv))/(Nx*Ny);
    int i = ((idx%(Nx*Ny*Nv))%(Nx*Ny))/Ny;
    int j = ((idx%(Nx*Ny*Nv))%(Nx*Ny))%Ny;
    if(idx < Nx*Ny*Nv*Ntheta){
        float hamiltonian_max = (float)FLT_MIN;
        float opt_a = 0.0;
        float opt_delta = 0.0;
        for(int a1 = 0; a1 < Na;a1++){
            for(int a2 = 0; a2 < Ndelta;a2++){
                float a = a_min + a1*(a_max - a_min)/(Na-1);
                float delta = delta_min + a2*(delta_max - delta_min)/(Ndelta - 1);
                float ham = approxHamiltonian(grid[idx],i,j,k,l,Nx,Ny,Nv,Ntheta,x_min,x_max,y_max,y_min,v_max,v_min,theta_max,theta_min,a,delta);
                if(ham > hamiltonian_max){
                    hamiltonian_max = ham;
                    opt_a = a;
                    opt_delta = delta;
                }
            }
        }
        grid[idx].opt_a = opt_a;
        grid[idx].opt_delta = opt_delta;
        hamiltonian_max = correctedHamiltonian(hamiltonian_max,grid[idx],i,j,k,l,Nx,Ny,Nv,Ntheta,x_min,x_max,y_max,y_min,v_max,v_min,theta_max,theta_min,opt_a,opt_delta);
        grid[idx].value = fminf(grid[idx].value,grid[idx].value + delta_t*hamiltonian_max);
    }
} 

void Grid::initializeGrid(float* r_obstacles,int num_obstacles,float angle_min,float angle_max,float angle_inc){
    if(Grid::grid != nullptr){
        cudaFree(Grid::grid);
    }
    cudaMalloc(&(Grid::grid),sizeof(Grid::Point)*Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta);
    float* cuda_SDF = computeSDF(r_obstacles,num_obstacles,angle_min,angle_max,angle_inc);
    dim3 gridSize((int)(Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta/1024)+1);
    dim3 blockSize(1024);
    initGridKernel<<<gridSize,blockSize>>>(cuda_SDF,Grid::grid,Grid::Nx,Grid::Ny,Grid::Nv,Grid::Ntheta);
    checkCUDAerror(cudaDeviceSynchronize());
    checkCUDAerror(cudaFree(cuda_SDF));
}
void Grid::computeDeltaT(){
    float lambda_x = 0;
    float lambda_y = 0;
    float lambda_v = Grid::a_max;
    float lambda_theta = 0;
    for(int a1 = 0;a1 < Nv;a1++){
        float v = (v_min + a1*(v_max - v_min)/(Nv - 1));
        for(int a2 = 0;a2 < Ntheta;a2++){
            float theta = (theta_min + a2*(theta_max - theta_min)/(Ntheta - 1));
            float x_dot = v*cosf(theta);
            float y_dot = v*sinf(theta);
            if(x_dot > lambda_x) lambda_x = x_dot;
            if(y_dot > lambda_y) lambda_y = y_dot;
        }
        for(int a3 = 0;a3 < Grid::Ndelta;a3++){
            float delta = (delta_min + a3*(delta_max - delta_min)/(Ndelta - 1));
            float theta_dot = v*tanf(delta)/length;
            if(theta_dot > lambda_theta) lambda_theta = theta_dot;
        }
    }
    delta_t = fminf((x_max-x_min)/((Nx-1)*lambda_x),fminf((y_max-y_min)/((Ny-1)*lambda_y),fminf((v_max-v_min)/((Nv-1)*lambda_v),(theta_max-theta_min)/((Ntheta-1)*lambda_theta))));
}
Grid::Point* Grid::computeReachability(int N){
    dim3 gridSize((int)(Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta/1024)+1);
    dim3 blockSize(1024);
    for(int i = 0;i < N;i++){
        partialDerivKernel<<<gridSize,blockSize>>>(grid, Nx, Ny, Nv, Ntheta, (x_max - x_min)/(Nx - 1), (y_max - y_min)/(Ny - 1), (v_max - v_min)/(Nv - 1), (theta_max - theta_min)/(Ntheta - 1));
        cudaDeviceSynchronize();
        updateValueKernel<<<gridSize,blockSize>>>(grid,Nx,Ny,Nv,Ntheta,x_min,x_max,y_max,y_min,v_max,v_min,theta_max,theta_min,Na,Ndelta,a_min,a_max,delta_min,delta_max,delta_t);
    }
    Point* tempGrid = (Point*)malloc(sizeof(Grid::Point)*Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta);
    cudaMemcpy(tempGrid,grid,sizeof(Grid::Point)*Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta,cudaMemcpyDeviceToHost);
    cudaFree(grid);
    grid = nullptr;
    cudaDeviceSynchronize();
    return tempGrid;
}