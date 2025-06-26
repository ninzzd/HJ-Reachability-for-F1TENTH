#include "rt_reachability/Grid.hpp"

#define getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta) grid[(l)*Nx*Ny*Nv + (k)*Nx*Ny + (i)*Ny + (j)].value
#define lli long long int
#define min(x,y) ((x) < (y)? (x) : (y))
#define max(x,y) ((x) > (y)? (x) : (y))
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
float Grid::length = 0.33f;
float Grid::delta_t = 0.01f;
float Grid::val_max = 0.0f;
float Grid::val_min = 0.0f;
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
void Grid::setCarLength(float length){
    Grid::length =length;
}
void Grid::setInputParams(int Na, int Ndelta, float a_min, float a_max, float delta_min, float delta_max){
    Grid::Na = Na;
    Grid::Ndelta = Ndelta;
    Grid::a_min = a_min;
    Grid::a_max = a_max;
    Grid::delta_min = delta_min;
    Grid::delta_max = delta_max;
}
void Grid::setValueFunctionBounds(float min, float max){
    Grid::val_min = min;
    Grid::val_max = max;
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
    // max(min(Ntheta-1,...),0)
    if(idx < Nx*Ny*Nv*Ntheta){
        int l = max(min(Ntheta-1,idx/(Nx*Ny*Nv)),0);
        int k = max(min(Nv-1,(idx - l*Nx*Ny*Nv)/(Nx*Ny)),0);
        int i = max(min(Nx-1,(idx - l*Nx*Ny*Nv - k*Nx*Ny)/Ny),0);
        int j = max(min(Ny-1,(idx - l*Nx*Ny*Nv - k*Nx*Ny - i*Ny)),0);
        cuda_grid[idx].value = cuda_SDF[i*Ny + j];
        cuda_grid[idx].opt_a = 0.0f;
        cuda_grid[idx].opt_delta = 0.0f;
        for(int t = 0;t < 4;t++){
            cuda_grid[idx].left_deriv[t] = 0.0f;
            cuda_grid[idx].right_deriv[t] = 0.0f;
        }
    }
}
__device__ void firstOrderUpwind(Grid::Point* grid, int i, int j, int k, int l, int Nx, int Ny, int Nv, int Ntheta,float delta_x,float delta_y, float delta_v, float delta_theta){
    int idx = l * Nx * Ny * Nv + k * Nx * Ny + i * Ny + j;
    for(int t = 0;t < 4;t++){
            int bx = (t==0?1:0);
            int by = (t==1?1:0);
            int bv = (t==2?1:0);
            int bt = (t==3?1:0);
            float diff;
            if(bx) diff = delta_x;
            else if(by) diff = delta_y;
            else if(bv) diff = delta_v;
            else if(bt) diff = delta_theta;
            if((bx && (i == 0))||(by && (j == 0))||(bv && (k == 0))||(bt && (l == 0))){
                grid[idx].right_deriv[t] = (getValue(grid,i+bx,j+by,k+bv,l+bt,Nx,Ny,Nv,Ntheta) - getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta))/diff;
                grid[idx].left_deriv[t] = grid[idx].right_deriv[t];
            }
            else if((bx && (i == Nx-1))||(by && (j == Ny-1))||(bv && (k == Nv-1))||(bt && (l == Ntheta-1))){
                grid[idx].left_deriv[t] = (-getValue(grid,i-bx,j-by,k-bv,l-bt,Nx,Ny,Nv,Ntheta) + getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta))/diff;
                grid[idx].right_deriv[t] = grid[idx].left_deriv[t];
            }
            else{
                grid[idx].right_deriv[t] = (getValue(grid,i+bx,j+by,k+bv,l+bt,Nx,Ny,Nv,Ntheta) - getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta))/diff;
                grid[idx].left_deriv[t] = (-getValue(grid,i-bx,j-by,k-bv,l-bt,Nx,Ny,Nv,Ntheta) + getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta))/diff;
            }
    }
}
__device__ void secondOrderUpwind(Grid::Point* grid, int i, int j, int k, int l, int Nx, int Ny, int Nv, int Ntheta,float delta_x,float delta_y, float delta_v, float delta_theta){
     int idx = l * Nx * Ny * Nv + k * Nx * Ny + i * Ny + j;
     float vals[5];
     vals[2] = getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta);
     for(int t = 0;t < 4;t++){
            int bx = (t==0?1:0);
            int by = (t==1?1:0);
            int bv = (t==2?1:0);
            int bt = (t==3?1:0);
            float diff;
            diff = bx*delta_x + by*delta_y + bv*delta_v + bt*delta_theta;
            float lastr, lastl;
            lastr = lastl = vals[2];
            for(int a = 1;a <= 2;a++){
                if(bx*(i+a) + by*(j+a) + bv*(k+a)+ bt*(l+a) >= bx*Nx + by*Ny + bv*Nv + bt*Ntheta){
                    vals[2+a] = lastr;
                }
                else{
                    lastr = getValue(grid,i+bx*a,j+by*a,k+bv*a,l+bt*a,Nx,Ny,Nv,Ntheta);
                    vals[2+a] = lastr;
                }
                if(bx*(i-a) + by*(j-a) + bv*(k-a)+ bt*(l-a) <= 0){
                    vals[2-a] = lastl;
                }
                else{
                    lastl = getValue(grid,i-bx*a,j-by*a,k-bv*a,l-bt*a,Nx,Ny,Nv,Ntheta);
                    vals[2-a] = lastl;
                }
            }
            float d1,d2,_d2;
            // For left derivative
            d1 = (vals[2] - vals[1])/diff; //i-0.5
            d2 = (vals[3] + vals[1] - vals[2])/(2*diff*diff); //i
            _d2 = (vals[2] + vals[0] - vals[1])/(2*diff*diff); //i-1
            grid[idx].left_deriv[t] = d1 + diff*(fabs(_d2) <= fabs(d2)?_d2:d2);
            //For right derivative
            d1 = (vals[3] - vals[2])/diff; //i+0.5
            d2 = (vals[3] + vals[1] - vals[2])/(2*diff*diff); //i
            _d2 = (vals[4] + vals[2] - vals[3])/(2*diff*diff); //i+1
            grid[idx].right_deriv[t] = d1 - diff*(fabs(_d2) <= fabs(d2)?_d2:d2);
    }
}
__device__ void thirdOrderUpwind(Grid::Point* grid, int i, int j, int k, int l, int Nx, int Ny, int Nv, int Ntheta,float delta_x,float delta_y, float delta_v, float delta_theta){
int idx = l * Nx * Ny * Nv + k * Nx * Ny + i * Ny + j;
     float vals[7];
     vals[3] = getValue(grid,i,j,k,l,Nx,Ny,Nv,Ntheta);
     for(int t = 0;t < 4;t++){
            int bx = (t==0?1:0);
            int by = (t==1?1:0);
            int bv = (t==2?1:0);
            int bt = (t==3?1:0);
            float diff;
            diff = bx*delta_x + by*delta_y + bv*delta_v + bt*delta_theta;
            float lastr, lastl;
            lastr = lastl = vals[3];
            for(int a = 1;a <= 3;a++){
                if(bx*(i+a) + by*(j+a) + bv*(k+a)+ bt*(l+a) >= bx*Nx + by*Ny + bv*Nv + bt*Ntheta){
                    vals[3+a] = lastr;
                }
                else{
                    lastr = getValue(grid,i+bx*a,j+by*a,k+bv*a,l+bt*a,Nx,Ny,Nv,Ntheta);
                    vals[3+a] = lastr;
                }
                if(bx*(i-a) + by*(j-a) + bv*(k-a)+ bt*(l-a) <= 0){
                    vals[3-a] = lastl;
                }
                else{
                    lastl = getValue(grid,i-bx*a,j-by*a,k-bv*a,l-bt*a,Nx,Ny,Nv,Ntheta);
                    vals[3-a] = lastl;
                }
            }
            float d1,d2,_d2,d3,_d3; // By convention, _d2 is sequentially or is indexed lesser than d2. Same applies to _d3 and d3
            // For left derivative
            // (i-3) -> 0 ; (i-2) -> 1 ; (i-1) -> 2 ; (i) -> 3 ; (i+1) -> 4 ; (i+2) -> 5; (i+3) -> 6 ;
            d1 = (vals[3] - vals[2])/diff; //i-0.5
            d2 = (vals[4] + vals[2] - vals[3])/(2*diff*diff); //i
            _d2 = (vals[3] + vals[1] - vals[2])/(2*diff*diff); //i-1
            grid[idx].left_deriv[t] = d1 + diff*(fabs(_d2) <= fabs(d2)?_d2:d2);
            if(fabs(_d2) <= fabs(d2)){
                _d3 = (vals[3] - 3*vals[2] + 3*vals[1] - vals[0])/(6*diff*diff*diff); //i-(3/2)
                d3 = (vals[4] - 3*vals[3] + 3*vals[2] - vals[1])/(6*diff*diff*diff); //i-(1/2)
                grid[idx].left_deriv[t] += 2*diff*diff*(fabs(_d3) <= fabs(d3)?_d3:d3);
            }
            else{
                _d3 = (vals[4] - 3*vals[3] + 3*vals[2] - vals[1])/(6*diff*diff*diff); //i-(1/2)
                d3 = (vals[5] - 3*vals[4] + 3*vals[3] - vals[2])/(6*diff*diff*diff); //i+(1/2)
                grid[idx].left_deriv[t] -= diff*diff*(fabs(_d3) <= fabs(d3)?_d3:d3);
            }
            
            //For right derivative
            d1 = (vals[4] - vals[3])/diff; //i+(1/2)
            d2 = (vals[5] + vals[3] - vals[4])/(2*diff*diff); //i+1
            _d2 = (vals[4] + vals[2] - vals[3])/(2*diff*diff); //i
            grid[idx].right_deriv[t] = d1 - diff*(fabs(_d2) <= fabs(d2)?_d2:d2);
            if(fabs(_d2) <= fabs(d2)){
                _d3 = (vals[4] - 3*vals[3] + 3*vals[2] - vals[1])/(6*diff*diff*diff); //i-(1/2)
                d3 = (vals[5] - 3*vals[4] + 3*vals[3] - vals[2])/(6*diff*diff*diff); //i+(1/2)
                grid[idx].left_deriv[t] -= diff*diff*(fabs(_d3) <= fabs(d3)?_d3:d3);
            }
            else{
                _d3 = (vals[5] - 3*vals[4] + 3*vals[3] - vals[2])/(6*diff*diff*diff); //i+(1/2)
                d3 = (vals[6] - 3*vals[5] + 3*vals[4] - vals[3])/(6*diff*diff*diff); //i+(3/2)
                grid[idx].left_deriv[t] += 2*diff*diff*(fabs(_d3) <= fabs(d3)?_d3:d3);
            }
    }
}
__global__ void partialDerivKernel(Grid::Point* grid, int Nx, int Ny, int Nv, int Ntheta,float delta_x,float delta_y, float delta_v, float delta_theta){
    __syncthreads();
    int idx = blockDim.x*blockIdx.x + threadIdx.x;
    if(idx < Nx*Ny*Nv*Ntheta){
        int l = max(min(Ntheta-1,idx/(Nx*Ny*Nv)),0);
        int k = max(min(Nv-1,(idx - l*Nx*Ny*Nv)/(Nx*Ny)),0);
        int i = max(min(Nx-1,(idx - l*Nx*Ny*Nv - k*Nx*Ny)/Ny),0);
        int j = max(min(Ny-1,(idx - l*Nx*Ny*Nv - k*Nx*Ny - i*Ny)),0);
        // firstOrderUpwind(grid,i,j,k,l,Nx,Ny,Nv,Ntheta,delta_x,delta_y,delta_v,delta_theta);
        // secondOrderUpwind(grid,i,j,k,l,Nx,Ny,Nv,Ntheta,delta_x,delta_y,delta_v,delta_theta);
        thirdOrderUpwind(grid,i,j,k,l,Nx,Ny,Nv,Ntheta,delta_x,delta_y,delta_v,delta_theta);
    }
    __syncthreads();
}
__device__ float approxHamiltonian(Grid::Point point, int i, int j, int k, int l, int Nx, int Ny, int Nv, int Ntheta, float x_min, float x_max, float y_min, float y_max, float v_min, float v_max, float theta_min, float theta_max, float a, float delta,float length){
    float ham = 0.0;
    float v = (v_min + k*(v_max - v_min)/(Nv - 1));
    float theta = (theta_min + l*(theta_max - theta_min)/(Ntheta - 1));
    float dot = 0;
    for(int t = 0;t < 4;t++){
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
                dot = v*tanf(delta)/length; // Find out car length
                break;
        }
        // Upwind Difference Scheme for Spacial Derivative Approximation:
        ham+=dot*((dot > 0?1:0)*point.left_deriv[t] + (dot <= 0?1:0)*point.right_deriv[t]);
    }
    return ham;
}
__device__ float correctedHamiltonian(float approx_ham, Grid::Point point, int i, int j, int k, int l, int Nx, int Ny, int Nv, int Ntheta, float x_min, float x_max, float y_min, float y_max, float v_min, float v_max, float theta_min, float theta_max, float a_max, float delta_max, float length){
    float ham = approx_ham;
    // float v = (v_min + k*(v_max - v_min)/(Nv - 1));
    // float theta = (theta_min + l*(theta_max - theta_min)/(Ntheta - 1));
    float dot = 0;
    for(int t = 0;t < 4;t++){
        switch(t){
            case 0:
                dot = v_max;
                break;
            case 1:
                dot = v_max;
                break;
            case 2:
                dot = a_max;
                break;
            case 3:
                dot = v_max*tanf(delta_max)/length; 
                break;
        }
        // Corrected Lax-Friedrichs Dissipation
        ham-=fabs(dot)*(point.right_deriv[t] - point.left_deriv[t])/2;
    }
    return ham;
}
__global__ void updateValueKernel(Grid::Point* grid, int Nx, int Ny, int Nv, int Ntheta, float x_min, float x_max, float y_min, float y_max, float v_min, float v_max, float theta_min, float theta_max, int Na, int Ndelta, float a_min, float a_max, float delta_min, float delta_max, float delta_t,float length,float val_min, float val_max){
    // The index logic is fine (double-checked)
    int idx = blockDim.x*blockIdx.x + threadIdx.x;
    if(idx < Nx*Ny*Nv*Ntheta){
        int l = max(min(Ntheta-1,idx/(Nx*Ny*Nv)),0);
        int k = max(min(Nv-1,(idx - l*Nx*Ny*Nv)/(Nx*Ny)),0);
        int i = max(min(Nx-1,(idx - l*Nx*Ny*Nv - k*Nx*Ny)/Ny),0);
        int j = max(min(Ny-1,(idx - l*Nx*Ny*Nv - k*Nx*Ny - i*Ny)),0);
        float hamiltonian_max = (float)FLT_MIN;
        float opt_a = 0.0;
        float opt_delta = 0.0;
        for(int a1 = 0; a1 < Na;a1++){
            for(int a2 = 0; a2 < Ndelta;a2++){
                float a = a_min + a1*(a_max - a_min)/(Na-1);
                float delta = delta_min + a2*(delta_max - delta_min)/(Ndelta - 1);
                float ham = approxHamiltonian(grid[idx],i,j,k,l,Nx,Ny,Nv,Ntheta,x_min,x_max,y_min,y_max,v_min,v_max,theta_min,theta_max,a,delta,length);
                if(ham > hamiltonian_max){
                    hamiltonian_max = ham;
                    opt_a = a;
                    opt_delta = delta;
                }
            }
        }
        grid[idx].opt_a = opt_a;
        grid[idx].opt_delta = opt_delta;
        hamiltonian_max = correctedHamiltonian(hamiltonian_max,grid[idx],i,j,k,l,Nx,Ny,Nv,Ntheta,x_min,x_max,y_min,y_max,v_min,v_max,theta_min,theta_max,a_max,delta_max,length);
        float temp = grid[idx].value + hamiltonian_max*delta_t;
        grid[idx].value = fmaxf(val_min,fminf(temp,grid[idx].value));
        // grid[idx].value -= 5.0f;
        // grid[idx].value = fmaxf(val_min,grid[idx].value);
    }
} 

void Grid::initializeGrid(float* r_obstacles,int num_obstacles,float angle_min,float angle_max,float angle_inc){
    if(Grid::grid != nullptr){
        CUDA_CHECK(cudaFree(Grid::grid));
    }
    CUDA_CHECK(cudaMalloc(&(Grid::grid),sizeof(Grid::Point)*Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta));
    float* cuda_SDF = computeSDF(r_obstacles,num_obstacles,angle_min,angle_max,angle_inc);

    // Given grid and block size ensures full occupancy
    dim3 gridSize((int)(Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta/256)+1);
    dim3 blockSize(256);

    initGridKernel<<<gridSize,blockSize>>>(cuda_SDF,Grid::grid,Grid::Nx,Grid::Ny,Grid::Nv,Grid::Ntheta);
    CUDA_CHECK(cudaDeviceSynchronize());
    CUDA_CHECK(cudaFree(cuda_SDF));
    CUDA_CHECK(cudaDeviceSynchronize());
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
            float delta = delta_min + a3*(delta_max - delta_min)/(Ndelta - 1);
            float theta_dot = v*tanf(delta)/length;
            if(theta_dot > lambda_theta) lambda_theta = theta_dot;
        }
    }
    // Revised Courant-Freidrichs-Lewy Condition
    // delta_t = fminf((x_max-x_min)/((Nx-1)*lambda_x),fminf((y_max-y_min)/((Ny-1)*lambda_y),fminf((v_max-v_min)/((Nv-1)*lambda_v),(theta_max-theta_min)/((Ntheta-1)*lambda_theta))));
    delta_t = 1 * 0.5/(lambda_x/((x_max-x_min)/(Nx-1)) + lambda_y/((y_max-y_min)/(Ny-1)) + lambda_v/((v_max-v_min)/(Nv-1)) + lambda_theta/((theta_max-theta_min)/(Ntheta-1)));
    std::cout << "Computed Iteration Timestep: " << delta_t << std::endl;
}
Grid::Point* Grid::computeReachability(int N){
    if(Grid::grid != nullptr){
        std::cout << "Grid has been initialized" << std::endl;
    }
    std::cout << "Value Function Bounds: [" << val_min << "," << val_max << "]" << std::endl;
    dim3 gridSize((int)((Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta)/256)+1);
    dim3 blockSize(256);
    for(int i = 0;i < N;i++){
        std::cout << "Iteration: " << i << std::endl;
        partialDerivKernel<<<gridSize,blockSize>>>(grid, Nx, Ny, Nv, Ntheta, (x_max - x_min)/(float)(Nx - 1), (y_max - y_min)/(float)(Ny - 1), (v_max - v_min)/(float)(Nv - 1), (theta_max - theta_min)/(float)(Ntheta - 1));
        CUDA_CHECK(cudaDeviceSynchronize());
        updateValueKernel<<<gridSize,blockSize>>>(grid,Nx,Ny,Nv,Ntheta,x_min,x_max,y_min,y_max,v_min,v_max,theta_min,theta_max,Na,Ndelta,a_min,a_max,delta_min,delta_max,delta_t,length,val_min,val_max);
        CUDA_CHECK(cudaDeviceSynchronize());
    }
    std::cout << "Reachability set computation is over" << std::endl;
    Point* tempGrid = (Point*)malloc(sizeof(Grid::Point)*Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta);
    CUDA_CHECK(cudaDeviceSynchronize());
    CUDA_CHECK(cudaMemcpy(tempGrid,grid,sizeof(Grid::Point)*Grid::Nx*Grid::Ny*Grid::Nv*Grid::Ntheta,cudaMemcpyDeviceToHost));
    CUDA_CHECK(cudaDeviceSynchronize());
    CUDA_CHECK(cudaFree(grid));
    grid = nullptr;
    return tempGrid;
}