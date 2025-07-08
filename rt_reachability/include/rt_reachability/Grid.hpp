#pragma once
#include <iostream>
#include <string>
#include <sstream>
#include "float.h"
#include "rt_reachability/SDF.hpp"
namespace rt_reachability {
    class Grid {
    public:
        typedef struct{
                float value;
                float opt_a;
                float opt_delta;
                float left_deriv[4];
                float right_deriv[4];
        } Point; // 44 bytes
        // Might cause misaligned memory accesses for CUDA threads
        static void setSize(int nx, int ny, int nv, int ntheta);
        static void setLowerBounds(float xmin, float ymin, float vmin, float thetamin);
        static void setUpperBounds(float xmax, float ymax, float vmax, float thetamax);
        static void setValueFunctionBounds(float min, float max);
        static void setCarLength(float length);
        static void setInputParams(int Na, int Ndelta, float a_min, float a_max, float delta_min, float delta_max);
        static void setHorizon(float horizon);

        static int getSizeX();
        static int getSizeY();
        static int getSizeV();
        static int getSizeTheta();
        static float getHorizon();
        
        static float getMinX();
        static float getMaxX();

        static float getMinY();
        static float getMaxY();

        static float getMinV();
        static float getMaxV();

        static float getMinTheta();
        static float getMaxTheta();

        static void getIndices(float x, float y, float v, float theta, int &i, int &j, int &k, int &l);
        static int getID(float x, float y, float v, float theta);

        static void initializeGrid(float* r_obstacles,int num_obstacles,float angle_min,float angle_max,float angle_inc);
        static Point* computeReachability();
        static Point* computeReachability(int N);
        static void computeDeltaT();
        static cudaEvent_t global_start, global_end;
        static float global_latency;
        // static void setMode(int mode = 0);

    private:
        static int Nx, Ny, Nv, Ntheta;
        static int Na, Ndelta;
        static float x_min, y_min, v_min, theta_min;
        static float x_max, y_max, v_max, theta_max;
        static float a_min, delta_min;
        static float a_max, delta_max;
        static float delta_t;
        static float length;
        static float val_min, val_max;
        static Point* grid;
        static float horizon;
        static int mode;
    };
}
