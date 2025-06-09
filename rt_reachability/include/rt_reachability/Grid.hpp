#pragma once

namespace rt_reachability {
    class Grid {
    public:
        static void setSize(int nx, int ny, int nv, int ntheta);
        static void setLowerBounds(float xmin, float ymin, float vmin, float thetamin);
        static void setUpperBounds(float xmax, float ymax, float vmax, float thetamax);

        static int getSizeX();
        static int getSizeY();
        static int getSizeV();
        static int getSizeTheta();
        
        static float getMinX();
        static float getMaxX();
        static float getMinY();
        static float getMaxY();
        static float getMinV();
        static float getMaxV();
        static float getMinTheta();
        static float getMaxTheta();
        static void initializeGrid(float* r_obstacles,int num_obstacles,float angle_min,float angle_max,float angle_inc);
        static void computeReachability();

    private:
        static int Nx, Ny, Nv, Ntheta;
        static float x_min, y_min, v_min, theta_min;
        static float x_max, y_max, v_max, theta_max;
        static float* grid;
    };
}
