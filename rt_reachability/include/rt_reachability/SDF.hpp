#include <cmath>
#include "rt_reachability/Grid.hpp"
#pragma once


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

}
