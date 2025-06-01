#include <cmath>
#include "rt_reachability/Grid.hpp"
#pragma once


namespace rt_reachability {

    void computeObstacleSet(
        float* r_obstacles,
        float angle_min,
        float angle_max,
        float angle_inc,
        float* value_func
    );

}
