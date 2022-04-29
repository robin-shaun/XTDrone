#ifndef HELPER
#define HELPER

#include <cmath>
#include <algorithm>

#include "hybrid_astar_test/global_definition/constants.hpp"
namespace HybridAStar {
namespace Helper {
static inline float normalizeHeading(float t) {
    if ((int)t <= 0 || (int)t >= 360) {
        if (t < -0.1) {
            t += 360.f;
        } else if ((int)t >= 360) {
            t -= 360.f;
        } else {
            t =  0;
        }
    }
    return t;
}
static inline float normalizeHeadingRad(float t) {
    if (t < 0) {
        t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
        return 2.f * M_PI + t;
    }

    return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}

static inline float toDeg(float t) {
    return normalizeHeadingRad(t) * 180.f / M_PI ;
}

static inline float toRad(float t) {
    return normalizeHeadingRad(t / 180.f * M_PI);
}

static inline float clamp(float n, float lower, float upper) {
    return std::max(lower, std::min(n, upper));
}

static inline float calculation_yaw_difference(float now_yaw, float last_yaw){
    if(now_yaw - last_yaw > M_PI){
        return (2 * M_PI - now_yaw + last_yaw);
    }
    else if( now_yaw - last_yaw < -M_PI){
        return (2 * M_PI +now_yaw - last_yaw);
    }
    else{
        return (now_yaw - last_yaw);
    }
}

}
}

#endif // HELPER

