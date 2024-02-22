
#include "utils.h"

float wrap_pi(const float heading) {
    //constrain heading to +/-pi

    const double wrapped = heading > M_PI ? heading - M_TWOPI : heading < -M_TWOPI ? heading + M_TWOPI : heading;
    return static_cast<float>(wrapped);
}