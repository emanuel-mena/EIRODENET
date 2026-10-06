#include "competition_strategy.hpp"

#include <assert.h>
#include <math.h>

int main(void)
{
    assert(competition_trim_from_yaw(10.0f) == 120.0f);
    assert(competition_trim_from_yaw(40.0f) == 300.0f);
    assert(competition_trim_from_yaw(-40.0f) == -300.0f);
    assert(isnan(competition_trim_from_yaw(NAN)));
    int16_t left = 0, right = 0;
    competition_trimmed_forward(120.0f, false, &left, &right);
    assert(left == 1000 && right == 880);
    competition_trimmed_forward(-120.0f, true, &left, &right);
    assert(left == -880 && right == -1000);
    assert(COMPETITION_OBSTACLE_MM == 30U);
    assert(COMPETITION_DETOUR_CELLS == 7.0f);
    assert(COMPETITION_DETOUR_MIN_DISTANCE_CELLS == 5.0f);
    assert(COMPETITION_DEPOT_STOP_CELLS == 6.0f);
    assert(COMPETITION_HALF_PWM == 700);
    return 0;
}
