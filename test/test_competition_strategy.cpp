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
    assert(COMPETITION_CUBE_DETECT_MM == 60U);
    assert(COMPETITION_CUBE_CLEARANCE_MM == 70U);
    assert(COMPETITION_CUBE_NEAR_CELLS == 6.0f);
    assert(COMPETITION_CUBE_HELD_CELLS == 3.5f);
    assert(!competition_cube_acquired(5.5f, 0, 100, 120, true, false, true));
    assert(!competition_cube_acquired(6.1f, 80, 100, 120, true, false, true));
    assert(!competition_cube_acquired(5.5f, 80, 100, 90, true, false, true));
    assert(!competition_cube_acquired(5.5f, 80, 100, 120, false, false, true));
    assert(competition_cube_acquired(5.5f, 80, 100, 120, true, false, true));
    assert(competition_cube_acquired(3.4f, 0, 0, 0, false, false, false));
    assert(!competition_cube_acquired(3.5f, 0, 0, 0, false, false, false));
    assert(!competition_cube_acquired(3.5f, 80, 100, 120, true, true, false));
    assert(COMPETITION_DETOUR_CELLS == 7.0f);
    assert(COMPETITION_DEPOT_APPROACH_CELLS == 2.5f);
    assert(competition_depot_needs_correction(2.5001f));
    assert(!competition_depot_needs_correction(2.5f));
    assert(!competition_depot_needs_correction(2.0f));
    assert(!competition_depot_needs_correction(NAN));
    assert(competition_vision_frame_is_new(12U, 11U));
    assert(!competition_vision_frame_is_new(12U, 12U));
    assert(COMPETITION_HALF_PWM == 700);
    assert(COMPETITION_DEPOT_SLOW_PWM == 700);
    assert(COMPETITION_DELIVERY_REVERSE_MS == 1000U);
    assert(COMPETITION_ASSIGNMENT_PATH_WIDTH_CELLS == 5.0f);
    assert(COMPETITION_ASSIGNMENT_CUBE_DIAMETER_CELLS == 3.0f);
    assert(COMPETITION_COMMANDER_START_DELAY_MS == 3000U);
    assert(competition_assignment_line_hits_cube(0, 0, 10, 0, 5, 0));
    assert(competition_assignment_line_hits_cube(0, 0, 10, 0, 5, 3.9f));
    assert(!competition_assignment_line_hits_cube(0, 0, 10, 0, 5, 4.1f));
    assert(!competition_assignment_line_hits_cube(0, 0, 10, 0, 5, 5));
    return 0;
}
