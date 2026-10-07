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
    assert(COMPETITION_CUBE_HELD_CELLS == 4.2f);
    assert(!competition_cube_acquired(5.5f, 0, 100, 120, true, false, true));
    assert(!competition_cube_acquired(6.1f, 80, 100, 120, true, false, true));
    assert(!competition_cube_acquired(5.5f, 80, 100, 90, true, false, true));
    assert(!competition_cube_acquired(5.5f, 80, 100, 120, false, false, true));
    assert(competition_cube_acquired(5.5f, 80, 100, 120, true, false, true));
    assert(competition_cube_acquired(4.1f, 0, 0, 0, false, false, false));
    assert(!competition_cube_acquired(4.2f, 0, 0, 0, false, false, false));
    assert(!competition_cube_acquired(4.2f, 80, 100, 120, true, true, false));
    assert(competition_stopping_distance(9.0f) > competition_stopping_distance(4.5f));
    assert(competition_stopping_distance(9.0f) > 2.0f);
    assert(COMPETITION_DETOUR_CELLS == 7.0f);
    assert(COMPETITION_DEPOT_APPROACH_CELLS == 2.5f);
    assert(competition_depot_needs_correction(2.5001f));
    assert(!competition_depot_needs_correction(2.5f));
    assert(!competition_depot_needs_correction(2.0f));
    assert(!competition_depot_needs_correction(NAN));
    assert(competition_vision_frame_is_new(12U, 11U));
    assert(!competition_vision_frame_is_new(12U, 12U));
    assert(COMPETITION_HALF_PWM == 700);
    assert(COMPETITION_CUBE_TURN_DUTY_PERCENT == 30U);
    assert(COMPETITION_DEPOT_SLOW_PWM == 700);
    assert(COMPETITION_DELIVERY_REVERSE_MS == 1000U);
    assert(COMPETITION_ASSIGNMENT_PATH_WIDTH_CELLS == 5.0f);
    assert(COMPETITION_ASSIGNMENT_CUBE_DIAMETER_CELLS == 3.0f);
    assert(COMPETITION_COMMANDER_START_DELAY_MS == 3000U);
    assert(competition_assignment_line_hits_cube(0, 0, 10, 0, 5, 0));
    assert(competition_assignment_line_hits_cube(0, 0, 10, 0, 5, 3.9f));
    assert(!competition_assignment_line_hits_cube(0, 0, 10, 0, 5, 4.1f));
    assert(!competition_assignment_line_hits_cube(0, 0, 10, 0, 5, 5));
    const competition_turn_obstacle_t turn_obstacles[] = {{4, -3}, {20, 3}};
    assert(competition_capture_turn_direction(0, 0, 0, 8, turn_obstacles, 2) == -1);
    assert(competition_capture_turn_direction(0, 0, 0, 4, turn_obstacles, 2) == 0);
    const competition_turn_obstacle_t right_obstacle[] = {{4, 3}};
    assert(competition_capture_turn_direction(0, 0, 0, 8, right_obstacle, 1) == 1);
    assert(competition_capture_turn_direction(0, 0, 0, NAN, right_obstacle, 1) == 0);
    return 0;
}
