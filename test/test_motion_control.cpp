#include "motion_control.hpp"
#include "navigation_geometry.hpp"

#include <assert.h>
#include <math.h>
#include <initializer_list>

int main(void)
{
    motion_bias_window_t window;
    motion_bias_reset(&window);
    const float still[3] = {0.0f, 0.0f, 1.0f};
    const float drift[3] = {0.0f, 0.0f, 0.35f};
    float bias = 0;
    for (unsigned i = 1; i <= 120; ++i) {
        motion_bias_add(&window, i * 10U, still, drift);
        motion_bias_add(&window, i * 10U, still, drift);
        if (i == 99) assert(!motion_bias_result(&window, &bias));
    }
    assert(window.count == 120);
    assert(motion_bias_result(&window, &bias) && fabsf(bias - 0.35f) < 0.001f);
    const float moving[3] = {0.4f, 0.0f, 1.0f};
    motion_bias_add(&window, 1210, moving, drift);
    assert(!motion_bias_result(&window, &bias));
    const float negative_drift[3] = {0.0f, 0.0f, -0.2f};
    for (unsigned i = 122; i <= 222; ++i)
        motion_bias_add(&window, i * 10U, still, negative_drift);
    assert(motion_bias_result(&window, &bias) && fabsf(bias + 0.2f) < 0.001f);
    const float spinning[3] = {4.0f, 0.0f, -0.2f};
    motion_bias_add(&window, 2230, still, spinning);
    assert(!motion_bias_result(&window, &bias));

    assert(fabsf(motion_wrap_degrees(181.0f) + 179.0f) < 0.001f);
    assert(fabsf(motion_wrap_degrees(-181.0f) - 179.0f) < 0.001f);
    int left, right;
    bool saturated;
    motion_drive_command(10, 0, 0, 0, &left, &right, &saturated);
    assert(left == 920 && right == 1000 && !saturated);
    motion_drive_command(0, 1, 0, 0, &left, &right, &saturated);
    assert(left == 1000 && right == 976 && !saturated);
    motion_drive_command(90, 0, 0, 0, &left, &right, &saturated);
    assert(left == 700 && right == 1000 && saturated);

    bool settled;
    assert(motion_turn_pwm(90, 0, 0, &settled) == 1000 && !settled);
    assert(motion_turn_pwm(-90, 0, 0, &settled) == -1000 && !settled);
    assert(motion_turn_pwm(10, 100, 0, &settled) == 0 && !settled);
    assert(motion_turn_pwm(-10, -100, 0, &settled) == 0 && !settled);
    // Braking must be symmetric, including when inertia opposes the turn.
    for (float error : {5.0f, 15.0f, 45.0f, 90.0f}) {
        for (float speed : {-180.0f, -50.0f, 0.0f, 50.0f, 180.0f}) {
            bool positive_settled, negative_settled;
            assert(motion_turn_pwm(error, speed, 0, &positive_settled) ==
                   -motion_turn_pwm(-error, -speed, 0, &negative_settled));
            assert(positive_settled == negative_settled);
        }
    }
    assert(motion_turn_pwm(2, 5, 0, &settled) == 0 && settled);
    assert(motion_turn_pwm(5, 0, 0, &settled) == 700 && !settled);
    assert(motion_turn_pwm(5, 0, 4, &settled) == 0 && !settled);
    float col = 10.0f, row = 10.0f, theta = 0.0f;
    motion_integrate_axle_pose(&col, &row, &theta, 0.0f, 90.0f,
                               navigation_axle_offset_cells(20.0f), 1.0f);
    assert(theta > 89.9f && theta < 90.1f);
    assert(row < 10.0f && fabsf(col - 10.0f) > 0.1f);
    col = row = 10.0f;
    theta = 0.0f;
    motion_integrate_axle_pose(&col, &row, &theta, 0.0f, -90.0f,
                               navigation_axle_offset_cells(20.0f), 1.0f);
    assert(theta < -89.9f && theta > -90.1f);
    assert(row > 10.0f && fabsf(col - 10.0f) > 0.1f);
    col = row = 10.0f;
    theta = 0.0f;
    motion_integrate_axle_pose(&col, &row, &theta, 1.0f, 0.0f,
                               navigation_axle_offset_cells(20.0f), 1.0f);
    assert(fabsf(col - 11.0f) < 0.001f && fabsf(row - 10.0f) < 0.001f);
    float axle_col, axle_row, center_col, center_row;
    navigation_axle_from_center(10.0f, 10.0f, 0.0f, 20.0f, &axle_col, &axle_row);
    navigation_center_from_axle(axle_col, axle_row, 0.0f, 20.0f, &center_col, &center_row);
    assert(fabsf(axle_col - 8.625f) < 0.001f && fabsf(axle_row - 10.0f) < 0.001f);
    assert(fabsf(center_col - 10.0f) < 0.001f && fabsf(center_row - 10.0f) < 0.001f);
    assert(navigation_pose_inside(3.75f, 14, 0, 43, 43));
    assert(!navigation_pose_inside(3.75f, 14, 180, 43, 43));
    assert(navigation_turn_inside(3.75f, 14, 90, -180, 43, 43));
    assert(!navigation_turn_inside(3.75f, 14, 90, 180, 43, 43));
    assert(!navigation_turn_inside_mm(3.75f, 14.0f, 0.0f, 90.0f, 43, 43, 20.0f));
    assert(navigation_segment_inside_mm(3.75f, 14.0f, 4.25f, 14.0f,
                                        0.0f, 43, 43, 20.0f));
    assert(navigation_turn_inside_mm(4.25f, 14.0f, 0.0f, 90.0f, 43, 43, 20.0f));
    assert(navigation_segment_inside_mm(4.25f, 14.0f, 4.25f, 13.0f,
                                        90.0f, 43, 43, 20.0f));
    assert(navigation_rovers_overlap(20, 20, 0, 25, 20, 180));
    assert(!navigation_rovers_overlap(20, 20, 0, 20, 26, 0));
    return 0;
}
