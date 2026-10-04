#include "motion_control.hpp"

#include <assert.h>
#include <math.h>

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
    assert(motion_turn_pwm(2, 5, 0, &settled) == 0 && settled);
    assert(motion_turn_pwm(5, 0, 0, &settled) == 700 && !settled);
    assert(motion_turn_pwm(5, 0, 4, &settled) == 0 && !settled);
    return 0;
}
