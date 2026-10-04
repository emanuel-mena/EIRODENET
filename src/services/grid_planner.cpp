#include "grid_planner.hpp"

#include <limits.h>
#include <stddef.h>

static int32_t s_cost[GRID_PLANNER_MAX_CELLS];
static uint16_t s_parent[GRID_PLANNER_MAX_CELLS];
static uint8_t s_state[GRID_PLANNER_MAX_CELLS];

static uint16_t cell_index(uint8_t col, uint8_t row, uint8_t cols)
{
    return (uint16_t)row * cols + col;
}

static int octile(uint8_t col, uint8_t row, grid_planner_cell_t goal)
{
    const int dx = col > goal.col ? col - goal.col : goal.col - col;
    const int dy = row > goal.row ? row - goal.row : goal.row - row;
    const int diagonal = dx < dy ? dx : dy;
    const int straight = dx + dy - 2 * diagonal;
    return 14 * diagonal + 10 * straight;
}

esp_err_t grid_planner_plan(uint8_t cols, uint8_t rows, const uint8_t *blocked,
                            grid_planner_cell_t start, grid_planner_cell_t goal,
                            grid_planner_route_t *route)
{
    if (blocked == NULL || route == NULL || cols == 0 || rows == 0 ||
        cols > GRID_PLANNER_MAX_DIM || rows > GRID_PLANNER_MAX_DIM ||
        start.col >= cols || start.row >= rows || goal.col >= cols || goal.row >= rows) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint16_t total = (uint16_t)cols * rows;
    const uint16_t start_index = cell_index(start.col, start.row, cols);
    const uint16_t goal_index = cell_index(goal.col, goal.row, cols);
    route->count = 0;
    if (blocked[goal_index]) return ESP_ERR_NOT_FOUND;
    for (uint16_t i = 0; i < total; ++i) {
        s_cost[i] = INT32_MAX;
        s_parent[i] = UINT16_MAX;
        s_state[i] = 0;
    }
    s_cost[start_index] = 0;
    s_state[start_index] = 1;
    static const int8_t directions[8][2] = {
        {1, 0}, {1, -1}, {0, -1}, {-1, -1},
        {-1, 0}, {-1, 1}, {0, 1}, {1, 1},
    };
    while (true) {
        uint16_t current = UINT16_MAX;
        int32_t best = INT32_MAX;
        for (uint16_t i = 0; i < total; ++i) {
            if (s_state[i] != 1) continue;
            const uint8_t col = (uint8_t)(i % cols);
            const uint8_t row = (uint8_t)(i / cols);
            const int32_t score = s_cost[i] + octile(col, row, goal);
            if (score < best) {
                best = score;
                current = i;
            }
        }
        if (current == UINT16_MAX) return ESP_ERR_NOT_FOUND;
        if (current == goal_index) break;
        s_state[current] = 2;
        const int current_col = current % cols;
        const int current_row = current / cols;
        for (size_t d = 0; d < 8; ++d) {
            const int next_col = current_col + directions[d][0];
            const int next_row = current_row + directions[d][1];
            if (next_col < 0 || next_row < 0 || next_col >= cols || next_row >= rows) continue;
            const uint16_t next = cell_index((uint8_t)next_col, (uint8_t)next_row, cols);
            if (blocked[next] || s_state[next] == 2) continue;
            const bool diagonal = directions[d][0] != 0 && directions[d][1] != 0;
            if (diagonal) {
                const uint16_t horizontal = cell_index((uint8_t)next_col,
                                                        (uint8_t)current_row, cols);
                const uint16_t vertical = cell_index((uint8_t)current_col,
                                                      (uint8_t)next_row, cols);
                if (blocked[horizontal] || blocked[vertical]) continue;
            }
            const int32_t candidate = s_cost[current] + (diagonal ? 14 : 10);
            if (candidate >= s_cost[next]) continue;
            s_cost[next] = candidate;
            s_parent[next] = current;
            s_state[next] = 1;
        }
    }
    uint16_t cursor = goal_index;
    while (true) {
        if (route->count >= GRID_PLANNER_MAX_CELLS) return ESP_ERR_INVALID_SIZE;
        route->cells[route->count++] = (grid_planner_cell_t){
            .col = (uint8_t)(cursor % cols), .row = (uint8_t)(cursor / cols)};
        if (cursor == start_index) break;
        cursor = s_parent[cursor];
        if (cursor == UINT16_MAX) return ESP_ERR_INVALID_STATE;
    }
    for (uint16_t left = 0, right = route->count - 1; left < right; ++left, --right) {
        const grid_planner_cell_t swap = route->cells[left];
        route->cells[left] = route->cells[right];
        route->cells[right] = swap;
    }
    return ESP_OK;
}
