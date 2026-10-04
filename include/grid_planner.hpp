#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#define GRID_PLANNER_MAX_DIM 64U
#define GRID_PLANNER_MAX_CELLS (GRID_PLANNER_MAX_DIM * GRID_PLANNER_MAX_DIM)

typedef struct {
    uint8_t col;
    uint8_t row;
} grid_planner_cell_t;

typedef struct {
    uint16_t count;
    grid_planner_cell_t cells[GRID_PLANNER_MAX_CELLS];
} grid_planner_route_t;

/** Planifica sobre ocho vecinos con costes 10/14 y sin cortar esquinas. */
esp_err_t grid_planner_plan(uint8_t cols, uint8_t rows, const uint8_t *blocked,
                            grid_planner_cell_t start, grid_planner_cell_t goal,
                            grid_planner_route_t *route);
