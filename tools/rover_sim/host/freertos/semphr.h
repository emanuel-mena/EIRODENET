#pragma once
using SemaphoreHandle_t = void*;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return (void*)1; }
inline void xSemaphoreTake(SemaphoreHandle_t, int) {}
inline void xSemaphoreGive(SemaphoreHandle_t) {}
