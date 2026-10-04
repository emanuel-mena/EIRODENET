#include "diagnostics_service.hpp"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

typedef struct {
    diagnostic_entry_t lines[DIAGNOSTIC_BUFFER_LINES];
    size_t head;
    size_t count;
} diagnostic_ring_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static diagnostic_ring_t s_local;
static diagnostic_ring_t s_peer;
static vprintf_like_t s_previous_vprintf;
static uint32_t s_boot_id;
static int s_reset_reason;
static uint32_t s_sequence;

static void append(diagnostic_ring_t *ring, const diagnostic_entry_t *entry)
{
    ring->lines[ring->head] = *entry;
    ring->head = (ring->head + 1) % DIAGNOSTIC_BUFFER_LINES;
    if (ring->count < DIAGNOSTIC_BUFFER_LINES) ++ring->count;
}

static int capture_log(const char *format, va_list args)
{
    char line[DIAGNOSTIC_TEXT_MAX];
    va_list copy;
    va_copy(copy, args);
    vsnprintf(line, sizeof(line), format, copy);
    va_end(copy);
    const int result = s_previous_vprintf(format, args);
    size_t length = strlen(line);
    while (length > 0 && (line[length - 1] == '\n' || line[length - 1] == '\r'))
        line[--length] = '\0';
    if (length > 0) {
        diagnostic_entry_t entry = {
            .boot_id = s_boot_id,
            .uptime_ms = (uint32_t)(esp_timer_get_time() / 1000),
            .reset_reason = (uint8_t)s_reset_reason,
        };
        memcpy(entry.text, line, length + 1);
        taskENTER_CRITICAL(&s_lock);
        entry.sequence = ++s_sequence;
        append(&s_local, &entry);
        taskEXIT_CRITICAL(&s_lock);
    }
    return result;
}

void diagnostics_service_start(uint32_t boot_id)
{
    s_boot_id = boot_id;
    s_reset_reason = (int)esp_reset_reason();
    s_previous_vprintf = esp_log_set_vprintf(capture_log);
}

uint32_t diagnostics_boot_id(void) { return s_boot_id; }
int diagnostics_reset_reason(void) { return s_reset_reason; }

size_t diagnostics_count(bool peer)
{
    taskENTER_CRITICAL(&s_lock);
    const size_t count = peer ? s_peer.count : s_local.count;
    taskEXIT_CRITICAL(&s_lock);
    return count;
}

bool diagnostics_get(bool peer, size_t index, diagnostic_entry_t *entry)
{
    if (entry == NULL) return false;
    taskENTER_CRITICAL(&s_lock);
    const diagnostic_ring_t *ring = peer ? &s_peer : &s_local;
    const bool found = index < ring->count;
    if (found) *entry = ring->lines[(ring->head + DIAGNOSTIC_BUFFER_LINES - ring->count + index)
                                    % DIAGNOSTIC_BUFFER_LINES];
    taskEXIT_CRITICAL(&s_lock);
    return found;
}

bool diagnostics_local_next(uint32_t after, diagnostic_entry_t *entry)
{
    if (entry == NULL) return false;
    taskENTER_CRITICAL(&s_lock);
    bool found = false;
    for (size_t i = 0; i < s_local.count; ++i) {
        const diagnostic_entry_t *candidate = &s_local.lines[
            (s_local.head + DIAGNOSTIC_BUFFER_LINES - s_local.count + i) % DIAGNOSTIC_BUFFER_LINES];
        if (candidate->sequence > after) { *entry = *candidate; found = true; break; }
    }
    taskEXIT_CRITICAL(&s_lock);
    return found;
}

void diagnostics_store_peer(const diagnostic_entry_t *entry)
{
    if (entry == NULL || entry->text[DIAGNOSTIC_TEXT_MAX - 1] != '\0') return;
    taskENTER_CRITICAL(&s_lock);
    if (s_peer.count > 0) {
        const diagnostic_entry_t *last = &s_peer.lines[
            (s_peer.head + DIAGNOSTIC_BUFFER_LINES - 1) % DIAGNOSTIC_BUFFER_LINES];
        if (last->boot_id > entry->boot_id ||
            (last->boot_id == entry->boot_id && last->sequence >= entry->sequence)) {
            taskEXIT_CRITICAL(&s_lock);
            return;
        }
    }
    append(&s_peer, entry);
    taskEXIT_CRITICAL(&s_lock);
}
