#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DIAGNOSTIC_TEXT_MAX 160U
#define DIAGNOSTIC_BUFFER_LINES 24U

typedef struct {
    uint32_t boot_id;
    uint32_t sequence;
    uint32_t uptime_ms;
    uint8_t reset_reason;
    char text[DIAGNOSTIC_TEXT_MAX];
} diagnostic_entry_t;

void diagnostics_service_start(uint32_t boot_id);
uint32_t diagnostics_boot_id(void);
int diagnostics_reset_reason(void);
size_t diagnostics_count(bool peer);
bool diagnostics_get(bool peer, size_t index, diagnostic_entry_t *entry);
bool diagnostics_local_next(uint32_t after, diagnostic_entry_t *entry);
void diagnostics_store_peer(const diagnostic_entry_t *entry);
