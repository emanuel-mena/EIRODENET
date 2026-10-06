#include "tinyml_policy.hpp"

#include <algorithm>
#include <math.h>
#include <new>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"

#ifndef EIRO_HOST_SIM
#include "esp_partition.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#endif

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr uint32_t kHeaderBytes = 16;
const char *TAG = "tinyml_policy";
tinyml_policy_status_t s_status = {};

float clip(float value)
{
    return fmaxf(-1.0f, fminf(1.0f, value));
}

void relative_body(const vision_status_t *v, float col, float row, float *forward, float *left)
{
    const float a = v->theta_deg * kPi / 180.0f;
    const float dx = col - v->col, dy = row - v->row;
    *forward = dx * cosf(a) - dy * sinf(a);
    *left = dx * sinf(a) + dy * cosf(a);
}

void put_relative(float *out, size_t &index, const vision_status_t *v,
                  float col, float row, float scale)
{
    float forward = 0, left = 0;
    relative_body(v, col, row, &forward, &left);
    out[index++] = clip(forward / scale);
    out[index++] = clip(left / scale);
}

float ray_to_edge(const vision_status_t *v, float dx, float dy)
{
    float distance = INFINITY;
    if (dx > 0.0001f) distance = fminf(distance, (v->grid_cols - v->col) / dx);
    if (dx < -0.0001f) distance = fminf(distance, -v->col / dx);
    if (dy > 0.0001f) distance = fminf(distance, (v->grid_rows - v->row) / dy);
    if (dy < -0.0001f) distance = fminf(distance, -v->row / dy);
    return isfinite(distance) ? fmaxf(0.0f, distance) : 0.0f;
}

uint32_t crc32_zlib(const uint8_t *data, size_t length)
{
    uint32_t crc = 0xffffffffU;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320U & (uint32_t)-(int32_t)(crc & 1U));
    }
    return crc ^ 0xffffffffU;
}

#ifndef EIRO_HOST_SIM
struct ImageHeader { char magic[4]; uint32_t version, length, crc32; };
static_assert(sizeof(ImageHeader) == kHeaderBytes, "Cabecera EIRM inesperada");
alignas(16) uint8_t s_arena[TINYML_POLICY_ARENA_BYTES];
alignas(tflite::MicroInterpreter) uint8_t s_interpreter_storage[sizeof(tflite::MicroInterpreter)];
tflite::MicroInterpreter *s_interpreter;
const uint8_t *s_mapped;
esp_partition_mmap_handle_t s_map_handle;

bool shape(const TfLiteTensor *tensor, int a, int b)
{
    return tensor && tensor->dims && tensor->dims->size == 2 &&
           tensor->dims->data[0] == a && tensor->dims->data[1] == b;
}

bool validate_graph(const tflite::Model *model)
{
    if (!model || model->version() != TFLITE_SCHEMA_VERSION || !model->subgraphs() ||
        model->subgraphs()->size() != 1) return false;
    const auto *graph = model->subgraphs()->Get(0);
    if (!graph || !graph->operators() || graph->operators()->size() != 4 ||
        !model->operator_codes()) return false;
    const int expected_widths[] = {64, 64, 2};
    for (int index = 0; index < 4; ++index) {
        const auto *op = graph->operators()->Get(index);
        const auto *code = model->operator_codes()->Get(op->opcode_index());
        if (!code) return false;
        const auto builtin = code->builtin_code();
        if (index < 3) {
            if (builtin != tflite::BuiltinOperator_FULLY_CONNECTED ||
                !op->outputs() || op->outputs()->size() != 1 ||
                !graph->tensors()) return false;
            const auto *tensor = graph->tensors()->Get(op->outputs()->Get(0));
            if (!tensor || !tensor->shape() || tensor->shape()->size() != 2 ||
                tensor->shape()->Get(1) != expected_widths[index]) return false;
            const auto *options = op->builtin_options_as_FullyConnectedOptions();
            if (!options) return false;
            const auto activation = options->fused_activation_function();
            if (activation != (index < 2 ? tflite::ActivationFunctionType_RELU
                                         : tflite::ActivationFunctionType_NONE)) return false;
        } else if (builtin != tflite::BuiltinOperator_TANH) return false;
    }
    return true;
}
#else
extern "C" bool tinyml_host_infer(const float observation[TINYML_POLICY_INPUTS],
                                  float output[TINYML_POLICY_OUTPUTS]);
#endif
} // namespace

esp_err_t tinyml_policy_build_observation(
    const tinyml_policy_context_t *context, const vision_status_t *vision,
    const rover_sensor_state_t *sensors, const rover_imu_state_t *imu,
    float out[TINYML_POLICY_INPUTS])
{
    if (!context || !vision || !sensors || !imu || !out ||
        context->phase >= TINYML_PHASE_COUNT || context->color >= VISION_MAX_CUBES)
        return ESP_ERR_INVALID_ARG;
    memset(out, 0, sizeof(float) * TINYML_POLICY_INPUTS);
    const float scale = fmaxf(1.0f, fmaxf(vision->grid_cols, vision->grid_rows));
    size_t n = 0;
    out[n++] = context->commander ? 1.0f : -1.0f;
    for (uint8_t phase = 0; phase < TINYML_PHASE_COUNT; ++phase)
        out[n++] = phase == context->phase ? 1.0f : -1.0f;
    for (uint8_t color = 0; color < VISION_MAX_CUBES; ++color)
        out[n++] = color == context->color ? 1.0f : -1.0f;

    const float radians = vision->theta_deg * kPi / 180.0f;
    out[n++] = clip(2.0f * vision->col / fmaxf(1.0f, vision->grid_cols) - 1.0f);
    out[n++] = clip(2.0f * vision->row / fmaxf(1.0f, vision->grid_rows) - 1.0f);
    out[n++] = sinf(radians);
    out[n++] = cosf(radians);
    out[n++] = clip(context->linear_speed_cells_s / 7.5f);
    out[n++] = clip(imu->sample.gyro_dps[2] / 180.0f);
    out[n++] = clip(2.0f * fminf(750.0f, (float)vision->age_ms) / 750.0f - 1.0f);
    out[n++] = vision->pose_valid ? 1.0f : -1.0f;
    out[n++] = clip(context->previous_linear);
    out[n++] = clip(context->previous_angular);

    put_relative(out, n, vision, vision->cubes[context->color].col,
                 vision->cubes[context->color].row, scale);
    put_relative(out, n, vision, vision->depot_col[context->color],
                 vision->depot_row[context->color], scale);
    out[n++] = clip((vision->depot_col[context->color] - vision->cubes[context->color].col) / scale);
    out[n++] = clip((vision->depot_row[context->color] - vision->cubes[context->color].row) / scale);
    out[n++] = vision->cube_valid[context->color] && vision->depot_valid[context->color] ? 1.0f : -1.0f;
    out[n++] = vision->cube_in_depot[context->color] ? 1.0f : -1.0f;

    put_relative(out, n, vision, vision->peer_col, vision->peer_row, scale);
    const float peer_angle = (vision->peer_theta_deg - vision->theta_deg) * kPi / 180.0f;
    out[n++] = sinf(peer_angle);
    out[n++] = cosf(peer_angle);
    out[n++] = vision->peer_valid ? 1.0f : -1.0f;
    out[n++] = context->peer_active ? 1.0f : -1.0f;
    out[n++] = clip(2.0f * fminf(1500.0f, (float)vision->peer_age_ms) / 1500.0f - 1.0f);
    out[n++] = context->conflict ? 1.0f : -1.0f;

    for (uint8_t color = 0; color < VISION_MAX_CUBES; ++color) {
        if (color == context->color) continue;
        put_relative(out, n, vision, vision->cubes[color].col, vision->cubes[color].row, scale);
        out[n++] = vision->cube_valid[color] ? 1.0f : -1.0f;
        out[n++] = vision->cube_in_depot[color] ? 1.0f : -1.0f;
    }

    struct Obstacle { float distance; uint8_t index; } nearest[VISION_MAX_OBSTACLES];
    uint8_t count = 0;
    for (uint8_t i = 0; i < vision->obstacle_count; ++i) {
        if (vision->obstacles[i].age_ms > 750U) continue;
        const float dx = vision->obstacles[i].col - vision->col;
        const float dy = vision->obstacles[i].row - vision->row;
        nearest[count++] = {dx * dx + dy * dy, i};
    }
    std::sort(nearest, nearest + count,
              [](const Obstacle &a, const Obstacle &b) { return a.distance < b.distance; });
    for (uint8_t slot = 0; slot < 4; ++slot) {
        if (slot < count) {
            const auto &obstacle = vision->obstacles[nearest[slot].index];
            put_relative(out, n, vision, obstacle.col, obstacle.row, scale);
            out[n++] = 1.0f;
        } else {
            out[n++] = 0.0f;
            out[n++] = 0.0f;
            out[n++] = -1.0f;
        }
    }

    const float forward_x = cosf(radians), forward_y = -sinf(radians);
    const float left_x = -forward_y, left_y = forward_x;
    out[n++] = clip(ray_to_edge(vision, forward_x, forward_y) / scale);
    out[n++] = clip(ray_to_edge(vision, -forward_x, -forward_y) / scale);
    out[n++] = clip(ray_to_edge(vision, left_x, left_y) / scale);
    out[n++] = clip(ray_to_edge(vision, -left_x, -left_y) / scale);

    out[n++] = sensors->ultrasonic_valid
        ? clip(2.0f * fminf(400.0f, (float)sensors->distance_mm) / 400.0f - 1.0f) : 1.0f;
    out[n++] = sensors->ultrasonic_valid ? 1.0f : -1.0f;
    const uint16_t infrared[] = {sensors->infrared.front_left, sensors->infrared.front_right,
                                 sensors->infrared.rear_left, sensors->infrared.rear_right};
    for (uint16_t value : infrared) out[n++] = clip((float)value / 2047.5f - 1.0f);
    out[n++] = imu->valid && imu->calibration_valid ? 1.0f : -1.0f;
    out[n++] = sensors->infrared_valid ? 1.0f : -1.0f;
    return n == TINYML_POLICY_INPUTS ? ESP_OK : ESP_ERR_INVALID_SIZE;
}

esp_err_t tinyml_policy_start(void)
{
    tinyml_policy_reset();
#ifdef EIRO_HOST_SIM
    s_status.available = true;
    s_status.version = TINYML_POLICY_MODEL_VERSION;
    s_status.arena_bytes = TINYML_POLICY_ARENA_BYTES;
    s_status.input_count = TINYML_POLICY_INPUTS;
    s_status.output_count = TINYML_POLICY_OUTPUTS;
    return ESP_OK;
#else
    const esp_partition_t *partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0x40, "model");
    if (!partition) { s_status.error = ESP_ERR_NOT_FOUND; return s_status.error; }
    ImageHeader header = {};
    esp_err_t err = esp_partition_read(partition, 0, &header, sizeof(header));
    if (err != ESP_OK || memcmp(header.magic, "EIRM", 4) != 0 ||
        header.version != TINYML_POLICY_MODEL_VERSION || header.length < 8 ||
        header.length > partition->size - sizeof(header)) {
        s_status.error = err == ESP_OK ? ESP_ERR_INVALID_RESPONSE : err;
        return s_status.error;
    }
    const void *mapped = nullptr;
    err = esp_partition_mmap(partition, 0, sizeof(header) + header.length,
                             ESP_PARTITION_MMAP_DATA, &mapped, &s_map_handle);
    if (err != ESP_OK) { s_status.error = err; return err; }
    s_mapped = static_cast<const uint8_t *>(mapped);
    const uint8_t *flatbuffer = s_mapped + sizeof(header);
    if (memcmp(flatbuffer + 4, "TFL3", 4) != 0 ||
        crc32_zlib(flatbuffer, header.length) != header.crc32) {
        s_status.error = ESP_ERR_INVALID_CRC;
        return s_status.error;
    }
    const tflite::Model *model = tflite::GetModel(flatbuffer);
    if (!validate_graph(model)) { s_status.error = ESP_ERR_NOT_SUPPORTED; return s_status.error; }
    static tflite::MicroMutableOpResolver<2> resolver;
    if (resolver.AddFullyConnected() != kTfLiteOk || resolver.AddTanh() != kTfLiteOk) {
        s_status.error = ESP_ERR_NO_MEM; return s_status.error;
    }
    s_interpreter = new (s_interpreter_storage) tflite::MicroInterpreter(
        model, resolver, s_arena, sizeof(s_arena));
    if (s_interpreter->AllocateTensors() != kTfLiteOk ||
        !shape(s_interpreter->input(0), 1, TINYML_POLICY_INPUTS) ||
        !shape(s_interpreter->output(0), 1, TINYML_POLICY_OUTPUTS) ||
        s_interpreter->input(0)->type != kTfLiteInt8 ||
        s_interpreter->output(0)->type != kTfLiteInt8 ||
        s_interpreter->input(0)->params.scale <= 0 ||
        s_interpreter->output(0)->params.scale <= 0) {
        s_status.error = ESP_ERR_INVALID_SIZE; return s_status.error;
    }
    s_status = {.available = true, .version = header.version, .length = header.length,
                .crc32 = header.crc32, .arena_bytes = TINYML_POLICY_ARENA_BYTES,
                .arena_used_bytes = (uint32_t)s_interpreter->arena_used_bytes(),
                .input_count = TINYML_POLICY_INPUTS, .output_count = TINYML_POLICY_OUTPUTS,
                .last_latency_us = 0, .inference_count = 0,
                .error = ESP_OK};
    ESP_LOGI(TAG, "EIRM v%lu: %lu bytes CRC32=%08lx tensor=[1,%u]->[1,%u] arena=%lu/%u",
             (unsigned long)header.version, (unsigned long)header.length,
             (unsigned long)header.crc32, (unsigned)TINYML_POLICY_INPUTS,
             (unsigned)TINYML_POLICY_OUTPUTS, (unsigned long)s_status.arena_used_bytes,
             (unsigned)TINYML_POLICY_ARENA_BYTES);
    return ESP_OK;
#endif
}

void tinyml_policy_reset(void)
{
    memset(&s_status, 0, sizeof(s_status));
    s_status.error = ESP_ERR_INVALID_STATE;
#ifndef EIRO_HOST_SIM
    s_interpreter = nullptr;
    if (s_mapped) {
        esp_partition_munmap(s_map_handle);
        s_mapped = nullptr;
    }
#endif
}

void tinyml_policy_get_status(tinyml_policy_status_t *status)
{
    if (status) *status = s_status;
}

bool tinyml_policy_ready(void) { return s_status.available && s_status.error == ESP_OK; }

esp_err_t tinyml_policy_infer(
    const tinyml_policy_context_t *context, const vision_status_t *vision,
    const rover_sensor_state_t *sensors, const rover_imu_state_t *imu,
    float *linear, float *angular)
{
    if (!linear || !angular) return ESP_ERR_INVALID_ARG;
    *linear = *angular = 0;
    if (!tinyml_policy_ready()) return s_status.error;
    float observation[TINYML_POLICY_INPUTS];
    esp_err_t err = tinyml_policy_build_observation(context, vision, sensors, imu, observation);
    if (err != ESP_OK) return err;
    const int64_t started = esp_timer_get_time();
#ifdef EIRO_HOST_SIM
    float output[TINYML_POLICY_OUTPUTS] = {};
    if (!tinyml_host_infer(observation, output)) err = ESP_FAIL;
    else { *linear = clip(output[0]); *angular = clip(output[1]); }
#else
    TfLiteTensor *input = s_interpreter->input(0);
    for (size_t i = 0; i < TINYML_POLICY_INPUTS; ++i) {
        const int value = (int)lroundf(observation[i] / input->params.scale) + input->params.zero_point;
        input->data.int8[i] = (int8_t)std::max(-128, std::min(127, value));
    }
    if (s_interpreter->Invoke() != kTfLiteOk) err = ESP_FAIL;
    else {
        const TfLiteTensor *output = s_interpreter->output(0);
        *linear = clip((output->data.int8[0] - output->params.zero_point) * output->params.scale);
        *angular = clip((output->data.int8[1] - output->params.zero_point) * output->params.scale);
    }
#endif
    s_status.last_latency_us = (uint32_t)(esp_timer_get_time() - started);
    if (err == ESP_OK) ++s_status.inference_count;
    else { s_status.available = false; s_status.error = err; }
    return err;
}

#ifdef EIRO_HOST_SIM
void tinyml_policy_set_host_model(uint32_t version, uint32_t length, uint32_t crc32,
                                  bool available)
{
    s_status.available = available;
    s_status.version = version;
    s_status.length = length;
    s_status.crc32 = crc32;
    s_status.arena_bytes = TINYML_POLICY_ARENA_BYTES;
    s_status.input_count = TINYML_POLICY_INPUTS;
    s_status.output_count = TINYML_POLICY_OUTPUTS;
    s_status.error = available ? ESP_OK : ESP_ERR_INVALID_STATE;
}
#endif
