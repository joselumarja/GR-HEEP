#ifndef TRAFFIC_GENERATOR_H_
#define TRAFFIC_GENERATOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "mmio.h"
#include "traffic_generator_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TRAFFIC_GENERATOR_WAIT_FOREVER UINT32_MAX

typedef struct traffic_generator {
    mmio_region_t base_addr;
} traffic_generator_t;

typedef enum traffic_generator_result {
    TRAFFIC_GENERATOR_RESULT_OK = 0,
    TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT,
    TRAFFIC_GENERATOR_RESULT_BUSY,
    TRAFFIC_GENERATOR_RESULT_TIMEOUT,
    TRAFFIC_GENERATOR_RESULT_HARDWARE_ERROR,
    TRAFFIC_GENERATOR_RESULT_NOT_STARTED,
    TRAFFIC_GENERATOR_RESULT_VERSION_MISMATCH,
} traffic_generator_result_t;

typedef enum traffic_generator_address_mode {
    TRAFFIC_GENERATOR_ADDRESS_FIXED = 0,
    TRAFFIC_GENERATOR_ADDRESS_SEQUENTIAL,
    TRAFFIC_GENERATOR_ADDRESS_STRIDE,
    TRAFFIC_GENERATOR_ADDRESS_UNIFORM,
    TRAFFIC_GENERATOR_ADDRESS_GAUSSIAN,
} traffic_generator_address_mode_t;

typedef enum traffic_generator_temporal_mode {
    TRAFFIC_GENERATOR_TEMPORAL_SATURATED = 0,
    TRAFFIC_GENERATOR_TEMPORAL_PERIODIC,
    TRAFFIC_GENERATOR_TEMPORAL_BERNOULLI,
    TRAFFIC_GENERATOR_TEMPORAL_BURST,
} traffic_generator_temporal_mode_t;

typedef enum traffic_generator_duration_mode {
    TRAFFIC_GENERATOR_DURATION_CYCLES = 0,
    TRAFFIC_GENERATOR_DURATION_TRANSACTIONS = 1,
    TRAFFIC_GENERATOR_DURATION_INFINITE = 3,
} traffic_generator_duration_mode_t;

typedef enum traffic_generator_rw_mode {
    TRAFFIC_GENERATOR_RW_READ = 0,
    TRAFFIC_GENERATOR_RW_WRITE,
    TRAFFIC_GENERATOR_RW_ALTERNATE,
    TRAFFIC_GENERATOR_RW_RANDOM,
} traffic_generator_rw_mode_t;

typedef enum traffic_generator_write_data_mode {
    TRAFFIC_GENERATOR_WRITE_DATA_FIXED = 0,
    TRAFFIC_GENERATOR_WRITE_DATA_ADDRESS,
    TRAFFIC_GENERATOR_WRITE_DATA_RANDOM,
    TRAFFIC_GENERATOR_WRITE_DATA_INCREMENT,
} traffic_generator_write_data_mode_t;

typedef struct traffic_generator_config {
    uintptr_t base_address;
    uint32_t address_mask;
    uint32_t stride;
    uint32_t seed;
    uint32_t write_data;
    uint8_t byte_enable;
    uint32_t injection_rate;
    uint32_t period;
    uint32_t burst_length;
    uint32_t idle_length;
    uint32_t duration_limit;
    traffic_generator_address_mode_t address_mode;
    traffic_generator_temporal_mode_t temporal_mode;
    traffic_generator_duration_mode_t duration_mode;
    traffic_generator_rw_mode_t rw_mode;
    traffic_generator_write_data_mode_t write_data_mode;
    uint32_t irq_enable_mask;
} traffic_generator_config_t;

typedef struct traffic_generator_status {
    uint32_t raw;
    uint8_t state;
    bool idle;
    bool running;
    bool draining;
    bool done;
    bool error;
    bool request_pending;
    bool response_pending;
} traffic_generator_status_t;

typedef struct traffic_generator_counters {
    uint32_t elapsed_cycles;
    uint32_t requests;
    uint32_t grants;
    uint32_t completed;
    uint32_t reads;
    uint32_t writes;
    uint32_t wait_grant_cycles;
    uint32_t wait_response_cycles;
    uint32_t latency_sum;
    uint32_t latency_max;
    uint32_t missed_opportunities;
    uint32_t last_address;
    uint32_t prng_state;
    uint32_t last_read_data;
    uint32_t temporal_prng_state;
} traffic_generator_counters_t;

traffic_generator_result_t traffic_generator_init(
    traffic_generator_t *generator, mmio_region_t base_addr);

traffic_generator_result_t traffic_generator_configure(
    traffic_generator_t *generator,
    const traffic_generator_config_t *config);

traffic_generator_result_t traffic_generator_start(
    traffic_generator_t *generator);
traffic_generator_result_t traffic_generator_stop(
    traffic_generator_t *generator);
traffic_generator_result_t traffic_generator_soft_reset(
    traffic_generator_t *generator);
traffic_generator_result_t traffic_generator_clear_done(
    traffic_generator_t *generator);
traffic_generator_result_t traffic_generator_clear_errors(
    traffic_generator_t *generator);

traffic_generator_result_t traffic_generator_get_status(
    const traffic_generator_t *generator,
    traffic_generator_status_t *status);
traffic_generator_result_t traffic_generator_get_counters(
    const traffic_generator_t *generator,
    traffic_generator_counters_t *counters);

uint32_t traffic_generator_get_errors(
    const traffic_generator_t *generator);
uint32_t traffic_generator_get_irq_status(
    const traffic_generator_t *generator);
traffic_generator_result_t traffic_generator_enable_irqs(
    traffic_generator_t *generator, uint32_t irq_mask);
traffic_generator_result_t traffic_generator_disable_irqs(
    traffic_generator_t *generator, uint32_t irq_mask);
traffic_generator_result_t traffic_generator_clear_irqs(
    traffic_generator_t *generator, uint32_t irq_mask);

uint32_t traffic_generator_get_version(
    const traffic_generator_t *generator);
traffic_generator_result_t traffic_generator_wait_done(
    const traffic_generator_t *generator, uint32_t timeout_iterations);
traffic_generator_result_t traffic_generator_wait_ready(
    const traffic_generator_t *generator, uint32_t timeout_iterations);

#ifdef __cplusplus
}
#endif

#endif /* TRAFFIC_GENERATOR_H_ */
