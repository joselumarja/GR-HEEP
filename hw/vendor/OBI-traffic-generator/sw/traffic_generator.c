#include "traffic_generator.h"

#include <stddef.h>

static inline bool generator_valid(const traffic_generator_t *generator) {
    return generator != NULL;
}

static inline uint32_t generator_read32(
    const traffic_generator_t *generator, ptrdiff_t offset) {
    return mmio_region_read32(generator->base_addr, offset);
}

static inline void generator_write32(
    traffic_generator_t *generator, ptrdiff_t offset, uint32_t value) {
    mmio_region_write32(generator->base_addr, offset, value);
}

static inline void generator_io_fence(void) {
#if defined(__riscv)
    __asm__ volatile("fence iorw, iorw" ::: "memory");
#else
    __asm__ volatile("" ::: "memory");
#endif
}

static inline bool bit_is_set(uint32_t value, uint32_t mask) {
    return (value & mask) != 0u;
}

static bool config_valid(const traffic_generator_config_t *config) {
    if (config == NULL ||
        (uint32_t)config->address_mode > TRAFFIC_GENERATOR_ADDRESS_GAUSSIAN ||
        (uint32_t)config->temporal_mode > TRAFFIC_GENERATOR_TEMPORAL_BURST ||
        ((uint32_t)config->duration_mode != TRAFFIC_GENERATOR_DURATION_CYCLES &&
         (uint32_t)config->duration_mode !=
             TRAFFIC_GENERATOR_DURATION_TRANSACTIONS &&
         (uint32_t)config->duration_mode != TRAFFIC_GENERATOR_DURATION_INFINITE) ||
        (uint32_t)config->rw_mode > TRAFFIC_GENERATOR_RW_RANDOM ||
        (uint32_t)config->write_data_mode >
            TRAFFIC_GENERATOR_WRITE_DATA_INCREMENT) {
        return false;
    }
    if ((config->base_address & 0x3u) != 0u ||
        (config->base_address & config->address_mask) != 0u ||
        (config->address_mask & 0x3u) != 0u ||
        (config->byte_enable & 0x0fu) == 0u) {
        return false;
    }
    if (config->temporal_mode == TRAFFIC_GENERATOR_TEMPORAL_PERIODIC &&
        config->period == 0u) {
        return false;
    }
    if (config->temporal_mode == TRAFFIC_GENERATOR_TEMPORAL_BURST &&
        config->burst_length == 0u) {
        return false;
    }
    if (config->address_mode == TRAFFIC_GENERATOR_ADDRESS_STRIDE &&
        (config->stride & 0x3u) != 0u) {
        return false;
    }
    return true;
}

static uint32_t encode_config(const traffic_generator_config_t *config) {
    return ((uint32_t)config->address_mode
            << TRAFFIC_GENERATOR_CONFIG_ADDRESS_MODE_OFFSET) |
           ((uint32_t)config->temporal_mode
            << TRAFFIC_GENERATOR_CONFIG_TEMPORAL_MODE_OFFSET) |
           ((uint32_t)config->duration_mode
            << TRAFFIC_GENERATOR_CONFIG_DURATION_MODE_OFFSET) |
           ((uint32_t)config->rw_mode
            << TRAFFIC_GENERATOR_CONFIG_RW_MODE_OFFSET) |
           ((uint32_t)config->write_data_mode
            << TRAFFIC_GENERATOR_CONFIG_WRITE_DATA_MODE_OFFSET);
}

static void decode_status(uint32_t raw, traffic_generator_status_t *status) {
    status->raw = raw;
    status->state = (uint8_t)((raw >> TRAFFIC_GENERATOR_STATUS_STATE_OFFSET) &
                              TRAFFIC_GENERATOR_STATUS_STATE_MASK);
    status->idle = bit_is_set(raw, TRAFFIC_GENERATOR_STATUS_IDLE_MASK);
    status->running = bit_is_set(raw, TRAFFIC_GENERATOR_STATUS_RUNNING_MASK);
    status->draining = bit_is_set(raw, TRAFFIC_GENERATOR_STATUS_DRAINING_MASK);
    status->done = bit_is_set(raw, TRAFFIC_GENERATOR_STATUS_DONE_MASK);
    status->error = bit_is_set(raw, TRAFFIC_GENERATOR_STATUS_ERROR_MASK);
    status->request_pending =
        bit_is_set(raw, TRAFFIC_GENERATOR_STATUS_REQUEST_PENDING_MASK);
    status->response_pending =
        bit_is_set(raw, TRAFFIC_GENERATOR_STATUS_RESPONSE_PENDING_MASK);
}

traffic_generator_result_t traffic_generator_init(
    traffic_generator_t *generator, mmio_region_t base_addr) {
    if (generator == NULL) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    generator->base_addr = base_addr;
    generator_write32(generator, TRAFFIC_GENERATOR_IRQ_ENABLE_REG_OFFSET, 0u);
    generator_write32(generator, TRAFFIC_GENERATOR_CONTROL_REG_OFFSET,
                      TRAFFIC_GENERATOR_CONTROL_SOFT_RESET_MASK);
    generator_io_fence();

    traffic_generator_result_t result =
        traffic_generator_wait_ready(generator, 1024u);
    if (result != TRAFFIC_GENERATOR_RESULT_OK) {
        return result;
    }
    return traffic_generator_get_version(generator) ==
                   TRAFFIC_GENERATOR_EXPECTED_VERSION
               ? TRAFFIC_GENERATOR_RESULT_OK
               : TRAFFIC_GENERATOR_RESULT_VERSION_MISMATCH;
}

traffic_generator_result_t traffic_generator_configure(
    traffic_generator_t *generator,
    const traffic_generator_config_t *config) {
    if (!generator_valid(generator) || !config_valid(config)) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
#if UINTPTR_MAX > UINT32_MAX
    if (config->base_address > UINT32_MAX) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
#endif

    traffic_generator_status_t status;
    traffic_generator_result_t result =
        traffic_generator_get_status(generator, &status);
    if (result != TRAFFIC_GENERATOR_RESULT_OK) {
        return result;
    }
    if (status.running || status.draining) {
        return TRAFFIC_GENERATOR_RESULT_BUSY;
    }

    generator_write32(generator, TRAFFIC_GENERATOR_BASE_ADDR_REG_OFFSET,
                      (uint32_t)config->base_address);
    generator_write32(generator, TRAFFIC_GENERATOR_ADDRESS_MASK_REG_OFFSET,
                      config->address_mask);
    generator_write32(generator, TRAFFIC_GENERATOR_STRIDE_REG_OFFSET,
                      config->stride);
    generator_write32(generator, TRAFFIC_GENERATOR_SEED_REG_OFFSET,
                      config->seed);
    generator_write32(generator, TRAFFIC_GENERATOR_WRITE_DATA_REG_OFFSET,
                      config->write_data);
    generator_write32(generator, TRAFFIC_GENERATOR_BYTE_ENABLE_REG_OFFSET,
                      config->byte_enable & 0x0fu);
    generator_write32(generator, TRAFFIC_GENERATOR_INJECTION_RATE_REG_OFFSET,
                      config->injection_rate);
    generator_write32(generator, TRAFFIC_GENERATOR_PERIOD_REG_OFFSET,
                      config->period);
    generator_write32(generator, TRAFFIC_GENERATOR_BURST_LENGTH_REG_OFFSET,
                      config->burst_length);
    generator_write32(generator, TRAFFIC_GENERATOR_IDLE_LENGTH_REG_OFFSET,
                      config->idle_length);
    generator_write32(generator, TRAFFIC_GENERATOR_DURATION_LIMIT_REG_OFFSET,
                      config->duration_limit);
    generator_write32(generator, TRAFFIC_GENERATOR_CONFIG_REG_OFFSET,
                      encode_config(config));
    generator_write32(generator, TRAFFIC_GENERATOR_IRQ_ENABLE_REG_OFFSET,
                      config->irq_enable_mask & TRAFFIC_GENERATOR_IRQ_ALL_MASK);
    generator_write32(generator, TRAFFIC_GENERATOR_CONTROL_REG_OFFSET,
                      TRAFFIC_GENERATOR_CONTROL_CLEAR_DONE_MASK |
                          TRAFFIC_GENERATOR_CONTROL_CLEAR_ERROR_MASK);
    generator_write32(generator, TRAFFIC_GENERATOR_IRQ_STATUS_REG_OFFSET,
                      TRAFFIC_GENERATOR_IRQ_ALL_MASK);
    generator_io_fence();
    return TRAFFIC_GENERATOR_RESULT_OK;
}

traffic_generator_result_t traffic_generator_start(
    traffic_generator_t *generator) {
    if (!generator_valid(generator)) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    traffic_generator_status_t status;
    traffic_generator_result_t result =
        traffic_generator_get_status(generator, &status);
    if (result != TRAFFIC_GENERATOR_RESULT_OK) {
        return result;
    }
    if (status.running || status.draining) {
        return TRAFFIC_GENERATOR_RESULT_BUSY;
    }
    generator_write32(generator, TRAFFIC_GENERATOR_CONTROL_REG_OFFSET,
                      TRAFFIC_GENERATOR_CONTROL_START_MASK |
                          TRAFFIC_GENERATOR_CONTROL_CLEAR_DONE_MASK |
                          TRAFFIC_GENERATOR_CONTROL_CLEAR_ERROR_MASK);
    generator_io_fence();
    return TRAFFIC_GENERATOR_RESULT_OK;
}

traffic_generator_result_t traffic_generator_stop(
    traffic_generator_t *generator) {
    if (!generator_valid(generator)) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    generator_write32(generator, TRAFFIC_GENERATOR_CONTROL_REG_OFFSET,
                      TRAFFIC_GENERATOR_CONTROL_STOP_MASK);
    generator_io_fence();
    return TRAFFIC_GENERATOR_RESULT_OK;
}

traffic_generator_result_t traffic_generator_soft_reset(
    traffic_generator_t *generator) {
    if (!generator_valid(generator)) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    traffic_generator_status_t status;
    traffic_generator_result_t result =
        traffic_generator_get_status(generator, &status);
    if (result != TRAFFIC_GENERATOR_RESULT_OK) {
        return result;
    }
    if (status.running || status.draining) {
        return TRAFFIC_GENERATOR_RESULT_BUSY;
    }
    generator_write32(generator, TRAFFIC_GENERATOR_CONTROL_REG_OFFSET,
                      TRAFFIC_GENERATOR_CONTROL_SOFT_RESET_MASK);
    generator_io_fence();
    return traffic_generator_wait_ready(generator, 1024u);
}

traffic_generator_result_t traffic_generator_clear_done(
    traffic_generator_t *generator) {
    if (!generator_valid(generator)) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    generator_write32(generator, TRAFFIC_GENERATOR_CONTROL_REG_OFFSET,
                      TRAFFIC_GENERATOR_CONTROL_CLEAR_DONE_MASK);
    return TRAFFIC_GENERATOR_RESULT_OK;
}

traffic_generator_result_t traffic_generator_clear_errors(
    traffic_generator_t *generator) {
    if (!generator_valid(generator)) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    generator_write32(generator, TRAFFIC_GENERATOR_CONTROL_REG_OFFSET,
                      TRAFFIC_GENERATOR_CONTROL_CLEAR_ERROR_MASK);
    return TRAFFIC_GENERATOR_RESULT_OK;
}

traffic_generator_result_t traffic_generator_get_status(
    const traffic_generator_t *generator,
    traffic_generator_status_t *status) {
    if (!generator_valid(generator) || status == NULL) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    decode_status(generator_read32(generator, TRAFFIC_GENERATOR_STATUS_REG_OFFSET),
                  status);
    return TRAFFIC_GENERATOR_RESULT_OK;
}

traffic_generator_result_t traffic_generator_get_counters(
    const traffic_generator_t *generator,
    traffic_generator_counters_t *counters) {
    if (!generator_valid(generator) || counters == NULL) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    counters->elapsed_cycles = generator_read32(
        generator, TRAFFIC_GENERATOR_ELAPSED_CYCLES_REG_OFFSET);
    counters->requests = generator_read32(
        generator, TRAFFIC_GENERATOR_REQUESTS_REG_OFFSET);
    counters->grants = generator_read32(
        generator, TRAFFIC_GENERATOR_GRANTS_REG_OFFSET);
    counters->completed = generator_read32(
        generator, TRAFFIC_GENERATOR_COMPLETED_REG_OFFSET);
    counters->reads = generator_read32(
        generator, TRAFFIC_GENERATOR_READS_REG_OFFSET);
    counters->writes = generator_read32(
        generator, TRAFFIC_GENERATOR_WRITES_REG_OFFSET);
    counters->wait_grant_cycles = generator_read32(
        generator, TRAFFIC_GENERATOR_WAIT_GRANT_CYCLES_REG_OFFSET);
    counters->wait_response_cycles = generator_read32(
        generator, TRAFFIC_GENERATOR_WAIT_RESPONSE_CYCLES_REG_OFFSET);
    counters->latency_sum = generator_read32(
        generator, TRAFFIC_GENERATOR_LATENCY_SUM_REG_OFFSET);
    counters->latency_max = generator_read32(
        generator, TRAFFIC_GENERATOR_LATENCY_MAX_REG_OFFSET);
    counters->missed_opportunities = generator_read32(
        generator, TRAFFIC_GENERATOR_MISSED_OPPORTUNITIES_REG_OFFSET);
    counters->last_address = generator_read32(
        generator, TRAFFIC_GENERATOR_LAST_ADDRESS_REG_OFFSET);
    counters->prng_state = generator_read32(
        generator, TRAFFIC_GENERATOR_PRNG_STATE_REG_OFFSET);
    counters->last_read_data = generator_read32(
        generator, TRAFFIC_GENERATOR_LAST_READ_DATA_REG_OFFSET);
    counters->temporal_prng_state = generator_read32(
        generator, TRAFFIC_GENERATOR_TEMPORAL_PRNG_STATE_REG_OFFSET);
    return TRAFFIC_GENERATOR_RESULT_OK;
}

uint32_t traffic_generator_get_errors(
    const traffic_generator_t *generator) {
    return generator_valid(generator)
               ? generator_read32(generator,
                                  TRAFFIC_GENERATOR_ERROR_STATUS_REG_OFFSET)
               : 0u;
}

uint32_t traffic_generator_get_irq_status(
    const traffic_generator_t *generator) {
    return generator_valid(generator)
               ? generator_read32(generator,
                                  TRAFFIC_GENERATOR_IRQ_STATUS_REG_OFFSET) &
                     TRAFFIC_GENERATOR_IRQ_ALL_MASK
               : 0u;
}

traffic_generator_result_t traffic_generator_enable_irqs(
    traffic_generator_t *generator, uint32_t irq_mask) {
    if (!generator_valid(generator)) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    uint32_t enabled = generator_read32(
        generator, TRAFFIC_GENERATOR_IRQ_ENABLE_REG_OFFSET);
    generator_write32(generator, TRAFFIC_GENERATOR_IRQ_ENABLE_REG_OFFSET,
                      enabled | (irq_mask & TRAFFIC_GENERATOR_IRQ_ALL_MASK));
    return TRAFFIC_GENERATOR_RESULT_OK;
}

traffic_generator_result_t traffic_generator_disable_irqs(
    traffic_generator_t *generator, uint32_t irq_mask) {
    if (!generator_valid(generator)) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    uint32_t enabled = generator_read32(
        generator, TRAFFIC_GENERATOR_IRQ_ENABLE_REG_OFFSET);
    generator_write32(generator, TRAFFIC_GENERATOR_IRQ_ENABLE_REG_OFFSET,
                      enabled & ~(irq_mask & TRAFFIC_GENERATOR_IRQ_ALL_MASK));
    return TRAFFIC_GENERATOR_RESULT_OK;
}

traffic_generator_result_t traffic_generator_clear_irqs(
    traffic_generator_t *generator, uint32_t irq_mask) {
    if (!generator_valid(generator)) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    generator_write32(generator, TRAFFIC_GENERATOR_IRQ_STATUS_REG_OFFSET,
                      irq_mask & TRAFFIC_GENERATOR_IRQ_ALL_MASK);
    return TRAFFIC_GENERATOR_RESULT_OK;
}

uint32_t traffic_generator_get_version(
    const traffic_generator_t *generator) {
    return generator_valid(generator)
               ? generator_read32(generator,
                                  TRAFFIC_GENERATOR_VERSION_REG_OFFSET)
               : 0u;
}

traffic_generator_result_t traffic_generator_wait_done(
    const traffic_generator_t *generator, uint32_t timeout_iterations) {
    if (!generator_valid(generator)) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    uint32_t iteration = 0u;
    bool observed_active = false;
    for (;;) {
        traffic_generator_status_t status;
        traffic_generator_result_t result =
            traffic_generator_get_status(generator, &status);
        if (result != TRAFFIC_GENERATOR_RESULT_OK) {
            return result;
        }
        observed_active = observed_active || status.running || status.draining ||
                          status.done;
        if (status.error) {
            return TRAFFIC_GENERATOR_RESULT_HARDWARE_ERROR;
        }
        if (status.done) {
            return TRAFFIC_GENERATOR_RESULT_OK;
        }
        if (!observed_active && status.idle) {
            return TRAFFIC_GENERATOR_RESULT_NOT_STARTED;
        }
        if (timeout_iterations != TRAFFIC_GENERATOR_WAIT_FOREVER &&
            iteration++ >= timeout_iterations) {
            return TRAFFIC_GENERATOR_RESULT_TIMEOUT;
        }
    }
}

traffic_generator_result_t traffic_generator_wait_ready(
    const traffic_generator_t *generator, uint32_t timeout_iterations) {
    if (!generator_valid(generator)) {
        return TRAFFIC_GENERATOR_RESULT_BAD_ARGUMENT;
    }
    uint32_t iteration = 0u;
    for (;;) {
        traffic_generator_status_t status;
        traffic_generator_result_t result =
            traffic_generator_get_status(generator, &status);
        if (result != TRAFFIC_GENERATOR_RESULT_OK) {
            return result;
        }
        if (status.idle || status.done) {
            return TRAFFIC_GENERATOR_RESULT_OK;
        }
        if (timeout_iterations != TRAFFIC_GENERATOR_WAIT_FOREVER &&
            iteration++ >= timeout_iterations) {
            return TRAFFIC_GENERATOR_RESULT_TIMEOUT;
        }
    }
}
