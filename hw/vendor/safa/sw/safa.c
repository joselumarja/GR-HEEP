#include "safa.h"

#include <stddef.h>
#include <stdio.h>

static inline bool safa_valid(const safa_t *safa) {
    return safa != NULL;
}

static inline uint32_t safa_read32(const safa_t *safa, ptrdiff_t offset) {
    return mmio_region_read32(safa->base_addr, offset);
}

static inline void safa_write32(safa_t *safa,
                                ptrdiff_t offset,
                                uint32_t value) {
    mmio_region_write32(safa->base_addr, offset, value);
}

/* Keep configuration writes ordered before START/DMA activity. */
static inline void safa_io_fence(void) {
#if defined(__riscv)
    __asm__ volatile("fence iorw, iorw" ::: "memory");
#else
    __asm__ volatile("" ::: "memory");
#endif
}

static inline bool bit_is_set(uint32_t value, uint32_t mask) {
    return (value & mask) != 0u;
}

static void decode_status(uint32_t raw, safa_status_t *status) {
    status->raw = raw;
    status->state = (uint8_t)((raw >> SAFA_STATUS_STATE_OFFSET) &
                              SAFA_STATUS_STATE_MASK);

    status->idle = bit_is_set(raw, SAFA_STATUS_IDLE_MASK);
    status->busy = bit_is_set(raw, SAFA_STATUS_BUSY_MASK);
    status->done = bit_is_set(raw, SAFA_STATUS_DONE_MASK);
    status->aborted = bit_is_set(raw, SAFA_STATUS_ABORTED_MASK);
    status->error = bit_is_set(raw, SAFA_STATUS_ERROR_MASK);
    status->start_wait = bit_is_set(raw, SAFA_STATUS_START_WAIT_MASK);
    status->running = bit_is_set(raw, SAFA_STATUS_RUNNING_MASK);
    status->draining = bit_is_set(raw, SAFA_STATUS_DRAINING_MASK);

    status->ap_idle = bit_is_set(raw, SAFA_STATUS_AP_IDLE_MASK);
    status->ap_ready = bit_is_set(raw, SAFA_STATUS_AP_READY_MASK);
    status->ap_start = bit_is_set(raw, SAFA_STATUS_AP_START_MASK);
    status->ap_done = bit_is_set(raw, SAFA_STATUS_AP_DONE_MASK);

    status->input_fifo_empty =
        bit_is_set(raw, SAFA_STATUS_IN_FIFO_EMPTY_MASK);
    status->input_fifo_full =
        bit_is_set(raw, SAFA_STATUS_IN_FIFO_FULL_MASK);
    status->output_fifo_empty =
        bit_is_set(raw, SAFA_STATUS_OUT_FIFO_EMPTY_MASK);
    status->output_fifo_full =
        bit_is_set(raw, SAFA_STATUS_OUT_FIFO_FULL_MASK);
    status->auto_start = bit_is_set(raw, SAFA_STATUS_AUTO_START_MASK);
    status->hls_done_seen =
        bit_is_set(raw, SAFA_STATUS_HLS_DONE_SEEN_MASK);
}

safa_result_t safa_init(safa_t *safa, mmio_region_t base_addr) {
    if (safa == NULL) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }

    safa->base_addr = base_addr;

    safa_write32(safa, SAFA_IRQ_ENABLE_REG_OFFSET, 0u);
    safa_write32(safa, SAFA_CONFIG_REG_OFFSET, 0u);
    safa_write32(safa, SAFA_INPUT_WORDS_REG_OFFSET, 0u);
    safa_write32(safa, SAFA_OUTPUT_WORDS_REG_OFFSET, 0u);
    safa_write32(safa, SAFA_CONTROL_REG_OFFSET, SAFA_CONTROL_SOFT_RESET_MASK);

    safa_io_fence();

    safa_result_t result = safa_wait_ready(safa, 1024u);
    if (result != SAFA_RESULT_OK) {
        return result;
    }

    safa_write32(safa, SAFA_IRQ_STATUS_REG_OFFSET, SAFA_IRQ_ALL_MASK);
    safa_write32(safa, SAFA_ERROR_STATUS_REG_OFFSET, SAFA_ERROR_ALL_MASK);

    return (safa_get_version(safa) == SAFA_EXPECTED_VERSION)
               ? SAFA_RESULT_OK
               : SAFA_RESULT_VERSION_MISMATCH;
}

safa_result_t safa_configure(safa_t *safa, const safa_config_t *config) {
    if (!safa_valid(safa) || config == NULL) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }

    safa_status_t status;
    safa_result_t result = safa_get_status(safa, &status);
    if (result != SAFA_RESULT_OK) {
        return result;
    }
    if (status.busy) {
        return SAFA_RESULT_BUSY;
    }

    safa_write32(safa, SAFA_INPUT_WORDS_REG_OFFSET, config->input_words);
    safa_write32(safa, SAFA_OUTPUT_WORDS_REG_OFFSET, config->output_words);
    safa_write32(safa,
                 SAFA_CONFIG_REG_OFFSET,
                 config->auto_start ? SAFA_CONFIG_AUTO_START_MASK : 0u);
    safa_write32(safa,
                 SAFA_IRQ_ENABLE_REG_OFFSET,
                 config->irq_enable_mask & SAFA_IRQ_ALL_MASK);

    /* Clear sticky state left by the previous transaction. */
    safa_write32(safa,
                 SAFA_CONTROL_REG_OFFSET,
                 SAFA_CONTROL_CLEAR_DONE_MASK |
                     SAFA_CONTROL_CLEAR_ERROR_MASK |
                     SAFA_CONTROL_CLEAR_ABORTED_MASK);
    safa_write32(safa, SAFA_IRQ_STATUS_REG_OFFSET, SAFA_IRQ_ALL_MASK);
    safa_io_fence();

    return SAFA_RESULT_OK;
}

safa_result_t safa_start(safa_t *safa) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }

    safa_status_t status;
    safa_result_t result = safa_get_status(safa, &status);
    if (result != SAFA_RESULT_OK) {
        return result;
    }
    if (status.busy) {
        return SAFA_RESULT_BUSY;
    }



    safa_write32(safa,
                 SAFA_CONTROL_REG_OFFSET,
                 SAFA_CONTROL_START_MASK |
                     SAFA_CONTROL_CLEAR_DONE_MASK |
                     SAFA_CONTROL_CLEAR_ERROR_MASK |
                     SAFA_CONTROL_CLEAR_ABORTED_MASK);
    safa_io_fence();
    return SAFA_RESULT_OK;
}

safa_result_t safa_abort(safa_t *safa) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }

    safa_write32(safa, SAFA_CONTROL_REG_OFFSET, SAFA_CONTROL_STOP_MASK);
    safa_io_fence();
    return SAFA_RESULT_OK;
}

safa_result_t safa_soft_reset(safa_t *safa) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }

    safa_write32(safa,
                 SAFA_CONTROL_REG_OFFSET,
                 SAFA_CONTROL_SOFT_RESET_MASK);
    safa_io_fence();
    return safa_wait_ready(safa, 1024u);
}

safa_result_t safa_clear_done(safa_t *safa) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }
    safa_write32(safa,
                 SAFA_CONTROL_REG_OFFSET,
                 SAFA_CONTROL_CLEAR_DONE_MASK);
    return SAFA_RESULT_OK;
}

safa_result_t safa_clear_aborted(safa_t *safa) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }
    safa_write32(safa,
                 SAFA_CONTROL_REG_OFFSET,
                 SAFA_CONTROL_CLEAR_ABORTED_MASK);
    return SAFA_RESULT_OK;
}

safa_result_t safa_clear_errors(safa_t *safa) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }
    safa_write32(safa,
                 SAFA_CONTROL_REG_OFFSET,
                 SAFA_CONTROL_CLEAR_ERROR_MASK);
    return SAFA_RESULT_OK;
}

safa_result_t safa_get_status(const safa_t *safa, safa_status_t *status) {
    if (!safa_valid(safa) || status == NULL) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }

    decode_status(safa_read32(safa, SAFA_STATUS_REG_OFFSET), status);
    return SAFA_RESULT_OK;
}

uint32_t safa_get_errors(const safa_t *safa) {
    if (!safa_valid(safa)) {
        return 0u;
    }
    return safa_read32(safa, SAFA_ERROR_STATUS_REG_OFFSET);
}

safa_result_t safa_clear_error_mask(safa_t *safa, uint32_t error_mask) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }
    safa_write32(safa,
                 SAFA_ERROR_STATUS_REG_OFFSET,
                 error_mask & SAFA_ERROR_ALL_MASK);
    return SAFA_RESULT_OK;
}

uint32_t safa_get_irq_status(const safa_t *safa) {
    if (!safa_valid(safa)) {
        return 0u;
    }
    return safa_read32(safa, SAFA_IRQ_STATUS_REG_OFFSET) & SAFA_IRQ_ALL_MASK;
}

safa_result_t safa_enable_irqs(safa_t *safa, uint32_t irq_mask) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }
    uint32_t enabled = safa_read32(safa, SAFA_IRQ_ENABLE_REG_OFFSET);
    enabled |= irq_mask & SAFA_IRQ_ALL_MASK;
    safa_write32(safa, SAFA_IRQ_ENABLE_REG_OFFSET, enabled);
    return SAFA_RESULT_OK;
}

safa_result_t safa_disable_irqs(safa_t *safa, uint32_t irq_mask) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }
    uint32_t enabled = safa_read32(safa, SAFA_IRQ_ENABLE_REG_OFFSET);
    enabled &= ~(irq_mask & SAFA_IRQ_ALL_MASK);
    safa_write32(safa, SAFA_IRQ_ENABLE_REG_OFFSET, enabled);
    return SAFA_RESULT_OK;
}

safa_result_t safa_clear_irqs(safa_t *safa, uint32_t irq_mask) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }
    safa_write32(safa,
                 SAFA_IRQ_STATUS_REG_OFFSET,
                 irq_mask & SAFA_IRQ_ALL_MASK);
    return SAFA_RESULT_OK;
}

safa_result_t safa_get_counters(const safa_t *safa,
                                safa_counters_t *counters) {
    if (!safa_valid(safa) || counters == NULL) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }

    counters->input_accepted =
        safa_read32(safa, SAFA_INPUT_ACCEPTED_REG_OFFSET);
    counters->input_consumed =
        safa_read32(safa, SAFA_INPUT_CONSUMED_REG_OFFSET);
    counters->output_generated =
        safa_read32(safa, SAFA_OUTPUT_GENERATED_REG_OFFSET);
    counters->output_popped =
        safa_read32(safa, SAFA_OUTPUT_POPPED_REG_OFFSET);
    counters->input_fifo_level =
        safa_read32(safa, SAFA_IN_FIFO_LEVEL_REG_OFFSET);
    counters->output_fifo_level =
        safa_read32(safa, SAFA_OUT_FIFO_LEVEL_REG_OFFSET);
    counters->active_cycles =
        safa_read32(safa, SAFA_ACTIVE_CYCLES_REG_OFFSET);
    counters->input_stall_cycles =
        safa_read32(safa, SAFA_INPUT_STALLS_REG_OFFSET);
    counters->output_stall_cycles =
        safa_read32(safa, SAFA_OUTPUT_STALLS_REG_OFFSET);
    counters->dma_push_stall_cycles =
        safa_read32(safa, SAFA_DMA_PUSH_STALLS_REG_OFFSET);
    counters->dma_pop_stall_cycles =
        safa_read32(safa, SAFA_DMA_POP_STALLS_REG_OFFSET);

    return SAFA_RESULT_OK;
}

uint32_t safa_get_version(const safa_t *safa) {
    if (!safa_valid(safa)) {
        return 0u;
    }
    return safa_read32(safa, SAFA_VERSION_REG_OFFSET);
}

safa_result_t safa_wait_done(const safa_t *safa,
                             uint32_t timeout_iterations) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }

    uint32_t iteration = 0u;
    bool observed_active = false;

    for (;;) {
        safa_status_t status;
        safa_result_t result = safa_get_status(safa, &status);
        if (result != SAFA_RESULT_OK) {
            return result;
        }

        observed_active = observed_active || status.busy || status.done;

        /* Count mismatches set ERROR in the same completion cycle, so errors
         * intentionally have priority over DONE. */
        if (status.error) {
            return SAFA_RESULT_HARDWARE_ERROR;
        }
        if (status.aborted) {
            return SAFA_RESULT_ABORTED;
        }
        if (status.done) {
            return SAFA_RESULT_OK;
        }

        if (!status.busy && !observed_active && !status.auto_start) {
            return SAFA_RESULT_NOT_STARTED;
        }

        if (timeout_iterations != SAFA_WAIT_FOREVER) {
            if (iteration >= timeout_iterations) {
                return SAFA_RESULT_TIMEOUT;
            }
            ++iteration;
        }
    }
}

safa_result_t safa_wait_ready(const safa_t *safa,
                              uint32_t timeout_iterations) {
    if (!safa_valid(safa)) {
        return SAFA_RESULT_BAD_ARGUMENT;
    }

    uint32_t iteration = 0u;
    for (;;) {
        safa_status_t status;
        safa_result_t result = safa_get_status(safa, &status);
        if (result != SAFA_RESULT_OK) {
            return result;
        }

        if (!status.busy && status.state != SAFA_STATE_ABORT_RESET) {
            return SAFA_RESULT_OK;
        }

        if (timeout_iterations != SAFA_WAIT_FOREVER) {
            if (iteration >= timeout_iterations) {
                return SAFA_RESULT_TIMEOUT;
            }
            ++iteration;
        }
    }
}
