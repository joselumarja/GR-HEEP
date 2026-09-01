#ifndef SAFA_H_
#define SAFA_H_

#include <stdbool.h>
#include <stdint.h>

#include "mmio.h"
#include "safa_regs.h"
#include "gr_heep.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SAFA_WAIT_FOREVER UINT32_MAX

typedef struct safa {
    mmio_region_t base_addr;
} safa_t;

typedef enum safa_result {
    SAFA_RESULT_OK = 0,
    SAFA_RESULT_BAD_ARGUMENT,
    SAFA_RESULT_BUSY,
    SAFA_RESULT_TIMEOUT,
    SAFA_RESULT_ABORTED,
    SAFA_RESULT_HARDWARE_ERROR,
    SAFA_RESULT_NOT_STARTED,
    SAFA_RESULT_VERSION_MISMATCH,
} safa_result_t;

typedef struct safa_config {
    uint32_t input_words;
    uint32_t output_words;
    bool auto_start;
    uint32_t irq_enable_mask;
} safa_config_t;

typedef struct safa_status {
    uint32_t raw;
    uint8_t state;

    bool idle;
    bool busy;
    bool done;
    bool aborted;
    bool error;
    bool start_wait;
    bool running;
    bool draining;

    bool ap_idle;
    bool ap_ready;
    bool ap_start;
    bool ap_done;

    bool input_fifo_empty;
    bool input_fifo_full;
    bool output_fifo_empty;
    bool output_fifo_full;
    bool auto_start;
    bool hls_done_seen;
} safa_status_t;

typedef struct safa_counters {
    uint32_t input_accepted;
    uint32_t input_consumed;
    uint32_t output_generated;
    uint32_t output_popped;
    uint32_t input_fifo_level;
    uint32_t output_fifo_level;
    uint32_t active_cycles;
    uint32_t input_stall_cycles;
    uint32_t output_stall_cycles;
    uint32_t dma_push_stall_cycles;
    uint32_t dma_pop_stall_cycles;
} safa_counters_t;

/**
 * Initialize a software handle and place the peripheral in a known state.
 * The supplied base address should normally be:
 * mmio_region_from_addr((uintptr_t)SAFA_START_ADDRESS)
 */
safa_result_t safa_init(safa_t *safa, mmio_region_t base_addr);

/** Configure word counts, auto-start mode and interrupt enables. */
safa_result_t safa_configure(safa_t *safa, const safa_config_t *config);

/** Send an explicit START command. Not needed when AUTO_START is enabled. */
safa_result_t safa_start(safa_t *safa);

/** Abort the current transaction and reset the HLS block and local FIFOs. */
safa_result_t safa_abort(safa_t *safa);

/** Reset all local state, FIFOs, counters, errors and interrupt status. */
safa_result_t safa_soft_reset(safa_t *safa);

/** Clear sticky completion, aborted and error flags. */
safa_result_t safa_clear_done(safa_t *safa);
safa_result_t safa_clear_aborted(safa_t *safa);
safa_result_t safa_clear_errors(safa_t *safa);

/** Read and decode the STATUS register. */
safa_result_t safa_get_status(const safa_t *safa, safa_status_t *status);

/** Read hardware error bits. */
uint32_t safa_get_errors(const safa_t *safa);

/** Clear selected ERROR_STATUS bits (W1C). */
safa_result_t safa_clear_error_mask(safa_t *safa, uint32_t error_mask);

/** Read or configure accelerator interrupts. */
uint32_t safa_get_irq_status(const safa_t *safa);
safa_result_t safa_enable_irqs(safa_t *safa, uint32_t irq_mask);
safa_result_t safa_disable_irqs(safa_t *safa, uint32_t irq_mask);
safa_result_t safa_clear_irqs(safa_t *safa, uint32_t irq_mask);

/** Read transaction counters and FIFO occupancy. */
safa_result_t safa_get_counters(const safa_t *safa,
                                safa_counters_t *counters);

/** Read the RTL version register. */
uint32_t safa_get_version(const safa_t *safa);

/** Wait until DONE, ABORTED, ERROR, or timeout. */
safa_result_t safa_wait_done(const safa_t *safa, uint32_t timeout_iterations);

/** Wait until the controller is no longer busy. DONE counts as ready. */
safa_result_t safa_wait_ready(const safa_t *safa,
                              uint32_t timeout_iterations);

#ifdef __cplusplus
}
#endif

#endif /* SAFA_H_ */
