#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "csr.h"
#include "gr_heep.h"
#include "traffic_generator.h"

#define REGION_WORDS 64u
#define REGION_BYTES (REGION_WORDS * sizeof(uint32_t))
#define REGION_MASK  (REGION_BYTES - sizeof(uint32_t))
#define WAIT_TIMEOUT 1000000u
#define ANY_COUNT    UINT32_MAX

static volatile uint32_t traffic_region[REGION_WORDS]
    __attribute__((aligned(REGION_BYTES)));

typedef struct trace_points {
    uint32_t configure_begin;
    uint32_t configure_end;
    uint32_t start_begin;
    uint32_t start_end;
    uint32_t stop_begin;
    uint32_t stop_end;
    uint32_t wait_begin;
    uint32_t done_seen;
} trace_points_t;

typedef struct case_result {
    traffic_generator_result_t result;
    traffic_generator_status_t status;
    traffic_generator_counters_t counters;
    trace_points_t trace;
    uint32_t irq_status;
    uint32_t error_status;
} case_result_t;

static uint32_t cycle_now(void) {
    uint32_t cycle;
    CSR_READ(CSR_REG_MCYCLE, &cycle);
    return cycle;
}

static void memory_fence(void) {
    __asm__ volatile("fence iorw, iorw" ::: "memory");
}

static void wait_cycles(uint32_t cycles) {
    uint32_t start = cycle_now();
    while ((uint32_t)(cycle_now() - start) < cycles) {
    }
}

static void prepare_region(uint32_t value) {
    for (uint32_t i = 0; i < REGION_WORDS; ++i) {
        traffic_region[i] = value;
    }
    memory_fence();
}

static traffic_generator_config_t default_config(void) {
    traffic_generator_config_t config = {
        .base_address = (uintptr_t)traffic_region,
        .address_mask = REGION_MASK,
        .stride = sizeof(uint32_t),
        .seed = 0x00000001u,
        .write_data = 0x10203040u,
        .byte_enable = 0x0fu,
        .injection_rate = UINT32_MAX,
        .period = 1,
        .burst_length = 1,
        .idle_length = 0,
        .duration_limit = 8,
        .address_mode = TRAFFIC_GENERATOR_ADDRESS_FIXED,
        .temporal_mode = TRAFFIC_GENERATOR_TEMPORAL_SATURATED,
        .duration_mode = TRAFFIC_GENERATOR_DURATION_TRANSACTIONS,
        .rw_mode = TRAFFIC_GENERATOR_RW_READ,
        .write_data_mode = TRAFFIC_GENERATOR_WRITE_DATA_FIXED,
        .irq_enable_mask = TRAFFIC_GENERATOR_IRQ_ALL_MASK,
    };
    return config;
}

static case_result_t execute_case(traffic_generator_t *generator,
                                  const traffic_generator_config_t *config,
                                  uint32_t manual_stop_after_cycles) {
    case_result_t outcome = {0};

    outcome.trace.configure_begin = cycle_now();
    outcome.result = traffic_generator_configure(generator, config);
    outcome.trace.configure_end = cycle_now();
    if (outcome.result != TRAFFIC_GENERATOR_RESULT_OK) {
        return outcome;
    }

    outcome.trace.start_begin = cycle_now();
    outcome.result = traffic_generator_start(generator);
    outcome.trace.start_end = cycle_now();
    if (outcome.result != TRAFFIC_GENERATOR_RESULT_OK) {
        return outcome;
    }

    if (manual_stop_after_cycles != 0) {
        wait_cycles(manual_stop_after_cycles);
        outcome.trace.stop_begin = cycle_now();
        outcome.result = traffic_generator_stop(generator);
        outcome.trace.stop_end = cycle_now();
        if (outcome.result != TRAFFIC_GENERATOR_RESULT_OK) {
            return outcome;
        }
    }

    outcome.trace.wait_begin = cycle_now();
    outcome.result = traffic_generator_wait_done(generator, WAIT_TIMEOUT);
    outcome.trace.done_seen = cycle_now();
    memory_fence();
    traffic_generator_get_status(generator, &outcome.status);
    traffic_generator_get_counters(generator, &outcome.counters);
    outcome.irq_status = traffic_generator_get_irq_status(generator);
    outcome.error_status = traffic_generator_get_errors(generator);
    return outcome;
}

static bool common_checks(const case_result_t *outcome,
                          const traffic_generator_config_t *config,
                          uint32_t expected_completed) {
    const traffic_generator_counters_t *counters = &outcome->counters;
    bool pass = outcome->result == TRAFFIC_GENERATOR_RESULT_OK &&
                outcome->status.done && !outcome->status.error &&
                outcome->error_status == 0 &&
                (outcome->irq_status & TRAFFIC_GENERATOR_IRQ_DONE_MASK) != 0 &&
                counters->requests == counters->grants &&
                counters->grants == counters->completed &&
                counters->reads + counters->writes == counters->completed;

    if (expected_completed != ANY_COUNT) {
        pass = pass && counters->completed == expected_completed;
    }
    if (config->duration_mode == TRAFFIC_GENERATOR_DURATION_CYCLES) {
        pass = pass && counters->elapsed_cycles == config->duration_limit;
    }
    if (config->rw_mode == TRAFFIC_GENERATOR_RW_READ) {
        pass = pass && counters->reads == counters->completed && counters->writes == 0;
    } else if (config->rw_mode == TRAFFIC_GENERATOR_RW_WRITE) {
        pass = pass && counters->writes == counters->completed && counters->reads == 0;
    } else if (config->rw_mode == TRAFFIC_GENERATOR_RW_ALTERNATE) {
        pass = pass && counters->reads == (counters->completed + 1) / 2 &&
               counters->writes == counters->completed / 2;
    }
    return pass;
}

static bool address_in_region(uint32_t address) {
    uint32_t base = (uint32_t)(uintptr_t)traffic_region;
    return (address & ~REGION_MASK) == base && (address & 0x3u) == 0;
}

static bool verify_sequential_writes(uint32_t count, uint32_t first_value) {
    memory_fence();
    for (uint32_t i = 0; i < count; ++i) {
        if (traffic_region[i] != first_value + i) {
            return false;
        }
    }
    return true;
}

static bool verify_stride_writes(uint32_t count, uint32_t stride_bytes) {
    memory_fence();
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t index = (i * stride_bytes) / sizeof(uint32_t);
        uint32_t expected = (uint32_t)(uintptr_t)&traffic_region[index];
        if (traffic_region[index] != expected) {
            return false;
        }
    }
    return true;
}

static void report_case(const char *name, const case_result_t *outcome, bool pass) {
    const trace_points_t *trace = &outcome->trace;
    const traffic_generator_counters_t *counters = &outcome->counters;

    printf("TRACE %-18s cfg=[%" PRIu32 ",%" PRIu32 "] "
           "start=[%" PRIu32 ",%" PRIu32 "] "
           "stop=[%" PRIu32 ",%" PRIu32 "] wait=%" PRIu32
           " done_seen=%" PRIu32 " hw_elapsed=%" PRIu32
           " req=%" PRIu32 " gnt=%" PRIu32 " done=%" PRIu32
           " rd=%" PRIu32 " wr=%" PRIu32 " last=0x%08" PRIx32 " %s\n",
           name, trace->configure_begin, trace->configure_end,
           trace->start_begin, trace->start_end, trace->stop_begin,
           trace->stop_end, trace->wait_begin, trace->done_seen,
           counters->elapsed_cycles, counters->requests, counters->grants,
           counters->completed, counters->reads, counters->writes,
           counters->last_address, pass ? "PASS" : "FAIL");

    if (!pass) {
        printf("  result=%u status=0x%08" PRIx32 " irq=0x%08" PRIx32
               " errors=0x%08" PRIx32 " wait_gnt=%" PRIu32
               " wait_rsp=%" PRIu32 " latency_sum=%" PRIu32
               " latency_max=%" PRIu32 "\n",
               (unsigned)outcome->result, outcome->status.raw,
               outcome->irq_status, outcome->error_status,
               counters->wait_grant_cycles, counters->wait_response_cycles,
               counters->latency_sum, counters->latency_max);
    }
}

int main(void) {
    traffic_generator_t generator;
    uint32_t failures = 0;

    CSR_CLEAR_BITS(CSR_REG_MCOUNTINHIBIT, 0x1);
    CSR_WRITE(CSR_REG_MCYCLEH, 0);
    CSR_WRITE(CSR_REG_MCYCLE, 0);

    uint32_t init_begin = cycle_now();
    traffic_generator_result_t init_result = traffic_generator_init(
        &generator,
        mmio_region_from_addr(OBI_TRAFFIC_GENERATOR_PERIPH_START_ADDRESS));
    uint32_t init_end = cycle_now();
    printf("TRACE init begin=%" PRIu32 " end=%" PRIu32
           " version=0x%08" PRIx32 " result=%u\n",
           init_begin, init_end, traffic_generator_get_version(&generator),
           (unsigned)init_result);
    if (init_result != TRAFFIC_GENERATOR_RESULT_OK) {
        return 1;
    }

    traffic_generator_config_t config = default_config();
    prepare_region(0xa5a5a5a5);
    case_result_t outcome = execute_case(&generator, &config, 0);
    bool pass = common_checks(&outcome, &config, config.duration_limit) &&
                outcome.counters.last_address == (uint32_t)(uintptr_t)traffic_region &&
                outcome.counters.last_read_data == 0xa5a5a5a5;
    report_case("fixed-read", &outcome, pass);
    failures += !pass;

    config = default_config();
    config.address_mode = TRAFFIC_GENERATOR_ADDRESS_SEQUENTIAL;
    config.rw_mode = TRAFFIC_GENERATOR_RW_WRITE;
    config.write_data_mode = TRAFFIC_GENERATOR_WRITE_DATA_INCREMENT;
    prepare_region(0xdeadbeef);
    outcome = execute_case(&generator, &config, 0);
    pass = common_checks(&outcome, &config, config.duration_limit) &&
           verify_sequential_writes(config.duration_limit, config.write_data);
    report_case("sequential-write", &outcome, pass);
    failures += !pass;

    config = default_config();
    config.address_mode = TRAFFIC_GENERATOR_ADDRESS_STRIDE;
    config.stride = 16;
    config.rw_mode = TRAFFIC_GENERATOR_RW_WRITE;
    config.write_data_mode = TRAFFIC_GENERATOR_WRITE_DATA_ADDRESS;
    prepare_region(0xdeadbeef);
    outcome = execute_case(&generator, &config, 0);
    pass = common_checks(&outcome, &config, config.duration_limit) &&
           verify_stride_writes(config.duration_limit, config.stride);
    report_case("stride-write", &outcome, pass);
    failures += !pass;

    config = default_config();
    config.address_mode = TRAFFIC_GENERATOR_ADDRESS_UNIFORM;
    config.seed = 0x13579bdf;
    config.duration_limit = 16;
    prepare_region(0x11223344);
    outcome = execute_case(&generator, &config, 0);
    pass = common_checks(&outcome, &config, config.duration_limit) &&
           address_in_region(outcome.counters.last_address);
    report_case("uniform-read", &outcome, pass);
    failures += !pass;

    config = default_config();
    config.address_mode = TRAFFIC_GENERATOR_ADDRESS_GAUSSIAN;
    config.seed = 0x2468ace1;
    config.duration_limit = 16;
    outcome = execute_case(&generator, &config, 0);
    pass = common_checks(&outcome, &config, config.duration_limit) &&
           address_in_region(outcome.counters.last_address);
    report_case("gaussian-read", &outcome, pass);
    failures += !pass;

    config = default_config();
    config.temporal_mode = TRAFFIC_GENERATOR_TEMPORAL_PERIODIC;
    config.period = 4;
    config.duration_mode = TRAFFIC_GENERATOR_DURATION_CYCLES;
    config.duration_limit = 64;
    outcome = execute_case(&generator, &config, 0);
    pass = common_checks(&outcome, &config, ANY_COUNT) &&
           outcome.counters.completed != 0;
    report_case("periodic-cycles", &outcome, pass);
    failures += !pass;

    config = default_config();
    config.temporal_mode = TRAFFIC_GENERATOR_TEMPORAL_BERNOULLI;
    config.injection_rate = 0x80000000;
    config.seed = 0xc001d00d;
    config.duration_limit = 16;
    outcome = execute_case(&generator, &config, 0);
    pass = common_checks(&outcome, &config, config.duration_limit);
    report_case("bernoulli-read", &outcome, pass);
    failures += !pass;

    config = default_config();
    config.temporal_mode = TRAFFIC_GENERATOR_TEMPORAL_BURST;
    config.burst_length = 3;
    config.idle_length = 5;
    config.duration_limit = 12;
    outcome = execute_case(&generator, &config, 0);
    pass = common_checks(&outcome, &config, config.duration_limit);
    report_case("burst-read", &outcome, pass);
    failures += !pass;

    config = default_config();
    config.rw_mode = TRAFFIC_GENERATOR_RW_ALTERNATE;
    config.write_data = 0xcafef00d;
    config.duration_limit = 8;
    prepare_region(0x55aa55aa);
    outcome = execute_case(&generator, &config, 0);
    pass = common_checks(&outcome, &config, config.duration_limit) &&
           traffic_region[0] == config.write_data;
    report_case("alternate-rw", &outcome, pass);
    failures += !pass;

    config = default_config();
    config.rw_mode = TRAFFIC_GENERATOR_RW_RANDOM;
    config.address_mode = TRAFFIC_GENERATOR_ADDRESS_UNIFORM;
    config.write_data_mode = TRAFFIC_GENERATOR_WRITE_DATA_RANDOM;
    config.seed = 0x31415927;
    config.duration_limit = 16;
    prepare_region(0);
    outcome = execute_case(&generator, &config, 0);
    pass = common_checks(&outcome, &config, config.duration_limit) &&
           outcome.counters.reads != 0 && outcome.counters.writes != 0;
    report_case("random-rw", &outcome, pass);
    failures += !pass;

    config = default_config();
    config.duration_mode = TRAFFIC_GENERATOR_DURATION_INFINITE;
    config.duration_limit = 0;
    outcome = execute_case(&generator, &config, 64);
    pass = common_checks(&outcome, &config, ANY_COUNT) &&
           outcome.counters.completed != 0 && outcome.trace.stop_begin != 0;
    report_case("manual-stop", &outcome, pass);
    failures += !pass;

    printf("TRAFFIC_GENERATOR_TEST failures=%" PRIu32
           " final_cycle=%" PRIu32 "\n",
           failures, cycle_now());
    return failures == 0 ? 0 : 1;
}
