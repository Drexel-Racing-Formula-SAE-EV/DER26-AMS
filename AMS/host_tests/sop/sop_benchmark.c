#define _POSIX_C_SOURCE 200809L

#include "estimator/ams_estimator_lut.h"
#include "soh/ams_soh.h"
#include "sop/ams_sop.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define BENCHMARK_ITERATIONS 2000u

static double elapsed_us(const struct timespec *start,
                         const struct timespec *end)
{
    const double seconds = (double)(end->tv_sec - start->tv_sec);
    const double nanoseconds = (double)(end->tv_nsec - start->tv_nsec);
    return (seconds * 1.0e6) + (nanoseconds / 1.0e3);
}

static ams_sop_input_t nominal_input(void)
{
    ams_sop_input_t input;
    memset(&input, 0, sizeof(input));
    input.measurement_sequence = 1u;
    input.measurement_timestamp_ms = 950u;
    input.now_ms = 1000u;
    input.pack_current_uncertainty_a = 0.5f;
    input.ambient_temp_c = 27.0f;
    input.operating_mode = AMS_SOP_MODE_DRIVE;
    input.measurement_valid = 1u;
    input.estimator_valid = 1u;
    input.estimator_acquired = 1u;
    input.estimator_segment_topology = 1u;
    input.current_calibrated = 1u;
    input.current_polarity_validated = 1u;
    input.ambient_measured = 1u;
    input.balance_recovered = 1u;
    input.discharge_authorized = 1u;
    input.regen_authorized = 1u;

    for(uint8_t segment = 0u; segment < AMS_SOP_SEGMENTS; segment++)
    {
        ams_sop_segment_input_t *state = &input.segment[segment];
        state->soc = 0.55f;
        state->r0_ohm = ams_p42a_r0_ohm(0.55f, 28.0f);
        state->core_temp_c = 28.0f;
        state->surface_max_temp_c = 27.0f;
        state->p_soc = 1.0e-6f;
        state->p_vp1 = 1.0e-7f;
        state->p_vp2 = 1.0e-7f;
        state->p_r0 = 1.0e-9f;
        state->capacity_soh_lower = 1.0f;
        state->resistance_soh_upper = 1.05f;
        state->cell_usable_mask = AMS_SOP_FULL_CELL_MASK;
        state->estimator_valid = 1u;
        state->capacity_soh_valid = 1u;
        state->resistance_soh_valid = 1u;
        for(uint8_t cell = 0u; cell < AMS_SOP_CELLS_PER_SEGMENT; cell++)
        {
            state->cell_voltage_v[cell] = 3.75f;
        }
    }
    return input;
}

int main(void)
{
    ams_sop_config_t config;
    ams_sop_default_config(&config);
    ams_sop_input_t input = nominal_input();
    ams_sop_result_t result;
    volatile float checksum = 0.0f;

    for(uint32_t warmup = 0u; warmup < 20u; warmup++)
    {
        if(ams_sop_solve(&input, &config, &result) != AMS_SOP_OK)
        {
            return 2;
        }
        checksum += result.model_discharge_current_a[warmup % AMS_SOP_HORIZONS];
    }

    double minimum_us = INFINITY;
    double maximum_us = 0.0;
    double total_us = 0.0;
    for(uint32_t iteration = 0u; iteration < BENCHMARK_ITERATIONS; iteration++)
    {
        struct timespec start;
        struct timespec end;
        (void)clock_gettime(CLOCK_MONOTONIC, &start);
        const ams_sop_status_t status = ams_sop_solve(&input, &config, &result);
        (void)clock_gettime(CLOCK_MONOTONIC, &end);
        if(status != AMS_SOP_OK)
        {
            return 3;
        }
        const double duration_us = elapsed_us(&start, &end);
        if(duration_us < minimum_us)
        {
            minimum_us = duration_us;
        }
        if(duration_us > maximum_us)
        {
            maximum_us = duration_us;
        }
        total_us += duration_us;
        checksum += result.charge_current_a[iteration % AMS_SOP_HORIZONS];
    }

    printf("DER26 SoP host resource report\n");
    printf("iterations=%u\n", BENCHMARK_ITERATIONS);
    printf("solve_min_us=%.3f\n", minimum_us);
    printf("solve_mean_us=%.3f\n", total_us / (double)BENCHMARK_ITERATIONS);
    printf("solve_max_us=%.3f\n", maximum_us);
    printf("feasibility_evaluations=%lu\n",
           (unsigned long)result.feasibility_evaluations);
    printf("prediction_steps=%lu\n", (unsigned long)result.prediction_steps);
    printf("sizeof_sop_input=%lu\n", (unsigned long)sizeof(ams_sop_input_t));
    printf("sizeof_sop_config=%lu\n", (unsigned long)sizeof(ams_sop_config_t));
    printf("sizeof_sop_result=%lu\n", (unsigned long)sizeof(ams_sop_result_t));
    printf("sizeof_soh_estimator=%lu\n",
           (unsigned long)sizeof(ams_soh_estimator_t));
    printf("checksum=%.3f\n", (double)checksum);
    return (maximum_us < 1000000.0) ? 0 : 4;
}
