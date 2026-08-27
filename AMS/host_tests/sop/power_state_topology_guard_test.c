#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "sop/ams_power_state.h"

#define CHECK(c) do { \
    if(!(c)) { \
        fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); \
        return 1; \
    } \
} while(0)

int main(void)
{
    _Static_assert(NSMBS < AMS_SOP_SEGMENTS,
                   "topology guard test must compile a reduced measurement store");

    ams_power_state_t state = {0};
    ams_measurement_snapshot_t measurement = {0};
    ams_estimator_t estimator = {0};
    ams_power_policy_t policy = {0};

    /* This is exactly the BENCH/HIL structural mismatch: a one-SMB snapshot
     * presented to a five-segment SoP/SoH model. It must fail before any
     * measurement array indexed by the physical segment count is touched. */
    CHECK(!ams_power_state_update(&state, &measurement, &estimator,
                                  &policy, 100u, 0.1f));
    CHECK(state.update_count == 1u);
    CHECK((state.published_result.reason_flags &
           AMS_SOP_REASON_INCOMPLETE_TOPOLOGY) != 0u);
    CHECK(state.published_result.valid == 0u);

    puts("PASS reduced-topology SoP/SoH guard fails closed before segment access");
    return 0;
}
