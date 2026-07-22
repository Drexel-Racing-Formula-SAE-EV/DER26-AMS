#include "fuse_replay.h"

#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void usage(const char *program)
{
    fprintf(stderr,
        "Usage: %s --trace TRACE.csv [options]\n"
        "\n"
        "Options:\n"
        "  --output FILE.csv             Detailed production/reference replay\n"
        "  --summary FILE.csv            One-row summary CSV\n"
        "  --startup cold-soak|known-cold|seeded:UTIL\n"
        "  --reset unknown|known-cold|restore|seeded:UTIL\n"
        "  --curve-time-fraction VALUE  Default 0.25 of typical melt time\n"
        "  --cooling-tau-s VALUE         Default 300\n"
        "  --soak-s VALUE                Default 300\n"
        "  --quiescent-current-a VALUE   Default 5\n"
        "  --uncertainty-a VALUE         Default 0.5\n"
        "  --temperature-c VALUE         Default 30\n"
        "  --state-abs-tol VALUE         Default 2e-5 utilization\n"
        "  --state-rel-tol VALUE         Default 0.001\n"
        "  --cap-tol-a VALUE             Default 0.20 A\n"
        "  --strict                      Exit nonzero on oracle mismatch\n",
        program);
}

static bool parse_double_arg(const char *text, double *value)
{
    if((text == NULL) || (value == NULL))
    {
        return false;
    }
    errno = 0;
    char *end = NULL;
    const double parsed = strtod(text, &end);
    if((errno != 0) || (end == text) || (*end != '\0') || !isfinite(parsed))
    {
        return false;
    }
    *value = parsed;
    return true;
}

static bool parse_startup(const char *text,
                          fuse_replay_init_policy_t *policy)
{
    if(strcmp(text, "cold-soak") == 0)
    {
        policy->kind = FUSE_REPLAY_INIT_COLD_SOAK;
        policy->utilization = 0.0;
        return true;
    }
    if(strcmp(text, "known-cold") == 0)
    {
        policy->kind = FUSE_REPLAY_INIT_KNOWN_COLD;
        policy->utilization = 0.0;
        return true;
    }
    if(strncmp(text, "seeded:", 7u) == 0)
    {
        double util;
        if(!parse_double_arg(text + 7, &util) || (util < 0.0))
        {
            return false;
        }
        policy->kind = FUSE_REPLAY_INIT_SEEDED_UTILIZATION;
        policy->utilization = util;
        return true;
    }
    return false;
}

static bool parse_reset(const char *text,
                        fuse_replay_reset_kind_t *kind,
                        double *utilization)
{
    if(strcmp(text, "unknown") == 0)
    {
        *kind = FUSE_REPLAY_RESET_UNKNOWN;
        return true;
    }
    if(strcmp(text, "known-cold") == 0)
    {
        *kind = FUSE_REPLAY_RESET_KNOWN_COLD;
        return true;
    }
    if(strcmp(text, "restore") == 0)
    {
        *kind = FUSE_REPLAY_RESET_RESTORE_PRE_RESET;
        return true;
    }
    if(strncmp(text, "seeded:", 7u) == 0)
    {
        double util;
        if(!parse_double_arg(text + 7, &util) || (util < 0.0))
        {
            return false;
        }
        *kind = FUSE_REPLAY_RESET_SEEDED_UTILIZATION;
        *utilization = util;
        return true;
    }
    return false;
}

int main(int argc, char **argv)
{
    fuse_replay_options_t options;
    fuse_replay_default_options(&options);

    for(int i = 1; i < argc; ++i)
    {
        const char *arg = argv[i];
#define REQUIRE_VALUE() do { \
    if((i + 1) >= argc) { \
        fprintf(stderr, "ERROR: %s requires a value\n", arg); \
        usage(argv[0]); \
        return EXIT_FAILURE; \
    } \
    ++i; \
} while(0)

        if(strcmp(arg, "--trace") == 0)
        {
            REQUIRE_VALUE();
            options.trace_path = argv[i];
        }
        else if(strcmp(arg, "--output") == 0)
        {
            REQUIRE_VALUE();
            options.output_path = argv[i];
        }
        else if(strcmp(arg, "--summary") == 0)
        {
            REQUIRE_VALUE();
            options.summary_path = argv[i];
        }
        else if(strcmp(arg, "--startup") == 0)
        {
            REQUIRE_VALUE();
            if(!parse_startup(argv[i], &options.startup_policy))
            {
                fprintf(stderr, "ERROR: invalid startup policy %s\n", argv[i]);
                return EXIT_FAILURE;
            }
        }
        else if(strcmp(arg, "--reset") == 0)
        {
            REQUIRE_VALUE();
            if(!parse_reset(argv[i], &options.reset_kind,
                            &options.reset_seed_utilization))
            {
                fprintf(stderr, "ERROR: invalid reset policy %s\n", argv[i]);
                return EXIT_FAILURE;
            }
        }
        else if(strcmp(arg, "--curve-time-fraction") == 0)
        {
            REQUIRE_VALUE();
            if(!parse_double_arg(argv[i], &options.curve_time_fraction)) return EXIT_FAILURE;
        }
        else if(strcmp(arg, "--cooling-tau-s") == 0)
        {
            REQUIRE_VALUE();
            if(!parse_double_arg(argv[i], &options.cooling_time_constant_s)) return EXIT_FAILURE;
        }
        else if(strcmp(arg, "--soak-s") == 0)
        {
            REQUIRE_VALUE();
            if(!parse_double_arg(argv[i], &options.initialization_soak_s)) return EXIT_FAILURE;
        }
        else if(strcmp(arg, "--quiescent-current-a") == 0)
        {
            REQUIRE_VALUE();
            if(!parse_double_arg(argv[i], &options.quiescent_current_a)) return EXIT_FAILURE;
        }
        else if(strcmp(arg, "--uncertainty-a") == 0)
        {
            REQUIRE_VALUE();
            if(!parse_double_arg(argv[i], &options.current_uncertainty_default_a)) return EXIT_FAILURE;
        }
        else if(strcmp(arg, "--temperature-c") == 0)
        {
            REQUIRE_VALUE();
            if(!parse_double_arg(argv[i], &options.temperature_default_c)) return EXIT_FAILURE;
        }
        else if(strcmp(arg, "--state-abs-tol") == 0)
        {
            REQUIRE_VALUE();
            if(!parse_double_arg(argv[i], &options.state_abs_tolerance)) return EXIT_FAILURE;
        }
        else if(strcmp(arg, "--state-rel-tol") == 0)
        {
            REQUIRE_VALUE();
            if(!parse_double_arg(argv[i], &options.state_rel_tolerance)) return EXIT_FAILURE;
        }
        else if(strcmp(arg, "--cap-tol-a") == 0)
        {
            REQUIRE_VALUE();
            if(!parse_double_arg(argv[i], &options.cap_tolerance_a)) return EXIT_FAILURE;
        }
        else if(strcmp(arg, "--strict") == 0)
        {
            options.strict = true;
        }
        else if((strcmp(arg, "--help") == 0) || (strcmp(arg, "-h") == 0))
        {
            usage(argv[0]);
            return EXIT_SUCCESS;
        }
        else
        {
            fprintf(stderr, "ERROR: unknown option %s\n", arg);
            usage(argv[0]);
            return EXIT_FAILURE;
        }
#undef REQUIRE_VALUE
    }

    if(options.trace_path == NULL)
    {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    fuse_replay_summary_t summary;
    return fuse_replay_run(&options, &summary) ? EXIT_SUCCESS : EXIT_FAILURE;
}
