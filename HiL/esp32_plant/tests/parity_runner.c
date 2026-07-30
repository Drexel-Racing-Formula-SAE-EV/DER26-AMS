#include <stdio.h>
#include <stdlib.h>

#include "plant_model_adapter.h"

static void print_header(void)
{
    fputs(
        "time_s,V_pack,T_core,T_surf,SoC_true,V_min,V_max,T_max,T_avg",
        stdout);
    for (unsigned int index = 0U; index < PLANT_NUM_GROUPS; index++)
    {
        printf(",V_group_%u", index + 1U);
    }
    for (unsigned int index = 0U; index < PLANT_NUM_SEGMENTS; index++)
    {
        printf(",V_segment_%u", index + 1U);
    }
    for (unsigned int index = 0U; index < PLANT_NUM_THERMISTORS; index++)
    {
        printf(",T_sensor_%u", index + 1U);
    }
    for (unsigned int index = 0U; index < PLANT_NUM_GROUPS; index++)
    {
        printf(",SoC_group_%u", index + 1U);
    }
    putchar('\n');
}

static void print_output(double time_s, const plant_output_t *output)
{
    printf(
        "%.17g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
        time_s,
        (double)output->V_pack,
        (double)output->T_core,
        (double)output->T_surf,
        (double)output->SoC_true,
        (double)output->V_min,
        (double)output->V_max,
        (double)output->T_max,
        (double)output->T_avg);
    for (unsigned int index = 0U; index < PLANT_NUM_GROUPS; index++)
    {
        printf(",%.9g", (double)output->V_group[index]);
    }
    for (unsigned int index = 0U; index < PLANT_NUM_SEGMENTS; index++)
    {
        printf(",%.9g", (double)output->V_segment[index]);
    }
    for (unsigned int index = 0U; index < PLANT_NUM_THERMISTORS; index++)
    {
        printf(",%.9g", (double)output->T_sensor[index]);
    }
    for (unsigned int index = 0U; index < PLANT_NUM_GROUPS; index++)
    {
        printf(",%.9g", (double)output->SoC_group[index]);
    }
    putchar('\n');
}

int main(int argc, char **argv)
{
    if (argc < 2 || argc > 4)
    {
        fprintf(
            stderr,
            "usage: %s input_profile.csv [initial_soc] [initial_temperature_C]\n",
            argv[0]);
        return EXIT_FAILURE;
    }
    const float initial_soc = (argc >= 3) ? strtof(argv[2], NULL) : 1.0f;
    const float initial_temperature_C =
        (argc >= 4) ? strtof(argv[3], NULL) : 25.0f;
    FILE *input = fopen(argv[1], "r");
    if (input == NULL)
    {
        perror(argv[1]);
        return EXIT_FAILURE;
    }

    char line[512];
    if (fgets(line, sizeof(line), input) == NULL ||
        !plant_reset(initial_soc, initial_temperature_C))
    {
        fclose(input);
        return EXIT_FAILURE;
    }
    print_header();

    plant_output_t output;
    while (fgets(line, sizeof(line), input) != NULL)
    {
        double time_s = 0.0;
        float current_A = 0.0f;
        float ambient_C = 0.0f;
        if (sscanf(line, "%lf,%f,%f", &time_s, &current_A, &ambient_C) != 3)
        {
            fprintf(stderr, "invalid input row: %s", line);
            fclose(input);
            plant_terminate();
            return EXIT_FAILURE;
        }
        if (!plant_step(current_A, ambient_C) ||
            !plant_get_outputs(&output))
        {
            fclose(input);
            plant_terminate();
            return EXIT_FAILURE;
        }
        print_output(time_s, &output);
    }

    fclose(input);
    plant_terminate();
    return EXIT_SUCCESS;
}
