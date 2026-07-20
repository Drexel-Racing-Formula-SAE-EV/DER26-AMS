/* Host unit tests for the shared DER26 NTCLE350E4103FHB0 model. */
#include "ext_drivers/thermistor_model.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static int failures;

#define EXPECT_TRUE(expr)                                                     \
    do                                                                        \
    {                                                                         \
        if(!(expr))                                                           \
        {                                                                     \
            printf("FAIL %s:%d expected true: %s\n", __FILE__, __LINE__, #expr); \
            failures++;                                                       \
        }                                                                     \
    } while(0)

#define EXPECT_FALSE(expr)                                                    \
    do                                                                        \
    {                                                                         \
        if((expr))                                                            \
        {                                                                     \
            printf("FAIL %s:%d expected false: %s\n", __FILE__, __LINE__, #expr); \
            failures++;                                                       \
        }                                                                     \
    } while(0)

#define EXPECT_NEAR(actual, expected, tolerance)                              \
    do                                                                        \
    {                                                                         \
        float a_ = (float)(actual);                                            \
        float e_ = (float)(expected);                                          \
        float t_ = (float)(tolerance);                                         \
        if(!isfinite(a_) || (fabsf(a_ - e_) > t_))                            \
        {                                                                     \
            printf("FAIL %s:%d actual=%f expected=%f tolerance=%f\n",        \
                   __FILE__, __LINE__, a_, e_, t_);                           \
            failures++;                                                       \
        }                                                                     \
    } while(0)

static void test_golden_table_points(void)
{
    static const struct
    {
        float temperature_c;
        float resistance_ohm;
    } points[] =
    {
        { -20.0f, 96761.15f },
        {   0.0f, 32624.23f },
        {  25.0f, 10000.00f },
        {  40.0f,  5323.88f },
        {  60.0f,  2483.82f },
        {  80.0f,  1251.80f },
        { 100.0f,   674.11f },
        { 120.0f,   384.41f },
    };

    for(uint32_t i = 0u; i < (uint32_t)(sizeof(points) / sizeof(points[0])); i++)
    {
        EXPECT_NEAR(thermistor_temperature_lut_c(points[i].resistance_ohm),
                    points[i].temperature_c,
                    0.0005f);
        EXPECT_NEAR(thermistor_temperature_steinhart_hart_c(points[i].resistance_ohm),
                    points[i].temperature_c,
                    0.012f);
        EXPECT_NEAR(thermistor_resistance_from_temperature_c(points[i].temperature_c),
                    points[i].resistance_ohm,
                    0.08f);
    }
}

static void test_dense_lut_and_equation_parity(void)
{
    for(int32_t deci_c = -200; deci_c <= 1200; deci_c++)
    {
        float temperature_c = (float)deci_c / 10.0f;
        float resistance_ohm = thermistor_resistance_from_temperature_c(temperature_c);
        float lut_c = thermistor_temperature_lut_c(resistance_ohm);
        float sh_c = thermistor_temperature_steinhart_hart_c(resistance_ohm);

        EXPECT_NEAR(sh_c, temperature_c, 0.012f);
        EXPECT_NEAR(lut_c, sh_c, 0.014f);
    }
}

static void test_raw_round_trip(void)
{
    for(int32_t deci_c = -200; deci_c <= 1200; deci_c++)
    {
        int16_t raw = 0;
        float temperature_c = (float)deci_c / 10.0f;
        EXPECT_TRUE(thermistor_adbms_raw_from_temperature_c(
            temperature_c, THERMISTOR_NOMINAL_VREG_V, &raw));

        thermistor_result_t result = thermistor_from_adbms_raw(
            raw, THERMISTOR_NOMINAL_VREG_V);
        EXPECT_TRUE(result.valid);
        EXPECT_NEAR(result.temperature_c, temperature_c, 0.020f);
    }
}

static void test_raw_zero_is_valid(void)
{
    thermistor_result_t result = thermistor_from_adbms_raw(
        0, THERMISTOR_NOMINAL_VREG_V);

    EXPECT_TRUE(result.valid);
    EXPECT_TRUE(result.status == THERMISTOR_STATUS_OK);
    EXPECT_NEAR(result.divider_voltage_v, 1.5f, 0.000001f);
    EXPECT_NEAR(result.resistance_ohm, 23333.333f, 0.01f);
    EXPECT_NEAR(result.temperature_c, 6.7124f, 0.001f);
}

static void test_sentinels_and_electrical_faults(void)
{
    thermistor_result_t reset = thermistor_from_adbms_raw(
        THERMISTOR_ADBMS_RESET_CODE, THERMISTOR_NOMINAL_VREG_V);
    thermistor_result_t clear = thermistor_from_adbms_raw(
        THERMISTOR_ADBMS_CLEAR_CODE, THERMISTOR_NOMINAL_VREG_V);
    thermistor_result_t open = thermistor_from_divider_voltage(0.05f, 5.0f);
    thermistor_result_t shorted = thermistor_from_divider_voltage(4.95f, 5.0f);

    EXPECT_FALSE(reset.valid);
    EXPECT_TRUE(reset.status == THERMISTOR_STATUS_ADC_SENTINEL);
    EXPECT_FALSE(clear.valid);
    EXPECT_TRUE(clear.status == THERMISTOR_STATUS_ADC_SENTINEL);
    EXPECT_FALSE(open.valid);
    EXPECT_TRUE(open.status == THERMISTOR_STATUS_OPEN_CIRCUIT);
    EXPECT_FALSE(shorted.valid);
    EXPECT_TRUE(shorted.status == THERMISTOR_STATUS_SHORT_CIRCUIT);
}

static void test_model_clamping(void)
{
    thermistor_result_t cold = thermistor_from_divider_voltage(0.20f, 5.0f);
    thermistor_result_t hot = thermistor_from_divider_voltage(4.85f, 5.0f);

    EXPECT_TRUE(cold.valid);
    EXPECT_TRUE(cold.status == THERMISTOR_STATUS_CLAMPED_COLD);
    EXPECT_TRUE(cold.model_clamped);
    EXPECT_NEAR(cold.temperature_c, -20.0f, 0.0001f);

    EXPECT_TRUE(hot.valid);
    EXPECT_TRUE(hot.status == THERMISTOR_STATUS_CLAMPED_HOT);
    EXPECT_TRUE(hot.model_clamped);
    EXPECT_NEAR(hot.temperature_c, 120.0f, 0.0001f);
    EXPECT_FALSE(thermistor_status_is_model_valid(cold.status));
    EXPECT_TRUE(thermistor_status_is_model_valid(THERMISTOR_STATUS_OK));
}

static void test_invalid_inputs(void)
{
    int16_t raw = 0;
    EXPECT_TRUE(isnan(thermistor_temperature_lut_c(NAN)));
    EXPECT_TRUE(isnan(thermistor_temperature_lut_c(-1.0f)));
    EXPECT_TRUE(isnan(thermistor_temperature_steinhart_hart_c(0.0f)));
    EXPECT_TRUE(isnan(thermistor_resistance_from_temperature_c(NAN)));
    EXPECT_FALSE(thermistor_adbms_raw_from_temperature_c(-20.1f, 5.0f, &raw));
    EXPECT_FALSE(thermistor_adbms_raw_from_temperature_c(120.1f, 5.0f, &raw));
    EXPECT_FALSE(thermistor_adbms_raw_from_temperature_c(25.0f, 4.4f, &raw));
    EXPECT_FALSE(thermistor_adbms_raw_from_temperature_c(25.0f, 5.0f, NULL));
}


static void test_divider_and_reference_boundaries(void)
{
    thermistor_result_t open_edge = thermistor_from_divider_voltage(0.10f, 5.0f);
    thermistor_result_t open_inside = thermistor_from_divider_voltage(0.1001f, 5.0f);
    thermistor_result_t short_edge = thermistor_from_divider_voltage(4.90f, 5.0f);
    thermistor_result_t short_inside = thermistor_from_divider_voltage(4.8999f, 5.0f);
    thermistor_result_t low_vreg = thermistor_from_divider_voltage(2.25f, 4.5f);
    thermistor_result_t high_vreg = thermistor_from_divider_voltage(2.75f, 5.5f);
    thermistor_result_t invalid_low_vreg = thermistor_from_divider_voltage(2.25f, 4.499f);
    thermistor_result_t invalid_high_vreg = thermistor_from_divider_voltage(2.75f, 5.501f);
    thermistor_result_t below_ground = thermistor_from_divider_voltage(-0.001f, 5.0f);
    thermistor_result_t above_reference = thermistor_from_divider_voltage(5.001f, 5.0f);

    EXPECT_FALSE(open_edge.valid);
    EXPECT_TRUE(open_edge.status == THERMISTOR_STATUS_OPEN_CIRCUIT);
    EXPECT_TRUE(open_inside.valid);
    EXPECT_TRUE(open_inside.status == THERMISTOR_STATUS_CLAMPED_COLD);

    EXPECT_FALSE(short_edge.valid);
    EXPECT_TRUE(short_edge.status == THERMISTOR_STATUS_SHORT_CIRCUIT);
    EXPECT_TRUE(short_inside.valid);
    EXPECT_TRUE(short_inside.status == THERMISTOR_STATUS_CLAMPED_HOT);

    EXPECT_TRUE(low_vreg.valid);
    EXPECT_NEAR(low_vreg.temperature_c, 25.0f, 0.001f);
    EXPECT_TRUE(high_vreg.valid);
    EXPECT_NEAR(high_vreg.temperature_c, 25.0f, 0.001f);

    EXPECT_FALSE(invalid_low_vreg.valid);
    EXPECT_TRUE(invalid_low_vreg.status == THERMISTOR_STATUS_REFERENCE_OUT_OF_RANGE);
    EXPECT_FALSE(invalid_high_vreg.valid);
    EXPECT_TRUE(invalid_high_vreg.status == THERMISTOR_STATUS_REFERENCE_OUT_OF_RANGE);
    EXPECT_FALSE(below_ground.valid);
    EXPECT_TRUE(below_ground.status == THERMISTOR_STATUS_VOLTAGE_OUT_OF_RANGE);
    EXPECT_FALSE(above_reference.valid);
    EXPECT_TRUE(above_reference.status == THERMISTOR_STATUS_VOLTAGE_OUT_OF_RANGE);
}

static void test_monotonicity_and_endpoint_behavior(void)
{
    float previous_r = thermistor_resistance_from_temperature_c(
        THERMISTOR_MODEL_MIN_TEMP_C);
    int16_t previous_raw = INT16_MIN;

    EXPECT_NEAR(thermistor_temperature_lut_c(previous_r),
                THERMISTOR_MODEL_MIN_TEMP_C,
                0.0005f);

    for(int32_t half_c = -39; half_c <= 240; half_c++)
    {
        float temperature_c = (float)half_c * 0.5f;
        float resistance_ohm = thermistor_resistance_from_temperature_c(temperature_c);
        int16_t raw = 0;

        EXPECT_TRUE(isfinite(resistance_ohm));
        EXPECT_TRUE(resistance_ohm < previous_r);
        EXPECT_TRUE(thermistor_adbms_raw_from_temperature_c(
            temperature_c, THERMISTOR_NOMINAL_VREG_V, &raw));
        EXPECT_TRUE(raw > previous_raw);
        EXPECT_NEAR(thermistor_temperature_lut_c(resistance_ohm),
                    temperature_c,
                    0.014f);

        previous_r = resistance_ohm;
        previous_raw = raw;
    }

    EXPECT_NEAR(thermistor_temperature_lut_c(1.0e9f),
                THERMISTOR_MODEL_MIN_TEMP_C,
                0.0001f);
    EXPECT_NEAR(thermistor_temperature_lut_c(1.0f),
                THERMISTOR_MODEL_MAX_TEMP_C,
                0.0001f);
}

static void test_status_strings_and_metadata(void)
{
    EXPECT_TRUE(THERMISTOR_MODEL_REVISION == 1u);
    EXPECT_TRUE(THERMISTOR_R25_OHM == 10000.0f);
    EXPECT_TRUE(THERMISTOR_B2585_K == 3984.0f);
    EXPECT_TRUE(thermistor_status_str(THERMISTOR_STATUS_OK)[0] == 'o');
    EXPECT_TRUE(thermistor_status_str(THERMISTOR_STATUS_ADC_SENTINEL)[0] == 'a');
    EXPECT_TRUE(thermistor_status_str((thermistor_status_t)99)[0] == 'u');
}

static void run_test(const char *name, void (*fn)(void))
{
    int before = failures;
    fn();
    printf("%s %s\n", (failures == before) ? "PASS" : "FAIL", name);
}

int main(void)
{
    run_test("golden table points", test_golden_table_points);
    run_test("dense LUT/equation parity", test_dense_lut_and_equation_parity);
    run_test("ADBMS raw round-trip", test_raw_round_trip);
    run_test("raw zero is valid", test_raw_zero_is_valid);
    run_test("sentinels and electrical faults", test_sentinels_and_electrical_faults);
    run_test("model clamping", test_model_clamping);
    run_test("divider/reference boundaries", test_divider_and_reference_boundaries);
    run_test("monotonicity/endpoints", test_monotonicity_and_endpoint_behavior);
    run_test("status strings/metadata", test_status_strings_and_metadata);
    run_test("invalid inputs", test_invalid_inputs);

    if(failures != 0)
    {
        printf("Thermistor model unit tests failed: %d\n", failures);
        return 1;
    }

    printf("All thermistor model unit tests passed.\n");
    return 0;
}
