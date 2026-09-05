/* Real current-window arithmetic and real CAN encoders, no MiL campaign. */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "app.h"
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); exit(1); } } while(0)
#define NEAR(a,b) CHECK(fabs((double)(a)-(double)(b)) < 0.00001)
app_data_t app;
static uint32_t tick;
static uint8_t packet[8];
uint32_t osKernelGetTickCount(void) { return tick; }
void vPortEnterCritical(void) {}
void vPortExitCritical(void) {}
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(const CAN_HandleTypeDef *h) { (void)h; return 1; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, const CAN_TxHeaderTypeDef *hdr, const uint8_t *p, uint32_t *m)
{ (void)h; (void)hdr; (void)m; memcpy(packet,p,8); return HAL_OK; }
HAL_StatusTypeDef canbus_tx_build_append(canbus_device_t *d,uint32_t ide,uint32_t id,const uint8_t p[8])
{ (void)d; (void)ide; CHECK(id==AMS_ECU_CAN_ID_CURRENT_DIAG); memcpy(packet,p,8); return HAL_OK; }
/* Snapshot tests must never fall back to mutable driver storage. */
uint8_t accumulator_configured_smb_count(const accumulator_t *d)
{ (void)d; CHECK(false); return 0; }
uint16_t accumulator_cell_voltage_mv(const accumulator_t *d,uint8_t s,uint8_t c)
{ (void)d; (void)s; (void)c; CHECK(false); return 0; }
bool accumulator_temp_sensor_usable(const accumulator_t *d,uint8_t s,uint8_t t)
{ (void)d; (void)s; (void)t; CHECK(false); return false; }
int16_t accumulator_temp_deci_c(const accumulator_t *d,uint8_t s,uint8_t t)
{ (void)d; (void)s; (void)t; CHECK(false); return 0; }
#include "Core/Src/tasks/canbus_task.c"
#include "Core/Src/sop/ams_power_state.c"
static void sample(ams_current_window_accumulator_t *a,uint32_t t,float i,uint16_t u,uint8_t r)
{
    ams_current_window_set_sensor_metadata(a,u,r);
    ams_current_window_update(a,t,i,i,true,true,42);
}
int main(void)
{
    ams_current_window_accumulator_t a;
    ams_current_window_t w;
    ams_current_window_init(&a,80);
    sample(&a,90,10,100,1); sample(&a,110,10,100,1);
    CHECK(!ams_current_window_rotate(&a,100,&w));
    CHECK(a.active.start_tick==80 && a.active.end_tick==110);
    sample(&a,130,10,100,1);
    CHECK(!ams_current_window_rotate(&a,190,&w));
    NEAR(w.charge_As,1.1); NEAR(w.average_A,10);
    sample(&a,210,10,100,1);
    CHECK(ams_current_window_rotate(&a,230,&w)); NEAR(w.average_A,10);
    puts("PASS late boundary rejection and clean recovery");

    ams_current_window_init(&a,0);
    sample(&a,10,30,500,1); CHECK(ams_current_window_rotate(&a,20,&w));
    sample(&a,30,10,100,2); sample(&a,40,10,100,2);
    CHECK(ams_current_window_rotate(&a,50,&w));
    CHECK(w.uncertainty_mA==500 && w.selected_range==0);
    NEAR(w.min_A,10); NEAR(w.max_A,30); NEAR(w.charge_As,0.4);
    sample(&a,60,10,100,2); CHECK(ams_current_window_rotate(&a,70,&w));
    CHECK(w.uncertainty_mA==100 && w.selected_range==2);
    puts("PASS carried uncertainty, extrema, sticky mixed range, next-window recovery");

    ams_current_window_init(&a,0);
    sample(&a,10,10,UINT16_MAX,1); CHECK(ams_current_window_rotate(&a,20,&w));
    sample(&a,30,10,100,1); CHECK(ams_current_window_rotate(&a,40,&w));
    CHECK(w.uncertainty_mA==UINT16_MAX);
    sample(&a,35,100,500,2); CHECK(a.last_sample_tick==30);
    CHECK(!ams_current_window_rotate(&a,50,&w));
    puts("PASS unknown uncertainty and late sample rejection");

    ams_current_window_init(&a,UINT32_MAX-20u);
    sample(&a,UINT32_MAX-10u,4,100,1); sample(&a,5,4,100,1);
    CHECK(ams_current_window_rotate(&a,20,&w)); NEAR(w.charge_As,.164);
    sample(&a,150,4,100,1); CHECK(!ams_current_window_rotate(&a,160,&w));
    puts("PASS tick wrap and excessive gap rejection");

    ams_current_window_init(&a,0);
    sample(&a,10,10,100,1); CHECK(ams_current_window_rotate(&a,90,&w));
    sample(&a,150,10,100,1); CHECK(!ams_current_window_rotate(&a,160,&w));
    puts("PASS rotation cannot hide a real-sample gap");

    ams_current_window_init(&a,50);
    sample(&a,60,10,100,1); CHECK(ams_current_window_rotate(&a,150,&w));
    CHECK(!ams_current_window_rotate(&a,170,&w));
    CHECK(!a.last_sample_valid);
    sample(&a,180,10,100,1); CHECK(ams_current_window_rotate(&a,190,&w));
    NEAR(w.charge_As,.2); NEAR(w.average_A,10);
    puts("PASS stale rotation cannot charge a new window for an old interval");

    static ams_measurement_snapshot_t s;
    can_measurement_view_t v;
    static canbus_device_t can;
    can.tx_builder.active=true;
    s.validity_flags=AMS_MEAS_VALID_CURRENT;
    s.current.valid=true; s.current.latest_sample_tick=100;
    s.current.sequence=0x1234; s.current.average_A=12;
    app.current_valid=true; app.current_sample_tick=120; app.current_sample_sequence=0x5678;
    tick=120; can_measurement_view_build(&app,&s,&v);
    CHECK(v.current_valid); NEAR(v.current_A,12);
    CHECK(send_ecu_current_diag(&can,&app,&v)==HAL_OK);
    CHECK(packet[0]==1 && packet[1]==1 && packet[2]==1);
    CHECK(packet[4]==0x12 && packet[5]==0x34 && packet[6]==0 && packet[7]==20);
    s.current.calibration_record_confident=true; s.current.calibration_id=42; s.current.uncertainty_mA=100;
    CHECK(send_ecu_current_diag(&can,&app,&v)==HAL_OK); CHECK(packet[1]==2);
    s.current.uncertainty_mA=UINT16_MAX;
    CHECK(send_ecu_current_diag(&can,&app,&v)==HAL_OK); CHECK(packet[1]==1);
    s.current.uncertainty_mA=0;
    CHECK(send_ecu_current_diag(&can,&app,&v)==HAL_OK); CHECK(packet[1]==1);
    tick=201; app.current_sample_tick=201;
    can_measurement_view_build(&app,&s,&v); CHECK(!v.current_valid);
    CHECK(send_ecu_current_diag(&can,&app,&v)==HAL_OK);
    CHECK(packet[0]==0 && packet[1]==0 && packet[2]==0);
    CHECK(send_ecu_current_diag(&can,&app,NULL)==HAL_OK); CHECK(packet[1]==1);
    tick=302; can_measurement_view_build(&app,NULL,&v); CHECK(!v.current_valid);
    CHECK(send_ecu_current_diag(&can,&app,&v)==HAL_OK); CHECK(packet[2]==0);
    puts("PASS CAN calibration quality, immutable sequence/age, expiry and fallback");
    memset(&s,0,sizeof(s));
    s.publication_tick=1000; s.voltage_complete_tick=1000;
    s.validity_flags=AMS_MEAS_VALID_VOLTAGE | AMS_MEAS_VALID_TEMPERATURE;
    s.cell_usable_mask[0]=3; s.temp_usable_mask[0]=3;
    s.cell_mv[0][0]=3700; s.cell_mv[0][1]=3800;
    s.temp_deci_c[0][0]=250; s.temp_deci_c[0][1]=300;
    s.cell_age_ms[0][0]=ACCUMULATOR_CELL_STALE_TIMEOUT_MS-10;
    s.temp_age_ms[0][0]=ACCUMULATOR_TEMP_STALE_TIMEOUT_MS-10;
    tick=1010; can_measurement_view_build(&app,&s,&v);
    CHECK(v.voltage_valid && v.temperature_valid);
    tick=1011; can_measurement_view_build(&app,&s,&v);
    CHECK(!v.voltage_valid && !v.temperature_valid);
    CHECK(v.usable_cell_count==1 && v.usable_temp_count==1);
    CHECK(v.min_cell_mv==3800 && v.min_temp_deci_c==300);
    CHECK(v.cell_usable_mask[0]==2 && v.temp_usable_mask[0]==2);
    CHECK(cell_mv_for_view(&app,&v,0,0)==0);
    CHECK(cell_mv_for_logger_view(&app,&v,0,0)==0);
    CHECK(temp_deci_c_for_view(&app,&v,0,0)==ECU_TEMP_INVALID_DECI_C);
    CHECK(temp_deci_c_for_logger_view(&app,&v,0,0)==AMS_LOGGER_TEMP_INVALID_DECI_C);
    CHECK(cell_mv_for_view(&app,&v,0,1)==3800);
    s.cell_age_ms[0][0]=UINT32_MAX; s.temp_age_ms[0][0]=UINT32_MAX;
    can_measurement_view_build(&app,&s,&v);
    CHECK(!v.voltage_valid && !v.temperature_valid);
    s.cell_age_ms[0][0]=0; s.temp_age_ms[0][0]=0;
    s.publication_tick=UINT32_MAX-5; s.voltage_complete_tick=UINT32_MAX-5;
    tick=5; can_measurement_view_build(&app,&s,&v);
    CHECK(v.voltage_valid && v.temperature_valid);
    puts("PASS CAN per-reading expiry, aggregates, overflow and tick wrap");

    static ams_power_state_t power;
    static ams_estimator_t est;
    ams_power_policy_t policy={0};
    ams_sop_input_t sop_input;
    ams_soh_input_t soh_input;
    float temperatures[AMS_SOP_SEGMENTS]={0};
    policy.current_calibrated=true;
    s.current.calibration_record_confident=true;
    s.current.calibration_id=42;
    const uint16_t uncertainties[]={500,0,UINT16_MAX};
    for(unsigned n=0;n<3;n++)
    {
        s.current.uncertainty_mA=uncertainties[n];
        build_sop_input(&power,&s,&est,&policy,temperatures,tick,&sop_input);
        build_soh_input(&s,&est,&policy,tick,0.1f,3.7f,3.8f,25,&soh_input);
        CHECK(sop_input.current_calibrated==(n==0));
        CHECK(soh_input.current_calibrated==(n==0));
    }
    puts("PASS SoP and SoH reject unknown/zero uncertainty as calibrated");
    return 0;
}
