#define main unused_wake_fixture_main
#include "adbms_wake_policy_test.c"
#undef main
static void setup(void) {
    reset_dev(5); model_diag=true; model_base_code=10000; model_no_stim=false; model_cfg_writes=model_fail_cfg_write=0; model_stimulus=0;
    for(unsigned i=0;i<5;i++) dev.ics[i].tx_cfga.refon=1;
    adbms6830_pack_cfga(&dev); adbms6830_pack_cfgb(&dev);
    for(unsigned i=0;i<5;i++) memcpy(model_cfga[i],dev.ics[i].configa.tx_data,6);
    read_clean(); sim_us+=5000;
    for(unsigned i=0;i<5;i++) dev.last_temp_updated_mask[i]=0x123456;
}
static void restored(void) {
    for(unsigned i=0;i<5;i++) {
        CHECK((model_cfga[i][2]&0x80)==0 && !dev.ics[i].tx_cfga.soakon);
        CHECK(dev.last_temp_updated_mask[i]==0x123456);
    }
    CHECK(test_lock_depth==0 && last_cs==GPIO_PIN_SET);
}
int main(void) {
    setup(); bool complete; uint64_t start=sim_us;
    CHECK(adbms6830_thermistor_open_wire_step(&dev,0,&complete)==HAL_OK && !complete);
    uint64_t first=sim_us-start;
    CHECK(dev.therm_phase.pending && dev.therm_ow_health.valid_mask==0);
    restored(); sim_us+=100000; model_base_code=10500; start=sim_us;
    CHECK(adbms6830_thermistor_open_wire_step(&dev,0,&complete)==HAL_OK && complete);
    uint64_t second=sim_us-start;
    CHECK(!dev.therm_phase.pending && dev.therm_ow_health.valid_mask==31);
    CHECK(dev.therm_ow_health.suspect_mask==0 && dev.therm_ow_health.count==1);
    CHECK(dev.therm_ow_health.baseline_raw[0]==10000 && dev.therm_ow_health.pullup_baseline_raw[0]==10500);
    restored(); printf("THERM phase wall model: down=%llu up=%llu us\n",(unsigned long long)first,(unsigned long long)second);
    setup(); CHECK(adbms6830_thermistor_open_wire_step(&dev,1,&complete)==HAL_OK && !complete);
    sim_us+=1001000;
    CHECK(adbms6830_thermistor_open_wire_step(&dev,1,&complete)!=HAL_OK && complete);
    CHECK(dev.therm_ow_health.valid_mask==0); restored();
    setup(); CHECK(adbms6830_thermistor_open_wire_step(&dev,1,&complete)==HAL_OK && !complete);
    adbms6830_link_forget(&dev);
    CHECK(adbms6830_thermistor_open_wire_step(&dev,1,&complete)!=HAL_OK && complete);
    CHECK(dev.therm_ow_health.valid_mask==0); restored();
    setup(); CHECK(adbms6830_thermistor_open_wire_step(&dev,1,&complete)==HAL_OK && !complete);
    dev.ics[0].tx_cfga.cth^=1; adbms6830_pack_cfga(&dev);
    CHECK(adbms6830_thermistor_open_wire_step(&dev,1,&complete)!=HAL_OK && complete);
    CHECK(dev.therm_ow_health.valid_mask==0);
    setup(); model_no_stim=true;
    CHECK(adbms6830_thermistor_open_wire_step(&dev,0,&complete)==HAL_OK && !complete);
    model_no_stim=false;
    CHECK(adbms6830_thermistor_open_wire_step(&dev,0,&complete)==HAL_ERROR && complete);
    CHECK(dev.therm_ow_health.valid_mask==31 && dev.therm_ow_health.suspect_mask==31);
    restored();
    setup(); model_fail_cfg_write=1;
    CHECK(adbms6830_thermistor_open_wire_step(&dev,0,&complete)!=HAL_OK && complete);
    CHECK(model_cfg_writes>=2); /* A failed stimulus write must still be restored. */
    restored();
    setup(); CHECK(adbms6830_run_thermistor_open_wire(&dev,2)==HAL_OK); restored();
    puts("PASS two-phase completion, normal configuration between phases, no primary publication, timeout/integrity cancellation and partial-write restoration");
    return failures ? 1 : 0;
}
