#define main existing_apm_tests_main
#include "ams_unit_test_runner.c"
#undef main
/* Link-only clock stubs for the bundled unit runner's unrelated 6830 code.
 * No scheduling or target timing claim is made by this harness. */
static uint32_t regression_tick;
uint32_t HAL_GetTick(void) { return regression_tick; }
uint32_t osKernelGetTickCount(void) { return 0u; }
static unsigned init_counter_case(uint16_t single_count, uint16_t continuous_count, bool calibrated, bool expected_ok) {
  adbms2950_driver_t dev; adbms2950_asic ics[1];
  SPI_HandleTypeDef spi={0}; GPIO_TypeDef ga={0},gb={0};
  const uint8_t sid[6]={1,2,3,4,5,(ADBMS2950B_DEVICE_ID<<1)};
  const uint8_t ref[6]={0,0,0,0,0,0x10};
  const uint8_t cfga[6]={0,0,0,0x3c,0x3c,1};
  const uint8_t cfgb[6]={0,0,0,0,0,0xf0};
  uint8_t stat[6]={0,calibrated?0x40:0,0,0,0,0x10};
  uint8_t flag1[6],flag2[6];
  unit_spi_reset();
  memset(&unit_delay_timer_instance,0,sizeof(unit_delay_timer_instance));
  unit_delay_timer.Instance=&unit_delay_timer_instance;unit_delay_timer_advances=true;
  unit_adbms2950_make_flag_payload(flag1,single_count,0);
  unit_adbms2950_make_flag_payload(flag2,continuous_count,0);
  const uint8_t *payloads[8]={sid,ref,cfga,cfgb,stat,flag1,stat,flag2};
  const uint8_t counters[8]={1,1,3,3,4,4,5,5};
  unit_spi_txrx_sequence_count=8;
  for(unsigned i=0;i<8;i++) unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[i][4],payloads[i],counters[i],false);
  HAL_StatusTypeDef st=adbms2950_init_mixed_chain(&dev,1,ics,1,&spi,&ga,&gb,5,6,&unit_delay_timer,STRING_B,false,false);
  unsigned bad=((st==HAL_OK)!=expected_ok || dev.health.initialized!=expected_ok || dev.health.i1_continuous_ready!=expected_ok);
  printf("Init single=%u continuous=%u calibrated=%u HAL=%u expected_ok=%u failures=%u\n",single_count,continuous_count,calibrated,st,expected_ok,bad);
  return bad;
}
int main(void) {
  int base = existing_apm_tests_main();
  run_test("ADBMS2950 single-shot/continuous initialization", test_adbms2950_mixed_chain_init_identity_and_readback);
  if(g_failures) base=1;
  unsigned failures = 0;
  failures+=init_counter_case(1,150,true,true);
  failures+=init_counter_case(1,150,false,false);
  failures+=init_counter_case(0,150,true,false);
  failures+=init_counter_case(1,0,true,false);
  failures+=init_counter_case(1,135,true,false);
  failures+=init_counter_case(1,136,true,true);

  {
    adbms2950_driver_t dev; adbms2950_asic ics[1];
    SPI_HandleTypeDef spi={0}; GPIO_TypeDef ga={0},gb={0};
    unit_adbms2950_init_driver(&dev,ics,&spi,&ga,&gb,1u);
    dev.spi_debug.enabled=true;
    memset(unit_spi_txrx_response,0,sizeof(unit_spi_txrx_response));
    HAL_StatusTypeDef st=adbms2950_spi_probe_sid(&dev);
    if(st==HAL_OK || dev.health.sid_valid ||
       dev.health.last_reason!=ADBMS2950_REASON_PEC ||
       dev.spi_debug.last_read_pec_fail_mask!=1u ||
       dev.spi_debug.last_tx_preview[4]!=0xffu ||
       dev.spi_debug.last_tx_preview[11]!=0xffu) failures++;
    printf("Absent-device SID error/trace regression failures=%u\n",failures);
  }

  for(unsigned byte=0;byte<2;byte++) for(unsigned bit=0;bit<6;bit++) {
    adbms2950_driver_t dev; adbms2950_asic ics[1];
    SPI_HandleTypeDef spi={0}; GPIO_TypeDef ga={0},gb={0};
    uint8_t flag[6]={0}; flag[byte]=(uint8_t)(1u<<bit);
    unit_adbms2950_init_driver(&dev,ics,&spi,&ga,&gb,1u);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_response[4],flag,2u,false);
    HAL_StatusTypeDef st=adbms2950_read_flag(&dev);
    uint32_t expected=1u << ((byte*6u)+(5u-bit));
    if(st!=HAL_OK || dev.health.fault_mask!=expected) {
      failures++;
      printf("FLAG byte%u bit%u expected_mask=0x%08x observed=0x%08lx\n",byte,bit,expected,(unsigned long)dev.health.fault_mask);
    }
  }
  {
    adbms2950_driver_t dev; adbms2950_asic ics[1];
    SPI_HandleTypeDef spi={0}; GPIO_TypeDef ga={0},gb={0};
    uint8_t stat[6]={0,0x40,0,0,0,0x10};
    uint8_t sample[6]={0x10,0x27,0,0,0x50,0x46};
    uint8_t flag[6];
    unit_adbms2950_init_driver(&dev,ics,&spi,&ga,&gb,1u);
    (void)adbms2950_set_calibration_profile(&dev, ADBMS2950_CAL_PROFILE_DER_APM);
    dev.health.initialized=true; dev.health.sid_valid=true; dev.health.config_valid=true; dev.health.i1_continuous_ready=true;
    unit_adbms2950_make_flag_payload(flag,10u,0u);
    flag[4]=0x04; /* Table 73: MED1, uncorrectable memory fault. */
    unit_spi_txrx_sequence_count=3;
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[0][4],stat,4,false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[1][4],sample,4,false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[2][4],flag,4,false);
    HAL_StatusTypeDef st=adbms2950_read_primary_sample(&dev,100u);
    if(st == HAL_OK || dev.health.sample_valid || dev.health.current_valid || dev.health.fault_mask != (1u << 17)) failures++;
    printf("MED1 injection: HAL=%u sample_valid=%u current_valid=%u fault_mask=0x%08lx\n",st,dev.health.sample_valid,dev.health.current_valid,(unsigned long)dev.health.fault_mask);
  }
#ifdef DER_APM_STANDALONE
  {
    adbms2950_driver_t dev; adbms2950_asic ics[1];
    SPI_HandleTypeDef spi={0}; GPIO_TypeDef ga={0},gb={0};
    unit_adbms2950_init_driver(&dev,ics,&spi,&ga,&gb,1u);
#if AMS_APM_EVAL_LEGACY_WAKE
    for(unsigned n=0;n<4;n++) {
      regression_tick=n;
      if(adbms2950_wakeup_checked(&dev)!=HAL_OK || dev.bridge_wakes!=n+1u ||
         dev.bridge_wake_skips!=0u || dev.bridge_ready) failures++;
    }
    printf("Legacy wake repeated-transaction regression failures=%u\n",failures);
#else
    regression_tick=0;
    if(adbms2950_wakeup_checked(&dev)!=HAL_OK || dev.bridge_wakes!=1) failures++;
    regression_tick=2;
    if(adbms2950_wakeup_checked(&dev)!=HAL_OK || dev.bridge_wakes!=1 || dev.bridge_wake_skips!=1) failures++;
    regression_tick=3;
    if(adbms2950_wakeup_checked(&dev)!=HAL_OK || dev.bridge_wakes!=2) failures++;
    dev.string = dev.string == STRING_A ? STRING_B : STRING_A;
    if(adbms2950_wakeup_checked(&dev)!=HAL_OK || dev.bridge_wakes!=3) failures++;
#endif
    printf("Bridge readiness idle/port regression failures=%u\n",failures);
  }
#endif

  for(unsigned persistent=0; persistent<2; persistent++) {
    adbms2950_driver_t dev; adbms2950_asic ics[1];
    SPI_HandleTypeDef spi={0}; GPIO_TypeDef ga={0},gb={0};
    uint8_t startup[6]={0x20,0x38,0,0,0xff,0xdb};
    uint8_t after[6]={0}; if(persistent) after[4]=4;
    unit_adbms2950_init_driver(&dev,ics,&spi,&ga,&gb,1u);
    dev.health.initialized=true; dev.health.sid_valid=true; dev.health.config_valid=true; dev.health.sid_valid=true; dev.health.config_valid=true;
    unit_spi_txrx_sequence_count=2;
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[0][4],startup,2,false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[1][4],after,3,false);
    HAL_StatusTypeDef st=adbms2950_startup_baseline(&dev);
    if((st==HAL_OK) != !persistent || dev.startup_baseline_valid != !persistent ||
       memcmp(dev.startup_raw_flag,startup,6) || dev.health.sample_valid) failures++;
    if(adbms2950_startup_baseline(&dev)!=HAL_ERROR) failures++;
    printf("Startup baseline persistent=%u status=%u valid=%u failures=%u\n",persistent,st,dev.startup_baseline_valid,failures);
  }

  {
    adbms2950_driver_t dev; adbms2950_asic ics[1];
    SPI_HandleTypeDef spi={0}; GPIO_TypeDef ga={0},gb={0};
    uint8_t thermal[6]={0,0,0,0,0,4};
    unit_adbms2950_init_driver(&dev,ics,&spi,&ga,&gb,1u);
    dev.health.initialized=true; dev.health.sid_valid=true; dev.health.config_valid=true;
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_response[4],thermal,2,false);
    if(adbms2950_startup_baseline(&dev)!=HAL_ERROR || dev.startup_baseline_valid ||
       dev.startup_raw_flag[5]!=4) failures++;
    printf("Thermal baseline refusal failures=%u\n",failures);
  }

  for(unsigned voltage_enabled=0; voltage_enabled<2; voltage_enabled++) {
    adbms2950_driver_t dev; adbms2950_asic ics[1];
    SPI_HandleTypeDef spi={0}; GPIO_TypeDef ga={0},gb={0};
    uint8_t stat[6]={0,0x40,0,0,0,0x10};
    uint8_t sample[6]={0x10,0x27,0,0,0xff,0x7f}, flag[6];
    unit_adbms2950_init_driver(&dev,ics,&spi,&ga,&gb,1u);
    (void)adbms2950_set_calibration_profile(&dev,ADBMS2950_CAL_PROFILE_DER_APM);
    dev.health.initialized=true; dev.health.sid_valid=true; dev.health.config_valid=true;
    dev.health.i1_continuous_ready=true; dev.health.hv_dividers_enabled=voltage_enabled;
    unit_adbms2950_make_flag_payload(flag,10,0);
    unit_spi_txrx_sequence_count=3;
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[0][4],stat,4,false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[1][4],sample,4,false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[2][4],flag,4,false);
    HAL_StatusTypeDef st=adbms2950_read_primary_sample(&dev,100);
    if((st==HAL_OK) != !voltage_enabled || dev.health.pack_voltage_valid) failures++;
    printf("Disabled-voltage sentinel isolation enabled=%u status=%u failures=%u\n",voltage_enabled,st,failures);
  }
  {
    adbms2950_driver_t dev; adbms2950_asic ics[1];
    SPI_HandleTypeDef spi={0}; GPIO_TypeDef ga={0},gb={0};
    unit_adbms2950_init_driver(&dev,ics,&spi,&ga,&gb,1u);
    (void)adbms2950_set_calibration_profile(&dev,ADBMS2950_CAL_PROFILE_DER_APM);
    dev.calibration.current_gain=1.25f; dev.health.recovery_count=4; dev.health.recovery_failure_count=5;
    unit_spi_tx_status=HAL_TIMEOUT;
    HAL_StatusTypeDef st=adbms2950_recover(&dev,true);
    if(st==HAL_OK || dev.calibration.current_gain!=1.25f || dev.health.recovery_count!=4 ||
       dev.health.recovery_failure_count!=6 || dev.health.sample_valid) failures++;
    printf("Failed recovery preservation status=%u failures=%u\n",st,failures);
  }
  /* Distinct raw packet must survive RDFLAG/UNSNAP and must not become
   * a qualified sample when a later fault or PEC failure occurs. */
  for(unsigned scenario=0; scenario<3; scenario++) {
    adbms2950_driver_t dev; adbms2950_asic ics[1];
    SPI_HandleTypeDef spi={0}; GPIO_TypeDef ga={0},gb={0};
    uint8_t stat[6]={0,0x40,0,0,0,0x10};
    uint8_t sample[6]={0x9c,0xff,0xff,0x5a,0,0}, flag[6]; /* -100 */
    unit_adbms2950_init_driver(&dev,ics,&spi,&ga,&gb,1u);
    (void)adbms2950_set_calibration_profile(&dev,ADBMS2950_CAL_PROFILE_DER_APM);
    dev.health.initialized=true; dev.health.sid_valid=true;
    dev.health.config_valid=true; dev.health.i1_continuous_ready=true;
    unit_adbms2950_make_flag_payload(flag,10,0);
    if(scenario==1) flag[4]=4; /* reject later memory fault */
    unit_spi_txrx_sequence_count=3;
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[0][4],stat,4,false);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[1][4],sample,4,scenario==2);
    unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[2][4],flag,4,false);
    uint8_t expected[8]; memcpy(expected,&unit_spi_txrx_sequence[1][4],8);
    HAL_StatusTypeDef st=adbms2950_read_primary_sample(&dev,1234);
    if(dev.health.primary_attempt!=1 || dev.health.primary_attempt_ms!=1234 ||
       dev.health.primary_packet_checked!=(scenario!=2) ||
       dev.health.primary_flag_checked!=(scenario!=2) ||
       memcmp(dev.health.primary_packet,expected,8) ||
       dev.health.sample_valid!=(scenario==0) || ((st==HAL_OK)!=(scenario==0))) failures++;
    if(scenario==0 && (dev.health.i1_raw!=-100 || fabsf(dev.health.current_a+1.0f)>0.001f ||
       dev.health.primary_status_counter!=4 || dev.health.primary_sample_counter!=4 ||
       dev.health.primary_flag_counter!=4)) failures++;
    /* Preflight rejection clears packet qualification, never repeats old bytes. */
    dev.health.config_valid=false;
    if(adbms2950_read_primary_sample(&dev,2345)==HAL_OK ||
       dev.health.primary_attempt!=2 || dev.health.primary_packet_checked ||
       dev.health.primary_flag_checked ||
       dev.health.primary_packet[0]!=0) failures++;
    printf("Primary diagnostic packet scenario=%u failures=%u\n",scenario,failures);
  }
  printf("Existing test exit=%d; independent FLAG mismatches=%u/12\n",base,failures);
  return base || failures;
}
