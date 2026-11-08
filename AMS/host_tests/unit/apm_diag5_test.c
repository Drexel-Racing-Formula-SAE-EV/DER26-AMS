#define main prior_test_main
#include "ams_unit_test_runner.c"
#undef main
static uint32_t diag_tick;
uint32_t HAL_GetTick(void) {return diag_tick;}
uint32_t osKernelGetTickCount(void) {return diag_tick;}
static void fill(unsigned i,const uint8_t *p,unsigned cc,bool corrupt) {
 unit_adbms_make_read_packet_from_data(&unit_spi_txrx_sequence[i][4],p,cc,corrupt);
}
int main(void) {
 adbms2950_driver_t d; adbms2950_asic ic[1];
 SPI_HandleTypeDef spi={0};GPIO_TypeDef a={0},b={0};
 adbms2950_debug_snapshot_t s;
 const uint8_t zero[6]={0};
 const uint8_t negative[3]={0x61,0xfa,0xff};
 EXPECT_TRUE(adbms2950_debug_s24(negative)==-1439);
 const uint8_t edge[3]={0,0,0x80};EXPECT_TRUE(adbms2950_debug_s24(edge)==-8388608);
 const uint8_t word[2]={0,0x80};EXPECT_TRUE(adbms2950_debug_s16(word)==-32768);
 /* All named fault bits: independent Table73 byte/mask fixture. */
 const uint8_t bytes[27]={0,0,0,0,0,0,1,1,1,1,1,1,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5};
 const uint8_t masks[27]={32,16,8,4,2,1,32,16,8,4,2,1,128,64,32,16,8,4,2,1,128,64,16,8,4,2,1};
 for(unsigned i=0;i<27;i++) {uint8_t f[6]={0};f[bytes[i]]=masks[i];EXPECT_TRUE(adbms2950_decode_fault_mask(f)==(1ul<<i));}
 for(unsigned fresh=0;fresh<2;fresh++) {
  unit_adbms2950_init_driver(&d,ic,&spi,&a,&b,1);unit_delay_timer_advances=true;
  d.health.current_a=12.5f;d.health.sample_valid=false;
  d.health.last_stage=ADBMS2950_STAGE_SAMPLE;
  d.health.last_reason=ADBMS2950_REASON_HARDWARE_FAULT;
  d.health.last_status=HAL_ERROR;
  unit_spi_txrx_sequence_count=24;
  for(unsigned i=0;i<24;i++)fill(i,zero,(fresh && i>=18)?3:2,i==1);
  EXPECT_TRUE(adbms2950_debug_capture(&d,&s,fresh)!=HAL_OK);
  EXPECT_TRUE(s.reg[1].status!=HAL_OK && s.reg[1].pec_fail==1);
  EXPECT_TRUE(s.reg[23].status==HAL_OK && unit_spi_txrx_calls==24);
  EXPECT_TRUE(d.health.current_a==12.5f && !d.health.sample_valid);
  EXPECT_TRUE(s.aux_completed==(fresh!=0));
  EXPECT_TRUE(d.health.last_stage==ADBMS2950_STAGE_SAMPLE &&
    d.health.last_reason==ADBMS2950_REASON_HARDWARE_FAULT && d.health.last_status==HAL_ERROR);
 }
 /* Fault ring retains changes, not every changing conversion counter. */
 unit_adbms2950_init_driver(&d,ic,&spi,&a,&b,1);
 unit_spi_txrx_sequence_count=12;
 for(unsigned i=0;i<12;i++){uint8_t f[6]={0};f[4]=(i&1)?64:0;fill(i,f,2,false);}
 for(unsigned i=0;i<12;i++){diag_tick=i;EXPECT_TRUE(adbms2950_read_flag(&d)==HAL_OK);}
 EXPECT_TRUE(d.fault_history_count==8 && d.fault_history_total==12);
 EXPECT_TRUE(d.fault_history[d.fault_history_next].tick_ms==4);
 /* Actual driver: cleared-to-fresh handshake, nominal screens and restore. */
 for(unsigned mode=4;mode<=6;mode++) for(unsigned scenario=0;scenario<7;scenario++) {
  unit_adbms2950_init_driver(&d,ic,&spi,&a,&b,1);unit_delay_timer_advances=true;
  d.health.sid_valid=true; d.health.device_id=6; d.health.initialized=true;
  uint8_t stat[6]={0,0x40,0,0,0,0};
  uint8_t original[6]={0,0,0,0,0,0xf0},test[6]={0,0,0,0,0,(uint8_t)(0xf0|mode)};
  uint8_t clear[6]={0,0,0xfc,0xff,0,0x80};
  int32_t value=mode==5?118000:(mode==6?-125000:0);
  uint16_t vb=mode==5?1180:(mode==6?23750:0);
  uint8_t result[6]={(uint8_t)value,(uint8_t)(value>>8),(uint8_t)(value>>16),0xff,(uint8_t)vb,(uint8_t)(vb>>8)};
  if(scenario==2) memcpy(result,clear,6); /* Ignored conversion must fail freshness. */
  if(scenario==3) {result[0]=0;result[1]=0;result[2]=0;result[4]=0x00;result[5]=0x10;} /* Fresh but wrong source. */
  unit_spi_txrx_sequence_count=32;
  fill(0,stat,2,false);fill(1,zero,2,false);fill(2,original,2,false);
  fill(3,test,4,false);fill(4,clear,5,false);fill(5,result,6,false);
  if(scenario==6) {fill(3,zero,4,false);fill(4,original,5,false);} /* Config verify mismatch. */
  else if(scenario==4) {fill(4,zero,5,false);fill(5,original,6,false);} /* Clear not observed. */
  else if(scenario==5) {uint8_t uncal[6]={0};fill(6,uncal,6,false);fill(7,original,7,false);}
  else if(scenario==2) fill(6,original,7,false); /* Immediate restore after sentinel rejection. */
  else {
   fill(6,stat,6,false);
   for(unsigned i=0;i<24;i++)fill(i+7,i==7?result:zero,6,false);
   fill(31,original,7,scenario==1);
  }
  HAL_StatusTypeDef restore;
  HAL_StatusTypeDef st=adbms2950_debug_adc_probe(&d,(uint8_t)mode,&s,&restore);
  EXPECT_TRUE((st==HAL_OK)==(scenario==0));
  EXPECT_TRUE((restore==HAL_OK)==(scenario!=1));
  EXPECT_TRUE(s.adc_probe.disturbed && s.adc_probe.restore_attempted);
  EXPECT_TRUE(s.adc_probe.clear_verified==(scenario!=4 && scenario!=6));
  EXPECT_TRUE(s.adc_probe.fresh==(scenario==0 || scenario==1 || scenario==3 || scenario==5));
  EXPECT_TRUE(s.adc_probe.expected_checked==(scenario==0 || scenario==1 || scenario==3));
  EXPECT_TRUE(s.adc_probe.expected_match==(scenario==0 || scenario==1));
  EXPECT_TRUE(!d.health.initialized && !d.health.i1_continuous_ready && !d.health.sample_valid);
 }
 /* Precheck failure: no write, no misleading restore attempt or readiness loss. */
 for(unsigned scenario=0;scenario<4;scenario++) {
  unit_adbms2950_init_driver(&d,ic,&spi,&a,&b,1);
  d.health.sid_valid=true;d.health.device_id=6;d.health.initialized=true;
  uint8_t stat[6]={0,0x40,0,0,0,0},cfg[6]={0};
  if(scenario==0) stat[1]=0;
  if(scenario==1) cfg[5]=0x20;
  unit_spi_txrx_sequence_count=3;
  fill(0,stat,2,false);fill(1,cfg,2,false);fill(2,zero,2,true);
  HAL_StatusTypeDef restore;
  EXPECT_TRUE(adbms2950_debug_adc_probe(&d,scenario==3?1:5,&s,&restore)!=HAL_OK);
  EXPECT_TRUE(!s.adc_probe.disturbed && !s.adc_probe.restore_attempted && d.health.initialized);
  EXPECT_TRUE(unit_spi_txrx_calls==(scenario==0?1:scenario==1?2:scenario==2?3:0));
 }
 printf("DIAG5 production-driver failures:%u\n",g_failures);
 return g_failures?1:0;
}
