/* Exercise the production 6830 driver + accumulator, replacing only HAL/time.
 * Simulated wire time is deterministic; this is not target WCET evidence. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ext_drivers/accumulator.h"
static uint64_t sim_us;
static uint64_t last_mux_end_us;
static uint32_t timer_base;
static uint32_t extra_delay_once;
static bool timer_stopped;
static uint32_t model_counter, transfers, largest_timeout;
static bool bad_pec, bad_counter, fail_spi;
static GPIO_PinState last_cs = GPIO_PIN_SET;
static uint32_t edge_count;
static unsigned failures;
#define CHECK(x) do { if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x); failures++;} }while(0)
static uint32_t timer_read(TIM_HandleTypeDef *h)
{
    if(!timer_stopped) sim_us++;
    if(extra_delay_once){ sim_us += extra_delay_once; extra_delay_once=0; }
    return (uint32_t)sim_us - timer_base;
}
static void timer_set(TIM_HandleTypeDef *h, uint32_t v){timer_base=(uint32_t)sim_us-v;}
#undef __HAL_TIM_GET_COUNTER
#undef __HAL_TIM_SET_COUNTER
#define __HAL_TIM_GET_COUNTER(h) timer_read(h)
#define __HAL_TIM_SET_COUNTER(h,v) timer_set(h,v)
static unsigned test_lock_depth;
static bool poison_on_unlock;
static bool poison_tx_on_acquire;
static void test_spi_lock(void);
static void test_spi_unlock(void);
#define adbms_spi_lock test_spi_lock
#define adbms_spi_unlock test_spi_unlock
/* Include the real source so static helpers are tested without exported APIs. */
#include "Core/Src/ext_drivers/adbms6830.c"
#include "Core/Src/ext_drivers/accumulator.c"

static void test_spi_lock(void) {
    /* Emulate another owner modifying the shared TX scratch before acquisition. */
    if(poison_tx_on_acquire && test_lock_depth==0u)
        memset(write_buf,0xcd,sizeof(write_buf));
    test_lock_depth++;
}
static void test_spi_unlock(void) {
    CHECK(test_lock_depth > 0);
    if(poison_on_unlock && test_lock_depth == 1u)
        memset(spi_txrx_rx_buf, 0xcd, sizeof(spi_txrx_rx_buf));
    test_lock_depth--;
}

uint32_t HAL_GetTick(void){return (uint32_t)(sim_us/1000u);}
uint32_t osKernelGetTickCount(void){return HAL_GetTick();}
void HAL_GPIO_WritePin(GPIO_TypeDef *p,uint16_t pin,GPIO_PinState value)
{
    if(value!=last_cs) edge_count++;
    last_cs=value;
}
static uint32_t clock_us(void *p){return (uint32_t)sim_us;}
static HAL_StatusTypeDef wait_us(void *p,uint32_t us){sim_us+=us;return HAL_OK;}
static void wire_time(SPI_HandleTypeDef *h,uint16_t n,uint32_t timeout)
{
    uint32_t div=1u<<(((h->Init.BaudRatePrescaler & SPI_CR1_BR)>>3u)+1u);
    uint32_t us=((uint32_t)n*8u*div+107u)/108u;
    CHECK(timeout*1000u >= us);
    if(timeout>largest_timeout) largest_timeout=timeout;
    transfers++; sim_us+=us;
}
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t timeout)
{
    CHECK(test_lock_depth > 0);
    wire_time(h,n,timeout);
    if(fail_spi) return HAL_TIMEOUT;
    CHECK(n>=4);
    uint16_t pec=Pec15_Calc(2,p);
    CHECK(p[2]==(pec>>8) && p[3]==(pec&255));
    if(p[0]==0x07u && p[1]==0x23u) last_mux_end_us=sim_us;
    if(p[0]==0x04u && p[1]>=0x11u && p[1]<=0x13u && last_mux_end_us!=0u)
        CHECK(sim_us-last_mux_end_us>=3000u);
    if(p[0]==0 && (p[1]==0x27 || p[1]==0x2e)) model_counter=0;
    else model_counter=(model_counter%63u)+1u;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,uint8_t *tx,uint8_t *rx,uint16_t n,uint32_t timeout)
{
    CHECK(test_lock_depth > 0);
    wire_time(h,n,timeout);
    if(fail_spi) return HAL_TIMEOUT;
    memset(rx,0,n);
    CHECK(n>=12 && (n-4)%8==0);
    uint16_t command=(uint16_t)((tx[0]<<8)|tx[1]);
    for(unsigned i=4;i+7<n;i+=8)
    {
        uint8_t *p=rx+i;
        /* Coherent normal cells/AUX codes, status C counter, COMM ACK. */
        for(unsigned k=0;k<6;k+=2){p[k]=0xa8; p[k+1]=0x61;}
        if(command==0x0722){p[0]=0x67;p[1]=0x98;p[2]=0x77;p[3]=1;p[4]=0x1f;p[5]=0xff;}
        if(command==0x0012 || command==0x0030){p[0]=p[1]=p[4]=p[5]=0;p[2]=0;p[3]=1;}
        p[6]=(uint8_t)(((model_counter+(bad_counter?1u:0u))&63u)<<2);
        uint16_t pec=pec10_calc(1,6,p); p[6]|=(uint8_t)(pec>>8);p[7]=(uint8_t)pec;
        if(bad_pec) p[7]^=1;
    }
    return HAL_OK;
}
static adbms6830_driver_t dev;
static adbms6830_asic ics[5];
static SPI_HandleTypeDef spi;
static TIM_HandleTypeDef tim;
static TIM_TypeDef timer;
static GPIO_TypeDef gpio;
static void reset_dev(unsigned count)
{
    memset(&dev,0,sizeof(dev));memset(ics,0,sizeof(ics));
    memset(&spi,0,sizeof(spi));memset(&tim,0,sizeof(tim));
    sim_us=0; timer_base=0; edge_count=0;model_counter=0;transfers=0;largest_timeout=0;
    last_mux_end_us=0;
    bad_pec=bad_counter=fail_spi=timer_stopped=false;extra_delay_once=0;last_cs=GPIO_PIN_SET;
    spi.Instance=SPI6;
    spi.Init.DataSize=SPI_DATASIZE_8BIT;
    spi.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;
    tim.Instance=&timer;dev.htim=&tim;dev.hspi=&spi;
    dev.ics=ics;dev.ics_capacity=count;dev.num_ics=count;dev.physical_chain_count=count;
    dev.monitored_cell_count=15;dev.string=dev.write_string=STRING_B;
    dev.cs_port[0]=dev.cs_port[1]=&gpio;dev.cs_pin[0]=2;dev.cs_pin[1]=4;
    adbms6830_bind_runtime_hooks(&dev,wait_us,clock_us,NULL);
}
static void read_clean(void)
{
    uint8_t response[40];
    CHECK(adbms6830_rd48_checked(&dev,RDCFGA,response)==HAL_OK);
    CHECK(adbms6830_last_read_integrity_ok(&dev));
    CHECK(dev.wake.verified);
}
/* Audit real entry points. Generic packets prove transport/wake behavior,
 * not valid hardware configuration/reference thresholds. */
#define WAKE_AUDIT(label, operation) do { \
    reset_dev(5);read_clean();sim_us+=100000u; \
    uint32_t cold0=dev.session.cold_wake_count,warm0=dev.session.warm_wake_count; \
    uint64_t t0=sim_us;HAL_StatusTypeDef result=(operation); \
    printf("WAKE_AUDIT %s status:%d cold:%lu warm:%lu time_us:%llu\n",label,(int)result, \
      (unsigned long)(dev.session.cold_wake_count-cold0), \
      (unsigned long)(dev.session.warm_wake_count-warm0),(unsigned long long)(sim_us-t0)); \
    CHECK(dev.session.cold_wake_count==cold0); \
}while(0)
int main(void)
{
    reset_dev(5); read_clean();
    poison_tx_on_acquire=true;
    uint8_t cfg_payload[30]={0};
    CHECK(adbms6830_cmd_checked(&dev,MUTE)==HAL_OK);
    CHECK(adbms6830_wr48_checked(&dev,WRCFGA,cfg_payload)==HAL_OK);
    CHECK(test_lock_depth==0u);
    poison_tx_on_acquire=false;
    puts("PASS command/write preparation protected against competing scratch owner");
    /* Exhaust millisecond tick phases at the fine-clock boundary. A long
     * HAL completion observation must not refresh the physical lower bound. */
    for(unsigned phase=0; phase<1000; phase++) {
        reset_dev(5);
        uint32_t start=100000u+phase;
        sim_us=start+834u+2999u;
        uint32_t proof=adbms6830_frame_activity_lower_bound(&dev,start,44);
        CHECK(proof==start+834u);
        adbms6830_session_note_activity(&dev,proof,start/1000u);
        adbms6830_session_open(&dev);
        dev.session.coherent_snapshot_active=true;
        uint32_t n=edge_count;
        CHECK(adbms6830_session_require_awake(&dev)==HAL_OK);
        CHECK(edge_count==n);
        CHECK(dev.session.last_gap_us==2999u);
        adbms6830_session_note_activity(&dev,(uint32_t)sim_us,HAL_GetTick());
        CHECK(dev.session.last_gap_us==2999u);
        /* Restore the physical proof for the exact-expiry assertion. */
        adbms6830_session_note_activity(&dev,proof,start/1000u);
        sim_us++;
        CHECK(adbms6830_session_require_awake(&dev)==HAL_BUSY);
        CHECK(edge_count==n); /* no wake spliced into SNAP */
    }
    /* A full microsecond wrap must not resurrect the cached awake proof. */
    reset_dev(5); sim_us=100000u;
    adbms6830_session_note_activity(&dev,(uint32_t)sim_us,HAL_GetTick());
    adbms6830_session_open(&dev); dev.session.coherent_snapshot_active=true;
    sim_us += (1ULL<<32)+100u;
    CHECK(adbms6830_session_require_awake(&dev)==HAL_BUSY);
    CHECK(edge_count==0);
    /* Short intervals across both clocks' rollover remain legitimate. */
    reset_dev(5); sim_us=((uint64_t)UINT32_MAX)*1000u+500u;
    adbms6830_session_note_activity(&dev,(uint32_t)sim_us,HAL_GetTick());
    adbms6830_session_open(&dev); dev.session.coherent_snapshot_active=true;
    sim_us+=2500u;
    CHECK(adbms6830_session_require_awake(&dev)==HAL_OK);
    CHECK(edge_count==0);
    /* Opening a logical session cannot renew an expired physical proof. */
    reset_dev(5); sim_us=100000u;
    adbms6830_session_note_activity(&dev,(uint32_t)sim_us,HAL_GetTick());
    sim_us+=8000u;
    adbms6830_session_open(&dev); dev.session.coherent_snapshot_active=true;
    CHECK(adbms6830_session_require_awake(&dev)==HAL_BUSY);
    CHECK(edge_count==0);
    puts("PASS 1000 tick phases, exact idle boundary, clock wraps, session reopen");
    reset_dev(5);
    CHECK(adbms6830_wakeup_checked(&dev)==HAL_OK);
    CHECK(dev.session.cold_wake_count==1 && dev.session.warm_wake_count==0);
    CHECK(sim_us>=10000 && sim_us<10100 && edge_count==10);
    uint8_t command[4] = {0, 0x2c, 0x59, 0x90};
    uint8_t reply[40];
    CHECK(adbms6830_spi_write_read(&dev, command, 4, reply, 40, 1)==HAL_OK);
    CHECK(dev.session.frame_trace_sequence == 1u);
    CHECK(dev.session.frame_trace[0].bytes == 44u);
    CHECK(dev.session.frame_trace[0].end_us - dev.session.frame_trace[0].start_us >= 834u);
    for(unsigned j=0; j<40; j++)
        CHECK(adbms6830_spi_write_read(&dev, command, 4, reply, 40, 1)==HAL_OK);
    CHECK(dev.session.frame_trace_sequence == 41u);
    CHECK(dev.session.frame_trace[8].status == HAL_OK);
    uint32_t frame_start=(uint32_t)sim_us;
    sim_us += 10000u; /* late observation must not become recent activity */
    CHECK(adbms6830_frame_activity_lower_bound(&dev, frame_start, 44)==frame_start+834u);
    frame_start=UINT32_MAX-400u; sim_us=(uint64_t)UINT32_MAX+1000u;
    CHECK(adbms6830_frame_activity_lower_bound(&dev, frame_start, 44)==frame_start+834u);
    sim_us=50000u;
    poison_on_unlock = true;
    CHECK(adbms6830_spi_write_read(&dev, command, 4, reply, 40, 1)==HAL_OK);
    CHECK(reply[0] != 0xcd); /* RX must be copied before ownership release. */
    CHECK(test_lock_depth == 0u);
    poison_on_unlock = false;
    uint32_t edges=edge_count;
    CHECK(adbms6830_wakeup_checked(&dev)==HAL_OK && edge_count==edges);
    read_clean(); sim_us+=100000;
    uint64_t before=sim_us;
    CHECK(adbms6830_wakeup_checked(&dev)==HAL_OK);
    CHECK(dev.session.warm_wake_count==1 && sim_us-before>=500 && sim_us-before<600);
    read_clean();sim_us+=1000000;
    CHECK(adbms6830_wakeup_checked(&dev)==HAL_OK && dev.session.cold_wake_count==2);
    read_clean();dev.string=STRING_A;
    CHECK(adbms6830_wakeup_checked(&dev)==HAL_OK && dev.session.cold_wake_count==3);
    dev.write_string=STRING_A;read_clean();
    bad_pec=true;uint8_t response[40];
    CHECK(adbms6830_rd48_checked(&dev,RDCFGA,response)==HAL_OK && !dev.wake.verified);
    bad_pec=false;before=dev.session.cold_wake_count;read_clean();
    CHECK(dev.session.cold_wake_count==before+1);
    bad_counter=true;CHECK(adbms6830_rd48_checked(&dev,RDCFGA,response)==HAL_OK && !dev.wake.verified);
    bad_counter=false;adbms6830_resync_command_counter_tracking(&dev);read_clean();
    bool owned=false;CHECK(adbms6830_transaction_session_begin(&dev,&owned)==HAL_OK);
    dev.session.coherent_snapshot_active=true;sim_us+=4300;
    edges=edge_count;CHECK(adbms6830_rd48_checked(&dev,RDCFGA,response)==HAL_BUSY);
    CHECK(edge_count==edges);dev.session.coherent_snapshot_active=false;
    adbms6830_transaction_session_end(&dev,owned);
    sim_us+=5000;extra_delay_once=5000;
    CHECK(adbms6830_wakeup_checked(&dev)==HAL_BUSY && !dev.wake.activity_valid);
    CHECK(last_cs==GPIO_PIN_SET && dev.session.wake_gap_abort_count==1);
    timer_stopped=true;CHECK(adbms6830_wakeup_checked(&dev)==HAL_TIMEOUT);
    timer_stopped=false;CHECK(last_cs==GPIO_PIN_SET);read_clean();
    fail_spi=true;CHECK(adbms6830_rd48_checked(&dev,RDCFGA,response)==HAL_TIMEOUT && !dev.wake.verified);
    fail_spi=false;read_clean();
    /* Explicit reset and clock-epoch replacement cannot retain warm proof. */
    CHECK(adbms6830_cmd_checked(&dev,SRST)==HAL_OK && !dev.wake.verified);
    read_clean();adbms6830_bind_runtime_hooks(&dev,wait_us,clock_us,NULL);CHECK(!dev.wake.verified);
    /* Timestamp zero and the 32-bit microsecond wrap are valid. */
    reset_dev(5);sim_us=UINT32_MAX-20000u;read_clean();sim_us+=30000u;
    before=dev.session.cold_wake_count;CHECK(adbms6830_wakeup_checked(&dev)==HAL_OK);
    CHECK(dev.session.cold_wake_count==before && dev.session.warm_wake_count==1);
    for(unsigned n=1;n<=5;n+=4)
    {
        reset_dev(n);read_clean();sim_us+=100000;
        before=sim_us;adbms6830_session_begin_scan(&dev);
        CHECK(smb_read_temp(&dev)==0);adbms6830_session_end_scan(&dev);
        printf("TEMP %u ICs: %llu us, wakes %lu, cold %lu warm %lu\n",n,
          (unsigned long long)(sim_us-before),(unsigned long)dev.session.wake_count_last_scan,
          (unsigned long)dev.session.cold_wake_count,(unsigned long)dev.session.warm_wake_count);
        CHECK(sim_us-before<31000u);CHECK(dev.session.cold_wake_count==1);
        CHECK(dev.session.wake_count_last_scan<=5);
        for(unsigned i=0;i<n;i++) CHECK(dev.last_temp_updated_mask[i]!=0);
        CHECK(largest_timeout<=3);
    }
    /* Real successful voltage+temperature production calls at 5 SMBs. */
    reset_dev(5);read_clean();sim_us+=100000;before=sim_us;
    CHECK(smb_read_voltage(&dev)==0);
#if AMS_ESTIMATOR_VOLTAGE_SOURCE == AMS_ESTIMATOR_VOLTAGE_SOURCE_RAW
    if(AMS_ADBMS_OPTIONAL_PRODUCT_DIVISOR > 1u) {
        uint64_t extra_start=sim_us;
        uint32_t reads=dev.health.avg8_read_count;
        CHECK(smb_read_voltage(&dev)==0);
        CHECK(dev.health.avg8_read_count==reads);
        CHECK(dev.last_acell_updated_mask[0]==0u);
        CHECK(dev.last_cell_updated_mask[0]!=0u);
        before += sim_us-extra_start;
    }
#endif
    CHECK(smb_read_temp(&dev)==0);
    printf("VOLT+TEMP nominal wire/delay model: %llu us\n",(unsigned long long)(sim_us-before));
    CHECK(sim_us-before<70000u);
    /* Status AUX conversion spans the scheduler gap, not a blocking 20 ms
     * extension of the measurement scan. Reference values in this wire model
     * are intentionally generic: test sequencing/transport, not thresholds. */
    CHECK(adbms6830_start_status_conversion(&dev)==HAL_OK);
    CHECK(dev.status_conversion_pending);
    CHECK(adbms6830_complete_status_conversion(&dev)==HAL_BUSY);
    CHECK(adbms6830_start_status_conversion(&dev)==HAL_OK);
    sim_us+=25000; before=sim_us;
    (void)adbms6830_complete_status_conversion(&dev);
    CHECK(!dev.status_conversion_pending && sim_us-before<6000u);
    printf("STATUS completion nominal wire model: %llu us\n",(unsigned long long)(sim_us-before));
    CHECK(adbms6830_start_status_conversion(&dev)==HAL_OK);
    uint8_t different_aux[2]={0x04,0x11};
    CHECK(adbms6830_cmd_checked(&dev,different_aux)==HAL_OK);
    sim_us+=25000;
    CHECK(adbms6830_complete_status_conversion(&dev)==HAL_BUSY);
    CHECK(adbms6830_start_status_conversion(&dev)==HAL_OK);
    sim_us+=1501000;
    CHECK(adbms6830_complete_status_conversion(&dev)==HAL_BUSY);
    /* Publication must not make an old physical sample appear new. */
    static accumulator_t acc;
    reset_dev(5);read_clean();sim_us+=100000;
    CHECK(smb_read_temp(&dev)==0);
    for(unsigned i=0;i<5;i++)
    {
        CHECK(dev.temp_scheduled_mask[i]==dev.last_temp_updated_mask[i]);
        CHECK(dev.temp_attempted_mask[i]==dev.temp_scheduled_mask[i]);
        CHECK((dev.temp_capture_tick_valid_mask[i]&dev.last_temp_updated_mask[i])
              ==dev.last_temp_updated_mask[i]);
    }
    memset(&acc,0,sizeof(acc));acc.smb=dev;
    unsigned sensor=0;while(!(dev.last_temp_updated_mask[0]&(1u<<sensor)))sensor++;
    /* Generic wire fixture codes are not calibrated thermistor values. */
    ics[0].temp.raw[sensor]=5000;
    uint32_t captured=dev.temp_capture_tick_ms[0][sensor];
    accumulator_update_temp_stats_at(&acc,captured+12001u);
    CHECK(acc.temp_last_update_ms[0][sensor]==captured);
    CHECK(!(acc.usable_temp_mask[0]&(1u<<sensor)));
    /* Timestamp zero and wrap are valid, not missing-data sentinels. */
    acc.smb.temp_capture_tick_ms[0][sensor]=0;
    acc.smb.temp_sample_generation[0][sensor]++;
    accumulator_update_temp_stats_at(&acc,1u);
    CHECK(acc.temp_last_update_ms[0][sensor]==0u);
    acc.smb.temp_capture_tick_ms[0][sensor]=UINT32_MAX-5u;
    acc.smb.temp_sample_generation[0][sensor]++;
    accumulator_update_temp_stats_at(&acc,4u);
    CHECK(acc.usable_temp_mask[0]&(1u<<sensor));
    /* Scheduled failures differ from unrelated mux positions and are counted
     * once even if the stats API evaluates the same generation repeatedly. */
    memset(acc.smb.last_temp_updated_mask,0,sizeof(acc.smb.last_temp_updated_mask));
    memset(acc.smb.temp_scheduled_mask,0,sizeof(acc.smb.temp_scheduled_mask));
    acc.smb.temp_scheduled_mask[0]=1u<<sensor;
    acc.smb.temp_visit_generation++;
    unsigned other=(sensor+1u)%24u;
    accumulator_update_temp_stats_at(&acc,5u);
    CHECK(acc.temp_failed_visits[0][sensor]==1u);
    CHECK(acc.temp_failed_visits[0][other]==0u);
    accumulator_update_temp_stats_at(&acc,5u);
    CHECK(acc.temp_failed_visits[0][sensor]==1u);
    acc.smb.temp_scheduled_mask[0]=0;
    accumulator_update_temp_stats_at(&acc,6u);
    CHECK(acc.temp_failed_visits[0][sensor]==1u);
    /* Early transport rejection must not replay the last update mask. */
    acc.smb.last_temp_updated_mask[0]=1u<<sensor;
    acc.smb_transport_ready=false;
    CHECK(accumulator_read_temp(&acc)==-1);
    CHECK(acc.smb.last_temp_updated_mask[0]==0u);
    /* The public wrapper must use RTOS time, not a constant zero. Test both
     * entry points and keep the producer timestamp unchanged throughout. */
    acc.smb.last_temp_updated_mask[0]=1u<<sensor;
    acc.smb.temp_capture_tick_ms[0][sensor]=100u;
    acc.smb.temp_sample_generation[0][sensor]++;
    sim_us=101000u;
    accumulator_update_temp_stats(&acc);
    CHECK(acc.temp_last_update_ms[0][sensor]==100u);
    CHECK(acc.usable_temp_mask[0]&(1u<<sensor));
    CHECK(accumulator_stat_temp(&acc)==0);
    CHECK(acc.usable_temp_mask[0]&(1u<<sensor));
    sim_us=(100u+ACCUMULATOR_TEMP_REVISIT_TIMEOUT_MS)*1000ull;
    accumulator_update_temp_stats(&acc);
    CHECK(acc.usable_temp_mask[0]&(1u<<sensor)); /* exact revisit limit */
    sim_us+=1000u;
    accumulator_update_temp_stats(&acc);
    CHECK(!(acc.usable_temp_mask[0]&(1u<<sensor)));
    /* The same API preserves a legitimate zero timestamp and wrap age. */
    acc.smb.temp_capture_tick_ms[0][sensor]=0u;
    acc.smb.temp_sample_generation[0][sensor]++;
    sim_us=1000u;
    accumulator_update_temp_stats(&acc);
    CHECK(acc.temp_last_update_ms[0][sensor]==0u);
    CHECK(acc.usable_temp_mask[0]&(1u<<sensor));
    acc.smb.temp_capture_tick_ms[0][sensor]=UINT32_MAX-5u;
    acc.smb.temp_sample_generation[0][sensor]++;
    sim_us=((uint64_t)UINT32_MAX+5u)*1000u;
    accumulator_update_temp_stats(&acc);
    CHECK(acc.usable_temp_mask[0]&(1u<<sensor));
    puts("PASS public temperature wrappers, stale boundary, zero and wrap");
    /* Exercise eight real mux positions with realistic cadence. The generic
     * wire fixture returns supply-like AUX values; substitute valid NTC codes
     * only after capture so scheduling/transport stay production paths. */
    reset_dev(5);read_clean();memset(&acc,0,sizeof(acc));
    for(unsigned step=0;step<8;step++)
    {
        sim_us=100000u+step*100000u;
        CHECK(smb_read_temp(&dev)==0);
        for(unsigned ic=0;ic<5;ic++)
            for(unsigned s=0;s<24;s++)
                if(dev.last_temp_updated_mask[ic]&(1u<<s))ics[ic].temp.raw[s]=5000;
        acc.smb=dev;
        accumulator_update_temp_stats_at(&acc,osKernelGetTickCount());
    }
    CHECK(acc.temp_full_usable && acc.usable_temp_count==120u);
    CHECK(acc.temp_sweep_count==1u);
    CHECK(acc.temp_oldest_required_age_ms<800u);
    uint32_t last_sweep=acc.temp_last_sweep_tick;
    int16_t filtered=acc.temp_filtered_deci_c[0][0];
    accumulator_update_temp_stats_at(&acc,osKernelGetTickCount());
    CHECK(acc.updated_temp_count==0u && acc.temp_sweep_count==1u);
    CHECK(acc.temp_filtered_deci_c[0][0]==filtered);
    /* One scheduled transport failure fails that channel immediately. */
    memset(acc.smb.last_temp_updated_mask,0,sizeof(acc.smb.last_temp_updated_mask));
    memset(acc.smb.temp_scheduled_mask,0,sizeof(acc.smb.temp_scheduled_mask));
    acc.smb.temp_visit_generation++;
    acc.smb.temp_scheduled_mask[0]=1u;
    accumulator_update_temp_stats_at(&acc,last_sweep+100u);
    CHECK(acc.temp_consecutive_misses[0][0]==1u);
    CHECK(acc.temp_consecutive_misses[0][1]==0u);
    CHECK(!(acc.usable_temp_mask[0]&1u));
    CHECK(acc.usable_temp_mask[0]&2u);
    CHECK(acc.temp_last_sweep_tick==last_sweep);
    acc.smb.temp_visit_generation++;
    acc.smb.last_temp_updated_mask[0]=1u;
    acc.smb.temp_sample_generation[0][0]++;
    acc.smb.temp_capture_tick_ms[0][0]=last_sweep+200u;
    accumulator_update_temp_stats_at(&acc,last_sweep+200u);
    CHECK(acc.temp_consecutive_misses[0][0]==0u && (acc.usable_temp_mask[0]&1u));
    /* A missing timestamp never becomes a newly fresh valid sample. */
    acc.smb.last_temp_updated_mask[0]=4u;
    acc.smb.temp_capture_tick_valid_mask[0]&=~4u;
    int16_t prior=acc.temp_deci_c[0][2];
    ics[0].temp.raw[2]=10000;
    accumulator_update_temp_stats_at(&acc,last_sweep+201u);
    CHECK(acc.updated_temp_count==0u && acc.temp_deci_c[0][2]==prior);
    /* An invalid electrical sample stays invalid between its scheduled
     * visits; unrelated scans must not erase the open-sensor diagnostic. */
    acc.smb.last_temp_updated_mask[0]=8u;
    acc.smb.temp_sample_generation[0][3]++;
    acc.smb.temp_capture_tick_ms[0][3]=last_sweep+202u;
    ics[0].temp.raw[3]=-10000;
    accumulator_update_temp_stats_at(&acc,last_sweep+202u);
    CHECK(acc.temp_open_mask[0]&8u);
    memset(acc.smb.last_temp_updated_mask,0,sizeof(acc.smb.last_temp_updated_mask));
    accumulator_update_temp_stats_at(&acc,last_sweep+203u);
    CHECK(acc.temp_open_mask[0]&8u);
    CHECK(!(acc.usable_temp_mask[0]&8u));
    /* Time-based filter is 1/8 at the configured nominal eight-step revisit. */
    CHECK(accumulator_iir_deci_c(0,800,800u)==100);
    CHECK(accumulator_iir_deci_c(0,800,1600u)>100);
    CHECK(accumulator_iir_deci_c(-100,100,0u)==-100);
    puts("PASS rolling sweep, duplicate rejection, failed visit, recovery, timestamp provenance and time filter");
    /* Persistent SPI failure remains bounded and cannot publish freshness. */
    reset_dev(5);read_clean();fail_spi=true;
    before=sim_us;CHECK(smb_read_temp(&dev)!=0);
    CHECK(sim_us-before<45000u);
    for(unsigned ic=0;ic<5;ic++)CHECK(dev.last_temp_updated_mask[ic]==0u);
    puts("PASS capture age, wrap, scheduled visits and transport rejection");
    WAKE_AUDIT("SID",adbms6830_read_sid(&dev));
    WAKE_AUDIT("configuration",adbms6830_verify_config_readback(&dev));
    WAKE_AUDIT("status",adbms6830_read_status(&dev,false));
    WAKE_AUDIT("references",adbms6830_refresh_diagnostics(&dev));
    WAKE_AUDIT("mute",adbms6830_mute_checked(&dev));
    WAKE_AUDIT("unmute",adbms6830_unmute_checked(&dev));
    reset_dev(5);read_clean();
    uint32_t cold0=dev.session.cold_wake_count,warm0=dev.session.warm_wake_count;
    for(unsigned burst=0;burst<20;burst++)read_clean();
    CHECK(dev.session.cold_wake_count==cold0 && dev.session.warm_wake_count==warm0);
    puts("PASS nested/burst wake checks do not emit additional wake trains");
    if(failures){fprintf(stderr,"%u failures\n",failures);return 1;}
    puts("PASS production 6822/6830 wake/idle/integrity/timing regressions");return 0;
}
