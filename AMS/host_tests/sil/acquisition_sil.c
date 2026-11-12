/* Exercise the production 6830 driver + accumulator, replacing only HAL/time.
 * Simulated wire time is deterministic; this is not target WCET evidence. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ext_drivers/accumulator.h"
/* Fake MMIO used by the real per-frame prescaler policy. */
static SPI_TypeDef spi_regs;
static RCC_TypeDef rcc_regs;
#undef SPI6
#define SPI6 (&spi_regs)
#undef RCC
#define RCC (&rcc_regs)
#undef __HAL_RCC_SPI6_FORCE_RESET
#undef __HAL_RCC_SPI6_RELEASE_RESET
#define __HAL_RCC_SPI6_FORCE_RESET() do { spi_regs.SR=0; spi_regs.CR1=0; spi_regs.CR2=0; } while(0)
#define __HAL_RCC_SPI6_RELEASE_RESET() ((void)0)
static uint64_t sim_us;
static adbms6830_driver_t *sil_device;
static unsigned coherent_reads;
static unsigned read_number, pause_every, corrupt_every, timeout_every;
static uint32_t pause_us;
static bool rounded_wait, persistent_pec;
static uint64_t last_mux_end_us;
static uint64_t model_aux_ready_us;
static bool reply_ref_off, reply_soak;
static uint32_t baseline_prescaler;
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
static void test_spi_lock(void);
static void test_spi_unlock(void);
#define adbms_spi_lock test_spi_lock
#define adbms_spi_unlock test_spi_unlock
/* Include the real source so static helpers are tested without exported APIs. */
#include "Core/Src/ext_drivers/adbms6830.c"
#include "Core/Src/ext_drivers/accumulator.c"

static void test_spi_lock(void) { test_lock_depth++; }
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
static HAL_StatusTypeDef wait_us(void *p,uint32_t us){
    sim_us += rounded_wait ? ((us+999u)/1000u)*1000u : us;
    return HAL_OK;
}
static void wire_time(SPI_HandleTypeDef *h,uint16_t n,uint32_t timeout)
{
    uint32_t div=1u<<(((h->Init.BaudRatePrescaler & SPI_CR1_BR)>>3u)+1u);
    uint32_t us=((uint32_t)n*8u*div+107u)/108u;
    CHECK(timeout*1000u >= us);
    if(timeout>largest_timeout) largest_timeout=timeout;
    transfers++; sim_us+=us;
}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p,uint16_t pin) { return last_cs; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *h,uint8_t *p,uint16_t n,uint32_t timeout)
{
    CHECK(test_lock_depth > 0);
    CHECK(h->Init.BaudRatePrescaler == ((p[0]==0x07 && p[1]==0x23) ? SPI_BAUDRATEPRESCALER_256 : baseline_prescaler));
    wire_time(h,n,timeout);
    if(fail_spi) {
        if(n>=2 && p[0]==0x07 && p[1]==0x23) {
            CHECK(h->Init.BaudRatePrescaler==SPI_BAUDRATEPRESCALER_256);
            h->Instance->SR |= SPI_SR_BSY;
            h->State=HAL_SPI_STATE_BUSY_TX;
        }
        return HAL_TIMEOUT;
    }
    CHECK(n>=4);
    uint16_t pec=Pec15_Calc(2,p);
    CHECK(p[2]==(pec>>8) && p[3]==(pec&255));
    if(p[0]==0x07u && p[1]==0x23u) {
        CHECK(h->Init.BaudRatePrescaler==SPI_BAUDRATEPRESCALER_256);
        CHECK((h->Instance->CR1 & SPI_CR1_BR)==SPI_BAUDRATEPRESCALER_256);
        last_mux_end_us=sim_us;
    }
    if(p[0]==0x04u && p[1]>=0x11u && p[1]<=0x13u && last_mux_end_us!=0u)
        CHECK(sim_us-last_mux_end_us>=3000u);
    if(p[0]==0x04u && p[1]>=0x11u && p[1]<=0x13u)
        model_aux_ready_us=sim_us+1112u; /* datasheet slowest AUX period */
    if(p[0]==0 && (p[1]==0x27 || p[1]==0x2e)) model_counter=0;
    else model_counter=(model_counter%63u)+1u;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *h,uint8_t *tx,uint8_t *rx,uint16_t n,uint32_t timeout)
{
    CHECK(test_lock_depth > 0);
    CHECK(h->Init.BaudRatePrescaler == ((tx[0]==0x07 && tx[1]==0x23) ? SPI_BAUDRATEPRESCALER_256 : baseline_prescaler));
    wire_time(h,n,timeout);
    read_number++;
    bool coherent = sil_device && sil_device->session.coherent_snapshot_active;
    if(coherent) coherent_reads++;
    if(coherent && pause_every && coherent_reads % pause_every == 0) sim_us += pause_us;
    if(fail_spi || (coherent && timeout_every && coherent_reads % timeout_every == 0)) return HAL_TIMEOUT;
    memset(rx,0,n);
    CHECK(n>=12 && (n-4)%8==0);
    uint16_t command=(uint16_t)((tx[0]<<8)|tx[1]);
    if(command==0x0019u && model_aux_ready_us) CHECK(sim_us>=model_aux_ready_us);
    for(unsigned i=4;i+7<n;i+=8)
    {
        uint8_t *p=rx+i;
        /* Coherent normal cells/AUX codes, status C counter, COMM ACK. */
        for(unsigned k=0;k<6;k+=2){p[k]=0xa8; p[k+1]=0x61;}
        if(command==0x0002){ memset(p,0,6); p[0]=reply_ref_off?0:0x80; p[2]=reply_soak?0x80:0; } /* REFON, no soak */
        if(command==0x0722){p[0]=0x67;p[1]=0x98;p[2]=0x77;p[3]=1;p[4]=0x1f;p[5]=0xff;}
        if(command==0x0012 || command==0x0030){p[0]=p[1]=p[4]=p[5]=0;p[2]=0;p[3]=1;}
        p[6]=(uint8_t)(((model_counter+(bad_counter?1u:0u))&63u)<<2);
        uint16_t pec=pec10_calc(1,6,p); p[6]|=(uint8_t)(pec>>8);p[7]=(uint8_t)pec;
        if(bad_pec || persistent_pec || (coherent && corrupt_every && coherent_reads % corrupt_every==0)) p[7]^=1;
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
    sil_device=&dev; coherent_reads=0;
    memset(&dev,0,sizeof(dev));memset(ics,0,sizeof(ics));
    memset(&spi,0,sizeof(spi));memset(&tim,0,sizeof(tim));
    sim_us=0; timer_base=0; edge_count=0;model_counter=0;transfers=0;largest_timeout=0;
    last_mux_end_us=0; model_aux_ready_us=0; reply_ref_off=reply_soak=false;
    bad_pec=bad_counter=fail_spi=timer_stopped=false;extra_delay_once=0;last_cs=GPIO_PIN_SET;
    memset(&spi_regs,0,sizeof(spi_regs));
    spi.Instance=SPI6;
    spi.State=HAL_SPI_STATE_READY;
    spi.Init.DataSize=SPI_DATASIZE_8BIT;
    #if AMS_ADBMS_SPI_PRESCALER_DIV == 64
    spi.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_64;
#elif AMS_ADBMS_SPI_PRESCALER_DIV == 128
    spi.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_128;
#else
    spi.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256;
#endif
    baseline_prescaler=spi.Init.BaudRatePrescaler;
    spi_regs.CR1=spi.Init.BaudRatePrescaler|SPI_CR1_SPE;
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
#ifdef SIL_CANDIDATE
    /* Fine idle proof must not be vetoed by frame-start millisecond age. */
    reset_dev(5); read_clean(); sim_us=100000u;
    adbms6830_session_open(&dev); dev.session.coherent_snapshot_active=true;
    pause_every=1; pause_us=2500u;
    uint8_t response[40];
    uint8_t frame[4]={0,0x2c,0x59,0x90};
    CHECK(adbms6830_spi_write_read(&dev,frame,4,response,40,1)==HAL_OK);
    CHECK((uint32_t)(sim_us-dev.wake.activity_us)<3000u);
#if AMS_ADBMS_SPI_PRESCALER_DIV == 256
    CHECK((uint32_t)(HAL_GetTick()-dev.wake.activity_ms)>=3u);
#endif
    CHECK(adbms6830_rd48_checked(&dev,RDCFGA,response)==HAL_OK);
    pause_every=0;
#endif
    puts("scenario,scans,accepted,rejected,temp_rejected,mean_us,max_us,over_100ms,transfers,restarts,warm_wakes");
    for(unsigned scenario=0;scenario<7;scenario++) {
        const char *names[]={"nominal","tick_rounding","preemption_2p5ms","preemption_8ms","transient_pec","transient_timeout","persistent_pec"};
        reset_dev(5); read_clean(); read_number=0;
        rounded_wait=scenario==1; pause_every=(scenario==2 ? 1 : scenario==3 ? 37 : 0);
        pause_us=scenario==2 ? 2500 : 8000;
        corrupt_every=scenario==4 ? 37 : 0; timeout_every=scenario==5 ? 37 : 0;
        persistent_pec=scenario==6;
        unsigned accepted=0,rejected=0,overrun=0,temp_rejected=0; uint64_t total=0,maximum=0;
        /* Cross the microsecond counter wrap in every scenario. */
        sim_us=(uint64_t)UINT32_MAX-100000u;
        for(unsigned scan=0;scan<200;scan++) {
            uint64_t begin=sim_us;
            adbms6830_session_begin_scan(&dev);
            (void)smb_prepare_temp(&dev);
            int rc=smb_read_voltage(&dev);
            if(rc==0) {
                accepted++;
                for(unsigned ic=0;ic<5;ic++) { CHECK((dev.last_cell_updated_mask[ic] & ams_observed_cell_mask(ic))==ams_observed_cell_mask(ic)); }
            } else rejected++;
            /* Temperature continues as the production task does; it must not
             * overwrite the voltage result used above. */
            if(smb_read_temp(&dev)!=0) temp_rejected++;
            adbms6830_session_end_scan(&dev);
            CHECK(last_cs==GPIO_PIN_SET && test_lock_depth==0);
            uint64_t elapsed=sim_us-begin;total+=elapsed;
            if(elapsed>maximum) maximum=elapsed;
            if(elapsed>100000) overrun++;
            if(elapsed<100000) sim_us=begin+100000;
            else sim_us+=1000; /* missed slot: yield rather than catch up */
        }
        if(scenario==0 || scenario==1) CHECK(accepted==200);
#ifdef SIL_CANDIDATE
        if(scenario==0 || scenario==1) {
            CHECK(temp_rejected==0);
            CHECK(overrun==0);
            CHECK(dev.session.warm_wake_count<=600);
            CHECK(dev.session.reference_probe_count<=2);
            CHECK(dev.session.reference_wait_count<=2);
            CHECK(dev.session.aux_short_wait_count==600);
        }
        if(scenario==2) { CHECK(accepted==200); CHECK(dev.session.coherent_restart_count==0); }
#endif
        if(scenario==6) CHECK(accepted==0);
        printf("%s,200,%u,%u,%u,%llu,%llu,%u,%u,%u,%u\n",names[scenario],accepted,rejected,temp_rejected,
            (unsigned long long)(total/200),(unsigned long long)maximum,overrun,transfers,
            dev.session.coherent_restart_count,dev.session.warm_wake_count);
        persistent_pec=false;pause_every=corrupt_every=timeout_every=0;rounded_wait=false;
    }
    return failures ? 1 : 0;
}
