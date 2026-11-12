#include "app.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static TIM_TypeDef timer8;
#undef TIM8
#define TIM8 (&timer8)
#undef __HAL_RCC_TIM8_CLK_ENABLE
#define __HAL_RCC_TIM8_CLK_ENABLE() ((void)0)
#define __DMB() ((void)0)
#include "Core/Src/ext_drivers/current_sensor.c"
static ADC_TypeDef high_regs,low_regs;
static ADC_HandleTypeDef hi,lo;
static current_sensor_t dev;
static uint32_t tick,poll_ms[2],elapsed;
static unsigned calls,starts,stops,polls,fail_config,fail_start,fail_poll,fail_stop;
uint32_t HAL_GetTick(void){return tick;}
HAL_StatusTypeDef stm32f767z_adc_switch_channel(ADC_HandleTypeDef *h,uint32_t ch) {
    calls++;return calls==fail_config ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h) {
    starts++;assert(polls==0);return starts==fail_start ? HAL_ERROR : HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *h,uint32_t ms) {
#if AMS_CURRENT_TIMER_TRIGGER
    assert(timer8.EGR==TIM_EGR_UG);
    assert((hi.Instance->CR2 & ADC_CR2_EXTSEL)==ADC_EXTERNALTRIGCONV_T8_TRGO);
#endif
    assert(starts==2);assert(ms<=5);poll_ms[polls]=ms;polls++;
    if(polls==1)tick+=elapsed;
    return polls==fail_poll ? HAL_TIMEOUT : HAL_OK;
}
uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h){return h==&hi ? 2001 : 2002;}
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h) {
    stops++;return stops==fail_stop ? HAL_ERROR : HAL_OK;
}
static void reset(void){
    memset(&timer8,0,sizeof(timer8));
    memset(&dev,0,sizeof(dev));memset(&hi,0,sizeof(hi));memset(&lo,0,sizeof(lo));
    hi.Instance=&high_regs;lo.Instance=&low_regs;dev.hadc_high=&hi;dev.hadc_low=&lo;
    tick=calls=starts=stops=polls=fail_config=fail_start=fail_poll=fail_stop=elapsed=0;
    dev.count_high=77;dev.count_low=88;dev.current_valid=true;
}
static void failed(void){
    assert(!current_sensor_read_adc(&dev));assert(!dev.last_read_ok && !dev.current_valid);
    assert(!dev.count_high_fresh && !dev.count_low_fresh);
    assert(dev.count_high==77 && dev.count_low==88);
}
int main(void){
    reset();elapsed=2;assert(current_sensor_read_adc(&dev));
    assert(starts==2 && stops==2 && poll_ms[0]==5 && poll_ms[1]==3);
    assert(dev.last_read_ok && dev.count_high_fresh && dev.count_low_fresh);
    assert(dev.count_high==2001 && dev.count_low==2002);
    reset();fail_config=2;failed();assert(!starts && !stops);
    reset();fail_start=1;failed();assert(starts==1 && stops==1);
    reset();fail_start=2;failed();assert(starts==2 && stops==2);
    reset();fail_poll=1;failed();assert(stops==2 && polls==1);
    reset();fail_poll=2;failed();assert(stops==2 && polls==2);
    reset();elapsed=5;failed();assert(stops==2 && polls==1);
    reset();fail_stop=1;failed();assert(stops==2);
    reset();dev.hadc_low=dev.hadc_high;failed();assert(!starts);
    reset();tick=UINT32_MAX-1;elapsed=2;assert(current_sensor_read_adc(&dev));assert(poll_ms[1]==3);
#if AMS_CURRENT_TIMER_TRIGGER
    reset();timer8.CR1=TIM_CR1_CEN;failed();assert(starts==0 && timer8.CR1==TIM_CR1_CEN);
    reset();hi.Instance->CR2=ADC_CR2_EXTSEL_0;elapsed=1;
    assert(current_sensor_read_adc(&dev));
    assert((hi.Instance->CR2 & (ADC_CR2_EXTSEL|ADC_CR2_EXTEN))==ADC_CR2_EXTSEL_0 && timer8.CR2==0);
#endif
    puts("PASS actual current ADC path: both starts before polling, shared timeout/wrap, cleanup and atomic pair rejection");
    return 0;
}
