#include <assert.h>
#include <stdint.h>
#include "ams_diag_budget.h"
int main(void) {
 bool forced; uint32_t ticks[24]={0};
 assert(ams_diag_admit(40,0,100,60,20,500,&forced) && !forced);
 assert(!ams_diag_admit(41,0,100,60,20,500,&forced) && !forced);
 assert(!ams_diag_admit(520,500,100,200,20,500,&forced) && forced);
 assert(ams_diag_admit(520,500,100,80,20,500,&forced) && forced);
 assert(ams_diag_admit(10,UINT32_MAX-19,100,70,0,500,&forced) && !forced);
 assert(ams_diag_overdue_mask(12000,0,ticks,0,12000)==0xffffff);
 ticks[3]=11999;
 assert(ams_diag_overdue_mask(12000,0,ticks,1UL<<3,12000)==0xfffff7);
 assert(ams_diag_overdue_mask(10,UINT32_MAX-19,ticks,0,31)==0);
 assert(ams_diag_overdue_mask(11,UINT32_MAX-19,ticks,0,31)==0xffffff);
 uint8_t retry=0, sensor=0;
 uint32_t visited=0;
 for(unsigned n=0;n<48;n++) {
     visited |= 1UL<<sensor;
     ams_diag_advance(false, &retry, &sensor);
 }
 assert(visited==0xffffff && retry==0 && sensor==0);
 ams_diag_advance(true, &retry, &sensor);
 assert(sensor==1 && retry==0);
 return 0;
}
