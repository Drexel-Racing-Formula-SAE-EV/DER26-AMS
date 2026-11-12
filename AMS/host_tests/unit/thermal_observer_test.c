#include "estimator/ams_thermal_observer.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static ams_thermal_config_t c;
static ams_thermal_input_t in;
static ams_thermal_observer_t o;
static void sample(uint32_t t,float surface)
{
    in.now_ms=t; in.source_ms=t;
    for(unsigned s=0;s<24;s++) { in.surface_c[s]=surface; in.surface_tick[s]=t; }
}
static void setup(uint32_t t)
{
    ams_thermal_default_config(&c); ams_thermal_init(&o);
    memset(&in,0,sizeof(in));
    in.heat_valid=true; in.current_valid=true; in.current_calibrated=true;
    in.boundary_valid=true; in.boundary_assumed=true; in.boundary_c=25;
    in.surface_mask=0xffffffu; sample(t,25);
    ams_thermal_step(&o,&c,&in); assert(o.available);
}
int main(void)
{
    setup(100);
    assert((o.flags&(AMS_THERMAL_UNCALIBRATED|AMS_THERMAL_BOUNDARY_ASSUMED))==3u);
    float p=o.p11;
    ams_thermal_step(&o,&c,&in);
    assert(o.p11==p && o.corrections==0 && o.duplicate_count==1 && o.available);
    /* Republishing cached thermistors must not reduce covariance. */
    in.now_ms=200; in.source_ms=200;
    ams_thermal_step(&o,&c,&in);
    assert(o.predictions==1 && o.corrections==0);
    assert(fabsf(o.core_c-25)<0.001f && fabsf(o.surface_c-25)<0.001f);
    sample(300,25); ams_thermal_step(&o,&c,&in); assert(o.corrections==1);
    /* Unknown current never becomes zero heat. */
    in.heat_valid=false; sample(400,25); float core=o.core_c;
    ams_thermal_step(&o,&c,&in);
    assert(!o.available && (o.flags&AMS_THERMAL_MODEL_INVALID) && o.core_c==core);
    in.heat_valid=true; sample(500,25); ams_thermal_step(&o,&c,&in); assert(o.available);
    /* A stopped publisher cannot be made fresh by a later consumer tick. */
    in.now_ms=2000; ams_thermal_step(&o,&c,&in); assert(!o.available);
    setup(100); in.boundary_valid=false; sample(200,25);
    ams_thermal_step(&o,&c,&in); assert(!o.available && (o.flags&AMS_THERMAL_BOUNDARY_INVALID));
    setup(100); in.surface_mask=0x7ff; sample(200,25);
    ams_thermal_step(&o,&c,&in); assert(!o.available && o.usable_sensors==11);
    setup(100); in.heat_w=NAN; sample(200,25);
    ams_thermal_step(&o,&c,&in); assert(!o.available);
    setup(100); c.rcs=0; ams_thermal_step(&o,&c,&in);
    assert(!o.available && (o.flags&AMS_THERMAL_MODEL_INVALID));
    /* Timestamp rollover and genuine per-channel newness. */
    setup(UINT32_MAX-50u); sample(49u,25); ams_thermal_step(&o,&c,&in);
    assert(o.available && o.predictions==1 && o.corrections==1);
    setup(100); sample(150,25); in.surface_tick[0]=99;
    ams_thermal_step(&o,&c,&in); assert(o.surface_tick[0]==100);
    /* Outlier consumption: cannot retry one rejected sample as new evidence. */
    setup(100); sample(200,100); ams_thermal_step(&o,&c,&in);
    assert(o.rejected==1 && (o.flags&AMS_THERMAL_INNOVATION_REJECT));
    in.source_ms=300; in.now_ms=300; ams_thermal_step(&o,&c,&in);
    assert(o.rejected==1 && o.corrections==0);
    /* Steady-state energy balance: Ts=Ta+q*Rsa, Tc=Ts+q*Rcs. */
    setup(100); o.surface_c=33; o.core_c=34.5f; in.heat_w=1;
    for(unsigned k=2;k<1000;k++) {
        sample(k*100u,33); ams_thermal_step(&o,&c,&in);
        assert(o.available && fabsf(o.core_c-34.5f)<0.005f);
        assert(o.p00*o.p11-o.p01*o.p01>=-0.001f);
    }
    /* Independent forward-Euler plant at 1 ms; observer 100 ms, fresh
     * thermistors every 800 ms. Heating/rest pulses and cooling change. */
    setup(100); double tc=25,ts=25; double max_error=0;
    for(unsigned k=2;k<10000;k++) {
        in.heat_w=(k<3500)?2.0f:0.0f;
        in.boundary_c=(k<6500)?25.0f:20.0f;
        for(unsigned j=0;j<100;j++) {
            double flow=(tc-ts)/1.5;
            double dc=(in.heat_w-flow)/55.0;
            double ds=(flow-(ts-in.boundary_c)/8.0)/15.0;
            tc+=0.001*dc; ts+=0.001*ds;
        }
        in.now_ms=k*100u; in.source_ms=in.now_ms;
        if(k%8==0) sample(k*100u,(float)ts);
        ams_thermal_step(&o,&c,&in); assert(o.available);
        double error=fabs(o.core_c-tc); if(error>max_error) max_error=error;
        assert(isfinite(o.core_upper_c) && o.core_upper_c>=o.core_c);
        assert(o.p00*o.p11-o.p01*o.p01>=-0.001f);
    }
    assert(max_error<0.1);
    printf("PASS thermal observer: faults, freshness, duplicates, rollover, Joseph PSD, equilibrium, independent plant max error %.5f C\n",max_error);
    return 0;
}
