/* Production EKF numerical/topology checks and native CPU scaling probe.
 * Native times are not Cortex-M7 WCET. No HAL or estimator algorithm mock. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "Core/Src/estimator/ams_soc_ekf.c"

static unsigned failures;
#define CHECK(x) do { if(!(x)){fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);failures++;} } while(0)
static void check_process_noise(void)
{
    const float dt_values[]={0.01f,0.05f,0.1f,0.2f,1.0f};
    for(unsigned i=0;i<5;i++)
    {
        ams_ekf_config_t cfg;ams_ekf_make_segment_config(&cfg,0);
        cfg.soc_init=0.6f;
        ams_ekf_instance_t inst;ams_ekf_init(&inst,&cfg);
        inst.step_count=AMS_EKF_ACQUISITION_STEPS;
        float before=inst.p_soc,before_r0=inst.p_r0;
        float before_vp=inst.p_vp1;
        float dt=dt_values[i];
        float f=1.0f+ams_p42a_neg_inv_tau1(inst.soc,25.0f)*dt;
        CHECK(ams_ekf_step_gated(&inst,0.0f,cfg.series_group_count*2.1f,
              25.0f,dt,false,NULL));
        CHECK(inst.fault_flags & AMS_EKF_FAULT_INNOVATION_REJECT);
        CHECK(fabsf(inst.p_soc-(before+AMS_EKF_Q_SOC*dt/0.1f))<2.0e-9f);
        CHECK(fabsf(inst.p_r0-(before_r0+AMS_EKF_Q_R0*dt/0.1f))<2.0e-11f);
        CHECK(fabsf(inst.p_vp1-(f*f*before_vp+AMS_EKF_Q_VP1*dt/0.1f))<2.0e-9f);
        CHECK(inst.innov_idx==0u); /* rejected samples cannot adapt R */
    }
}
static void check_topology(void)
{
    ams_estimator_t est;ams_estimator_init_default(&est);
    CHECK(est.instance_count==5);
    CHECK(ams_estimator_configure_subsegments(&est));
    CHECK(est.instance_count==10);
    uint16_t seen[5]={0};unsigned total=0;
    for(unsigned i=0;i<10;i++)
    {
        ams_ekf_config_t *cfg=&est.inst[i].cfg;
        CHECK(cfg->first_series_group/15u==i/2u);
        CHECK((cfg->first_series_group+cfg->series_group_count-1u)/15u==i/2u);
        CHECK(cfg->series_group_count==7u || cfg->series_group_count==8u);
        for(unsigned k=0;k<cfg->series_group_count;k++)
        {
            unsigned group=cfg->first_series_group+k,seg=group/15u,bit=1u<<(group%15u);
            CHECK(!(seen[seg]&bit));seen[seg]|=bit;total++;
        }
    }
    CHECK(total==74u);
    for(unsigned seg=0;seg<5;seg++) CHECK(seen[seg]==ams_observed_cell_mask(seg));
    est.instance_count=AMS_EKF_MAX_INSTANCES+1u;
    ams_estimator_refresh_summary(&est,AMS_ESTIMATOR_INPUT_HARDWARE,0);
    CHECK(est.fault_flags==AMS_EKF_FAULT_BAD_CONFIG);
    est.instance_count=0u;
    ams_estimator_refresh_summary(&est,AMS_ESTIMATOR_INPUT_HARDWARE,0);
    CHECK(est.fault_flags==AMS_EKF_FAULT_BAD_CONFIG);
}
static void bench(unsigned count,bool acquisition)
{
    static ams_estimator_t est;
    if(count==5) CHECK(ams_estimator_configure_segments(&est));
    else CHECK(ams_estimator_configure_subsegments(&est));
    for(unsigned i=0;i<count;i++)
    {
        est.inst[i].soc=0.6f;est.inst[i].cfg.soc_init=0.6f;
    }
    const unsigned repeats=100000;
    clock_t start=clock();
    for(unsigned n=0;n<repeats;n++)
    {
        for(unsigned i=0;i<count;i++)
        {
            ams_ekf_instance_t *inst=&est.inst[i];
            float v=inst->cfg.series_group_count*ams_p42a_ocv_v(inst->soc,25.0f);
            bool ok=acquisition ? ams_ekf_step_acquiring_gated(inst,0,v,25,0.1f,NULL) :
                                  ams_ekf_step_gated(inst,0,v,25,0.1f,false,NULL);
            CHECK(ok && !(inst->fault_flags & AMS_EKF_FAULT_INNOVATION_REJECT));
        }
    }
    double sweep_us=(double)(clock()-start)*1.0e6/CLOCKS_PER_SEC/repeats;
    printf("NATIVE %u instances %s: %.3f us/sweep; %.3f us/instance\n",count,
           acquisition?"acquiring":"normal",sweep_us,sweep_us/count);
}
int main(void)
{
    check_process_noise();check_topology();
    printf("Native sizeof: instance=%zu estimator=%zu acquisition=%zu bytes\n",
           sizeof(ams_ekf_instance_t),sizeof(ams_estimator_t),sizeof(ams_ekf_acquisition_t));
    if(getenv("EKF_SKIP_BENCHMARK")==NULL)
    {
        bench(5,false);bench(10,false);bench(5,true);bench(10,true);
    }
    if(failures)return 1;
    puts("PASS elapsed-time Q, rejected-input isolation, 10-way cell coverage and count guards");
    return 0;
}
