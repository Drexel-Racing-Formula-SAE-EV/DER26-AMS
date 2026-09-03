#include "fuse_reference_oracle.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

typedef struct { long double current_a; long double time_s; } curve_point_t;
static const curve_point_t curve[] = {
    {154.0L,100.0L},{168.0L,50.0L},{180.9L,30.0L},{192.8L,20.0L},
    {219.2L,10.0L},{249.2L,5.0L},{274.9L,3.0L},{300.8L,2.0L},
    {350.9L,1.0L},{411.1L,0.5L},{458.9L,0.3L},{500.2L,0.2L},
    {576.6L,0.1L},{652.8L,0.05L},{705.8L,0.03L},{752.5L,0.02L},
    {800.0L,0.01253125L},{850.0L,0.01078L}
};
#define CURVE_COUNT ((uint32_t)(sizeof(curve)/sizeof(curve[0])))

static long double clamp_ld(long double x,long double lo,long double hi)
{ return x<lo?lo:(x>hi?hi:x); }
static long double lerp(long double x,long double x0,long double y0,long double x1,long double y1)
{ if(x1<=x0)return y0; long double f=clamp_ld((x-x0)/(x1-x0),0,1); return y0+f*(y1-y0); }
static long double loglerp(long double x,long double x0,long double y0,long double x1,long double y1)
{
    if(x<=0||x0<=0||x1<=x0||y0<=0||y1<=0)return y0;
    long double f=clamp_ld((logl(x)-logl(x0))/(logl(x1)-logl(x0)),0,1);
    return expl(logl(y0)+f*(logl(y1)-logl(y0)));
}

bool fuse_ref_config_valid(const fuse_ref_config_t *c)
{
    if(c==NULL)return false;
    if(!isfinite(c->rated_current_a)||fabsl(c->rated_current_a-80.0L)>1e-9L)return false;
    if(!isfinite(c->curve_time_fraction)||c->curve_time_fraction<=0||c->curve_time_fraction>1)return false;
    if(!isfinite(c->cooling_time_constant_s)||c->cooling_time_constant_s<=0)return false;
    if(!isfinite(c->initialization_soak_s)||c->initialization_soak_s<0)return false;
    if(!isfinite(c->quiescent_current_a)||c->quiescent_current_a<0)return false;
    if(!isfinite(c->fuse_temperature_margin_c)||c->fuse_temperature_margin_c<0)return false;
    if(!isfinite(c->minimum_temperature_derating)||c->minimum_temperature_derating<=0||c->minimum_temperature_derating>1)return false;
    if(!isfinite(c->maximum_state_multiple)||c->maximum_state_multiple<1)return false;
    if(!isfinite(c->low_current_fit_scale_s)||c->low_current_fit_scale_s<=0)return false;
    if(!isfinite(c->low_current_fit_exponent)||c->low_current_fit_exponent<=0)return false;
    if(!isfinite(c->maximum_curve_time_s)||c->maximum_curve_time_s<=0)return false;
    if(!isfinite(c->minimum_curve_time_s)||c->minimum_curve_time_s<=0||c->minimum_curve_time_s>=c->maximum_curve_time_s)return false;
    for(uint32_t h=0;h<FUSE_REF_HORIZON_COUNT;++h)
        if(!isfinite(c->horizons_s[h])||c->horizons_s[h]<=0||!isfinite(c->discharge_static_cap_a[h])||c->discharge_static_cap_a[h]<0)return false;
    return true;
}
void fuse_ref_state_init(fuse_ref_state_t *s){if(s!=NULL)memset(s,0,sizeof(*s));}
bool fuse_ref_state_seed_utilization(fuse_ref_state_t *s,const fuse_ref_config_t *c,long double u)
{
    if(s==NULL||!fuse_ref_config_valid(c)||!isfinite(u)||u<0||u>c->maximum_state_multiple)return false;
    s->thermal_utilization=u;s->quiescent_time_s=c->initialization_soak_s;s->thermal_state_initialized=1u;s->budget_exhausted=(u>=1)?1u:0u;return true;
}
long double fuse_ref_temperature_derating(long double temp,long double min_d)
{
    static const long double tc[]={-40,0,25,40,60,80,100,125};
    static const long double f[]={1.15L,1.06L,1.00L,0.97L,0.93L,0.89L,0.85L,0.80L};
    if(!isfinite(temp)||!isfinite(min_d)||min_d<=0||min_d>1)return 0;
    long double d=f[7];
    if(temp<=tc[0])d=f[0]; else for(uint32_t i=1;i<8;++i){if(temp<=tc[i]){d=lerp(temp,tc[i-1],f[i-1],tc[i],f[i]);break;}}
    return clamp_ld(d,min_d,1.0L);
}
long double fuse_ref_typical_melt_time_s(const fuse_ref_config_t *c,long double I,uint8_t *ex)
{
    if(ex!=NULL)*ex=0u;
    if(!fuse_ref_config_valid(c)||!isfinite(I)||I<0)return NAN;
    if(I<=c->rated_current_a)return INFINITY;
    if(I<curve[0].current_a){if(ex!=NULL)*ex=1u; long double over=I/c->rated_current_a-1; if(over<=0)return INFINITY; return clamp_ld(c->low_current_fit_scale_s*powl(over,-c->low_current_fit_exponent),c->minimum_curve_time_s,c->maximum_curve_time_s);}
    for(uint32_t i=1;i<CURVE_COUNT;++i)if(I<=curve[i].current_a)return loglerp(I,curve[i-1].current_a,curve[i-1].time_s,curve[i].current_a,curve[i].time_s);
    if(ex!=NULL)*ex=1u;
    const curve_point_t *a=&curve[CURVE_COUNT-2],*b=&curve[CURVE_COUNT-1];
    long double slope=(logl(b->time_s)-logl(a->time_s))/(logl(b->current_a)-logl(a->current_a));
    return clamp_ld(expl(logl(b->time_s)+slope*(logl(I)-logl(b->current_a))),c->minimum_curve_time_s,c->maximum_curve_time_s);
}
static long double source_rate(const fuse_ref_config_t *c,long double I,uint8_t *ex,long double *typ,long double *usable)
{
    long double t=fuse_ref_typical_melt_time_s(c,I,ex);if(typ)*typ=t;
    if(!isfinite(t)){if(usable)*usable=INFINITY;return 0;}
    long double u=clamp_ld(t*c->curve_time_fraction,c->minimum_curve_time_s,c->maximum_curve_time_s);if(usable)*usable=u;
    long double kernel=-c->cooling_time_constant_s*expm1l(-u/c->cooling_time_constant_s);
    if(!isfinite(kernel)||kernel<=LDBL_MIN)return 1/c->minimum_curve_time_s;
    return 1/kernel;
}
static long double predict_exact(const fuse_ref_state_t *s,const fuse_ref_config_t *c,long double candidate,long double uncertainty,long double derating,long double horizon,uint8_t *ex)
{
    long double eff=fmaxl(0,candidate)+uncertainty;long double eq=eff/derating;long double q=source_rate(c,eq,ex,NULL,NULL);long double d=expl(-horizon/c->cooling_time_constant_s);return s->thermal_utilization*d+q*c->cooling_time_constant_s*(1-d);
}
static long double solve_cap(const fuse_ref_state_t *s,
                             const fuse_ref_config_t *c,
                             long double cap,
                             long double uncertainty,
                             long double derating,
                             long double h,
                             uint8_t *ex)
{
    if((s->budget_exhausted != 0u) || (cap <= 0.0L))
    {
        return 0.0L;
    }
    uint8_t e = 0u;
    const long double at = predict_exact(s,c,cap,uncertainty,derating,h,&e);
    if((e != 0u) && (ex != NULL))
    {
        *ex = 1u;
    }
    if(at <= 1.0L)
    {
        return cap;
    }
    long double lo = 0.0L;
    long double hi = cap;
    for(uint32_t k = 0u; k < 64u; ++k)
    {
        const long double mid = (lo + hi) / 2.0L;
        e = 0u;
        const long double predicted = predict_exact(
            s,c,mid,uncertainty,derating,h,&e);
        if((e != 0u) && (ex != NULL))
        {
            *ex = 1u;
        }
        if(predicted <= 1.0L)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    return lo;
}
static void invalid_result(fuse_ref_result_t *r){if(r){memset(r,0,sizeof(*r));r->utilization=1;r->reason_flags=FUSE_REF_REASON_INPUT_INVALID;}}
bool fuse_ref_step_exact_zoh(fuse_ref_state_t *s,const fuse_ref_config_t *c,const fuse_ref_input_t *in,fuse_ref_result_t *r)
{
    if((s == NULL) || (r == NULL))
    {
        return false;
    }
    invalid_result(r);
    const bool valid=fuse_ref_config_valid(c)&&in!=NULL&&isfinite(in->pack_current_a)&&isfinite(in->current_uncertainty_a)&&in->current_uncertainty_a>=0&&isfinite(in->temperature_proxy_c)&&isfinite(in->elapsed_s)&&in->elapsed_s>0&&in->measurement_valid&&in->current_calibrated&&in->current_polarity_validated;
    if(!valid)
    {
        if(s->invalid_count != UINT64_MAX)
        {
            ++s->invalid_count;
        }
        return false;
    }
    if(s->update_count != UINT64_MAX)
    {
        ++s->update_count;
    }
    r->reason_flags=FUSE_REF_REASON_NONE;
    r->effective_current_a=fabsl(in->pack_current_a)+in->current_uncertainty_a;
    if(!s->thermal_state_initialized){if(r->effective_current_a<=c->quiescent_current_a){s->quiescent_time_s+=in->elapsed_s;if(s->quiescent_time_s>=c->initialization_soak_s){s->thermal_state_initialized=1;s->thermal_utilization=0;s->budget_exhausted=0;}}else s->quiescent_time_s=0;}
    r->estimated_fuse_temperature_c=in->temperature_proxy_c;if(!in->temperature_measured_at_fuse){r->estimated_fuse_temperature_c+=c->fuse_temperature_margin_c;r->reason_flags|=FUSE_REF_REASON_TEMPERATURE_PROXY;}
    r->temperature_derating=fuse_ref_temperature_derating(r->estimated_fuse_temperature_c,c->minimum_temperature_derating);r->continuous_current_a=c->rated_current_a*r->temperature_derating;r->equivalent_25c_current_a=r->effective_current_a/r->temperature_derating;
    uint8_t ex=0;long double q=source_rate(c,r->equivalent_25c_current_a,&ex,&r->typical_melt_time_s,&r->usable_melt_time_s);if(ex){r->curve_extrapolated=1;r->reason_flags|=FUSE_REF_REASON_CURVE_EXTRAPOLATED;}
    long double d=expl(-in->elapsed_s/c->cooling_time_constant_s);s->thermal_utilization=s->thermal_utilization*d+q*c->cooling_time_constant_s*(1-d);s->thermal_utilization=clamp_ld(s->thermal_utilization,0,c->maximum_state_multiple);
    r->utilization=s->thermal_utilization;r->remaining_utilization=fmaxl(0,1-s->thermal_utilization);if(r->utilization>=1)s->budget_exhausted=1;else if(r->utilization<=0.5L)s->budget_exhausted=0;
    for(uint32_t h=0;h<FUSE_REF_HORIZON_COUNT;++h){uint8_t ce=0;r->discharge_current_cap_a[h]=solve_cap(s,c,c->discharge_static_cap_a[h],in->current_uncertainty_a,r->temperature_derating,c->horizons_s[h],&ce);if(ce){r->curve_extrapolated=1;r->reason_flags|=FUSE_REF_REASON_CURVE_EXTRAPOLATED;}if(r->discharge_current_cap_a[h]+1e-9L<c->discharge_static_cap_a[h])r->reason_flags|=FUSE_REF_REASON_CURVE_DERATED;}
    r->valid=1;r->budget_exhausted=s->budget_exhausted;if(!in->model_validated)r->reason_flags|=FUSE_REF_REASON_MODEL_UNVALIDATED;if(!s->thermal_state_initialized)r->reason_flags|=FUSE_REF_REASON_INITIAL_STATE_UNKNOWN;if(s->budget_exhausted)r->reason_flags|=FUSE_REF_REASON_BUDGET_EXHAUSTED;r->authority_valid=(in->model_validated&&s->thermal_state_initialized)?1u:0u;return true;
}
long double fuse_ref_integrate_trapezoidal(long double x0,long double q,long double dt,long double tau,uint32_t n)
{
    if(!isfinite(x0)||x0<0||!isfinite(q)||q<0||!isfinite(dt)||dt<0||!isfinite(tau)||tau<=0||n==0)return NAN;
    long double h=dt/(long double)n,x=x0;for(uint32_t k=0;k<n;++k){long double f0=-x/tau+q;long double pred=x+h*f0;long double f1=-pred/tau+q;x+=0.5L*h*(f0+f1);}return x;
}
