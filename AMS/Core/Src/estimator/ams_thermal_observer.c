#include "estimator/ams_thermal_observer.h"
#include <math.h>
#include <stddef.h>
#include <string.h>
static bool positive(float v) { return isfinite(v) && v > 0.0f; }
static bool temperature(float v) { return isfinite(v) && v >= -40.0f && v <= 120.0f; }
static void inc(uint32_t *v) { if(*v != UINT32_MAX) ++*v; }
void ams_thermal_default_config(ams_thermal_config_t *c)
{
    if(!c) return;
    memset(c, 0, sizeof(*c));
    c->cc=55.0f; c->cs=15.0f; c->rcs=1.5f; c->rsa=8.0f;
    c->process_variance_rate=0.04f; c->measurement_variance=2.25f;
    c->model_margin_c=5.0f; c->innovation_gate_sigma=5.0f;
    c->max_step_ms=1000u; c->max_surface_age_ms=1000u;
    c->sensor_mask=0x00ffffffu; c->minimum_sensors=12u;
    /* These are provisional SoP model values, not installed-pack calibration. */
    c->calibrated=false;
}
void ams_thermal_init(ams_thermal_observer_t *o)
{
    if(o) { memset(o,0,sizeof(*o)); o->flags=AMS_THERMAL_UNCALIBRATED; }
}
static bool config_ok(const ams_thermal_config_t *c)
{
    return c && positive(c->cc) && positive(c->cs) && positive(c->rcs) &&
        positive(c->rsa) && positive(c->process_variance_rate) &&
        positive(c->measurement_variance) && positive(c->innovation_gate_sigma) &&
        isfinite(c->model_margin_c) && c->model_margin_c>=0.0f &&
        c->max_step_ms>0u && c->max_step_ms<=10000u &&
        c->max_surface_age_ms>0u && c->max_surface_age_ms<=60000u &&
        c->minimum_sensors>0u && c->minimum_sensors<=AMS_THERMAL_SENSORS &&
        (c->sensor_mask & ~0x00ffffffu)==0u;
}
void ams_thermal_step(ams_thermal_observer_t *o,
                      const ams_thermal_config_t *c,
                      const ams_thermal_input_t *in)
{
    if(!o || !in) return;
    o->available=false;
    o->flags=AMS_THERMAL_UNCALIBRATED;
    if(!config_ok(c)) { o->flags|=AMS_THERMAL_MODEL_INVALID; return; }
    o->flags=c->calibrated ? 0u : AMS_THERMAL_UNCALIBRATED;
    if(in->boundary_assumed) o->flags|=AMS_THERMAL_BOUNDARY_ASSUMED;
    if(!in->current_calibrated) o->flags|=AMS_THERMAL_CURRENT_UNCALIBRATED;
    if(!in->current_valid) o->flags|=AMS_THERMAL_CURRENT_INVALID;
    if(!in->heat_valid || !isfinite(in->heat_w) || in->heat_w<0.0f || in->heat_w>100.0f)
        o->flags|=AMS_THERMAL_MODEL_INVALID;
    if(!in->boundary_valid || !temperature(in->boundary_c))
        o->flags|=AMS_THERMAL_BOUNDARY_INVALID;
    if((uint32_t)(in->now_ms-in->source_ms)>c->max_step_ms)
        o->flags|=AMS_THERMAL_TIMING_INVALID;

    float all_sum=0.0f, fresh_sum=0.0f, fresh_sq=0.0f;
    uint32_t new_mask=0u;
    unsigned all_n=0u, fresh_n=0u;
    for(unsigned s=0u;s<AMS_THERMAL_SENSORS;s++) {
        uint32_t bit=1u<<s;
        if(!(in->surface_mask & c->sensor_mask & bit) ||
           (uint32_t)(in->now_ms-in->surface_tick[s])>c->max_surface_age_ms ||
           !temperature(in->surface_c[s])) continue;
        ++all_n; all_sum+=in->surface_c[s];
        if(!(o->seen_mask&bit) || (int32_t)(in->surface_tick[s]-o->surface_tick[s])>0) {
            ++fresh_n; fresh_sum+=in->surface_c[s];
            fresh_sq+=in->surface_c[s]*in->surface_c[s]; new_mask|=bit;
        }
    }
    o->usable_sensors=(uint8_t)all_n;
    if(all_n<c->minimum_sensors) o->flags|=AMS_THERMAL_SURFACE_INVALID;
    const uint32_t fatal=AMS_THERMAL_CURRENT_INVALID|AMS_THERMAL_SURFACE_INVALID|
        AMS_THERMAL_BOUNDARY_INVALID|AMS_THERMAL_TIMING_INVALID|AMS_THERMAL_MODEL_INVALID;
    if(o->flags&fatal) {
        /* No invented zero heat or last-good extrapolation through unknown time. */
        o->initialized=false;
        return;
    }
    if(!o->initialized) {
        /* Only seed on a genuinely new observation after a gap. */
        if(fresh_n==0u) { o->flags|=AMS_THERMAL_SURFACE_INVALID; return; }
        o->core_c=all_sum/(float)all_n; o->surface_c=o->core_c;
        o->p00=25.0f; o->p01=0.0f; o->p11=c->measurement_variance;
        o->tick_ms=in->source_ms; o->initialized=true;
        /* Seeding consumes every included observation, not just the newest mux. */
        for(unsigned s=0u;s<AMS_THERMAL_SENSORS;s++) {
            uint32_t bit=1u<<s;
            if((in->surface_mask&c->sensor_mask&bit) && temperature(in->surface_c[s]) &&
               (uint32_t)(in->now_ms-in->surface_tick[s])<=c->max_surface_age_ms) {
                o->surface_tick[s]=in->surface_tick[s]; o->seen_mask|=bit;
            }
        }
    } else {
        uint32_t elapsed=in->source_ms-o->tick_ms;
        if(elapsed==0u) { inc(&o->duplicate_count); o->available=true; return; }
        if(elapsed>c->max_step_ms) {
            o->flags|=AMS_THERMAL_TIMING_INVALID; o->initialized=false; return;
        }
        float dt=(float)elapsed*0.001f, h=dt*0.5f;
        float a=1.0f/(c->rcs*c->cc), b=1.0f/(c->rcs*c->cs), d=1.0f/(c->rsa*c->cs);
        float m00=1.0f+h*a, m01=-h*a, m10=-h*b, m11=1.0f+h*(b+d);
        float det=m00*m11-m01*m10;
        float n00=1.0f-h*a, n01=h*a, n10=h*b, n11=1.0f-h*(b+d);
        float f00=(m11*n00-m01*n10)/det, f01=(m11*n01-m01*n11)/det;
        float f10=(-m10*n00+m00*n10)/det, f11=(-m10*n01+m00*n11)/det;
        float u0=dt*in->heat_w/c->cc, u1=dt*d*in->boundary_c;
        float core=f00*o->core_c+f01*o->surface_c+(m11*u0-m01*u1)/det;
        float surface=f10*o->core_c+f11*o->surface_c+(-m10*u0+m00*u1)/det;
        float q=c->process_variance_rate*dt;
        float p00=f00*f00*o->p00+2.0f*f00*f01*o->p01+f01*f01*o->p11+q;
        float p01=f00*f10*o->p00+(f00*f11+f01*f10)*o->p01+f01*f11*o->p11;
        float p11=f10*f10*o->p00+2.0f*f10*f11*o->p01+f11*f11*o->p11+q;
        inc(&o->predictions);
        if(fresh_n>0u) {
            float mean=fresh_sum/(float)fresh_n;
            float spread=fmaxf(0.0f,fresh_sq/(float)fresh_n-mean*mean);
            /* Spatial/model errors are correlated: no R/N confidence collapse.
             * Age allowance is 1 C/s, provisional, not sensor accuracy. */
            float age_s=(float)c->max_surface_age_ms*0.001f;
            float r=c->measurement_variance+spread+age_s*age_s;
            float innovation=mean-surface, variance=p11+r;
            o->innovation_c=innovation;
            if(innovation*innovation<=c->innovation_gate_sigma*c->innovation_gate_sigma*variance) {
                float k0=p01/variance, k1=p11/variance;
                core+=k0*innovation; surface+=k1*innovation;
                /* Joseph covariance update with H=[0,1]. */
                float old01=p01, old11=p11;
                p00=p00-2.0f*k0*old01+k0*k0*(old11+r);
                p01=(1.0f-k1)*(old01-k0*old11)+k0*k1*r;
                p11=(1.0f-k1)*(1.0f-k1)*old11+k1*k1*r;
                inc(&o->corrections);
            } else { o->flags|=AMS_THERMAL_INNOVATION_REJECT; inc(&o->rejected); }
            /* Rejected readings are consumed too; never retry as independent evidence. */
            for(unsigned s=0u;s<AMS_THERMAL_SENSORS;s++) if(new_mask&(1u<<s)) {
                o->surface_tick[s]=in->surface_tick[s]; o->seen_mask|=1u<<s;
            }
        }
        if(!temperature(core) || !temperature(surface) || !positive(p00) ||
           !positive(p11) || !isfinite(p01) || p00>10000.0f || p11>10000.0f ||
           p00*p11-p01*p01 < -0.001f) {
            o->flags|=AMS_THERMAL_NUMERIC; o->initialized=false; return;
        }
        o->core_c=core; o->surface_c=surface;
        o->p00=p00; o->p01=p01; o->p11=p11; o->tick_ms=in->source_ms;
    }
    o->heat_w=in->heat_w;
    o->core_upper_c=o->core_c+3.0f*sqrtf(o->p00)+c->model_margin_c;
    o->available=true; /* availability is not calibration or authority */
}
