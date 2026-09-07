/* DER26 AMS MiL production State-of-Power runner.
 * Links the checked-in ams_sop.c and estimator LUTs directly on the host.
 * It executes the pure finite-horizon production solve; publication slew and
 * mission/fuse supervisory overlays are qualified separately.
 */
#include "sop/ams_sop.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SEGMENTS AMS_SOP_SEGMENTS
#define CELLS AMS_SOP_CELLS_PER_SEGMENT
#define COMMON_FIELDS 15u
#define SEGMENT_SCALAR_FIELDS 19u
#define INPUT_FIELDS (COMMON_FIELDS + SEGMENTS * (SEGMENT_SCALAR_FIELDS + CELLS))
#define LINE_MAX_LEN 32768u

static bool parse_double(const char *text,double *value)
{
    if((text==NULL)||(value==NULL))return false;
    errno=0;char *end=NULL;double v=strtod(text,&end);
    if((end==text)||(errno==ERANGE))return false;
    while((*end==' ')||(*end=='\t')||(*end=='\r')||(*end=='\n'))end++;
    if(*end!='\0') return false;
    *value=v;
    return true;
}
static bool parse_u32(const char *text,uint32_t *value)
{
    double v=0.0;if(!parse_double(text,&v)||!isfinite(v)||(v<0.0)||(v>4294967295.0))return false;
    *value=(uint32_t)llround(v);return true;
}
static size_t split_csv(char *line,char **fields,size_t capacity)
{
    size_t count=0u;char *save=NULL;
    for(char *tok=strtok_r(line,",",&save);tok!=NULL&&count<capacity;tok=strtok_r(NULL,",",&save))fields[count++]=tok;
    return count;
}
static uint16_t parse_mask(double v)
{
    if(!isfinite(v)||v<0.0) return 0u;
    if(v>65535.0) v=65535.0;
    return (uint16_t)llround(v);
}
static uint8_t parse_u8(double v)
{
    if(!isfinite(v)||v<=0.0) return 0u;
    if(v>255.0) v=255.0;
    return (uint8_t)llround(v);
}
static void header(FILE *out)
{
    fprintf(out,"time_s,sequence,status,valid,authority_valid,fallback_active,reason_flags,feasibility_evaluations,prediction_steps");
    for(unsigned h=0;h<AMS_SOP_HORIZONS;h++)
    {
        fprintf(out,",h%u_s,h%u_model_discharge_A,h%u_model_charge_A,h%u_discharge_A,h%u_charge_A,h%u_discharge_W,h%u_charge_W"
                    ",h%u_d_bind,h%u_c_bind,h%u_d_seg,h%u_d_cell,h%u_c_seg,h%u_c_cell"
                    ",h%u_d_minV,h%u_d_maxV,h%u_d_minSoC,h%u_d_maxSoC,h%u_d_maxCoreC,h%u_d_maxSurfC"
                    ",h%u_c_minV,h%u_c_maxV,h%u_c_minSoC,h%u_c_maxSoC,h%u_c_maxCoreC,h%u_c_maxSurfC",
                    h,h,h,h,h,h,h,h,h,h,h,h,h,h,h,h,h,h,h,h,h,h,h,h,h);
    }
    fputc('\n',out);
}
static int usage(const char *p)
{
    fprintf(stderr,"usage: %s INPUT.csv OUTPUT.csv\n",p);return 2;
}
int main(int argc,char **argv)
{
    if(argc!=3)return usage(argv[0]);
    FILE *in=fopen(argv[1],"r");if(!in){perror("open input");return 2;}
    FILE *out=fopen(argv[2],"w");if(!out){perror("open output");fclose(in);return 2;}
    char line[LINE_MAX_LEN];if(!fgets(line,sizeof(line),in)){fprintf(stderr,"empty input\n");fclose(in);fclose(out);return 2;}
    header(out);unsigned long row=1;
    ams_sop_config_t cfg;ams_sop_default_config(&cfg);
    while(fgets(line,sizeof(line),in))
    {
        row++;if(line[0]=='\n'||line[0]=='\r'||line[0]=='\0')continue;
        char *f[INPUT_FIELDS+4u];size_t n=split_csv(line,f,INPUT_FIELDS+4u);
        if(n!=INPUT_FIELDS){fprintf(stderr,"row %lu expected %u fields got %zu\n",row,(unsigned)INPUT_FIELDS,n);fclose(in);fclose(out);return 3;}
        size_t k=0;double d=0.0;uint32_t sequence=0,timestamp_ms=0,now_ms=0;double time_s=0;
        ams_sop_input_t input;memset(&input,0,sizeof(input));
        if(!parse_double(f[k++],&time_s)||!parse_u32(f[k++],&sequence)||!parse_u32(f[k++],&timestamp_ms)||!parse_u32(f[k++],&now_ms))goto bad;
        input.measurement_sequence=sequence;input.measurement_timestamp_ms=timestamp_ms;input.now_ms=now_ms;
        if(!parse_double(f[k++],&d)) goto bad;
        input.pack_current_a=(float)d;
        if(!parse_double(f[k++],&d)) goto bad;
        input.pack_current_uncertainty_a=(float)d;
        if(!parse_double(f[k++],&d)) goto bad;
        input.ambient_temp_c=(float)d;
#define READ_BOOL(member) do{if(!parse_double(f[k++],&d))goto bad;input.member=(d!=0.0)?1u:0u;}while(0)
        READ_BOOL(measurement_valid);READ_BOOL(estimator_valid);READ_BOOL(estimator_acquired);READ_BOOL(estimator_segment_topology);
        READ_BOOL(current_calibrated);READ_BOOL(current_polarity_validated);READ_BOOL(ambient_measured);READ_BOOL(balance_recovered);
#undef READ_BOOL
        input.operating_mode=AMS_SOP_MODE_DRIVE;input.discharge_authorized=1u;input.charger_authorized=1u;input.regen_authorized=1u;
        for(unsigned s=0;s<SEGMENTS;s++)
        {
            ams_sop_segment_input_t *seg=&input.segment[s];
#define READ_FLOAT(member) do{if(!parse_double(f[k++],&d))goto bad;seg->member=(float)d;}while(0)
            READ_FLOAT(soc);READ_FLOAT(vp1_v);READ_FLOAT(vp2_v);READ_FLOAT(r0_ohm);READ_FLOAT(core_temp_c);READ_FLOAT(surface_max_temp_c);
            READ_FLOAT(p_soc);READ_FLOAT(p_vp1);READ_FLOAT(p_vp2);READ_FLOAT(p_r0);READ_FLOAT(innovation_v);
            READ_FLOAT(capacity_soh_lower);READ_FLOAT(resistance_soh_upper);
            if(!parse_double(f[k++],&d)) goto bad;
            seg->max_cell_age_ms=(uint32_t)((d<0.0)?0.0:d);
            if(!parse_double(f[k++],&d)) goto bad;
            seg->cell_usable_mask=parse_mask(d);
            if(!parse_double(f[k++],&d)) goto bad;
            seg->estimator_valid=(d!=0.0)?1u:0u;
            if(!parse_double(f[k++],&d)) goto bad;
            seg->model_domain_flags=parse_u8(d);
            if(!parse_double(f[k++],&d)) goto bad;
            seg->capacity_soh_valid=(d!=0.0)?1u:0u;
            if(!parse_double(f[k++],&d)) goto bad;
            seg->resistance_soh_valid=(d!=0.0)?1u:0u;
#undef READ_FLOAT
            for(unsigned c=0;c<CELLS;c++){if(!parse_double(f[k++],&d))goto bad;seg->cell_voltage_v[c]=(float)d;}
        }
        if(k!=INPUT_FIELDS)goto bad;
        ams_sop_result_t r;ams_sop_status_t st=ams_sop_solve(&input,&cfg,&r);
        fprintf(out,"%.9g,%u,%u,%u,%u,%u,%u,%u,%u",time_s,sequence,(unsigned)st,(unsigned)r.valid,(unsigned)r.authority_valid,(unsigned)r.fallback_active,(unsigned)r.reason_flags,(unsigned)r.feasibility_evaluations,(unsigned)r.prediction_steps);
        for(unsigned h=0;h<AMS_SOP_HORIZONS;h++)
        {
            const ams_sop_prediction_extrema_t *de=&r.discharge_extrema[h];const ams_sop_prediction_extrema_t *ce=&r.charge_extrema[h];
            fprintf(out,",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u,%u,%u,%u,%u"
                        ",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g",
                    cfg.horizons_s[h],r.model_discharge_current_a[h],r.model_charge_current_a[h],r.discharge_current_a[h],r.charge_current_a[h],r.discharge_power_w[h],r.charge_power_w[h],
                    (unsigned)r.discharge_binding[h],(unsigned)r.charge_binding[h],(unsigned)r.discharge_limiting_segment[h],(unsigned)r.discharge_limiting_cell[h],(unsigned)r.charge_limiting_segment[h],(unsigned)r.charge_limiting_cell[h],
                    de->minimum_cell_voltage_v,de->maximum_cell_voltage_v,de->minimum_soc,de->maximum_soc,de->maximum_core_temp_c,de->maximum_surface_temp_c,
                    ce->minimum_cell_voltage_v,ce->maximum_cell_voltage_v,ce->minimum_soc,ce->maximum_soc,ce->maximum_core_temp_c,ce->maximum_surface_temp_c);
        }
        fputc('\n',out);continue;
bad:
        fprintf(stderr,"row %lu invalid field near index %zu\n",row,k);fclose(in);fclose(out);return 3;
    }
    if(ferror(in)||ferror(out)){fprintf(stderr,"I/O error\n");fclose(in);fclose(out);return 4;}
    fclose(in);if(fclose(out)!=0){perror("close output");return 4;}return 0;
}
