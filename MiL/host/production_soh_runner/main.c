/* DER26 AMS MiL production SoH runner: direct ams_soh.c host execution. */
#include "soh/ams_soh.h"
#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define SEGMENTS AMS_SOH_SEGMENTS
#define COMMON 19u
#define INPUT_FIELDS (COMMON + 3u*SEGMENTS)
#define LINE_MAX_LEN 8192u
static bool pd(const char *t,double *v){if(!t||!v)return false;errno=0;char *e=NULL;double x=strtod(t,&e);if(e==t||errno==ERANGE)return false;while(*e==' '||*e=='\t'||*e=='\r'||*e=='\n')e++;if(*e)return false;*v=x;return true;}
static bool pu32(const char *t,uint32_t *v){double x;if(!pd(t,&x)||!isfinite(x)||x<0||x>4294967295.0)return false;*v=(uint32_t)llround(x);return true;}
static size_t split(char *l,char **f,size_t cap){size_t n=0;char *save=NULL;for(char *t=strtok_r(l,",",&save);t&&n<cap;t=strtok_r(NULL,",",&save))f[n++]=t;return n;}
int main(int argc,char **argv){
 if(argc!=3){fprintf(stderr,"usage: %s INPUT.csv OUTPUT.csv\n",argv[0]);return 2;}
 FILE *in=fopen(argv[1],"r");if(!in){perror("input");return 2;}FILE *out=fopen(argv[2],"w");if(!out){perror("output");fclose(in);return 2;}
 char line[LINE_MAX_LEN];if(!fgets(line,sizeof(line),in)){fprintf(stderr,"empty input\n");fclose(in);fclose(out);return 2;}
 fprintf(out,"time_s,sequence,update_ok,reason_flags,rest_elapsed_s,anchor_valid,accepted_windows,rejected_windows,capacity_Ah,capacity_sigma_Ah,capacity_soh,capacity_soh_lower,capacity_confidence_pct,capacity_valid,resistance_growth,resistance_growth_upper,resistance_confidence_pct,resistance_valid,combined_soh\n");
 ams_soh_config_t cfg;ams_soh_default_config(&cfg);ams_soh_estimator_t est;ams_soh_init(&est,&cfg);unsigned long row=1;
 while(fgets(line,sizeof(line),in)){row++;if(line[0]=='\n'||line[0]=='\r'||line[0]=='\0')continue;char *f[INPUT_FIELDS+2];size_t n=split(line,f,INPUT_FIELDS+2);if(n!=INPUT_FIELDS){fprintf(stderr,"row %lu expected %u got %zu\n",row,(unsigned)INPUT_FIELDS,n);goto fail;}
  size_t k=0;double time=0,d=0;uint32_t seq=0,ts=0,now=0;ams_soh_input_t x;memset(&x,0,sizeof(x));
  if(!pd(f[k++],&time)||!pu32(f[k++],&seq)||!pu32(f[k++],&ts)||!pu32(f[k++],&now)) goto bad;
  x.measurement_sequence=seq;x.measurement_timestamp_ms=ts;x.now_ms=now;
#define RF(m) do{if(!pd(f[k++],&d))goto bad;x.m=(float)d;}while(0)
  RF(elapsed_s);RF(pack_current_a);RF(pack_current_uncertainty_a);if(!pd(f[k++],&d))goto bad;x.total_charge_as=d;RF(pack_soc);RF(average_cell_temp_c);RF(cell_voltage_spread_v);RF(maximum_soc_sigma);RF(maximum_abs_innovation_v_per_cell);RF(maximum_abs_polarization_v);
#undef RF
#define RB(m) do{if(!pd(f[k++],&d))goto bad;x.m=(d!=0.0)?1u:0u;}while(0)
  RB(measurement_valid);RB(estimator_valid);RB(current_calibrated);RB(current_polarity_validated);RB(balance_recovered);
#undef RB
  for(unsigned s=0;s<SEGMENTS;s++){if(!pd(f[k++],&d))goto bad;x.segment_resistance_growth_ratio[s]=(float)d;if(!pd(f[k++],&d))goto bad;x.segment_resistance_confidence_pct[s]=(uint8_t)fmin(fmax(d,0),100);if(!pd(f[k++],&d))goto bad;x.segment_resistance_valid[s]=(d!=0.0)?1u:0u;}
  if(k!=INPUT_FIELDS) goto bad;
  bool ok=ams_soh_update(&est,&cfg,&x);const ams_soh_result_t *r=&est.result;
  fprintf(out,"%.9g,%u,%u,%u,%.9g,%u,%u,%u,%.9g,%.9g,%.9g,%.9g,%u,%u,%.9g,%.9g,%u,%u,%.9g\n",time,seq,ok?1u:0u,(unsigned)r->last_reason_flags,est.rest_elapsed_s,(unsigned)est.anchor_valid,(unsigned)r->accepted_capacity_windows,(unsigned)r->rejected_capacity_windows,r->capacity_ah,r->capacity_sigma_ah,r->capacity_soh,r->capacity_soh_lower,(unsigned)r->capacity_confidence_pct,(unsigned)r->capacity_valid,r->resistance_growth_ratio,r->resistance_growth_upper,(unsigned)r->resistance_confidence_pct,(unsigned)r->resistance_valid,r->combined_soh);continue;
 bad: fprintf(stderr,"row %lu bad field near %zu\n",row,k);goto fail;
 }
 if(ferror(in)||ferror(out)) goto fail;
 fclose(in);
 if(fclose(out)!=0) return 4;
 return 0;
 fail:fclose(in);fclose(out);return 3;
}
