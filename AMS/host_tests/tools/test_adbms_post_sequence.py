#!/usr/bin/env python3
"""Execute extracted production POST functions with ADC-gated diagnostic flags.
Extraction keeps the sequencing/matcher implementation authoritative; only the
transport and ASIC flag behavior are substituted. This is not silicon validation.
"""
from pathlib import Path
import subprocess
import tempfile
import sys
root = Path(__file__).resolve().parents[2]
source = Path(sys.argv[1]) if len(sys.argv) > 1 else root / 'Core/Src/ext_drivers/adbms6830.c'
s = source.read_text()
def function(signature):
    start = s.index(signature)
    brace = s.index('{', start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (s[end] == '{') - (s[end] == '}')
        end += 1
    return s[start:end]
functions = '\n'.join(function(x) for x in (
    'static bool adbms6830_post_stage_matches(',
    'static HAL_StatusTypeDef adbms6830_run_startup_post_once('))
pre = r'''
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ext_drivers/adbms6830_functions.h"
static unsigned conversions, waits;
static bool conversion_pending, completed;
static unsigned fail_stage;
static bool fail_restore, fail_spi_baseline, unexpected_spi_flag;
static unsigned spi_reads;
static uint8_t stimulus;
static void apply_stimulus(adbms6830_driver_t *d);
#define CHECK(x) do { if(!(x)) { fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x); exit(1); } } while(0)
static bool adbms6830_topology_valid(const adbms6830_driver_t *d) { return d && d->num_ics == 1 && d->ics; }
HAL_StatusTypeDef adbms6830_wrcfga_checked(adbms6830_driver_t *d) {
    stimulus = d->ics[0].tx_cfga.flag_d;
    completed = false;
    apply_stimulus(d);
    if(fail_restore && d->post.stage == ADBMS6830_POST_RESTORE_CONFIG) return HAL_ERROR;
    return HAL_OK;
}
HAL_StatusTypeDef adbms6830_clear_all_flags(adbms6830_driver_t *d) {
    memset(&d->diag[0], 0, sizeof(d->diag[0])); return HAL_OK;
}
HAL_StatusTypeDef adbms6830_establish_diagnostic_baseline(adbms6830_driver_t *d) {
    CHECK(stimulus == 0);
    if(fail_spi_baseline && d->post.stage == ADBMS6830_POST_SPIFLT) return HAL_ERROR;
    memset(&d->diag[0], 0, sizeof(d->diag[0])); d->diag[0].statc_valid = true; return HAL_OK;
}
static HAL_StatusTypeDef adbms6830_adcv_checked(adbms6830_driver_t *d, RD rd, CONT cont, DCP dcp, RSTF rstf, OW_C_S ow) {
    CHECK(rd == RD_ON && cont == SINGLE && dcp == DCP_OFF && rstf == RSTF_ON && ow == OW_OFF_ALL_CH);
    CHECK(d->diag[0].va_uv == 0 && d->diag[0].thsd == 0 && d->diag[0].tmodchk == 0);
    conversions++; conversion_pending = true; return HAL_OK;
}
HAL_StatusTypeDef adbms6830_wait_cooperative(adbms6830_driver_t *d, uint32_t us) {
    CHECK(conversion_pending && us == ADBMS6830_REDUNDANT_CONVERSION_WAIT_US);
    waits++; conversion_pending = false;
    if((unsigned)d->post.stage == fail_stage) return HAL_TIMEOUT;
    completed = true; apply_stimulus(d); return HAL_OK;
}
static void apply_stimulus(adbms6830_driver_t *d) {
    adbms6830_ic_diag_t *f = &d->diag[0];
    /* Supply detectors require a conversion. Immediate flag stimuli still
     * work, allowing the old implementation to reach and fail at supply UV. */
    if(stimulus & 3u) f->oscchk = 1;
    if(completed && stimulus == 4u) f->va_uv = 1;
    if(completed && stimulus == 12u) f->va_ov = 1;
    if(stimulus & 16u) f->thsd = 1;
    if(stimulus & 32u) f->ced = f->sed = 1;
    if(stimulus & 64u) f->cmed = f->smed = 1;
    if(stimulus & 128u) f->tmodchk = 1;
 }
HAL_StatusTypeDef adbms6830_read_status(adbms6830_driver_t *d, bool spi) {
    d->diag[0].statc_valid = true;
    if(spi) {
        spi_reads++;
        d->diag[0].spiflt = 1;
        /* Active TMOD stimulus can reassert after a latch clear, as observed
         * in the submitted target SPIFLT failure. */
        if(stimulus & 128u) d->diag[0].tmodchk = 1;
        if(unexpected_spi_flag) d->diag[0].thsd = 1;
    }
    return HAL_OK;
}
HAL_StatusTypeDef adbms6830_verify_config_readback(adbms6830_driver_t *d) {
    CHECK(d->ics[0].tx_cfga.flag_d == 0); return HAL_OK;
}
'''
post = r'''
static adbms6830_driver_t dev;
static adbms6830_asic ic;
static cfa6830_ production[ADBMS6830_MAX_TRACKED_ICS];
static void reset(void) {
    memset(&dev,0,sizeof(dev)); memset(&ic,0,sizeof(ic));
    memset(production,0,sizeof(production));
    dev.num_ics=1; dev.ics=&ic;
    conversions=waits=0; conversion_pending=completed=fail_restore=false;
    fail_stage=0; stimulus=0; spi_reads=0;
    fail_spi_baseline=unexpected_spi_flag=false;
}
int main(void) {
    reset();
    HAL_StatusTypeDef status = adbms6830_run_startup_post_once(&dev,production);
    if(status != HAL_OK) fprintf(stderr,"POST result %u failed_step %u conversions %u waits %u\n", (unsigned)status, (unsigned)dev.post.failed_stage, conversions, waits);
    CHECK(status==HAL_OK);
    CHECK(conversions==8 && waits==8);
    CHECK(dev.post.failed_stage==ADBMS6830_POST_IDLE);
    CHECK(dev.post.restore_status==HAL_OK && stimulus==0);
    CHECK(!dev.post.passed); /* Only the outer POST owner can publish pass. */
    reset(); fail_stage=ADBMS6830_POST_SUPPLY_UV;
    CHECK(adbms6830_run_startup_post_once(&dev,production)==HAL_ERROR);
    CHECK(dev.post.failed_stage==ADBMS6830_POST_SUPPLY_UV);
    CHECK(dev.post.restore_status==HAL_OK && stimulus==0);
    CHECK(conversions==3 && waits==3);
    reset(); fail_restore=true;
    CHECK(adbms6830_run_startup_post_once(&dev,production)==HAL_ERROR);
    CHECK(dev.post.failed_stage==ADBMS6830_POST_RESTORE_CONFIG);
    CHECK(dev.post.restore_status==HAL_ERROR);
    reset(); fail_spi_baseline=true;
    CHECK(adbms6830_run_startup_post_once(&dev,production)==HAL_ERROR);
    CHECK(dev.post.failed_stage==ADBMS6830_POST_SPIFLT);
    CHECK(dev.post.restore_status==HAL_OK && stimulus==0 && spi_reads==0);
    reset(); unexpected_spi_flag=true;
    CHECK(adbms6830_run_startup_post_once(&dev,production)==HAL_ERROR);
    CHECK(dev.post.failed_stage==ADBMS6830_POST_SPIFLT);
    CHECK(dev.post.failed_ic_mask==0 && dev.post.unexpected_ic_mask==1);
    CHECK(dev.post.restore_status==HAL_OK && stimulus==0 && spi_reads==1);
    puts("PASS production POST stimulus/conversion/wait/read sequencing and mandatory restoration");
    return 0;
}
'''
incs=['host_tests/include','.','Core/Inc','Core/Inc/tasks','Core/Inc/ext_drivers','Drivers/CMSIS/Include','Drivers/CMSIS/Device/ST/STM32F7xx/Include','Drivers/STM32F7xx_HAL_Driver/Inc','Drivers/STM32F7xx_HAL_Driver/Inc/Legacy','Middlewares/Third_Party/FreeRTOS/Source/include','Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2','Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1']
with tempfile.TemporaryDirectory() as tmp:
    c=Path(tmp)/'post.c'; exe=Path(tmp)/'post'; c.write_text(pre+functions+post)
    subprocess.run(['cc','-std=c11','-Werror','-Wno-pointer-to-int-cast','-Wno-int-to-pointer-cast','-DSTM32F767xx','-DUSE_HAL_DRIVER','-DAMS_HOST_TEST=1','-DAMS_BUILD_PROFILE=5','-DAMS_BENCH_VALIDATION_SINGLE_SMB=1']+['-I'+str(root/x) for x in incs]+[str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
