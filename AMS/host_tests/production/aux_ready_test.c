#define main unused_wake_fixture_main
#include "adbms_wake_policy_test.c"
#undef main
int main(void) {
    for(unsigned count=1;count<=5;count+=4) {
        reset_dev(count);read_clean();model_poll_stuck=false;
        uint64_t start=sim_us;model_poll_ready_us=start+1112;
        CHECK(adbms6830_wait_aux_ready(&dev,false,1250)==HAL_OK);
        CHECK(sim_us-start>=1112 && sim_us-start<=1750);
        CHECK(dev.session.aux_poll_count>0 && test_lock_depth==0 && last_cs==GPIO_PIN_SET);
        read_clean(); /* additional poll commands retain counter agreement */
    }
    reset_dev(5);read_clean();model_poll_ready_us=sim_us+8000;model_poll_stuck=false;
    uint32_t warm=dev.session.warm_wake_count;
    uint64_t start=sim_us;
    CHECK(adbms6830_wait_aux_ready(&dev,true,10000)==HAL_OK);
    CHECK(sim_us-start>=8000 && sim_us-start<10000);
    CHECK(dev.session.warm_wake_count-warm<=1);
    uint32_t elapsed_wait=(uint32_t)(sim_us-start);
    read_clean();
    printf("PASS AUX2 simulated 8ms readiness: %luus, %lu warm wake(s), counter integrity retained\n",
        (unsigned long)elapsed_wait,(unsigned long)(dev.session.warm_wake_count-warm));
    reset_dev(5);read_clean();model_poll_stuck=true;start=sim_us;
    CHECK(adbms6830_wait_aux_ready(&dev,false,1250)==HAL_TIMEOUT);
    CHECK(sim_us-start<=1750 && !dev.wake.verified && dev.session.aux_poll_fail_count==1);
    CHECK(test_lock_depth==0 && last_cs==GPIO_PIN_SET);
    reset_dev(5);read_clean();model_poll_stuck=false;fail_spi=true;
    CHECK(adbms6830_wait_aux_ready(&dev,false,1250)!=HAL_OK);
    CHECK(!dev.wake.verified && test_lock_depth==0 && last_cs==GPIO_PIN_SET);
    reset_dev(5);read_clean();sim_us=(uint64_t)UINT32_MAX-500;read_clean();
    model_poll_ready_us=sim_us+1112;model_poll_stuck=false;
    CHECK(adbms6830_wait_aux_ready(&dev,false,1250)==HAL_OK);
    reset_dev(5);read_clean();model_poll_stuck=false;
    CHECK(smb_prepare_temp(&dev)==0);CHECK(smb_read_voltage(&dev)==0);
    CHECK(smb_read_temp(&dev)==0 && dev.session.aux_poll_count>=3);
    for(unsigned ic=0;ic<5;ic++) CHECK(dev.last_temp_updated_mask[ic]!=0);
    reset_dev(5);read_clean();dev.session.coherent_snapshot_active=true;
    uint32_t before=transfers;
    CHECK(adbms6830_wait_aux_ready(&dev,false,1250)==HAL_ERROR && transfers==before);
    reset_dev(5);read_clean();dev.spi_debug.enabled=true;
    CHECK(adbms6830_start_adc_cell_voltage_measurement(&dev)==HAL_OK);
    CHECK(dev.spi_debug.last_cmd[0]==0x02u); /* RD_OFF, no S conversion */
    puts("PASS bounded AUX polling: 1/5-device propagation, busy/ready, timeout, transport failure, fine-clock wrap; no continuous C/S polling");
    return failures ? 1 : 0;
}
