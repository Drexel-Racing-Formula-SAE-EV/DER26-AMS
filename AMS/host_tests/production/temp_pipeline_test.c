#define main unused_wake_fixture_main
#include "adbms_wake_policy_test.c"
#undef main
int main(void) {
    reset_dev(5); read_clean();
    CHECK(smb_prepare_temp(&dev)==0); sim_us+=1500;
    CHECK(smb_read_temp(&dev)==0 && dev.temp_settle_saved_us==1500);
    CHECK(!dev.temp_prepare_attempted && !dev.temp_prepare_ok);
    reset_dev(5); read_clean();
    CHECK(smb_prepare_temp(&dev)==0);
    CHECK(smb_read_voltage(&dev)==0);
    CHECK(smb_read_temp(&dev)==0 && dev.temp_settle_saved_us==3000);
    for(unsigned i=0;i<5;i++) CHECK(dev.last_temp_updated_mask[i]!=0 && dev.temp_scheduled_mask[i]==dev.temp_attempted_mask[i]);
    reset_dev(5); read_clean(); fail_spi=true;
    CHECK(smb_prepare_temp(&dev)!=0); uint32_t count=transfers;
    fail_spi=false; CHECK(smb_read_temp(&dev)!=0 && transfers==count);
    CHECK(smb_read_temp(&dev)==0); /* next explicit visit can recover */
    reset_dev(5); read_clean(); CHECK(smb_prepare_temp(&dev)==0);
    dev.mux_selected_channel[0][0]^=1; count=transfers;
    CHECK(smb_read_temp(&dev)!=0 && transfers==count);
    reset_dev(5); read_clean(); CHECK(smb_prepare_temp(&dev)==0);
    sim_us+=201000; count=transfers;
    CHECK(smb_read_temp(&dev)!=0 && transfers==count);
    reset_dev(5); read_clean(); CHECK(smb_prepare_temp(&dev)==0);
    adbms6830_link_forget(&dev); count=transfers;
    CHECK(smb_read_temp(&dev)!=0 && transfers==count);
    reset_dev(5); read_clean(); sim_us=(uint64_t)UINT32_MAX-1000;
    CHECK(smb_prepare_temp(&dev)==0); sim_us+=5000;
    CHECK(smb_read_temp(&dev)==0 && dev.temp_settle_saved_us==3000);
    CHECK(test_lock_depth==0 && last_cs==GPIO_PIN_SET);
    puts("PASS residual settling, voltage overlap, single-use preparation, no immediate failure retry, stale/mux/integrity rejection and clock wrap");
    return failures ? 1 : 0;
}
