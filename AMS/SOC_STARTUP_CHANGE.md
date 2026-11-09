# Vehicle SoC startup prior — v0.5.57-r1

- `AMS_EKF_DEFAULT_SOC_INIT` is now **0.27f** for **both vehicle profiles (3 and 4)**; no profile-conditional 100% fallback remains in this vehicle source tree.
- The default initializes every configured segment EKF and the fallback coulomb counter consistently.
- This 27% is a **temporary prior**, based on earlier approximately 3.55 V/cell Molicel P42A readings assuming a rested pack near 25 C. It is not a current observation or persisted SoC.
- Future work may restore a verified persisted SoC (pack identity, freshness, CRC) or use rested OCV; neither is implemented here.
- No change to BMS_OK logic, cell monitoring requirements, drive gates, AIR auxiliary feedback, charging, or CAN bitrate.
