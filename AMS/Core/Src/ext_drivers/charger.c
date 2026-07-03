/*
 * charger.c
 *
 *  Created on: Jun 9, 2026
 *      Author: logan
 */


#include "ext_drivers/charger.h"

void charger_init(charger_t *dev, canbus_device_t *canbus)
{
    if(dev == NULL)
    {
        return;
    }

    dev->target_voltage     = 0.0f;
    dev->target_current     = 0.0f;
    dev->read_voltage       = 0.0f;
    dev->read_current       = 0.0f;
    dev->hardware_fail      = false;
    dev->overtemp_fail      = false;
    dev->input_volt_fail    = false;
    dev->voltage_sense_fail = false;
    dev->communication_fail = false;
    dev->tx_fail            = false;
    dev->last_tx_status     = HAL_OK;
    dev->disable_reason_mask = CHARGER_DISABLE_REASON_NONE;
    dev->last_rx_tick       = 0;
    dev->canbus             = canbus;
    dev->tx_count           = 0;
    dev->rx_count           = 0;
    dev->tx_fail_count      = 0;
    dev->flags              = 0;
}
