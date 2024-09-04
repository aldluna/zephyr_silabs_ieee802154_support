/*
 * Copyright (c) 2021 Telink Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * Copyright (c) 2021 Telink Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_rail_ieee802154
#define LOG_MODULE_NAME ieee802154_erf32

#if defined(CONFIG_IEEE802154_DRIVER_LOG_LEVEL)
#define LOG_LEVEL CONFIG_IEEE802154_DRIVER_LOG_LEVEL
#else
#define LOG_LEVEL LOG_LEVEL_NONE
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#include <zephyr/random/random.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/irq.h>
#if defined(CONFIG_NET_L2_OPENTHREAD)
#include <zephyr/net/openthread.h>
#endif

#include "em_system.h"
#include "rail_types.h"
#include "rail.h"
#include "rail_ieee802154.h"
#include "pa_conversions_efr32.h"
#include "sl_rail_util_pa_config.h"
#include "sl_rail_util_protocol.h"

#include "ieee802154_silabs_rail.h"

/* ERF32 data structure */
static struct efr32_data data;

/* Enum declaration */
static efr32_state rail_state;

/* RAIL handler callback declaration */
static void efr32_rail_cb(RAIL_Handle_t rail_handle, RAIL_Events_t a_events);
/* RAIL config declartion */
static RAIL_Config_t rail_config = {
	.eventsCallback = &efr32_rail_cb,
};

/*##### RAIL api configuration #####*/
/* CSMA configuration options */
static const RAIL_CsmaConfig_t rail_csma_config = RAIL_CSMA_CONFIG_802_15_4_2003_2p4_GHz_OQPSK_CSMA;

/* RAIL IEE802154 config struct */
static RAIL_IEEE802154_Config_t rail_ieee802154_config = {
    .addresses = NULL,
    .ackConfig = {
      .enable = true,
      .ackTimeout = 672,
      .rxTransitions = {
        .success = RAIL_RF_STATE_RX,
        .error   = RAIL_RF_STATE_RX,
      },
      .txTransitions = {
        .success = RAIL_RF_STATE_RX,
        .error   = RAIL_RF_STATE_RX,
      }
    },
    .timings = {
      .idleToRx = 100,
      .idleToTx = 100,
      .rxToTx   = 192,
      .txToRx   = 192,
      .rxSearchTimeout     = 0,
      .txToRxSearchTimeout = 0,
    },
    .framesMask          			   = RAIL_IEEE802154_ACCEPT_STANDARD_FRAMES,
    .promiscuousMode     			   = false,
    .isPanCoordinator    			   = false,
    .defaultFramePendingInOutgoingAcks = false,
};

/* Get MAC address */
static void *efr32_get_mac(uint8_t *mac_addr)
{
	uint64_t uniqueID = SYSTEM_GetUnique();
	memcpy(mac_addr, &uniqueID, sizeof(uniqueID));
}

/* API implementation: iface_init */
static void efr32_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct efr32_data *efr32 = dev->data;
	efr32_get_mac(efr32->mac_addr);

	net_if_set_link_addr(iface, efr32->mac_addr, ERF32_IEEE_ADDRESS_SIZE, NET_LINK_IEEE802154);
	efr32->iface = iface;
	ieee802154_init(iface);
}

/* API implementation: get_capabilities */
static enum ieee802154_hw_caps efr32_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	return IEEE802154_HW_FCS |
           IEEE802154_HW_FILTER |
           IEEE802154_HW_TX_RX_ACK |
           IEEE802154_HW_CSMA;
}

/* API implementation: cca */
static int efr32_cca(const struct device *dev)
{
	// RAIL handles this in tx()
	ARG_UNUSED(dev);
	return 0;
}

/* API implementation: set_channel */
static int efr32_set_channel(const struct device *dev, uint16_t channel)
{
    RAIL_Status_t status;
	struct efr32_data *efr32 = dev->data;

    // Set the radio channel
    status = RAIL_PrepareChannel(efr32->rail_handle, channel);

    if (status != RAIL_STATUS_NO_ERROR) {
        return -EIO;
    }

	efr32->channel = channel;

    return 0;
}

/* API implementation: filter */
static int efr32_filter(const struct device *dev,
		      			bool set,
		      			enum ieee802154_filter_type type,
		      			const struct ieee802154_filter *filter)
{
	LOG_DBG("Applying filter %u", type);

	// RAIL status
	RAIL_Status_t status = RAIL_STATUS_NO_ERROR;
	struct efr32_data *efr32 = dev->data;

	if (!set) {
		return -ENOTSUP;
	}

	if (type == IEEE802154_FILTER_TYPE_IEEE_ADDR) {
		status = RAIL_IEEE802154_SetLongAddress(efr32->rail_handle, 
												filter->ieee_addr, 
												DEFAULT_ADDRESS_INDEX);
	} else if (type == IEEE802154_FILTER_TYPE_SHORT_ADDR) {
		status = RAIL_IEEE802154_SetShortAddress(efr32->rail_handle, 
												 filter->short_addr, 
												 DEFAULT_ADDRESS_INDEX);
	} else if (type == IEEE802154_FILTER_TYPE_PAN_ID) {
		status = RAIL_IEEE802154_SetPanId(efr32->rail_handle, 
										  filter->pan_id, 
										  DEFAULT_ADDRESS_INDEX);
	}
	
	if (status != RAIL_STATUS_NO_ERROR)
	{
		LOG_ERR("Error setting address via RAIL");
		return -EIO;
	}
	
	return 0;
}

/* API implementation: set_txpower */
static int efr32_set_txpower(const struct device *dev, int16_t dbm)
{
	// RAIL status
	RAIL_Status_t status = RAIL_STATUS_NO_ERROR;
	// Init erf32 data struct
	struct efr32_data *efr32 = dev->data;
	
	// Set new dbm value
	status = RAIL_SetTxPowerDbm (efr32->rail_handle, (RAIL_TxPower_t)DBM_TO_DECI_DBM(dbm));
	if (status != RAIL_STATUS_NO_ERROR) {
        return -EIO;
    }

	return 0;
}

/* API implementation: start */
static int efr32_start(const struct device *dev)
{
	struct efr32_data *efr32 = dev->data;
	RAIL_Status_t status;

	status = RAIL_StartRx(efr32->rail_handle, efr32->channel, NULL);
	if (status != RAIL_STATUS_NO_ERROR) {
		LOG_DBG("RAIL_StartRx returned an error %d", status);
		return -EIO;
	}

	return 0;
}

/* API implementation: stop */
static int efr32_stop(const struct device *dev)
{
#if 0
    struct efr32_context *efr32 = dev->driver_data;
    ARG_UNUSED(efr32);

    rail_state = EFR32_STATE_DISABLED;

    LOG_DBG("EFR32 802154 radio stopped");

    return 0;
#else
	// RAIL status
	RAIL_Status_t status = RAIL_STATUS_NO_ERROR;
	// Init erf32 data struct
	struct efr32_data *efr32 = dev->data;

	RAIL_Idle(efr32->rail_handle, RAIL_IDLE_ABORT, true);
	status = RAIL_ConfigEvents(efr32->rail_handle, RAIL_EVENTS_ALL, 0);
	if (status != RAIL_STATUS_NO_ERROR) {
        return -EIO;
    }

	status = RAIL_IEEE802154_Deinit(efr32->rail_handle);
	if (status != RAIL_STATUS_NO_ERROR) {
        return -EIO;
    }

	return 0;
#endif
}

/* API implementation: tx */
static int efr32_tx(const struct device *dev,
		  			enum ieee802154_tx_mode mode,
		  			struct net_pkt *pkt,
		  			struct net_buf *frag)
{
	struct efr32_data *efr32 = dev->data;

	RAIL_TxOptions_t txOptions = RAIL_TX_OPTIONS_DEFAULT;
    RAIL_Status_t status = RAIL_STATUS_NO_ERROR;
	uint16_t number_written_bytes = 0;
    uint16_t payload_length = frag->len;
    uint8_t *payload = frag->data;

	LOG_DBG("rx %p (%u)", payload, payload_length);
	efr32->tx_success = false;

    memcpy(efr32->tx_buffer + 1, payload, payload_length);
    efr32->tx_buffer[0] = payload_length + EFR32_FCS_LENGTH;

    /* Reset semaphore in case ACK was received after timeout */
    k_sem_reset(&efr32->tx_wait);

	// Load data into FIFO
    number_written_bytes = RAIL_WriteTxFifo(efr32->rail_handle, payload, payload_length, true);
    // Begin transmitting
    status = RAIL_StartCcaCsmaTx(efr32->rail_handle, efr32->channel, txOptions, &rail_csma_config, NULL);
    if (status != RAIL_STATUS_NO_ERROR) {
		efr32->tx_success = 0;
        return -EIO;
    }

	LOG_DBG("Sending frame: channel=%d, written=%u", efr32->channel, number_written_bytes);
	k_sem_give(&efr32->rx_wait);
	LOG_DBG("Result: %d", efr32->tx_success);
    return efr32->tx_success ? 0 : -EBUSY;
}

#if 1
static void efr32_rx(void *arg1, void *arg2, void *arg3)
{
    struct efr32_data *efr32 = (struct efr32_data *) arg1;

    while (1)
    {
        LOG_DBG("Waiting for frame");
        k_sem_take(&efr32->rx_wait, K_FOREVER);

		LOG_DBG("Frame received!");

        RAIL_Idle(efr32->rail_handle, RAIL_IDLE_ABORT, true);
        RAIL_StartRx(efr32->rail_handle, efr32->channel, NULL);
    }
}
#endif

#define RAIL_IRQ_PRIO 0

void ieee802154_gecko_irq_config(void)
{
	//IRQ_DIRECT_CONNECT(RFSENSE_IRQn, RAIL_IRQ_PRIO, RFSENSE_IRQHandler, RAIL_IRQ_FLAGS);
	//irq_enable(RFSENSE_IRQn);
	IRQ_DIRECT_CONNECT(AGC_IRQn, RAIL_IRQ_PRIO, AGC_IRQHandler, 0);
	irq_enable(AGC_IRQn);
	IRQ_DIRECT_CONNECT(BUFC_IRQn, RAIL_IRQ_PRIO, BUFC_IRQHandler, 0);
	irq_enable(BUFC_IRQn);
	IRQ_DIRECT_CONNECT(FRC_IRQn, RAIL_IRQ_PRIO, FRC_IRQHandler, 0);
	irq_enable(FRC_IRQn);
	IRQ_DIRECT_CONNECT(FRC_IRQn, RAIL_IRQ_PRIO, FRC_IRQHandler, 0);
	irq_enable(FRC_IRQn);
	IRQ_DIRECT_CONNECT(FRC_PRI_IRQn, RAIL_IRQ_PRIO, FRC_PRI_IRQHandler, 0);
	irq_enable(FRC_PRI_IRQn);
	IRQ_DIRECT_CONNECT(MODEM_IRQn, RAIL_IRQ_PRIO, MODEM_IRQHandler, 0);
	irq_enable(MODEM_IRQn);
	IRQ_DIRECT_CONNECT(PROTIMER_IRQn, RAIL_IRQ_PRIO, PROTIMER_IRQHandler, 0);
	irq_enable(PROTIMER_IRQn);
	IRQ_DIRECT_CONNECT(RAC_RSM_IRQn, RAIL_IRQ_PRIO, RAC_RSM_IRQHandler, 0);
	irq_enable(RAC_RSM_IRQn);
	IRQ_DIRECT_CONNECT(RAC_SEQ_IRQn, RAIL_IRQ_PRIO, RAC_SEQ_IRQHandler, 0);
	irq_enable(RAC_SEQ_IRQn);
	IRQ_DIRECT_CONNECT(SYNTH_IRQn, RAIL_IRQ_PRIO, SYNTH_IRQHandler, 0);
	irq_enable(SYNTH_IRQn);
}

#if 0
#define RAIL_IRQ_PRIO 0

void ieee802154_gecko_irq_config(void)
{
	//IRQ_DIRECT_CONNECT(RFSENSE_IRQn, RAIL_IRQ_PRIO, RFSENSE_IRQHandler, RAIL_IRQ_FLAGS);
	//irq_enable(RFSENSE_IRQn);
	IRQ_DIRECT_CONNECT(AGC_IRQn, RAIL_IRQ_PRIO, AGC_IRQHandler, 0);
	irq_enable(AGC_IRQn);
	IRQ_DIRECT_CONNECT(BUFC_IRQn, RAIL_IRQ_PRIO, BUFC_IRQHandler, 0);
	irq_enable(BUFC_IRQn);
	IRQ_DIRECT_CONNECT(FRC_IRQn, RAIL_IRQ_PRIO, FRC_IRQHandler, 0);
	irq_enable(FRC_IRQn);
	IRQ_DIRECT_CONNECT(FRC_IRQn, RAIL_IRQ_PRIO, FRC_IRQHandler, 0);
	irq_enable(FRC_IRQn);
	IRQ_DIRECT_CONNECT(FRC_PRI_IRQn, RAIL_IRQ_PRIO, FRC_PRI_IRQHandler, 0);
	irq_enable(FRC_PRI_IRQn);
	IRQ_DIRECT_CONNECT(MODEM_IRQn, RAIL_IRQ_PRIO, MODEM_IRQHandler, 0);
	irq_enable(MODEM_IRQn);
	IRQ_DIRECT_CONNECT(PROTIMER_IRQn, RAIL_IRQ_PRIO, PROTIMER_IRQHandler, 0);
	irq_enable(PROTIMER_IRQn);
	IRQ_DIRECT_CONNECT(RAC_RSM_IRQn, RAIL_IRQ_PRIO, RAC_RSM_IRQHandler, 0);
	irq_enable(RAC_RSM_IRQn);
	IRQ_DIRECT_CONNECT(RAC_SEQ_IRQn, RAIL_IRQ_PRIO, RAC_SEQ_IRQHandler, 0);
	irq_enable(RAC_SEQ_IRQn);
	IRQ_DIRECT_CONNECT(SYNTH_IRQn, RAIL_IRQ_PRIO, SYNTH_IRQHandler, 0);
	irq_enable(SYNTH_IRQn);
}

static RAIL_Status_t efr32_config_rail(const struct device *dev){
	struct efr32_data *efr32 = dev->data;
	RAIL_Status_t status;

	efr32->rail_handle = RAIL_Init(&rail_config, NULL);
	if (efr32->rail_handle == NULL) {
		LOG_DBG("Failed to get RAIL handle");
		return -EIO;
	}
	RAIL_Idle(efr32->rail_handle, RAIL_IDLE, true);

	RAIL_EnablePaCal(true);
	RAIL_ConfigCal(efr32->rail_handle, RAIL_CAL_ALL);

	/* Configure RAIL callbacks */
	RAIL_ConfigEvents(efr32->rail_handle, RAIL_EVENTS_ALL,
			  RAIL_EVENT_RX_PACKET_RECEIVED |
			  RAIL_EVENT_TX_PACKET_SENT |
			  RAIL_EVENT_TX_ABORTED |
			  RAIL_EVENT_TX_BLOCKED |
			  RAIL_EVENT_TX_UNDERFLOW |
			  RAIL_EVENT_TX_CHANNEL_BUSY |
			  RAIL_EVENT_CAL_NEEDED);

	RAIL_DECLARE_TX_POWER_VBAT_CURVES_ALT;
	const RAIL_TxPowerCurvesConfigAlt_t tx_power_curves = RAIL_DECLARE_TX_POWER_CURVES_CONFIG_ALT;
	status = RAIL_InitTxPowerCurvesAlt(&tx_power_curves);
	if (status != RAIL_STATUS_NO_ERROR) {
		LOG_DBG("RAIL_RAIL_InitTxPowerCurvesAlt returned an error %d", status);
		return -EIO;
	}

	/* Initialize the PA */
	RAIL_TxPowerConfig_t txPowerConfig = {
		.mode = RAIL_TX_POWER_MODE_2P4_HP,
		.voltage = 1800U,
		.rampTime = 10U,
	};

	status = RAIL_ConfigTxPower(efr32->rail_handle, &txPowerConfig);
	if (status != RAIL_STATUS_NO_ERROR) {
		LOG_DBG("RAIL_ConfigTxPower returned an error %d", status);
		return -EIO;
	}

	status = RAIL_SetTxPower(efr32->rail_handle, 255);
	if (status != RAIL_STATUS_NO_ERROR) {
		LOG_DBG("RAIL_SetTxPower returned an error %d", status);
		return -EIO;
	}

	const RAIL_DataConfig_t rail_data_cfg = {
		.txSource = TX_PACKET_DATA,
		.rxSource = RX_PACKET_DATA,
		.txMethod = PACKET_MODE,
		.rxMethod = PACKET_MODE,
	};

	status = RAIL_ConfigData(efr32->rail_handle, &rail_data_cfg);
	if (status != RAIL_STATUS_NO_ERROR) {
		LOG_DBG("RAIL_ConfigData returned an error %d", status);
		return -EIO;
	}

	RAIL_SetTxFifo(efr32->rail_handle, efr32->tx_buffer, 0, ERF32_TX_BUFFER_LENGTH);

	return status;
}


static RAIL_Status_t efr32_config_ieee802154_2p4ghz(RAIL_Handle_t handle){
	RAIL_Status_t status;

	status = RAIL_IEEE802154_Init(handle, &rail_ieee802154_config);
	if (RAIL_STATUS_NO_ERROR != status) {
		(void) RAIL_IEEE802154_Deinit(handle);
		LOG_DBG("RAIL_IEEE802154_Init returned an error %d", status);
		return -EIO;
	}
	
	status = RAIL_IEEE802154_Config2p4GHzRadio(handle);
	if (RAIL_STATUS_NO_ERROR == status) {
		LOG_DBG("RAIL_IEEE802154_Config2p4GHzRadio returned an error %d", status);
		return -EIO;
	}


	return status;
}

void RAILCb_AssertFailed(RAIL_Handle_t railHandle, uint32_t errorCode){
        
	static const char* railErrorMessages[] = RAIL_ASSERT_ERROR_MESSAGES;
	const char *errorMessage ="Unknown";
 
  	// If this error code is within the range of known error messages then use       
	// the appropriate error message    
  	if(errorCode < (sizeof (railErrorMessages) / sizeof(char*))){
		errorMessage = railErrorMessages[errorCode];
	}
	printf(errorMessage);
         
	// Reset the chip since an assert is a fatal error
	NVIC_SystemReset();
}

static int ieee802154_gecko_init(struct device *dev)
{
	struct efr32_data *efr32 = dev->data;
	int ret;

	k_sem_init(&efr32->tx_wait, 0, 1);

	ret = efr32_config_ieee802154_2p4ghz(efr32->rail_handle);
	if (ret != 0) {
		LOG_ERR("%d: Failed to initialize RAIL", ret);
		return ret;
	}

	ieee802154_gecko_irq_config();

	ret = efr32_config_rail(dev);
	if (ret != 0) {
		LOG_ERR("%d: Failed to initialize IEEE 802.15.4 Radio", ret);
		return ret;
	}

	LOG_INF("Device %s initialized", dev->name);

	return 0;
}
#endif

#if 1
static void tx_power_config_update(const RAIL_TxPowerConfig_t *tx_pwr_config, int8_t tx_power, struct efr32_data *efr32){
	RAIL_Status_t       status;
    RAIL_TxPowerLevel_t tx_power_lvl;
    RAIL_TxPower_t      tx_power_dbm = DBM_TO_DECI_DBM(tx_power);

    tx_power_lvl = RAIL_GetTxPower(efr32->rail_handle);

    // Always need to call RAIL_SetTxPowerDbm after RAIL_ConfigTxPower
    // First need to get existing power setting and reassert value after config

    if (tx_power_lvl != RAIL_TX_POWER_LEVEL_INVALID)
    {
        tx_power_dbm = RAIL_GetTxPowerDbm(efr32->rail_handle);
    }

    status = RAIL_ConfigTxPower(efr32->rail_handle, tx_pwr_config);
	if (status != RAIL_STATUS_NO_ERROR)
    {
        LOG_ERR("Error with config data.");
    }

    status = RAIL_SetTxPowerDbm(efr32->rail_handle, tx_power_dbm);
    if (status != RAIL_STATUS_NO_ERROR)
    {
        LOG_ERR("Error with config data.");
    }
}

static RAIL_Handle_t efr32_init_rail(struct efr32_data *efr32){
    RAIL_Status_t status;
    RAIL_Handle_t handle;

    handle = RAIL_Init(&efr32->rail_config, NULL);
    if (handle == NULL)
    {
        LOG_ERR("Error with config data.");
        return NULL;
    }

#if 0
    status = RAIL_InitPowerManager();
    OT_ASSERT(status == RAIL_STATUS_NO_ERROR);
#endif // SL_CATALOG_POWER_MANAGER_PRESENT

    status = RAIL_ConfigCal(handle, RAIL_CAL_ALL);
    if (status != RAIL_STATUS_NO_ERROR)
    {
        LOG_ERR("Error with config calibrations.");
        return NULL;
    }

    status = RAIL_SetPtiProtocol(handle, RAIL_PTI_PROTOCOL_THREAD);
    if (status != RAIL_STATUS_NO_ERROR)
    {
        LOG_ERR("Error with Pti protocol set up");
        return NULL;
    }
	
    //status = RAIL_IEEE802154_Init(handle, &rail_ieee802154_config);
    status = efr32_config_ieee802154_2p4ghz(handle);
	if (status != RAIL_STATUS_NO_ERROR)
    {
        LOG_ERR("Error with RAIL IEE802154 Init.");
        return NULL;
    }

#if 0
    // Enhanced Frame Pending
    status = RAIL_IEEE802154_EnableEarlyFramePending(handle, true);
    OT_ASSERT(status == RAIL_STATUS_NO_ERROR);

    status = RAIL_IEEE802154_EnableDataFramePending(handle, true);
    OT_ASSERT(status == RAIL_STATUS_NO_ERROR);

    // Copies of MAC keys for encrypting at the radio layer
    memset(sMacKeys, 0, sizeof(sMacKeys));
#endif // (OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2)

    efr32->tx_buffer_size = RAIL_SetTxFifo(handle, efr32->tx_buffer, 0, sizeof(efr32->tx_buffer));
    if (efr32->tx_buffer_size != sizeof(efr32->tx_buffer))
    {
        LOG_ERR("Error in Tx FIFO setup");
        return NULL;
    }

    // Enable RAIL multi-timer
    RAIL_ConfigMultiTimer(true);

    return handle;
}

static void efr32_rail_config_load(struct efr32_data *efr32, int8_t aTxPower){
	RAIL_Status_t        status;
    RAIL_TxPowerConfig_t txPowerConfig = {SL_RAIL_UTIL_PA_SELECTION_2P4GHZ,
                                          SL_RAIL_UTIL_PA_VOLTAGE_MV,
                                          SL_RAIL_UTIL_PA_RAMP_TIME_US};

    if (efr32->rail_channel_config != NULL)
    {
        status = RAIL_IEEE802154_SetPtiRadioConfig(efr32->rail_handle, RAIL_IEEE802154_PTI_RADIO_CONFIG_915MHZ_R23_NA_EXT);
		if (status != RAIL_STATUS_NO_ERROR)
    	{
			LOG_ERR("Error with config data.");
    	}

        uint16_t firstChannel = 0xFF;
        firstChannel = RAIL_ConfigChannels(efr32->rail_handle, efr32->rail_channel_config, NULL);
		if (firstChannel != efr32->mChannelMin)
    	{
			LOG_ERR("Error with config data.");
    	}
        
        txPowerConfig.mode = SL_RAIL_UTIL_PA_SELECTION_SUBGHZ;
        status = RAIL_IEEE802154_ConfigGOptions(efr32->rail_handle, RAIL_IEEE802154_G_OPTION_GB868, RAIL_IEEE802154_G_OPTION_GB868);
        if (status != RAIL_STATUS_NO_ERROR)
    	{
			LOG_ERR("Error with config data.");
    	}
    }
    else
    {
#if 0
        status = sl_rail_util_ieee802154_config_radio(gRailHandle);
#else
        status = RAIL_IEEE802154_Config2p4GHzRadio(efr32->rail_handle);
#endif // SL_CATALOG_RAIL_UTIL_IEEE802154_PHY_SELECT_PRESENT
        if (status != RAIL_STATUS_NO_ERROR)
    	{
			LOG_ERR("Error with config data.");
    	}
    }

#if 0
    // 802.15.4E support (only on platforms that support it, so error checking is disabled)
    // Note: This has to be called after RAIL_IEEE802154_Config2p4GHzRadio due to a bug where this call
    // can overwrite options set below.
    RAIL_IEEE802154_ConfigEOptions(gRailHandle,
                                   (RAIL_IEEE802154_E_OPTION_GB868 | RAIL_IEEE802154_E_OPTION_ENH_ACK),
                                   (RAIL_IEEE802154_E_OPTION_GB868 | RAIL_IEEE802154_E_OPTION_ENH_ACK));
#endif // (OPENTHREAD_CONFIG_THREAD_VERSION >= OT_THREAD_VERSION_1_2)

    if (aTxPower != 127)
    {
        tx_power_config_update(&txPowerConfig, aTxPower, efr32);
    }
}

static void efr32_config_init(struct efr32_data *efr32){
	RAIL_Status_t status;

	efr32->aEventCallback = efr32_rail_cb;
	efr32->rail_config.protocol = NULL;
	efr32->rail_config.scheduler = NULL;
	efr32->rail_channel_config = NULL;
	efr32->mChannelMin = 1; 
	efr32->mChannelMin = 10;
	
	efr32->rail_handle = efr32_init_rail(efr32);
	if (efr32->rail_handle != NULL)
	{
		LOG_ERR("Fail RAIL handle init");
	}

	status = RAIL_ConfigEvents(efr32->rail_handle, RAIL_EVENTS_ALL, 0);
	if (status != RAIL_STATUS_NO_ERROR)
	{
		LOG_ERR("Fail RAIL on event config");
	}
  
	//efr32_rail_config_load(efr32, 0);
}

#if 0
/* Driver initialization */
static int efr32_radio_init(const struct device *dev)
{
    RAIL_Status_t status;

	struct efr32_data *efr32 = dev->data;

    efr32_config_init(efr32);

   //status = RAIL_ConfigSleep(efr32->rail_handle, RAIL_SLEEP_CONFIG_TIMERSYNC_ENABLED);
   //if (status != RAIL_STATUS_NO_ERROR)
	//{
	//	LOG_ERR("Fail RAIL config sleep");
	//}
#if 0
    sReceive.frame.mLength       = 0;
    sReceive.frame.mPsdu         = NULL;

    sReceiveAck.frame.mLength    = 0;
    sReceiveAck.frame.mPsdu      = sReceiveAckPsdu;

    // Initialize the queue for received packets.
    queueStatus = queueInit(&sPendingCommandQueue, RADIO_REQUEST_BUFFER_COUNT);
    OT_ASSERT(queueStatus);

    // Specify a callback to be called upon queue overflow.
    queueStatus = queueOverflow(&sPendingCommandQueue, &pendingCommandQueueOverflowCallback);
    OT_ASSERT(queueStatus);

    for (uint8_t i = 0; i < RADIO_REQUEST_BUFFER_COUNT; i++)
    {
        // Initialize the tx buffer params.
        sTransmitBuffer[i].iid                = INVALID_INTERFACE_INDEX;
        sTransmitBuffer[i].frame.mLength      = 0;
        sTransmitBuffer[i].frame.mPsdu        = sTransmitPsdu[i];

        sTransmitBuffer[i].frame.mInfo.mTxInfo.mIeInfo = &sTransmitIeInfo[i];
    }

    otLinkMetricsInit(EFR32_RECEIVE_SENSITIVITY);
#endif
    //sCurrentBandConfig = efr32RadioGetBandConfig(OPENTHREAD_CONFIG_DEFAULT_CHANNEL);
    //OT_ASSERT(sCurrentBandConfig != NULL);

    sl_rail_util_pa_init();
    efr32_set_txpower(dev ,0);

    status = RAIL_ConfigRxOptions(efr32->rail_handle,
                                  RAIL_RX_OPTION_TRACK_ABORTED_FRAMES,
                                  RAIL_RX_OPTION_TRACK_ABORTED_FRAMES);
    if (status != RAIL_STATUS_NO_ERROR)
	{
		LOG_ERR("Fail RAIL config Rx Options");
	}

    //efr32PhyStackInit();
    //efr32RadioSetCcaMode(SL_OPENTHREAD_RADIO_CCA_MODE); // Change for the following call
	RAIL_IEEE802154_ConfigCcaMode(efr32->rail_handle, RAIL_IEEE802154_CCA_MODE_RSSI);

    //sEnergyScanStatus = ENERGY_SCAN_STATUS_IDLE;

#if 0
    sChannelSwitchingCfg.bufferBytes = RAIL_IEEE802154_RX_CHANNEL_SWITCHING_BUF_BYTES;
    sChannelSwitchingCfg.buffer      = sChannelSwitchingBuffer;
    for (uint8_t i = 0U; i < RAIL_IEEE802154_RX_CHANNEL_SWITCHING_NUM_CHANNELS; i++)
    {
        sChannelSwitchingCfg.channels[i] = UNINITIALIZED_CHANNEL;
    }
#endif

    // Initialize the queue for received packets.
	k_fifo_init(&efr32->tx_fifo);

    LOG_DBG("Initialized");
	return 0;
}
#else

#ifndef SL_COMPONENT_CATALOG_PRESENT
__WEAK void sl_openthread_init(void)
{
    // Placeholder for enabling Silabs specific features available only through Simplicity Studio
}
#else
void sl_openthread_init(void);
#endif // SL_COMPONENT_CATALOG_PRESENT

void RAILCb_AssertFailed(RAIL_Handle_t railHandle, uint32_t errorCode){
        
	static const char* railErrorMessages[] = RAIL_ASSERT_ERROR_MESSAGES;
	const char *errorMessage ="Unknown";
 
  	// If this error code is within the range of known error messages then use       
	// the appropriate error message    
  	if(errorCode < (sizeof (railErrorMessages) / sizeof(char*))){
		errorMessage = railErrorMessages[errorCode];
	}
	printf(errorMessage);
         
	// Reset the chip since an assert is a fatal error
	NVIC_SystemReset();
}

/// RAIL schedule configuration
static RAILSched_Config_t s_rail_sched_config;
/// RAIL configuration
static RAIL_Config_t s_rail_config = {
  .eventsCallback = &efr32_rail_cb,
  .scheduler = &s_rail_sched_config,
};

static int efr32_radio_init(const struct device *dev){
	RAIL_Status_t status;
	struct efr32_data *efr32 = dev->data;
	efr32->rail_config = s_rail_config;

	// intializes openthread
	sl_openthread_init();

	// Semaphore init
	k_sem_init(&efr32->tx_wait, 0, 1);

	// initializes the RAIL core
	efr32->rail_handle = RAIL_Init(&efr32->rail_config,
								NULL
								);
	LOG_DBG("s_rail_config pointer: %p", &s_rail_config);
	LOG_DBG("efr32->rail_config pointer: %p", &efr32->rail_config);

	if (efr32->rail_handle == NULL)
	{
		LOG_ERR("Fail init handle");
	}

	// configures the data management
	RAIL_DataConfig_t data_config = {
		.txSource = TX_PACKET_DATA,
		.rxSource = RX_PACKET_DATA,
		.txMethod = PACKET_MODE,
		.rxMethod = PACKET_MODE,
	};
	status = RAIL_ConfigData(efr32->rail_handle, &data_config);
	if((RAIL_STATUS_NO_ERROR != status)){
		LOG_ERR("RAIL_ConfigData failed, return value: %d", status);
	}
				

	// configures the channels
	const RAIL_ChannelConfig_t *channel_config = NULL;
	(void) RAIL_ConfigChannels(efr32->rail_handle,
								channel_config,
								NULL);

	// configures the IEEE 802.15.4 protocol based on the chosen protocol by user
	status = sl_rail_util_protocol_config(efr32->rail_handle,
	(sl_rail_util_protocol_type_t) SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ);
	if((RAIL_STATUS_NO_ERROR != status)){
		LOG_ERR("sl_rail_util_protocol_config failed, return value: %d", status);
	}

	// configures RAIL calibration
	status = RAIL_ConfigCal(efr32->rail_handle,
							0U
							| (0
								? RAIL_CAL_TEMP : 0U)
							| (0
								? RAIL_CAL_ONETIME : 0U));
	if((RAIL_STATUS_NO_ERROR != status)){
		LOG_ERR("RAIL_ConfigCal failed, return value: %d", status);
	}

	// configures the using RAIL events
	status = RAIL_ConfigEvents(efr32->rail_handle,
								RAIL_EVENTS_ALL,
								RAIL_EVENT_RX_PACKET_RECEIVED
								| RAIL_EVENT_TX_PACKET_SENT
								| RAIL_EVENT_CAL_NEEDED
								| RAIL_EVENT_TXACK_PACKET_SENT
								| RAIL_EVENT_RX_PACKET_ABORTED
								| RAIL_EVENT_RX_FRAME_ERROR
								| RAIL_EVENT_RX_FIFO_OVERFLOW
								| RAIL_EVENT_RX_ADDRESS_FILTERED
								| RAIL_EVENT_RX_SCHEDULED_RX_MISSED
								| RAIL_EVENT_TX_ABORTED
								| RAIL_EVENT_TX_BLOCKED
								| RAIL_EVENT_TX_UNDERFLOW
								| RAIL_EVENT_TX_CHANNEL_BUSY
								| RAIL_EVENT_TX_SCHEDULED_TX_MISSED
								| RAIL_EVENT_TXACK_ABORTED
								| RAIL_EVENT_TXACK_BLOCKED
								| RAIL_EVENT_TXACK_UNDERFLOW
								);
	
	ieee802154_gecko_irq_config();

	k_thread_create(&efr32->rx_thread, efr32->rx_stack,
		2048, efr32_rx, efr32, NULL, NULL,
		K_PRIO_COOP(2), 0, K_NO_WAIT);

	k_thread_name_set(&efr32->rx_thread, "efr32_rx");
	
	LOG_DBG("Initialized");
	return 0;
}
#endif
#endif

static void efr32_rail_cb(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
    LOG_DBG("Processing events 0x%llX", events);

	if (events & RAIL_EVENT_RX_PACKET_RECEIVED) {
		LOG_DBG("Received packet frame");
		//RAIL_RxPacketHandle_t packetHandle = RAIL_RX_PACKET_HANDLE_INVALID;
		//RAIL_RxPacketInfo_t packetInfo;
		//RAIL_RxPacketDetails_t packetDetails;
		//RAIL_Status_t status;
		//uint16_t length;
		//packetHandle = RAIL_GetRxPacketInfo(efr32_data.rail_handle, RAIL_RX_PACKET_HANDLE_OLDEST, &packetInfo);
	}
	if (events & RAIL_EVENT_RX_PACKET_ABORTED) {
		LOG_DBG("RX packet aborted");
	}
	if (events & RAIL_EVENT_RX_FRAME_ERROR) {
		LOG_DBG("RX frame error");
	}
	if (events & RAIL_EVENT_RX_FIFO_OVERFLOW) {
		LOG_DBG("RX FIFO overflow");
	}
	if (events & RAIL_EVENT_RX_ADDRESS_FILTERED) {
		LOG_DBG("RX Address filtered");
	}
    	if (events & RAIL_EVENT_TX_PACKET_SENT) {
		LOG_DBG("TX packet sent");
	}
	if (events & RAIL_EVENT_TX_ABORTED) {
		LOG_DBG("TX was aborted");
	}
	if (events & RAIL_EVENT_TX_BLOCKED) {
		LOG_DBG("TX is blocked");
	}
	if (events & RAIL_EVENT_TX_UNDERFLOW) {
		LOG_DBG("TX is underflow");
	}
	if (events & RAIL_EVENT_TX_CHANNEL_BUSY) {
		LOG_DBG("TX channel busy");
	}

	if (events & RAIL_EVENT_TXACK_PACKET_SENT) {
		LOG_DBG("TXACK packet sent");
	}
	if (events & RAIL_EVENT_TXACK_ABORTED) {
		LOG_DBG("TXACK aborted");
	}
    	if (events & RAIL_EVENT_TXACK_BLOCKED) {
		LOG_DBG("TXACK blocked");
	}
	if (events & RAIL_EVENT_TXACK_UNDERFLOW) {
		LOG_DBG("TXACK underflow");
	}

    if (events & RAIL_EVENT_TX_FIFO_ALMOST_EMPTY) {
        LOG_DBG("TX FIFO almost empty");
	}
	if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
		LOG_DBG("RX FIFO almost full");
	}
	if (events & RAIL_EVENT_RX_ACK_TIMEOUT) {
		LOG_DBG("RX AutoAck occurred");
	}
	if (events & RAIL_EVENT_CAL_NEEDED) {
		LOG_DBG("Calibration needed");
		RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
	}
	if (events & RAIL_EVENT_IEEE802154_DATA_REQUEST_COMMAND) {
		LOG_DBG("IEEE802154 Data request command");
	}
}

/* API implementation: ed_scan */
static int efr32_ed_scan(const struct device *dev, uint16_t duration,
		       energy_scan_done_cb_t done_cb)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(duration);
	ARG_UNUSED(done_cb);

	/* ed_scan not supported */

	return -ENOTSUP;
}

/* API implementation: configure */
static int efr32_configure(const struct device *dev,
			 enum ieee802154_config_type type,
			 const struct ieee802154_config *config)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(type);
	ARG_UNUSED(config);

	/* configure not supported */

	return -ENOTSUP;
}

/* API implementation: attr_get */
static int efr32_attr_get(const struct device *dev, enum ieee802154_attr attr,
			struct ieee802154_attr_value *value)
{
	/* Do nothing - mock up */
	return 0;
}

/* driver-allocated attribute memory - constant across all driver instances */
IEEE802154_DEFINE_PHY_SUPPORTED_CHANNELS(drv_attr, 11, 26);

/* IEEE802154 driver APIs structure */
static const struct ieee802154_radio_api efr32_radio_api = {
	.iface_api.init = efr32_iface_init,
	.get_capabilities = efr32_get_capabilities,
	.cca = efr32_cca,
	.set_channel = efr32_set_channel,
	.filter = efr32_filter,
	.set_txpower = efr32_set_txpower,
	.start = efr32_start,
	.stop = efr32_stop,
	.tx = efr32_tx,
	.ed_scan = efr32_ed_scan,
	.configure = efr32_configure,
	.attr_get = efr32_attr_get,
};

#if defined(CONFIG_NET_L2_IEEE802154)
#define L2 IEEE802154_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(IEEE802154_L2)
#define MTU 125
#elif defined(CONFIG_NET_L2_OPENTHREAD)
#define L2 OPENTHREAD_L2
#define L2_CTX_TYPE NET_L2_GET_CTX_TYPE(OPENTHREAD_L2)
#define MTU 1280
#endif


/* IEEE802154 driver registration */
#if defined(CONFIG_NET_L2_IEEE802154) || defined(CONFIG_NET_L2_OPENTHREAD)
NET_DEVICE_DT_INST_DEFINE(0, efr32_radio_init, NULL, &data, NULL,
			  CONFIG_IEEE802154_SILABS_RAIL_INIT_PRIO,
			  &efr32_radio_api, L2, L2_CTX_TYPE, MTU);
#else
DEVICE_DT_INST_DEFINE(0, ieee802154_gecko_init, NULL, &data, NULL,
		      POST_KERNEL, CONFIG_IEEE802154_SILABS_RAIL_INIT_PRIO,
		      &efr32_radio_api);
#endif
