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
/* ######### efr32 driver structures ########### */
/* ERF32 data structure */
static struct efr32_data data;

/* Enum declaration */
//static efr32_state rail_state;

/* ######### RAIL config variables ########### */
// RAIL handler callback declaration
static void efr32_rail_cb(RAIL_Handle_t rail_handle, RAIL_Events_t a_events);

// RAIL config declartion */
static RAIL_Config_t rail_config = {
	.eventsCallback = &efr32_rail_cb,
};

/* ########## RAIL Tx defines ########## */
/* Maximum time to wait for RAIL to send the packet */
#define TX_PACKET_SENT_TIMEOUT         K_MSEC(100)

// CSMA configuration options
static const RAIL_CsmaConfig_t rail_csma_config = RAIL_CSMA_CONFIG_802_15_4_2003_2p4_GHz_OQPSK_CSMA;

/* Configuration structure for the LBT transmit algorithm */
static const RAIL_LbtConfig_t rail_lbt_config = {
	.lbtMinBoRand = 0,
	.lbtMaxBoRand = 10,
	.lbtTries = 5,
	.lbtThreshold = -75,
	.lbtBackoff = 320,   /* 20 symbols at 16 us/symbol */
	.lbtDuration = 128,  /* 8 symbols at 16 us/symbol */
	.lbtTimeout = 0,     /* No timeout */
};

/* ieee802154_radio_api callbacks */
/* Get MAC address */
static inline uint8_t *efr32_get_mac(struct device *net)
{
	uint64_t uniqueID = SYSTEM_GetUnique();
	uint8_t *mac = (uint8_t *)&uniqueID;
	printk("mac:%d\n",uniqueID);
	printk("mac:%d\n",*mac);
	return mac;
}

/* API implementation: iface_init */
static void efr32_iface_init(struct net_if *iface)
{
	struct device *dev = net_if_get_device(iface);
	struct efr32_data *efr32 = dev->data;
	uint8_t *mac = efr32_get_mac(dev);

	net_if_set_link_addr(iface, mac, ERF32_IEEE_ADDRESS_SIZE, NET_LINK_IEEE802154);
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
}

/* API implementation: tx */
static int efr32_tx(const struct device *dev,
		  			enum ieee802154_tx_mode mode,
		  			struct net_pkt *pkt,
		  			struct net_buf *frag)
{
#if 0
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
#endif
	struct efr32_data *efr32 = dev->data;
	uint8_t frame_len = frag->len + IEEE802154_FCS_LENGTH;
	RAIL_Status_t status;

	/* Write packet length at rail_tx_fifo[0] */
	if (RAIL_WriteTxFifo(efr32->rail_handle, &frame_len, 1, true) != 1) {
		LOG_DBG("Writing packet length to TxFifo failed");
		return -EIO;
	}
	/* Add packet payload */
	if (RAIL_WriteTxFifo(efr32->rail_handle, frag->data, frag->len, false)
	    != frag->len) {
		LOG_DBG("Writing packet payload to TxFifo failed");
		return -EIO;
	}

	switch (mode) {
	case IEEE802154_TX_MODE_DIRECT:
		status = RAIL_StartTx(efr32->rail_handle, efr32->channel,
				RAIL_TX_OPTIONS_DEFAULT, NULL);
		break;
	case IEEE802154_TX_MODE_CCA:
		status = RAIL_StartCcaLbtTx(efr32->rail_handle,
				efr32->channel,
				RAIL_TX_OPTIONS_DEFAULT,
				&rail_lbt_config,
				NULL);
		break;
	case IEEE802154_TX_MODE_CSMA_CA:
		status = RAIL_StartCcaCsmaTx(efr32->rail_handle,
				efr32->channel,
				RAIL_TX_OPTIONS_DEFAULT,
				&rail_csma_config, NULL);
		break;
	case IEEE802154_TX_MODE_TXTIME:
	case IEEE802154_TX_MODE_TXTIME_CCA:
	default:
		NET_ERR("TX mode %d not supported", mode);
		return -ENOTSUP;
	}

	if (status != RAIL_STATUS_NO_ERROR) {
		LOG_ERR("Failed to start Tx");
		return -EIO;
	}

	/* Wait for the callback from the radio driver. */
	if (0 != k_sem_take(&efr32->tx_wait, TX_PACKET_SENT_TIMEOUT)) {
		LOG_DBG("Failed to take tx_wait semaphore");
		return -EIO;
	}

	return efr32->tx_status;
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

/* ######### Interrupt set up ########### */
#define RAIL_IRQ_PRIO 0

static void ieee802154_gecko_irq_config(void)
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

/* ########## Init helper functions ########## */
// PA init function
static void railInitPa(RAIL_Handle_t myRailHandle)
{
  // Initialize the RAIL Tx power curves for all PAs on this chip
  const RAIL_TxPowerCurvesConfigAlt_t txPowerCurvesConfig = RAIL_TxPowerCurvesDcdc;
  if (RAIL_InitTxPowerCurvesAlt(&txPowerCurvesConfig) != RAIL_STATUS_NO_ERROR) {
    // Could not initialize transmit power curves so something is configured
    // wrong. Please fix and rebuild.
    while(1);
  }

  // Switch to the 2.4GHz HP PA powered off the 1.8V DCDC connection
  RAIL_TxPowerConfig_t railTxPowerConfig = {
    RAIL_TX_POWER_MODE_2P4GIG_HP, // 2.4GHz HP Power Amplifier mode
    1800,                         // 1.8V vPA voltage for DCDC connection
    10                            // Desired ramp time in us
  };
  if (RAIL_ConfigTxPower(myRailHandle, &railTxPowerConfig)
      != RAIL_STATUS_NO_ERROR) {
    // Error: The requested PA could not be selected. Fix your configuration
    // and try again.
    while(1);
  }

  // Set the output power to the maximum supported by this chip
  RAIL_SetTxPower(myRailHandle, 255);
}


__WEAK void sl_openthread_init(void)
{
    // Placeholder for enabling Silabs specific features available only through Simplicity Studio
}

void RAILCb_AssertFailed(RAIL_Handle_t railHandle, uint32_t errorCode){
        
	static const char* railErrorMessages[] = RAIL_ASSERT_ERROR_MESSAGES;
	const char *errorMessage ="Unknown";
 
  	// If this error code is within the range of known error messages then use       
	// the appropriate error message    
  	if(errorCode < (sizeof (railErrorMessages) / sizeof(char*))){
		errorMessage = railErrorMessages[errorCode];
	}
	LOG_DBG("Callback Error: %s", errorMessage);
         
	// Reset the chip since an assert is a fatal error
	NVIC_SystemReset();
}

static RAIL_Status_t efr32_iee802154_init(const struct device *dev){
    RAIL_Status_t status;
	struct efr32_data *efr32 = dev->data;
	efr32->rail_config = rail_config;

    // intializes openthread
	sl_openthread_init();
    
    efr32->rail_handle = RAIL_Init(&rail_config, NULL);

    // configures the IEEE 802.15.4 protocol based on the chosen protocol by user
	status = sl_rail_util_protocol_config(efr32->rail_handle,
	(sl_rail_util_protocol_type_t) SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ);
	if((RAIL_STATUS_NO_ERROR != status)){
		LOG_ERR("sl_rail_util_protocol_config failed, return value: %d", status);
        return status;
	}

    railInitPa(efr32->rail_handle);

    RAIL_StateTransitions_t transitions = {RAIL_RF_STATE_RX, RAIL_RF_STATE_RX};
    status = RAIL_SetRxTransitions(efr32->rail_handle, &transitions);
    status = RAIL_SetTxTransitions(efr32->rail_handle, &transitions);
    if((RAIL_STATUS_NO_ERROR != status)){
		LOG_ERR("RAIL_SetRxTransitions failed, return value: %d", status);
        return status;
	}

    /* Configure RAIL callbacks */
	RAIL_ConfigEvents(efr32->rail_handle, RAIL_EVENTS_ALL,
			  RAIL_EVENT_RX_PACKET_RECEIVED |
			  RAIL_EVENT_TX_PACKET_SENT |
			  RAIL_EVENT_TX_ABORTED |
			  RAIL_EVENT_TX_BLOCKED |
			  RAIL_EVENT_TX_UNDERFLOW |
			  RAIL_EVENT_TX_CHANNEL_BUSY |
			  RAIL_EVENT_CAL_NEEDED);

    RAIL_SetTxFifo(efr32->rail_handle, efr32->tx_buffer, 0, ERF32_TX_BUFFER_LENGTH);
    static const RAIL_DataConfig_t railDataConfig = {
        .txSource = TX_PACKET_DATA,
        .rxSource = RX_PACKET_DATA,
        .txMethod = FIFO_MODE, //doesn't do anything
        .rxMethod = FIFO_MODE,
    };
    status = RAIL_ConfigData(efr32->rail_handle, &railDataConfig);
    RAIL_SetTxFifoThreshold(efr32->rail_handle, ERF32_TX_BUFFER_LENGTH - ERF32_TX_BUFFER_LENGTH / 10);
    RAIL_SetRxFifoThreshold(efr32->rail_handle, ERF32_RX_BUFFER_LENGTH - ERF32_RX_BUFFER_LENGTH / 10);
    RAIL_ConfigEvents(efr32->rail_handle, RAIL_EVENT_TX_FIFO_ALMOST_EMPTY|RAIL_EVENT_RX_FIFO_ALMOST_FULL,
                        RAIL_EVENT_TX_FIFO_ALMOST_EMPTY|RAIL_EVENT_RX_FIFO_ALMOST_FULL);
    
    return status;
}

/* ########## Rx main thread ########## */
static void efr32_rx_thread(void *arg1, RAIL_RxPacketHandle_t arg2, void *arg3)
{
#if 0
    // Do nothing
    struct efr32_data *efr32 = (struct efr32_data *)arg1;

    while (1)
    {
        LOG_DBG("Waiting for frame");
        //k_sem_take(&efr32->rx_wait, K_FOREVER);

		LOG_DBG("Frame received!");

        RAIL_Idle(efr32->rail_handle, RAIL_IDLE_ABORT, true);
        RAIL_StartRx(efr32->rail_handle, efr32->channel, NULL);
		k_yield();
    }
#else
	struct efr32_data *efr32 = (struct efr32_data *)arg1;
	RAIL_RxPacketHandle_t packet_handle = (RAIL_RxPacketHandle_t) arg2;
	RAIL_RxPacketInfo_t *packet_info = (RAIL_RxPacketInfo_t *) arg3;
	struct net_pkt *pkt;

	while (1){
		pkt = NULL;
		LOG_DBG("Rx packet received");

		pkt = net_pkt_alloc_with_buffer(efr32->iface, packet_info->packetBytes,
						AF_UNSPEC, 0, K_FOREVER);
		if (pkt == NULL) {
			LOG_ERR("No net_pkt available");
			return;
		}

		/* Skip Frame Length field, 1 byte at index 0 */
		if (net_pkt_write(pkt, packet_info->firstPortionData + 1,
				packet_info->firstPortionBytes - 1)) {
			goto drop;
		}
		if (net_pkt_write(pkt, packet_info->lastPortionData,
				packet_info->packetBytes - packet_info->firstPortionBytes)) {
			goto drop;
		}

		/* Fill packet information */
		RAIL_RxPacketDetails_t packet_details = {
			.timeReceived = {
				.timePosition = RAIL_PACKET_TIME_AT_SYNC_END,
			},
		};
		RAIL_GetRxPacketDetails(efr32->rail_handle, packet_handle, &packet_details);

		net_pkt_set_ieee802154_lqi(pkt, packet_details.lqi);
		net_pkt_set_ieee802154_rssi(pkt, packet_details.rssi);

#if defined(CONFIG_NET_PKT_TIMESTAMP)
		struct net_ptp_time timestamp = {
			.second = packet_details.timeReceived.packetTime / USEC_PER_SEC,
			.nanosecond =
				(packet_details.timeReceived.packetTime % USEC_PER_SEC)
				* NSEC_PER_USEC
		};

		net_pkt_set_timestamp(pkt, &timestamp);
#endif

		if (net_recv_data(efr32->iface, pkt) < 0) {
			LOG_ERR("Packet dropped by NET stack");
			goto drop;
		}

		continue;

drop:
	net_pkt_unref(pkt);
	}
#endif
}

/* ########## RAIL general callback ########## */
static void efr32_rail_cb(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
	if (events & RAIL_EVENT_CAL_NEEDED) {
		RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
	}

	if (events & (RAIL_EVENT_TX_ABORTED |
		     RAIL_EVENT_TX_BLOCKED |
		     RAIL_EVENT_TX_UNDERFLOW |
		     RAIL_EVENT_TX_CHANNEL_BUSY)) {
		LOG_DBG("RAIL_Events_t 0x%llx", events);
		data.tx_status = -EIO;
		k_sem_give(&data.tx_wait);
	}

	if (events & RAIL_EVENT_TX_PACKET_SENT) {
		LOG_DBG("RAIL_Events_t: TX_PACKET_SENT");
		data.tx_status = 0;
		k_sem_give(&data.tx_wait);
	}

	if (events & RAIL_EVENT_RX_PACKET_RECEIVED) {
		RAIL_RxPacketInfo_t packet_info;
		RAIL_RxPacketHandle_t rx_packet_handle;

		LOG_DBG("RAIL_Events_t: RX_PACKET_RECEIVED");

		rx_packet_handle = RAIL_GetRxPacketInfo(
				data.rail_handle,
				RAIL_RX_PACKET_HANDLE_NEWEST,
				&packet_info);
		if ((rx_packet_handle != RAIL_RX_PACKET_HANDLE_INVALID) &&
		    (packet_info.packetStatus == RAIL_RX_PACKET_READY_SUCCESS)) {
			efr32_rx_thread(&data, rx_packet_handle, &packet_info);
		}
	}
}

/* ########## Init Driver START ########## */
#if 1
static int efr32_radio_init(const struct device *dev){
	struct efr32_data *efr32 = dev->data;

	// Semaphore init
	k_sem_init(&efr32->tx_wait, 0, 1);
	k_sem_init(&efr32->rx_wait, 0, 1);
    efr32_iee802154_init(dev);
	ieee802154_gecko_irq_config();
	LOG_DBG("Initialized");
	return 0;
}
#else
static int efr32_radio_init(const struct device *dev){
	struct efr32_data *efr32 = dev->data;

	// Semaphore init
	k_sem_init(&efr32->tx_wait, 0, 1);
	k_sem_init(&efr32->rx_wait, 0, 1);
    efr32_iee802154_init(dev);
	ieee802154_gecko_irq_config();

	k_thread_create(&efr32->rx_thread, efr32->rx_stack,
		800, efr32_rx_thread, efr32, NULL, NULL,
		K_PRIO_COOP(2), 0, K_NO_WAIT);

	k_thread_name_set(&efr32->rx_thread, "efr32_rx");
	
	LOG_DBG("Initialized");
	return 0;
}
#endif
/* ########## Init Driver END ########## */

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
