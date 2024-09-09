/*
 * Copyright (c) 2021 Telink Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel/thread_stack.h>
#include "rail.h"
#include "rail_types.h"

#ifndef ZEPHYR_DRIVERS_IEEE802154_IEEE802154_ERF32_H
#define ZEPHYR_DRIVERS_IEEE802154_IEEE802154_ERF32_H

/* RAIL ieee802154 config defines */
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_AUTO_ACK_ENABLE 1
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_AUTO_ACK_RX_TRANSITION_STATE  RAIL_RF_STATE_RX
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_AUTO_ACK_TX_TRANSITION_STATE  RAIL_RF_STATE_RX
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_TIMING_IDLE_TO_TX_US  100
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_TIMING_IDLE_TO_RX_US  100
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_TIMING_RX_TO_TX_US  192
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_TIMING_TX_TO_RX_US  182
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_TIMING_RX_SEARCH_TIMEOUT_AFTER_IDLE_ENABLE 0
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_TIMING_RX_SEARCH_TIMEOUT_AFTER_IDLE_US  65535
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_TIMING_RX_SEARCH_TIMEOUT_AFTER_TX_ENABLE 0
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_TIMING_RX_SEARCH_TIMEOUT_AFTER_TX_US  65535
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_ACCEPT_BEACON_FRAME_ENABLE 1
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_ACCEPT_DATA_FRAME_ENABLE 1
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_ACCEPT_COMMAND_FRAME_ENABLE 1
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_PAN_COORDINATOR_ENABLE 0
#define SL_RAIL_UTIL_PROTOCOL_IEEE802154_2P4GHZ_DEFAULT_FRAME_PENDING_STATE 0

/* Address filter index define */
#define DEFAULT_ADDRESS_INDEX (0)


#define ERF32_IEEE_ADDRESS_SIZE					(8u)

/* Generic */
#define ERF32_RX_BUFFER_LENGTH					(512u)
#define ERF32_TX_BUFFER_LENGTH					(256u)
#define IEEE802154_MAX_LENGTH 					(256u)
#define EFR32_FCS_LENGTH	  					(2u)

/* TX power lookup table */

/* RAIL macro functions */
#define DBM_TO_DECI_DBM(n) (n*10u)

/* ieee802154 RAIL enums */
typedef enum e_efr32_state
{
    EFR32_STATE_RX,
    EFR32_STATE_TX,
    EFR32_STATE_DISABLED,
    EFR32_STATE_CCA,
    EFR32_STATE_SLEEP,
} efr32_state;

/* data structure */
struct efr32_data {
	struct  net_if *iface;
	uint8_t mac_addr[ERF32_IEEE_ADDRESS_SIZE];

	// RAIL handle struct
	RAIL_Handle_t         rail_handle;
	RAIL_RxPacketHandle_t packet_handle;
    
    // Handler configuration options
    RAIL_Config_t rail_config;

    /// RAIL schedule configuration
    RAILSched_Config_t rail_sched_config;

    // configures the channels
    const RAIL_ChannelConfig_t *rail_channel_config;
    uint8_t                    mChannelMin;
    uint8_t                    mChannelMax;

    // RAIL hanlde callback
    void (*aEventCallback)(RAIL_Handle_t railHandle, RAIL_Events_t events);
	
    // RAIL FIFO buffer
    uint8_t rx_buffer[ERF32_RX_BUFFER_LENGTH];
	uint8_t tx_buffer[ERF32_TX_BUFFER_LENGTH];

    // RAIL buffer size
    uint16_t rx_buffer_size;
    uint16_t tx_buffer_size;

	K_KERNEL_STACK_MEMBER(rx_stack, CONFIG_IEEE802154_EFR32_RX_STACK_SIZE);
    struct k_thread rx_thread;
	
	/* CCA complete sempahore. Unlocked when CCA is complete. */
    struct k_sem cca_wait;
    /* RX synchronization semaphore. Unlocked when frame has been
	 * received.
	 */
    struct k_sem rx_wait;
    /* TX synchronization semaphore. Unlocked when frame has been
	 * sent or CCA failed.
	 */
    struct k_sem tx_wait;

    /* TX FIFO queue
     *
     */
    struct k_fifo tx_fifo;
    

    /* TX result. Set to 1 on success, 0 otherwise. */
    bool tx_success;

    /* TX result, updated in radio transmit callbacks. */
	int tx_status;

    uint8_t channel;
};

#endif
