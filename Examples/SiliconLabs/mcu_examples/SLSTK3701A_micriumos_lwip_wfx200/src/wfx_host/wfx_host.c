/***************************************************************************//**
 * @file
 * @brief WFX FMAC driver host implementation
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include  <rtos_description.h>

#include "sl_wfx.h"
#include "sl_wfx_host_api.h"
#include "sl_wfx_bus.h"
#include "wfx_host_cfg.h"

#include "em_gpio.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_ldma.h"
#include "em_bus.h"
#if   defined(SLEXP802X)
#include "brd8022a_pds.h"
#include "brd8023a_pds.h"
#elif defined(EFM32GG11B820F2048GM64) //WGM160PX22KGA2
#include "brd4001a_pds.h"
#else
#error "WFX200 EXP board type must be specified"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <kernel/include/os.h>
#include <common/include/rtos_utils.h>
#include <common/include/rtos_err.h>

#if   defined(WF200_ALPHA_KEY)
#include "sl_wfx_wf200_A0.h"
#elif defined(WF200_BETA_KEY)
#include "sl_wfx_wf200_B0.h"
#elif defined(WF200_PROD_KEY)
#include "sl_wfx_wf200_C0.h"
#elif defined(WF200_DEV_KEY)
#include "sl_wfx_wf200.h"
#else
#error Must define either WF200_ALPHA_KEY/WF200_BETA_KEY/WF200_PROD_KEY/WF200_DEV_KEY
#endif
#include "wfx_task.h"
#include "udelay.h"
#include "demo_config.h"
#include "dhcp_server.h"
#include "wfx_host.h"

extern char event_log[];

OS_SEM    wf200_confirmation;
/// WiFi event flags
OS_FLAG_GRP  sl_wfx_event_group;
static OS_MUTEX wfx_mutex;

#define SL_WFX_EVENT_MAX_SIZE  512
#define SL_WFX_EVENT_LIST_SIZE 1
#define SL_WFX_MAX_STATIONS    8
#define SL_WFX_MAX_SCAN_RESULTS 50

scan_result_list_t scan_list[SL_WFX_MAX_SCAN_RESULTS];
static uint8_t scan_count = 0;
uint8_t scan_count_web = 0;

struct {
  uint32_t wf200_firmware_download_progress;
  MEM_DYN_POOL buf_pool;
  MEM_DYN_POOL buf_pool_rx_tx;
  int wf200_initialized;
  uint8_t waited_event_id;
  uint8_t posted_event_id;
} host_context;

#define BUFFER_SIZE 1024
#define TX_RX_BUFFER_SIZE 1550

#ifdef SL_WFX_USE_SDIO
#ifdef SLEEP_ENABLED
sl_status_t sl_wfx_host_enable_sdio (void);
sl_status_t sl_wfx_host_disable_sdio (void);
#endif
#endif

#ifdef SL_WFX_USE_SPI
#ifdef SLEEP_ENABLED
sl_status_t sl_wfx_host_enable_spi (void);
sl_status_t sl_wfx_host_disable_spi (void);
#endif
#endif

/* WF200 host callbacks */
void sl_wfx_connect_callback(uint8_t* mac, uint32_t status);
void sl_wfx_disconnect_callback(uint8_t* mac, uint16_t reason);
void sl_wfx_start_ap_callback(uint32_t status);
void sl_wfx_stop_ap_callback(void);
void sl_wfx_host_received_frame_callback(sl_wfx_received_ind_t* rx_buffer);
void sl_wfx_scan_result_callback(sl_wfx_scan_result_ind_body_t* scan_result);
void sl_wfx_scan_complete_callback(uint32_t status);
void sl_wfx_generic_status_callback(sl_wfx_generic_ind_t* frame);
void sl_wfx_client_connected_callback(uint8_t* mac);
void sl_wfx_ap_client_disconnected_callback(uint32_t status, uint8_t* mac);

/**************************************************************************//**
 * Set up memory pools for WFX host interface
 *****************************************************************************/
sl_status_t wfx_host_setup_memory_pools(void)
{
  RTOS_ERR err;
  Mem_DynPoolCreate("WFX Buffers",
                    &host_context.buf_pool,
                    DEF_NULL,
                    BUFFER_SIZE,
                    sizeof(CPU_INT32U),
                    0,
                    LIB_MEM_BLK_QTY_UNLIMITED,
                    &err);
  Mem_DynPoolCreate("WFX RX TX Buffers",
                    &host_context.buf_pool_rx_tx,
                    DEF_NULL,
                    TX_RX_BUFFER_SIZE,
                    sizeof(CPU_INT32U),
                    0,
                    LIB_MEM_BLK_QTY_UNLIMITED,
                    &err);
  if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE) {
#ifdef DEBUG
    printf("wfx_host_setup_memory_pools: unable to set up memory pools for wfx\n");
#endif
    return SL_STATUS_ALLOCATION_FAILED;
  }
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * WFX FMAC driver host interface initialization
 *****************************************************************************/
sl_status_t sl_wfx_host_init(void)
{
  RTOS_ERR err;
  UDELAY_Calibrate();
  OSFlagCreate(&sl_wfx_event_group, "wifi events", 0, &err);
  host_context.wf200_firmware_download_progress = 0;
  host_context.wf200_initialized = 0;
  OSSemCreate(&wf200_confirmation, "wf200 confirmation", 0, &err);
  if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE) {
    printf("OS error: sl_wfx_host_init");
  }
  OSMutexCreate(&wfx_mutex, "wfx host mutex",&err);
  if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE) {
      printf("OS error: sl_wfx_host_init");
  }
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get firmware data
 *****************************************************************************/
sl_status_t sl_wfx_host_get_firmware_data(const uint8_t** data, uint32_t data_size)
{
  *data = &sl_wfx_firmware[host_context.wf200_firmware_download_progress];
  host_context.wf200_firmware_download_progress += data_size;
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get firmware size
 *****************************************************************************/
sl_status_t sl_wfx_host_get_firmware_size(uint32_t* firmware_size)
{
  *firmware_size = sizeof(sl_wfx_firmware);
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get PDS data
 *****************************************************************************/
sl_status_t sl_wfx_host_get_pds_data(const char **pds_data, uint16_t index)
{
#ifdef SLEXP802X
  // Manage dynamically the PDS in function of the chip connected
  if (strncmp("WFM200", (char *)sl_wfx_context->wfx_opn, 6) == 0) {
    *pds_data = pds_table_brd8023a[index];
  } else {
    *pds_data = pds_table_brd8022a[index];
  }
#else
  *pds_data = pds_table_brd4001a[index];
#endif
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Get PDS size
 *****************************************************************************/
sl_status_t sl_wfx_host_get_pds_size(uint16_t *pds_size)
{
#ifdef SLEXP802X
  // Manage dynamically the PDS in function of the chip connected
  if (strncmp("WFM200", (char *)sl_wfx_context->wfx_opn, 6) == 0) {
    *pds_size = SL_WFX_ARRAY_COUNT(pds_table_brd8023a);
  } else {
    *pds_size = SL_WFX_ARRAY_COUNT(pds_table_brd8022a);
  }
#else
  *pds_size = SL_WFX_ARRAY_COUNT(pds_table_brd4001a);
#endif
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Deinit host interface
 *****************************************************************************/
sl_status_t sl_wfx_host_deinit(void)
{
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Allocate buffer
 *****************************************************************************/
sl_status_t sl_wfx_host_allocate_buffer(void **buffer,
                                        sl_wfx_buffer_type_t type,
                                        uint32_t buffer_size)
{
  RTOS_ERR err;
  if (type == SL_WFX_RX_FRAME_BUFFER || type == SL_WFX_TX_FRAME_BUFFER) {
    if (buffer_size > host_context.buf_pool_rx_tx.BlkSize) {
#ifdef DEBUG
      printf("Unable to allocate frame buffer\n");
      if (type == SL_WFX_RX_FRAME_BUFFER) {
        printf("RX ");
      } else {
        printf("TX ");
      }
      printf("type = %d, buffer_size requested = %d, mem pool blksize = %d\n", (int)type, (int)buffer_size, (int)host_context.buf_pool_rx_tx.BlkSize);
#endif
      return SL_STATUS_ALLOCATION_FAILED;
    }
    *buffer = Mem_DynPoolBlkGet(&host_context.buf_pool_rx_tx, &err);
    if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE) {
      printf("Mem_DynPoolBlkGet error\r\n");
    }
  } else {
    if (buffer_size > host_context.buf_pool.BlkSize) {
#ifdef DEBUG
      printf("Unable to allocate wf200 buffer\n");
      printf("type = %d, buffer_size requested = %d, mem pool blksize = %d\n", (int)type, (int)buffer_size, (int)host_context.buf_pool.BlkSize);
#endif
      return SL_STATUS_ALLOCATION_FAILED;
    }
    *buffer = Mem_DynPoolBlkGet(&host_context.buf_pool, &err);
    if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE) {
      printf("Mem_DynPoolBlkGet error control buffer\r\n");
    }
  }
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Free host buffer
 *****************************************************************************/
sl_status_t sl_wfx_host_free_buffer(void* buffer, sl_wfx_buffer_type_t type)
{
  RTOS_ERR err;
  if (type == SL_WFX_RX_FRAME_BUFFER || type == SL_WFX_TX_FRAME_BUFFER) {
    Mem_DynPoolBlkFree(&host_context.buf_pool_rx_tx, buffer, &err);
  } else {
    Mem_DynPoolBlkFree(&host_context.buf_pool, buffer, &err);
  }
  if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE) {
    printf("Mem_DynPoolBlkFree error \r\n");
  }
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Set reset pin low
 *****************************************************************************/
sl_status_t sl_wfx_host_hold_in_reset(void)
{
  GPIO_PinOutClear(WFX_HOST_CFG_RESET_PORT, WFX_HOST_CFG_RESET_PIN);
  host_context.wf200_initialized = 0;
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Set wakeup pin status
 *****************************************************************************/
sl_status_t sl_wfx_host_set_wake_up_pin(uint8_t state)
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_ATOMIC();
  if ( state > 0 ) {
#ifdef SLEEP_ENABLED
#ifdef SL_WFX_USE_SDIO
    sl_wfx_host_enable_sdio();
#endif
#ifdef SL_WFX_USE_SPI
    sl_wfx_host_enable_spi();
#endif
#endif
    GPIO_PinOutSet(WFX_HOST_CFG_WUP_PORT, WFX_HOST_CFG_WUP_PIN);
  } else {
    GPIO_PinOutClear(WFX_HOST_CFG_WUP_PORT, WFX_HOST_CFG_WUP_PIN);
#ifdef SLEEP_ENABLED
#ifdef SL_WFX_USE_SDIO
    sl_wfx_host_disable_sdio();
#endif
#ifdef SL_WFX_USE_SPI
    sl_wfx_host_disable_spi();
#endif
#endif
  }
  CORE_EXIT_ATOMIC();
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_reset_chip(void)
{
  RTOS_ERR err;
  // Pull it low for at least 1 ms to issue a reset sequence
  GPIO_PinOutClear(WFX_HOST_CFG_RESET_PORT, WFX_HOST_CFG_RESET_PIN);
  // Delay for 10ms
  OSTimeDly(10, OS_OPT_TIME_DLY, &err);

  // Hold pin high to get chip out of reset
  GPIO_PinOutSet(WFX_HOST_CFG_RESET_PORT, WFX_HOST_CFG_RESET_PIN);
  // Delay for 3ms
  OSTimeDly(3, OS_OPT_TIME_DLY, &err);
  host_context.wf200_initialized = 0;
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait_for_wake_up(void)
{
  RTOS_ERR err;
  UDELAY_Delay(1000);
  UDELAY_Delay(1000);
  OSFlagPost(&wfxtask_evts, WFX_EVENT_WAKE, OS_OPT_POST_FLAG_SET, &err);
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait(uint32_t wait_time)
{
  RTOS_ERR err;
  OSTimeDly(wait_time, OS_OPT_TIME_DLY, &err);
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_setup_waited_event(uint8_t event_id)
{
  host_context.waited_event_id = event_id;
  host_context.posted_event_id = 0;

  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait_for_confirmation(uint8_t confirmation_id, uint32_t timeout, void** event_payload_out)
{
  RTOS_ERR err;
  while (timeout > 0u) {
    timeout--;
    OSSemPend(&wf200_confirmation, 1, OS_OPT_PEND_BLOCKING, 0, &err);
    if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE) {
      if (confirmation_id == host_context.posted_event_id) {
        if ( event_payload_out != NULL ) {
          *event_payload_out = sl_wfx_context->event_payload_buffer;
        }

        return SL_STATUS_OK;
      }
    } else if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_TIMEOUT) {
      printf("OS error: sl_wfx_host_wait_for_confirmation\r\n");
    }
  }

  return SL_STATUS_TIMEOUT;
}

/**************************************************************************//**
 * Called when the driver needs to lock its access
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_lock(void)
{
  RTOS_ERR err;
  OSMutexPend(&wfx_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);
  if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE)
  {
    return SL_STATUS_FAIL;
  }
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Called when the driver needs to unlock its access
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_unlock(void)
{
  RTOS_ERR err;
  OSMutexPost(&wfx_mutex, OS_OPT_POST_NONE, &err);
  if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE)
  {
    return SL_STATUS_FAIL;
  }
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Called when the driver needs to post an event
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_post_event(sl_wfx_generic_message_t *network_rx_buffer)
{
  RTOS_ERR err;
  int i;
  switch (network_rx_buffer->header.id ) {
    /******** INDICATION ********/
    case SL_WFX_CONNECT_IND_ID:
    {
      sl_wfx_connect_ind_t* connect_indication = (sl_wfx_connect_ind_t*) network_rx_buffer;
      sl_wfx_connect_callback(connect_indication->body.mac, connect_indication->body.status);
      break;
    }
    case SL_WFX_DISCONNECT_IND_ID:
    {
      sl_wfx_disconnect_ind_t* disconnect_indication = (sl_wfx_disconnect_ind_t*) network_rx_buffer;
      sl_wfx_disconnect_callback(disconnect_indication->body.mac, disconnect_indication->body.reason);
      break;
    }
    case SL_WFX_START_AP_IND_ID:
    {
      sl_wfx_start_ap_ind_t* start_ap_indication = (sl_wfx_start_ap_ind_t*) network_rx_buffer;
      sl_wfx_start_ap_callback(start_ap_indication->body.status);
      break;
    }
    case SL_WFX_STOP_AP_IND_ID:
    {
      sl_wfx_stop_ap_callback();
      break;
    }
    case SL_WFX_RECEIVED_IND_ID:
    {
      sl_wfx_received_ind_t* ethernet_frame = (sl_wfx_received_ind_t*) network_rx_buffer;
      if ( ethernet_frame->body.frame_type == 0 ) {
        sl_wfx_host_received_frame_callback(ethernet_frame);
      }
      break;
    }
    case SL_WFX_SCAN_RESULT_IND_ID:
    {
      sl_wfx_scan_result_ind_t* scan_result = (sl_wfx_scan_result_ind_t*)network_rx_buffer;
      sl_wfx_scan_result_callback(&scan_result->body);
      break;
    }
    case SL_WFX_SCAN_COMPLETE_IND_ID:
    {
      sl_wfx_scan_complete_ind_t* scan_complete = (sl_wfx_scan_complete_ind_t*)network_rx_buffer;
      sl_wfx_scan_complete_callback(scan_complete->body.status);
      break;
    }
    case SL_WFX_AP_CLIENT_CONNECTED_IND_ID:
    {
      sl_wfx_ap_client_connected_ind_t* client_connected_indication = (sl_wfx_ap_client_connected_ind_t*) network_rx_buffer;
      sl_wfx_client_connected_callback(client_connected_indication->body.mac);
      break;
    }
    case SL_WFX_AP_CLIENT_REJECTED_IND_ID:
    {
      break;
    }
    case SL_WFX_AP_CLIENT_DISCONNECTED_IND_ID:
    {
      sl_wfx_ap_client_disconnected_ind_t* ap_client_disconnected_indication = (sl_wfx_ap_client_disconnected_ind_t*) network_rx_buffer;
      sl_wfx_ap_client_disconnected_callback(ap_client_disconnected_indication->body.reason, ap_client_disconnected_indication->body.mac);
      break;
    }
    case SL_WFX_GENERIC_IND_ID:
    {
      sl_wfx_generic_ind_t* generic_status = (sl_wfx_generic_ind_t*) network_rx_buffer;
      sl_wfx_generic_status_callback(generic_status);
      break;
    }
    case SL_WFX_EXCEPTION_IND_ID:
    {
      printf("Firmware Exception\r\n");
      sl_wfx_exception_ind_body_t *firmware_exception = (sl_wfx_exception_ind_body_t*)network_rx_buffer;
      printf("Exception data = \n");
      for (i = 0; i < SL_WFX_EXCEPTION_DATA_SIZE; i++) {
        printf("%X, ", firmware_exception->data[i]);
      }
      printf("\n");
      break;
    }
    case SL_WFX_ERROR_IND_ID:
    {
      printf("Firmware Error\r\n");
      sl_wfx_error_ind_body_t *firmware_error = (sl_wfx_error_ind_body_t*)network_rx_buffer;
      printf("Error type = %lu\r\n", firmware_error->type);
      break;
    }
    /******** CONFIRMATION ********/
    case SL_WFX_SEND_FRAME_CNF_ID:
    {
      break;
    }
  }

  if ( host_context.waited_event_id == network_rx_buffer->header.id ) {
    /* Post the event in the queue */
    memcpy(sl_wfx_context->event_payload_buffer, (void*)network_rx_buffer, network_rx_buffer->header.length);
    host_context.posted_event_id = network_rx_buffer->header.id;
    OSSemPost(&wf200_confirmation, OS_OPT_POST_1, &err);
  }

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Called when the driver needs to transmit a frame
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_transmit_frame(void* frame, uint32_t frame_len)
{
  return sl_wfx_data_write(frame, frame_len);
}

/**************************************************************************//**
 * Callback for individual scan result
 *****************************************************************************/
void sl_wfx_scan_result_callback(sl_wfx_scan_result_ind_body_t* scan_result)
{
  scan_count++;
  printf(
    "# %2d %2d  %03d %02X:%02X:%02X:%02X:%02X:%02X  %s",
    scan_count,
    scan_result->channel,
    ((int16_t)(scan_result->rcpi - 220) / 2),
    scan_result->mac[0], scan_result->mac[1],
    scan_result->mac[2], scan_result->mac[3],
    scan_result->mac[4], scan_result->mac[5],
    scan_result->ssid_def.ssid);
  /*Report one AP information*/
  printf("\r\n");
  if (scan_count <= SL_WFX_MAX_SCAN_RESULTS) {
    scan_list[scan_count - 1].ssid_def = scan_result->ssid_def;
	scan_list[scan_count - 1].channel = scan_result->channel;
	scan_list[scan_count - 1].security_mode = scan_result->security_mode;
	scan_list[scan_count - 1].rcpi = scan_result->rcpi;
	memcpy(scan_list[scan_count - 1].mac, scan_result->mac, 6);
  }
}

/**************************************************************************//**
 * Callback for scan complete
 *****************************************************************************/
void sl_wfx_scan_complete_callback(uint32_t status)
{
  RTOS_ERR err;
  scan_count_web = scan_count;
  scan_count = 0;
  OSFlagPost(&sl_wfx_event_group, SL_WFX_SCAN_COMPLETE, OS_OPT_POST_FLAG_SET, &err);
}

/**************************************************************************//**
 * Callback when station connects
 *****************************************************************************/
void sl_wfx_connect_callback(uint8_t* mac, uint32_t status)
{
  RTOS_ERR err;

  switch(status){
    case WFM_STATUS_SUCCESS:
    {
      printf("Connected\r\n");
      sl_wfx_context->state |= SL_WFX_STA_INTERFACE_CONNECTED;
      OSFlagPost(&sl_wfx_event_group, SL_WFX_CONNECT, OS_OPT_POST_FLAG_SET, &err);
      break;
    }
    case WFM_STATUS_NO_MATCHING_AP:
    {
      strcpy(event_log, "Connection failed, access point not found");
      printf("%s\r\n", event_log);
      break;
    }
    case WFM_STATUS_CONNECTION_ABORTED:
    {
      strcpy(event_log, "Connection aborted");
      printf("%s\r\n", event_log);
      break;
    }
    case WFM_STATUS_CONNECTION_TIMEOUT:
    {
      strcpy(event_log, "Connection timeout");
      printf("%s\r\n", event_log);
      break;
    }
    case WFM_STATUS_CONNECTION_REJECTED_BY_AP:
    {
      strcpy(event_log, "Connection rejected by the access point");
      printf("%s\r\n", event_log);
      break;
    }
    case WFM_STATUS_CONNECTION_AUTH_FAILURE:
    {
      strcpy(event_log, "Connection authentication failure");
      printf("%s\r\n", event_log);
      break;
    }
    default:
    {
      strcpy(event_log, "Connection attempt error");
      printf("%s\r\n", event_log);
    }
  }
}

/**************************************************************************//**
 * Callback for station disconnect
 *****************************************************************************/
void sl_wfx_disconnect_callback(uint8_t* mac, uint16_t reason)
{
  RTOS_ERR err;
  printf("Disconnected %d\r\n", reason);
  sl_wfx_context->state &= ~SL_WFX_STA_INTERFACE_CONNECTED;
  OSFlagPost(&sl_wfx_event_group, SL_WFX_DISCONNECT, OS_OPT_POST_FLAG_SET, &err);
}

/**************************************************************************//**
 * Callback for AP started
 *****************************************************************************/
void sl_wfx_start_ap_callback(uint32_t status)
{
  RTOS_ERR err;
  if (status == 0) {
    printf("AP started\r\n");
    printf("Join the AP with SSID: %s\r\n", softap_ssid);
    sl_wfx_context->state |= SL_WFX_AP_INTERFACE_UP;
    OSFlagPost(&sl_wfx_event_group, SL_WFX_START_AP, OS_OPT_POST_FLAG_SET, &err);
  } else {
    printf("AP start failed\r\n");
    strcpy(event_log, "AP start failed");
  }
}

/**************************************************************************//**
 * Callback for AP stopped
 *****************************************************************************/
void sl_wfx_stop_ap_callback(void)
{
  RTOS_ERR err;
  dhcpserver_clear_stored_mac ();
  printf("SoftAP stopped\r\n");
  sl_wfx_context->state &= ~SL_WFX_AP_INTERFACE_UP;
  OSFlagPost(&sl_wfx_event_group, SL_WFX_STOP_AP, OS_OPT_POST_FLAG_SET, &err);
}

/**************************************************************************//**
 * Callback for client connect to AP
 *****************************************************************************/
void sl_wfx_client_connected_callback(uint8_t* mac)
{
  printf("Client connected, MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
         mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  printf("Open a web browser and go to http://%d.%d.%d.%d\r\n",
         ap_ip_addr0, ap_ip_addr1, ap_ip_addr2, ap_ip_addr3);
}

/**************************************************************************//**
 * Callback for client rejected from AP
 *****************************************************************************/
void sl_wfx_ap_client_rejected_callback(uint32_t status, uint8_t* mac)
{
  struct eth_addr mac_addr;
  memcpy(&mac_addr, mac, SL_WFX_BSSID_SIZE);
  dhcpserver_remove_mac(&mac_addr);
  printf("Client rejected, reason: %d, MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
         (int)status, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/**************************************************************************//**
 * Callback for AP client disconnect
 *****************************************************************************/
void sl_wfx_ap_client_disconnected_callback(uint32_t status, uint8_t* mac)
{
  struct eth_addr mac_addr;
  memcpy(&mac_addr, mac, SL_WFX_BSSID_SIZE);
  dhcpserver_remove_mac(&mac_addr);
  printf("Client disconnected, reason: %d, MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
         (int)status, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

/**************************************************************************//**
 * Callback for generic status received
 *****************************************************************************/
void sl_wfx_generic_status_callback(sl_wfx_generic_ind_t* frame)
{
  printf("Generic status received\r\n");
}

/**************************************************************************//**
 * Called when the driver is considering putting the WFx in
 * sleep mode
 * @returns SL_WIFI_SLEEP_GRANTED to let the WFx go to sleep,
 * SL_WIFI_SLEEP_NOT_GRANTED otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_sleep_grant(sl_wfx_host_bus_transfer_type_t type,
                                    sl_wfx_register_address_t address,
                                    uint32_t length)
{
  return SL_STATUS_WIFI_SLEEP_GRANTED;
}

#if SL_WFX_DEBUG_MASK
void sl_wfx_host_log(const char *string, ...)
{
  va_list valist;

  va_start(valist, string);
  vprintf(string, valist);
  va_end(valist);
}
#endif
