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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

#include "em_gpio.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_ldma.h"
#include "em_bus.h"
#include "em_core.h"

#include "cmsis_os.h"

#include "demo_config.h"
#include "dhcp_server.h"

#include "sl_wfx.h"
#include "sl_wfx_host_api.h"
#include "bus/sl_wfx_bus.h"
#include "sl_wfx_host.h"
#include "sl_wfx_host_cfg.h"
#include "sl_wfx_host_events.h"
#include "sl_wfx_task.h"

#include "sl_wfx_wf200_C0.h"

#if   defined(SLEXP802X)
#include "brd8022a_pds.h"
#include "brd8023a_pds.h"
#elif defined(EFM32GG11B820F2048GM64) //WGM160PX22KGA2
#include "brd4001a_pds.h"
#else
#error "WFX200 EXP board type must be specified"
#endif

QueueHandle_t eventQueue;
osSemaphoreId wfx_wakeup_sem;
extern SemaphoreHandle_t s_xDriverSemaphore;
extern char event_log[];

#define SL_WFX_EVENT_MAX_SIZE     512
#define SL_WFX_EVENT_LIST_SIZE    1
#define SL_WFX_MAX_STATIONS       8
#define SL_WFX_MAX_SCAN_RESULTS   50

scan_result_list_t scan_list[SL_WFX_MAX_SCAN_RESULTS];
uint8_t scan_count = 0;
uint8_t scan_count_t = 0;
uint8_t scan_count_web = 0;

struct {
  uint32_t wf200_firmware_download_progress;
  int wf200_initialized;
  uint8_t waited_event_id;
  uint8_t posted_event_id;
} host_context;

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
void sl_wfx_ap_client_rejected_callback(uint32_t status, uint8_t* mac);

/**************************************************************************//**
 * WFX FMAC driver host interface initialization
 *****************************************************************************/
sl_status_t sl_wfx_host_init(void)
{
  host_context.wf200_firmware_download_progress = 0;
  eventQueue = xQueueCreate(SL_WFX_EVENT_LIST_SIZE, sizeof(uint8_t));
  wfx_wakeup_sem = xSemaphoreCreateBinary();
  return SL_STATUS_OK;
}

#if SL_WFX_DEBUG_MASK
/**************************************************************************//**
 * Host debug output
 *****************************************************************************/
void sl_wfx_host_log(const char *string, ...)
{
  va_list args;
  va_start(args, string);
  vprintf(string, args);
  va_end(args);
}
#endif

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
  *buffer = pvPortMalloc( buffer_size );
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Free host buffer
 *****************************************************************************/
sl_status_t sl_wfx_host_free_buffer(void* buffer, sl_wfx_buffer_type_t type)
{
  vPortFree( buffer );
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Set reset pin low
 *****************************************************************************/
sl_status_t sl_wfx_host_hold_in_reset(void)
{
  GPIO_PinOutClear(SL_WFX_HOST_CFG_RESET_PORT, SL_WFX_HOST_CFG_RESET_PIN);
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
    GPIO_PinOutSet(SL_WFX_HOST_CFG_WUP_PORT, SL_WFX_HOST_CFG_WUP_PIN);
  } else {
    GPIO_PinOutClear(SL_WFX_HOST_CFG_WUP_PORT, SL_WFX_HOST_CFG_WUP_PIN);
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
  // Pull it low for at least 1 ms to issue a reset sequence
  GPIO_PinOutClear(SL_WFX_HOST_CFG_RESET_PORT, SL_WFX_HOST_CFG_RESET_PIN);

  // Delay for 10ms
  osDelay(10);

  // Hold pin high to get chip out of reset
  GPIO_PinOutSet(SL_WFX_HOST_CFG_RESET_PORT, SL_WFX_HOST_CFG_RESET_PIN);

  // Delay for 3ms
  osDelay(3);

  host_context.wf200_initialized = 0;
  return SL_STATUS_OK;
}


sl_status_t sl_wfx_host_wait_for_wake_up(void)
{
  xSemaphoreTake(wfx_wakeup_sem, 0);
  xSemaphoreTake(wfx_wakeup_sem, 3/portTICK_PERIOD_MS);

  return SL_STATUS_OK;
}
sl_status_t sl_wfx_host_wait(uint32_t wait_time)
{
  osDelay(wait_time);
  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_setup_waited_event(uint8_t event_id)
{
  host_context.waited_event_id = event_id;
  host_context.posted_event_id = 0;

  return SL_STATUS_OK;
}

sl_status_t sl_wfx_host_wait_for_confirmation(uint8_t confirmation_id,
                                              uint32_t timeout,
                                              void** event_payload_out)
{
  uint8_t posted_event_id;
  for(uint32_t i = 0; i < timeout; i++)
  {
    /* Wait for an event posted by the function sl_wfx_host_post_event() */
    if(xQueueReceive(eventQueue, &posted_event_id, 1) == pdTRUE)
    {
      /* Once a message is received, check if it is the expected ID */
      if (confirmation_id == posted_event_id)
      {
        /* Pass the confirmation reply and return*/
        if ( event_payload_out != NULL )
        {
          *event_payload_out = sl_wfx_context->event_payload_buffer;
        }
        return SL_STATUS_OK;
      }
    }
  }
  /* The wait for the confirmation timed out, return */
  return SL_STATUS_TIMEOUT;
}

/**************************************************************************//**
 * Called when the driver needs to lock its access
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_lock(void)
{

  sl_status_t status= SL_STATUS_OK;

  if (xSemaphoreTake(s_xDriverSemaphore, 500) == pdTRUE) {
    status = SL_STATUS_OK;
  } else {
    printf("Wi-Fi driver mutex timeout\r\n");
    status = SL_STATUS_TIMEOUT;
  }

  return status;
}

/**************************************************************************//**
 * Called when the driver needs to unlock its access
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_unlock(void)
{
  xSemaphoreGive(s_xDriverSemaphore);

  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Called when the driver needs to post an event
 *
 * @returns Returns SL_STATUS_OK if successful, SL_STATUS_FAIL otherwise
 *****************************************************************************/
sl_status_t sl_wfx_host_post_event(sl_wfx_generic_message_t *event_payload)
{

  switch(event_payload->header.id){
    /******** INDICATION ********/
    case SL_WFX_CONNECT_IND_ID:
    {
      sl_wfx_connect_ind_t* connect_indication = (sl_wfx_connect_ind_t*) event_payload;
      sl_wfx_connect_callback(connect_indication->body.mac, connect_indication->body.status);
      break;
    }
    case SL_WFX_DISCONNECT_IND_ID:
    {
      sl_wfx_disconnect_ind_t* disconnect_indication = (sl_wfx_disconnect_ind_t*) event_payload;
      sl_wfx_disconnect_callback(disconnect_indication->body.mac, disconnect_indication->body.reason);
      break;
    }
    case SL_WFX_START_AP_IND_ID:
    {
      sl_wfx_start_ap_ind_t* start_ap_indication = (sl_wfx_start_ap_ind_t*) event_payload;
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
      sl_wfx_received_ind_t* ethernet_frame = (sl_wfx_received_ind_t*) event_payload;
      if (ethernet_frame->body.frame_type == 0) {
        sl_wfx_host_received_frame_callback( ethernet_frame );
      }
      break;
    }
    case SL_WFX_SCAN_RESULT_IND_ID:
    {
      sl_wfx_scan_result_ind_t* scan_result = (sl_wfx_scan_result_ind_t*) event_payload;
      sl_wfx_scan_result_callback(&scan_result->body);
      break;
    }
    case SL_WFX_SCAN_COMPLETE_IND_ID:
    {
      sl_wfx_scan_complete_ind_t* scan_complete = (sl_wfx_scan_complete_ind_t*) event_payload;
      sl_wfx_scan_complete_callback(scan_complete->body.status);
      break;
    }
    case SL_WFX_AP_CLIENT_CONNECTED_IND_ID:
    {
      sl_wfx_ap_client_connected_ind_t* client_connected_indication = (sl_wfx_ap_client_connected_ind_t*) event_payload;
      sl_wfx_client_connected_callback(client_connected_indication->body.mac);
      break;
    }
    case SL_WFX_AP_CLIENT_REJECTED_IND_ID:
    {
      sl_wfx_ap_client_rejected_ind_t* ap_client_rejected_indication = (sl_wfx_ap_client_rejected_ind_t*) event_payload;
      sl_wfx_ap_client_rejected_callback(ap_client_rejected_indication->body.reason, ap_client_rejected_indication->body.mac);
      break;
    }
    case SL_WFX_AP_CLIENT_DISCONNECTED_IND_ID:
    {
      sl_wfx_ap_client_disconnected_ind_t* ap_client_disconnected_indication = (sl_wfx_ap_client_disconnected_ind_t*) event_payload;
      sl_wfx_ap_client_disconnected_callback(ap_client_disconnected_indication->body.reason, ap_client_disconnected_indication->body.mac);
      break;
    }
#ifdef SL_WFX_USE_SECURE_LINK
    case SL_WFX_SECURELINK_EXCHANGE_PUB_KEYS_IND_ID:
    {
      if(host_context.waited_event_id != SL_WFX_SECURELINK_EXCHANGE_PUB_KEYS_IND_ID) {
        memcpy((void*)&sl_wfx_context->secure_link_exchange_ind,(void*)event_payload, event_payload->header.length);
      }
      break;
    }
#endif
    case SL_WFX_GENERIC_IND_ID:
    {
      sl_wfx_generic_ind_t* generic_status = (sl_wfx_generic_ind_t*) event_payload;
      sl_wfx_generic_status_callback(generic_status);
      break;
    }
    case SL_WFX_EXCEPTION_IND_ID:
    {
      sl_wfx_exception_ind_t *firmware_exception = (sl_wfx_exception_ind_t*) event_payload;
      uint8_t *exception_tmp = (uint8_t *) firmware_exception;
      printf("firmware exception %lu\r\n", firmware_exception->body.reason);
      for (uint16_t i = 0; i < firmware_exception->header.length; i += 16) {
        printf("hif: %.8x:", i);
        for (uint8_t j = 0; (j < 16) && ((i + j) < firmware_exception->header.length); j ++) {
          printf(" %.2x", *exception_tmp);
          exception_tmp++;
        }
        printf("\r\n");
      }
      break;
    }
    case SL_WFX_ERROR_IND_ID:
    {
      printf("Firmware Error\r\n");
      sl_wfx_error_ind_t *firmware_error = (sl_wfx_error_ind_t*)event_payload;
      printf ("Error type = %lu\r\n",firmware_error->body.type);
      break;
    }
  }

  if(host_context.waited_event_id == event_payload->header.id) {
    if(event_payload->header.length < SL_WFX_EVENT_MAX_SIZE) {
      /* Post the event in the queue */
      memcpy( sl_wfx_context->event_payload_buffer,
              (void*) event_payload,
              event_payload->header.length );
      xQueueOverwrite(eventQueue, (void *) &event_payload->header.id);
    }
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

  (void)(status);
  scan_count_web = scan_count;
  scan_count = 0;
  xEventGroupSetBits(sl_wfx_event_group, SL_WFX_SCAN_COMPLETE);
}

/**************************************************************************//**
 * Callback when station connects
 *****************************************************************************/
void sl_wfx_connect_callback(uint8_t* mac, uint32_t status)
{
  (void)(mac);
  switch(status){
    case WFM_STATUS_SUCCESS:
    {
      printf("Connected\r\n");
      sl_wfx_context->state |= SL_WFX_STA_INTERFACE_CONNECTED;
      xEventGroupSetBits(sl_wfx_event_group, SL_WFX_CONNECT);
      break;
    }
    case WFM_STATUS_NO_MATCHING_AP:
    {
      strcpy(event_log, "Connection failed, access point not found");
      printf(event_log);
      printf("\r\n");
      break;
    }
    case WFM_STATUS_CONNECTION_ABORTED:
    {
      strcpy(event_log, "Connection aborted");
      printf(event_log);
      printf("\r\n");
      break;
    }
    case WFM_STATUS_CONNECTION_TIMEOUT:
    {
      strcpy(event_log, "Connection timeout");
      printf(event_log);
      printf("\r\n");
      break;
    }
    case WFM_STATUS_CONNECTION_REJECTED_BY_AP:
    {
      strcpy(event_log, "Connection rejected by the access point");
      printf(event_log);
      printf("\r\n");
      break;
    }
    case WFM_STATUS_CONNECTION_AUTH_FAILURE:
    {
      strcpy(event_log, "Connection authentication failure");
      printf(event_log);
      printf("\r\n");
      break;
    }
    default:
    {
      strcpy(event_log, "Connection attempt error");
      printf(event_log);
      printf("\r\n");
    }
  }
}

/**************************************************************************//**
 * Callback for station disconnect
 *****************************************************************************/
void sl_wfx_disconnect_callback(uint8_t* mac, uint16_t reason)
{
  (void)(mac);
  printf("Disconnected %d\r\n", reason);
  sl_wfx_context->state &= ~SL_WFX_STA_INTERFACE_CONNECTED;
  xEventGroupSetBits(sl_wfx_event_group, SL_WFX_DISCONNECT);
}

/**************************************************************************//**
 * Callback for AP started
 *****************************************************************************/
void sl_wfx_start_ap_callback(uint32_t status)
{
  if (status == 0) {
    printf("AP started\r\n");
    printf("Join the AP with SSID: %s\r\n", softap_ssid);
    sl_wfx_context->state |= SL_WFX_AP_INTERFACE_UP;
    xEventGroupSetBits(sl_wfx_event_group, SL_WFX_START_AP);
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
  dhcpserver_clear_stored_mac ();
  printf("SoftAP stopped\r\n");
  sl_wfx_context->state &= ~SL_WFX_AP_INTERFACE_UP;
  xEventGroupSetBits(sl_wfx_event_group, SL_WFX_STOP_AP);
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
  (void)(frame);
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
  (void)(type);
  (void)(address);
  (void)(length);

  return SL_STATUS_WIFI_SLEEP_GRANTED;
}

