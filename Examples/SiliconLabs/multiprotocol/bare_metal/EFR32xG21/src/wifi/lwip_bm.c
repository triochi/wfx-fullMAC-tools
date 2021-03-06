/***************************************************************************//**
 * @file
 * @brief LwIP task and related functions
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
#include <string.h>
#include <stdbool.h>

#include "bsp.h"

#include "interface.h"
#include "bluetooth_app.h"

#include "sl_wfx.h"

// LwIP includes.
#include "ethernetif.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/tcpip.h"
#include "lwip/apps/httpd.h"
#include "lwip/timeouts.h"
#include "netif/ethernet.h"
#include "lwip_bm.h"
#include "dhcp_server.h"
#include "sl_wfx_host.h"
#ifdef LWIP_IPERF_SERVER
#include "lwip/ip_addr.h"
#include "lwip/apps/lwiperf.h"
#endif

#include "lwip/stats.h"

#define HTTP_MAX_ITEM_LENGTH      100

uint8_t print_addr = 0;

/// Enable or disable DHCP client for station.
static uint8_t use_dhcp_client = USE_DHCP_CLIENT_DEFAULT;

static int netif_config(void);

/// LwIP station network interface structure.
struct netif sta_netif;
/// LwIP AP network interface structure.
struct netif ap_netif;

sl_wfx_context_t wifi;
char wlan_ssid[32]                        = WLAN_SSID_DEFAULT;
char wlan_passkey[64]                     = WLAN_PASSKEY_DEFAULT;
sl_wfx_security_mode_t wlan_security      = WLAN_SECURITY_DEFAULT;
char softap_ssid[32]                      = SOFTAP_SSID_DEFAULT;
char softap_passkey[64]                   = SOFTAP_PASSKEY_DEFAULT;
sl_wfx_security_mode_t softap_security    = SOFTAP_SECURITY_DEFAULT;
uint8_t softap_channel                    = SOFTAP_CHANNEL_DEFAULT;

/// Station IP address octet 0.
uint8_t sta_ip_addr0 = STA_IP_ADDR0_DEFAULT;
/// Station IP address octet 1.
uint8_t sta_ip_addr1 = STA_IP_ADDR1_DEFAULT;
/// Station IP address octet 2.
uint8_t sta_ip_addr2 = STA_IP_ADDR2_DEFAULT;
/// Station IP address octet 3.
uint8_t sta_ip_addr3 = STA_IP_ADDR3_DEFAULT;

/// Station net mask octet 0.
uint8_t sta_netmask_addr0 = STA_NETMASK_ADDR0_DEFAULT;
/// Station net mask octet 1.
uint8_t sta_netmask_addr1 = STA_NETMASK_ADDR1_DEFAULT;
/// Station net mask octet 2.
uint8_t sta_netmask_addr2 = STA_NETMASK_ADDR2_DEFAULT;
/// Station net mask octet 3.
uint8_t sta_netmask_addr3 = STA_NETMASK_ADDR3_DEFAULT;

/// Station gateway IP octet 0.
uint8_t sta_gw_addr0 = STA_GW_ADDR0_DEFAULT;
/// Station gateway IP octet 1.
uint8_t sta_gw_addr1 = STA_GW_ADDR1_DEFAULT;
/// Station gateway IP octet 2.
uint8_t sta_gw_addr2 = STA_GW_ADDR2_DEFAULT;
/// Station gateway IP octet 3.
uint8_t sta_gw_addr3 = STA_GW_ADDR3_DEFAULT;

/// AP IP address octet 0.
uint8_t ap_ip_addr0 = AP_IP_ADDR0_DEFAULT;
/// AP IP address octet 1.
uint8_t ap_ip_addr1 = AP_IP_ADDR1_DEFAULT;
/// AP IP address octet 2.
uint8_t ap_ip_addr2 = AP_IP_ADDR2_DEFAULT;
/// AP IP address octet 3.
uint8_t ap_ip_addr3 = AP_IP_ADDR3_DEFAULT;

/// AP net mask octet 0.
uint8_t ap_netmask_addr0 = AP_NETMASK_ADDR0_DEFAULT;
/// AP net mask octet 1.
uint8_t ap_netmask_addr1 = AP_NETMASK_ADDR1_DEFAULT;
/// AP net mask octet 2.
uint8_t ap_netmask_addr2 = AP_NETMASK_ADDR2_DEFAULT;
/// AP net mask octet 3.
uint8_t ap_netmask_addr3 = AP_NETMASK_ADDR3_DEFAULT;

/// AP gateway IP octet 0.
uint8_t ap_gw_addr0 = AP_GW_ADDR0_DEFAULT;
/// AP gateway IP octet 1.
uint8_t ap_gw_addr1 = AP_GW_ADDR1_DEFAULT;
/// AP gateway IP octet 2.
uint8_t ap_gw_addr2 = AP_GW_ADDR2_DEFAULT;
/// AP gateway IP octet 3.
uint8_t ap_gw_addr3 = AP_GW_ADDR3_DEFAULT;

/// Memory to store an event to display in the web page
char event_log[50];
/// AP channel
static uint8_t ap_channel;
/// AP mac address.
static struct eth_addr ap_mac;
#ifdef LWIP_HTTP_SERVER
/// SSI handler
static uint16_t ssi_handler(int index, char *pc_insert, int insert_len);
static char from_hex(char ch);
static sl_status_t url_decode(char *str);
/// CGI handlers
static const char *toggle_leds_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);
static const char *get_leds_state_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);
static const char *get_interface_states_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);
static const char *start_ble_advertising_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);
static const char *stop_ble_advertising_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);
static const char *disconnect_ble_master_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);
static const char *start_softap_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);
static const char *stop_softap_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);
static const char *disconnect_client_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);
static const char *start_station_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);
static const char *stop_station_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);
static const char *start_scan_cgi_handler(int index, int num_params, char *pc_param[], char *pc_value[]);

/// Table of the CGI names and handlers.
static const tCGI cgi_table[] = {
  {"/toggle_leds.cgi", toggle_leds_cgi_handler},
  {"/get_leds_state.cgi", get_leds_state_cgi_handler},
  {"/get_interface_states.cgi", get_interface_states_cgi_handler},
  {"/start_ble_beacon.cgi", start_ble_advertising_cgi_handler},
  {"/stop_ble_beacon.cgi", stop_ble_advertising_cgi_handler},
  {"/disconnect_master.cgi", disconnect_ble_master_cgi_handler},
  {"/start_softap.cgi", start_softap_cgi_handler},
  {"/stop_softap.cgi", stop_softap_cgi_handler},
  {"/disconnect_client.cgi", disconnect_client_cgi_handler},
  {"/start_station.cgi", start_station_cgi_handler},
  {"/stop_station.cgi", stop_station_cgi_handler},
  {"/start_scan.cgi", start_scan_cgi_handler}
};

static char const *ssi_tags[] = {
  "leds_state",
  "scan_list",
  "softap_state",
  "softap_ssid",
  "softap_ip",
  "softap_mac",
  "softap_secu",
  "softap_channel",
  "clients_list",
  "station_state",
  "station_ip",
  "station_mac",
  "ap_ssid",
  "ap_mac",
  "ap_secu",
  "ap_channel",
  "event",
  "ble_state",
  "ble_name",
  "ble_mac",
  "master_list"
};

/***************************************************************************//**
 * @brief Initialize the web server CGI handlers
 ******************************************************************************/
static void cgi_ssi_init(void)
{
  //give the CGI and SSI tables to the HTTP server
  http_set_cgi_handlers(cgi_table, sizeof(cgi_table)/sizeof (tCGI));
  http_set_ssi_handler(ssi_handler, ssi_tags, sizeof(ssi_tags)/sizeof(char *));
}

/***************************************************************************//**
 * @brief Web server CGI handler for controlling the LEDs.
 ******************************************************************************/
static const char *toggle_leds_cgi_handler (int index,
                                            int num_params,
                                            char *pc_param[],
                                            char *pc_value[])
{
  interface_light_toggle(interface_light_trigger_src_wifi,
                         (interface_mac_t *)wifi.mac_addr_1.octet);

  return "/empty.json";
}

/***************************************************************************//**
 * @brief Web server CGI handler to get the LED states.
 ******************************************************************************/
static const char *get_leds_state_cgi_handler (int index,
                                               int num_params,
                                               char *pc_param[],
                                               char *pc_value[])
{
  return "/leds_state.json";
}

/***************************************************************************//**
 * @brief Web server CGI handler to get the interface states.
 ******************************************************************************/
static const char *get_interface_states_cgi_handler(int index,
                                                    int num_params,
                                                    char *pc_param[],
                                                    char *pc_value[])
{
  return "/interface_states.json";
}

/***************************************************************************//**
 * @brief Web server CGI handler to start the BLE advertising.
 ******************************************************************************/
static const char *start_ble_advertising_cgi_handler(int index,
                                                     int num_params,
                                                     char *pc_param[],
                                                     char *pc_value[])
{
  bluetooth_app_start_advertising();
  return "/empty.json";
}

/***************************************************************************//**
 * @brief Web server CGI handler to stop the BLE advertising.
 ******************************************************************************/
static const char *stop_ble_advertising_cgi_handler(int index,
                                                    int num_params,
                                                    char *pc_param[],
                                                    char *pc_value[])
{
  bluetooth_app_stop_advertising();
  return "/empty.json";
}

/***************************************************************************//**
 * @brief Web server CGI handler to disconnect the BLE master.
 ******************************************************************************/
static const char *disconnect_ble_master_cgi_handler(int index,
                                                     int num_params,
                                                     char *pc_param[],
                                                     char *pc_value[])
{
  printf("Disconnect BLE\r\n");
  bluetooth_app_disconnect_master();
  return "/empty.json";
}

/***************************************************************************//**
 * @brief Web server CGI handler to start the station interface.
 ******************************************************************************/
static const char *start_station_cgi_handler(int index,
                                             int num_params,
                                             char *pc_param[],
                                             char *pc_value[])
{
  sl_status_t status;
  int ssid_length = 0, passkey_length = 0;

  if (num_params == 3) {
    if (strcmp(pc_param[0], "ssid") == 0)
    {
      url_decode(pc_value[0]);
      ssid_length = strlen(pc_value[0]);
      memset(wlan_ssid, 0, 32);
      strncpy(wlan_ssid, pc_value[0], ssid_length);
    }
    if (strcmp(pc_param[1], "pwd") == 0)
    {
      url_decode(pc_value[1]);
      passkey_length = strlen(pc_value[1]);
      memset(wlan_passkey, 0, 64);
      strncpy(wlan_passkey, pc_value[1], passkey_length);
    }
    if (strcmp(pc_param[2], "secu") == 0)
    {
      url_decode(pc_value[2]);
      if ((strcmp(pc_value[2], "WPA2") == 0) || (strcmp(pc_value[2], "WPA") == 0))
      {
        wlan_security = WFM_SECURITY_MODE_WPA2_WPA1_PSK;
      }else if (strcmp(pc_value[2], "WEP") == 0)
      {
        wlan_security = WFM_SECURITY_MODE_WEP;
      }else if (strcmp(pc_value[2], "OPEN") == 0)
      {
        wlan_security = WFM_SECURITY_MODE_OPEN;
      }
    }
    if (!(wifi.state & SL_WFX_STA_INTERFACE_CONNECTED))
    {
      status = sl_wfx_send_join_command((uint8_t*) wlan_ssid, ssid_length,
                                        NULL, 0, wlan_security, 0, 0,
                                        (uint8_t*) wlan_passkey, passkey_length,
                                        NULL, 0);
      if (status != SL_STATUS_OK) {
        printf("Connection command error\r\n");
        strcpy(event_log, "Connection command error");
      } else {
        //if connected, latch the MAC address and the channel from the scan list
        for (uint16_t i = 0; i < SL_WFX_MAX_SCAN_RESULTS; i++) {
          if (strcmp((char *) scan_list[i].ssid_def.ssid, wlan_ssid) == 0) {
           ap_channel = scan_list[i].channel;
            memcpy(&ap_mac, scan_list[i].mac, SL_WFX_BSSID_SIZE);
          }
        }
      }
    }
  }else{
    printf("Invalid Connection Request\r\n");
    strcpy(event_log, "Invalid Connection Request");
  }

  return "/empty.json";
}

/***************************************************************************//**
 * @brief Web server CGI handler to stop the station interface.
 ******************************************************************************/
static const char *stop_station_cgi_handler(int index,
                                            int num_params,
                                            char *pc_param[],
                                            char *pc_value[])
{
  sl_wfx_send_disconnect_command();

  return "/empty.json";
}

/***************************************************************************//**
 * @brief Web server CGI handler to start the softAP interface.
 ******************************************************************************/
static const char *start_softap_cgi_handler(int index,
                                            int num_params,
                                            char *pc_param[],
                                            char *pc_value[])
{
  sl_wfx_start_ap_command(softap_channel, (uint8_t*) softap_ssid,
                          strlen(softap_ssid), 0, 0, softap_security,
                          0, (uint8_t*) softap_passkey, strlen(softap_passkey),
                          NULL, 0, NULL, 0);
  return "/empty.json";
}

/***************************************************************************//**
 * @brief Web server CGI handler to start the softAP interface.
 ******************************************************************************/
static const char *stop_softap_cgi_handler(int index,
                                           int num_params,
                                           char *pc_param[],
                                           char *pc_value[])
{
  sl_wfx_stop_ap_command();
  return "/empty.json";
}

/***************************************************************************//**
 * @brief Web server CGI handler to disconnect a client from the softAP
 * interface.
 ******************************************************************************/
static const char *disconnect_client_cgi_handler(int index,
                                                 int num_params,
                                                 char *pc_param[],
                                                 char *pc_value[])
{
  sl_wfx_mac_address_t mac_address;
  const char separator[] = ":";
  char *mac_byte = NULL;

  for (uint16_t i = 0; i < num_params; i++) {
    if (strcmp(pc_param[i], "mac") == 0)
    {
      mac_byte = strtok(pc_value[i], separator);
      for(uint8_t j = 0; j < 6; j++)
      {
        mac_address.octet[j] = (uint8_t)strtoul(mac_byte, NULL, 16);
        mac_byte = strtok(NULL, separator);
      }
      sl_wfx_disconnect_ap_client_command(&mac_address);
    }
  }

  return "/empty.json";
}

/***************************************************************************//**
 * @brief Web server CGI handler to start a scan.
 ******************************************************************************/
static const char *start_scan_cgi_handler(int index,
                                          int num_params,
                                          char *pc_param[],
                                          char *pc_value[])
{
  sl_status_t result;

  // Reset scan list
  scan_count = 0;
  scan_count_saved = 0;
  memset(scan_list, 0, sizeof(scan_result_list_t) * SL_WFX_MAX_SCAN_RESULTS);

  // perform a scan on every Wi-Fi channel in active mode
  result = sl_wfx_send_scan_command(WFM_SCAN_MODE_ACTIVE,
                                    NULL,
                                    0,
                                    NULL,
                                    0,
                                    NULL,
                                    0,
                                    NULL);
  if ((result == SL_STATUS_OK) || (result == SL_STATUS_WIFI_WARNING))
  {
    sl_wfx_host_setup_waited_event(SL_WFX_SCAN_COMPLETE_IND_ID);
    result = sl_wfx_host_wait_for_confirmation(SL_WFX_SCAN_COMPLETE_IND_ID,
                                      SL_WFX_DEFAULT_REQUEST_TIMEOUT_MS,
                                      NULL);
  }

  return "/scan_results.json";
}

static uint16_t ssi_handler(int index, char *pc_insert, int insert_len)
{
  int value, result = 0;
  char string_field[HTTP_MAX_ITEM_LENGTH];
  char client_name[9] = {'C', 'l', 'i', 'e', 'n', 't', ' ', ' ', '\0'};

  pc_insert[0] = 0;

  switch (index) {
    case 0: // <!--#leds_state-->
      value = interface_light_get_state();
      result = snprintf(pc_insert, 2, "%d", value);
      break;
    case 1: // <!--#scan_list-->
      for (int i = 0; i < scan_count_saved; i++) {
        value = snprintf(string_field, HTTP_MAX_ITEM_LENGTH, "{\"ssid\":\"%s\", \"rssi\":\"%d\", \"secu\":\"%s\"},",
                          (char *)((strlen((char *) scan_list[i].ssid_def.ssid) != 0) ? \
                              (char*)scan_list[i].ssid_def.ssid : "Hidden"),
                              ((int16_t)(scan_list[i].rcpi - 220)/2),
                              (scan_list[i].security_mode.wpa2) ? "WPA2" : \
                                  ((scan_list[i].security_mode.wpa) ? "WPA" : \
                                      ((scan_list[i].security_mode.wep) ? "WEP": "OPEN")));
        if ((value > 0) && (value < HTTP_MAX_ITEM_LENGTH)) {
          // Item format Ok, check that it doesn't exceed the response size before concatenating it
          if ((result + value) < LWIP_HTTPD_MAX_TAG_INSERT_LEN) {
            strcat(pc_insert, string_field);
            result += value;
          } else {
            // HTTP response size exceeded, exit the loop and send what is possible
            break;
          }
        }
      }
      // Strip the last comma
      if (pc_insert[result-1] == ',') {
        pc_insert[result-1] = '\0';
        result -= 1;
      }
      break;
    case 2: // <!--#softap_state-->
      if (wifi.state & SL_WFX_AP_INTERFACE_UP) {
        result = snprintf(pc_insert, 2, "1");
      } else {
        result = snprintf(pc_insert, 2, "0");
      }
      break;
    case 3: // <!--#softap_ssid-->
      result = snprintf(pc_insert, 32, "%s", softap_ssid);
      break;
    case 4: // <!--#softap_ip-->
      result = sprintf(pc_insert, "%d.%d.%d.%d",
                       (int)(ap_netif.ip_addr.addr & 0xff),
                       (int)((ap_netif.ip_addr.addr >> 8) & 0xff),
                       (int)((ap_netif.ip_addr.addr >> 16) & 0xff),
                       (int)((ap_netif.ip_addr.addr >> 24) & 0xff));
      break;
    case 5: // <!--#softap_mac-->
      result = sprintf(pc_insert,
                       "%02X:%02X:%02X:%02X:%02X:%02X",
                       wifi.mac_addr_1.octet[0],
                       wifi.mac_addr_1.octet[1],
                       wifi.mac_addr_1.octet[2],
                       wifi.mac_addr_1.octet[3],
                       wifi.mac_addr_1.octet[4],
                       wifi.mac_addr_1.octet[5]);
      break;
    case 6: // <!--#softap_secu-->
      if(softap_security == WFM_SECURITY_MODE_OPEN)                result = sprintf(pc_insert, "OPEN");
      else if(softap_security == WFM_SECURITY_MODE_WEP)            result = sprintf(pc_insert, "WEP");
      else if(softap_security == WFM_SECURITY_MODE_WPA2_WPA1_PSK)  result = sprintf(pc_insert, "WPA1/WPA2");
      else if(softap_security == WFM_SECURITY_MODE_WPA2_PSK)       result = sprintf(pc_insert, "WPA2");
      break;
    case 7: // <!--#softap_channel-->
      result = sprintf(pc_insert, "%d", softap_channel);
      break;
    case 8: // <!--#clients_list-->
      for(uint8_t i = 0; i < DHCPS_MAX_CLIENT; i++) {
        struct eth_addr mac;
        dhcpserver_get_mac(i, &mac);
        if (!(mac.addr[0] == 0 && mac.addr[1] == 0 &&
            mac.addr[2] == 0 && mac.addr[3] == 0 &&
            mac.addr[4] == 0 && mac.addr[5] == 0)) {
          ip_addr_t ip_addr = dhcpserver_get_ip(&mac);
          // Add an index to the client name
          client_name[7] = i + 1 + 0x30;

          value = snprintf(string_field, HTTP_MAX_ITEM_LENGTH,
                           "{\"name\":\"%s\", \"ip\":\"%d.%d.%d.%d\", \"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\"},",
                           client_name,
                           (int)(ip_addr.addr & 0xff),
                           (int)((ip_addr.addr >> 8) & 0xff),
                           (int)((ip_addr.addr >> 16) & 0xff),
                           (int)((ip_addr.addr >> 24) & 0xff),
                           mac.addr[0],
                           mac.addr[1],
                           mac.addr[2],
                           mac.addr[3],
                           mac.addr[4],
                           mac.addr[5]);

          if ((value > 0) && (value < HTTP_MAX_ITEM_LENGTH)) {
            // Item format Ok, check that it doesn't exceed the response size before concatenating it
            if ((result + value) < LWIP_HTTPD_MAX_TAG_INSERT_LEN) {
              strcat(pc_insert, string_field);
              result += value;
            } else {
              // HTTP response size exceeded, exit the loop and send what is possible
              break;
            }
          }
        }
      }
      // Strip the last comma
      if (pc_insert[result-1] == ',') {
        pc_insert[result-1] = '\0';
        result -= 1;
      }
      break;
    case 9: /* <!--#station_state--> */
      if (wifi.state & SL_WFX_STA_INTERFACE_CONNECTED) {
        result = snprintf(pc_insert, 2, "1");
      } else {
        result = snprintf(pc_insert, 2, "0");
      }
      break;
    case 10: /* <!--#station_ip--> */
      result = sprintf(pc_insert, "%d.%d.%d.%d",
                       (int)(sta_netif.ip_addr.addr & 0xff),
                       (int)((sta_netif.ip_addr.addr >> 8) & 0xff),
                       (int)((sta_netif.ip_addr.addr >> 16) & 0xff),
                       (int)((sta_netif.ip_addr.addr >> 24) & 0xff));
      break;
    case 11: /* <!--#station_mac--> */
      result = sprintf(pc_insert,
                       "%02X:%02X:%02X:%02X:%02X:%02X",
                       wifi.mac_addr_0.octet[0],
                       wifi.mac_addr_0.octet[1],
                       wifi.mac_addr_0.octet[2],
                       wifi.mac_addr_0.octet[3],
                       wifi.mac_addr_0.octet[4],
                       wifi.mac_addr_0.octet[5]);
      break;
    case 12: /* <!--#ap_ssid--> */
      result = snprintf(pc_insert, 32, "%s", wlan_ssid);
      break;
    case 13: /* <!--#ap_mac--> */
      result = sprintf(pc_insert,
                       "%02X:%02X:%02X:%02X:%02X:%02X",
                       ap_mac.addr[0],
                       ap_mac.addr[1],
                       ap_mac.addr[2],
                       ap_mac.addr[3],
                       ap_mac.addr[4],
                       ap_mac.addr[5]);
      break;
    case 14: // <!--#ap_secu-->
      if(wlan_security == WFM_SECURITY_MODE_OPEN)                result = sprintf(pc_insert, "OPEN");
      else if(wlan_security == WFM_SECURITY_MODE_WEP)            result = sprintf(pc_insert, "WEP");
      else if(wlan_security == WFM_SECURITY_MODE_WPA2_WPA1_PSK)  result = sprintf(pc_insert, "WPA1/WPA2");
      else if(wlan_security == WFM_SECURITY_MODE_WPA2_PSK)       result = sprintf(pc_insert, "WPA2");
      break;
    case 15: // <!--#ap_channel-->
      result = sprintf(pc_insert, "%d", ap_channel);
      break;
    case 16: /* <!--#event--> */
      result = sprintf(pc_insert, "%s", event_log);
      strcpy(event_log, "");
      break;
    case 17: // <!--#ble_state-->
      value = bluetooth_app_get_ble_state();
      result = snprintf(pc_insert, 2, "%d", value);
      break;
    case 18: // <!--#ble_name-->
      bluetooth_app_get_own_name(pc_insert, LWIP_HTTPD_MAX_TAG_INSERT_LEN);
      result = strlen(pc_insert);
      break;
    case 19: // <!--#ble_mac-->
    {
      bd_addr ble_id = {0};
      bluetooth_app_get_own_mac(&ble_id);
      result = sprintf(pc_insert,
                       "%02X:%02X:%02X:%02X:%02X:%02X",
                       ble_id.addr[5],
                       ble_id.addr[4],
                       ble_id.addr[3],
                       ble_id.addr[2],
                       ble_id.addr[1],
                       ble_id.addr[0]);
      break;
    }
    case 20: // <!--#master_list-->
    {
      bd_addr master_id = {0};
      int res = bluetooth_app_get_master_mac(&master_id);
      if (res == 0) {
        bluetooth_app_get_master_name(string_field, HTTP_MAX_ITEM_LENGTH);
        result = sprintf(pc_insert,
                         "{\"name\":\"%s\",\"mac\":\"%02X:%02X:%02X:%02X:%02X:%02X\"}",
                         string_field,
                         master_id.addr[5],
                         master_id.addr[4],
                         master_id.addr[3],
                         master_id.addr[2],
                         master_id.addr[1],
                         master_id.addr[0]);
      }
      break;
    }
    default:
      // Error, unsupported Id
      break;
  }

  return result;
}

/***************************************************************************//**
 * @brief Converts a hex character to its integer value
 *
 * @param ch Character to convert to integer.
 * @returns Returns integer result.
 ******************************************************************************/
static char from_hex(char ch)
{
  return isdigit(ch) ? ch - '0' : tolower(ch) - 'a' + 10;
}

/***************************************************************************//**
 * @brief Returns a url-decoded version of the string str
 *
 * @param str String to decode.
 * @returns Success or fail.
 ******************************************************************************/
static sl_status_t url_decode(char *str)
{
  char *pstr = str, rstr[64];
  int i = 0;

  if (strlen(str) > 64) {
    return SL_STATUS_FAIL;
  }

  while (*pstr) {
    if (*pstr == '%') {
      if (pstr[1] && pstr[2]) {
        rstr[i++] = from_hex(pstr[1]) << 4 | from_hex(pstr[2]);
        pstr += 2;
      }
    } else if (*pstr == '+') {
      rstr[i++]  = ' ';
    } else {
      rstr[i++] = *pstr;
    }
    pstr++;
  }
  rstr[i] = '\0';
  strcpy(str, &rstr[0]);
  return SL_STATUS_OK;
}

#endif

#ifdef LWIP_IPERF_SERVER
/***************************************************************************//**
 * @brief Function to handle iperf results report
 ******************************************************************************/
static void lwip_iperf_results(void *arg, enum lwiperf_report_type report_type,
                               const ip_addr_t* local_addr, u16_t local_port, const ip_addr_t* remote_addr, u16_t remote_port,
                               u32_t bytes_transferred, u32_t ms_duration, u32_t bandwidth_kbitpsec)
{
  printf("\r\nIperf Server Report:\r\n");
  printf("Interval %d.%d sec\r\n", (int)(ms_duration / 1000), (int)(ms_duration % 1000));
  printf("Bytes transferred %d.%dMBytes\r\n", (int)(bytes_transferred / 1024 / 1024), (int)((((bytes_transferred / 1024) * 1000) / 1024) % 1000));
  printf("Bandwidth %d.%d Mbits/sec\r\n\r\n", (int)(bandwidth_kbitpsec / 1024), (int)(((bandwidth_kbitpsec * 1000) / 1024) % 1000));
}

#endif

/***************************************************************************//**
 * Initialize and start LwIP stack.
 ******************************************************************************/
int lwip_bm_init (void)
{
  int res;

  // Initialize the LwIP stack without RTOS
  lwip_init();

  // Initialize the LwIP stack
  res = netif_config();

#ifdef LWIP_HTTP_SERVER
  // Initialize web server demo
  httpd_init();
  // Initialize the CGI handlers
  cgi_ssi_init();
#endif
#ifdef LWIP_IPERF_SERVER
  lwiperf_start_tcp_server_default(lwip_iperf_results, 0);
#endif

  return res;
}

/**************************************************************************//**
 * Set station link status to up.
 *****************************************************************************/
sl_status_t lwip_set_sta_link_up(void)
{
  netif_set_up(&sta_netif);
  netif_set_link_up(&sta_netif);
  if (use_dhcp_client) {
    dhcp_start(&sta_netif);
    print_addr = 1;
  }
  return SL_STATUS_OK;
}
/**************************************************************************//**
 * Set station link status to down.
 *****************************************************************************/
sl_status_t lwip_set_sta_link_down(void)
{
  if (use_dhcp_client) {
    dhcp_stop(&sta_netif);
  }
  netif_set_link_down(&sta_netif);
  netif_set_down(&sta_netif);
  return SL_STATUS_OK;
}
/**************************************************************************//**
 * Set AP link status to up.
 *****************************************************************************/
sl_status_t lwip_set_ap_link_up(void)
{
  netif_set_up(&ap_netif);
  netif_set_link_up(&ap_netif);
  dhcpserver_start();
  return SL_STATUS_OK;
}

/**************************************************************************//**
 * Enable DHCP client.
 *****************************************************************************/
void lwip_enable_dhcp_client(void)
{
  use_dhcp_client = 1;
}

/**************************************************************************//**
 * Disable DHCP client.
 *****************************************************************************/
void lwip_disable_dhcp_client(void)
{
  use_dhcp_client = 0;
}

/**************************************************************************//**
 * Set AP link status to down.
 *****************************************************************************/
sl_status_t lwip_set_ap_link_down(void)
{
  dhcpserver_stop();
  netif_set_link_down(&ap_netif);
  netif_set_down(&ap_netif);
  return SL_STATUS_OK;
}

/***************************************************************************//**
 * Initializes LwIP network interface.
 ******************************************************************************/
static int netif_config(void)
{
  sl_status_t status;
  ip_addr_t ap_ipaddr, ap_netmask, ap_gw;
  ip_addr_t sta_ipaddr, sta_netmask, sta_gw;
  int res = -1;
  bool wfx_init_ok = false;

  if (use_dhcp_client) {
    ip_addr_set_zero_ip4(&sta_ipaddr);
    ip_addr_set_zero_ip4(&sta_netmask);
    ip_addr_set_zero_ip4(&sta_gw);
  } else {
    IP_ADDR4(&sta_ipaddr, sta_ip_addr0, sta_ip_addr1, sta_ip_addr2, sta_ip_addr3);
    IP_ADDR4(&sta_netmask, sta_netmask_addr0, sta_netmask_addr1, sta_netmask_addr2, sta_netmask_addr3);
    IP_ADDR4(&sta_gw, sta_gw_addr0, sta_gw_addr1, sta_gw_addr2, sta_gw_addr3);
  }

  /* Initialize the SoftAP information */
  IP_ADDR4(&ap_ipaddr, ap_ip_addr0, ap_ip_addr1, ap_ip_addr2, ap_ip_addr3);
  IP_ADDR4(&ap_netmask, ap_netmask_addr0, ap_netmask_addr1, ap_netmask_addr2, ap_netmask_addr3);
  IP_ADDR4(&ap_gw, ap_gw_addr0, ap_gw_addr1, ap_gw_addr2, ap_gw_addr3);

  /* Initialize the WF200 used by the two interfaces */
  status = sl_wfx_init(&wifi);
  printf("FMAC Driver version    %s\r\n", FMAC_DRIVER_VERSION_STRING);
  switch (status) {
    case SL_STATUS_OK:
      wifi.state = SL_WFX_STARTED;
      printf("WF200 Firmware version %d.%d.%d\r\n",
             wifi.firmware_major,
             wifi.firmware_minor,
             wifi.firmware_build);
      printf("WF200 initialization successful\r\n");
      wfx_init_ok = true;
      break;
    case SL_STATUS_WIFI_INVALID_KEY:
      printf("Failed to init WF200: Firmware keyset invalid\r\n");
      break;
    case SL_STATUS_WIFI_FIRMWARE_DOWNLOAD_TIMEOUT:
      printf("Failed to init WF200: Firmware download timeout\r\n");
      break;
    case SL_STATUS_TIMEOUT:
      printf("Failed to init WF200: Poll for value timeout\r\n");
      break;
    case SL_STATUS_FAIL:
      printf("Failed to init WF200: Error\r\n");
      break;
    default:
      printf("Failed to init WF200: Unknown error\r\n");
  }

  if (wfx_init_ok) {
    // Add station and softAP interfaces
    netif_add(&sta_netif, &sta_ipaddr, &sta_netmask, &sta_gw, NULL, &sta_ethernetif_init, &ethernet_input);
    netif_add(&ap_netif, &ap_ipaddr, &ap_netmask, &ap_gw, NULL, &ap_ethernetif_init, &ethernet_input);

    //  Registers the default network interface.
    netif_set_default(&ap_netif);

    sl_wfx_start_ap_command(softap_channel, (uint8_t*) softap_ssid,
                            strlen(softap_ssid), 0, 0, softap_security, 0,
                            (uint8_t*) softap_passkey, strlen(softap_passkey),
                            NULL, 0, NULL, 0);
    printf ("IP address : %d.%d.%d.%d\r\n",
            ap_ip_addr0,
            ap_ip_addr1,
            ap_ip_addr2,
            ap_ip_addr3);

    interface_display_wifi_state(true);
    res = 0;
  }

  return res;
}

/***************************************************************************//**
 * @brief
 *    LwIP main loop
 *
 * @param[in]
 *    none
 *
 * @return
 *    none
 ******************************************************************************/
void lwip_bm_process(void)
{
  sys_check_timeouts();
}

