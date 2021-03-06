#ifdef  SL_WFX_USE_SDIO
#include  <rtos_description.h>
#include "sl_wfx.h"
#include "sl_wfx_host_api.h"
#include "bus/sl_wfx_bus.h"

#include "em_gpio.h"
#include "em_usart.h"
#include "em_cmu.h"
#include "em_ldma.h"
#include "em_bus.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <kernel/include/os.h>
#include <common/include/rtos_utils.h>
#include <common/include/rtos_err.h>
#include <common/source/kal/kal_priv.h>

#include <common/include/rtos_err.h>

#include <io/include/sd.h>
#include <io/include/sd_card.h>

#include "sl_wfx_host_sdio_fnct.h"
#include "sl_wfx_host_cfg.h"

#include "sl_wfx_task.h"

#ifndef SL_WIFI_CFG_SD_CONTROLLER_NAME
#define SL_WIFI_CFG_SD_CONTROLLER_NAME             "sd0"
#endif

static SD_BUS_HANDLE sd_bus_handle = 0;
static bool sdio_enabled = false;
/****************************************************************************************************//**
 *                                     sl_wfx_host_init_bus()
 *
 * @brief    Initializes the communications bus.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_init_bus(void)
{
  RTOS_ERR err;

  GPIO_PinOutSet(SL_WFX_HOST_CFG_RESET_PORT, SL_WFX_HOST_CFG_RESET_PIN);
  sdio_enabled = true;
  // Check to see if the bus was already added
  if (SD_BusHandleGetFromName(SL_WIFI_CFG_SD_CONTROLLER_NAME) == SD_BusHandleNull) {
    (void)SD_BusAdd(SL_WIFI_CFG_SD_CONTROLLER_NAME, &err);
    if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE) {
      return SL_STATUS_FAIL;
    }
  }

  sdio_fnct_init(&err);

  if (sd_bus_handle == SD_BusHandleNull) {
    sd_bus_handle = SD_BusHandleGetFromName(SL_WIFI_CFG_SD_CONTROLLER_NAME);

    SD_BusStart(sd_bus_handle, &err);

    if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE) {
      return SL_STATUS_FAIL;
    }
  }

  return SL_STATUS_OK;
}

/****************************************************************************************************//**
 *                                     sl_wfx_host_deinit_bus()
 *
 * @brief    De-initializes the communications bus.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_deinit_bus(void)
{
  RTOS_ERR err;

  if (sd_bus_handle != SD_BusHandleNull) {
    SD_BusStop(sd_bus_handle, &err);
    if (RTOS_ERR_CODE_GET(err) != RTOS_ERR_NONE) {
      return SL_STATUS_FAIL;
    }
    sd_bus_handle = SD_BusHandleNull;
  }

  return SL_STATUS_OK;
}

static sl_status_t sdio_io_write_direct(uint8_t function, uint32_t address, uint8_t* data)
{
  RTOS_ERR    err;
  sl_status_t result;

  sdio_fnct_wrbyte(address,
                         *data,
                         &err);

  if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE) {
    result = SL_STATUS_OK;
  } else {
    result = SL_STATUS_FAIL;
  }

  return result;
}

static sl_status_t sdio_io_read_direct(uint8_t function, uint32_t address, uint8_t* data)
{
  RTOS_ERR    err;
  sl_status_t result;

  *data = sdio_fnct_rdbyte(address, &err);

  if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE) {
    result = SL_STATUS_OK;
  } else {
    result = SL_STATUS_FAIL;
  }

  return result;
}

static sl_status_t sdio_io_write_extended(uint8_t function, uint32_t address, uint8_t* data, uint32_t data_length)
{
  RTOS_ERR    err;
  sl_status_t result;
  uint32_t    block_count;

  if (data_length >= 512) {
    block_count = (data_length / SL_WFX_SDIO_BLOCK_SIZE) + ( ( (data_length % SL_WFX_SDIO_BLOCK_SIZE) == 0) ? 0 : 1);
    sdio_fnct_wrblk(address,
                          data,
                          block_count,
                          1,
                          &err);
  } else {
    sdio_fnct_wr(address,
                       data,
                       data_length,
                       1,
                       &err);
  }

  if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE) {
    result = SL_STATUS_OK;
  } else {
    result = SL_STATUS_FAIL;
  }

  return result;
}

static sl_status_t sdio_io_read_extended(uint8_t function, uint32_t address, uint8_t* data, uint32_t data_length)
{
  RTOS_ERR    err;
  sl_status_t result;
  uint32_t    block_count;

  if (data_length >= 512) {
    block_count = (data_length / SL_WFX_SDIO_BLOCK_SIZE) + ( ( (data_length % SL_WFX_SDIO_BLOCK_SIZE) == 0) ? 0 : 1);

    sdio_fnct_rdblk(address,
                          data,
                          block_count,
                          1,
                          &err);
  } else {
    sdio_fnct_rd(address,
                       data,
                       data_length,
                       1,
                       &err);
  }

  if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE) {
    result = SL_STATUS_OK;
  } else {
    result = SL_STATUS_FAIL;
  }

  return result;
}

sl_status_t sl_wfx_host_sdio_transfer_cmd52(sl_wfx_host_bus_transfer_type_t type, uint8_t function, uint32_t address, uint8_t* buffer)
{
  sl_status_t status;

  if (type == SL_WFX_BUS_WRITE) {
    status = sdio_io_write_direct(function, address, buffer);
  } else {
    status = sdio_io_read_direct(function, address, buffer);
  }

  return status;
}

sl_status_t sl_wfx_host_sdio_transfer_cmd53(sl_wfx_host_bus_transfer_type_t type, uint8_t function, uint32_t address, uint8_t* buffer, uint16_t buffer_length)
{
  sl_status_t status;

  if (type == SL_WFX_BUS_WRITE) {
    status = sdio_io_write_extended(function, address, buffer, buffer_length);
  } else {
    status = sdio_io_read_extended(function, address, buffer, buffer_length);
  }

  return status;
}

sl_status_t sl_wfx_host_sdio_enable_high_speed_mode(void)
{
  sl_status_t result;
  uint8_t     value_u8;

  result = sdio_io_read_direct(0, 0x13, &value_u8);
  value_u8 |= 0x2;   // Set EHS to 1
  result = sdio_io_write_direct(0, 0x13, &value_u8);
  SDIO->HOSTCTRL1 |= SDIO_HOSTCTRL1_HSEN;   // Enable HS mode at the host
  return result;
}

static void sdio_irq_callback(void* arg)
{
  RTOS_ERR err;
  OSFlagPost(&bus_events, SL_WFX_BUS_EVENT_FLAG_RX, OS_OPT_POST_FLAG_SET, &err);
}

/****************************************************************************************************//**
 *                                  sl_wfx_host_enable_platform_interrupt()
 *
 * @brief    Enable interrupts on the host side.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_enable_platform_interrupt(void)
{
  RTOS_ERR err;
  sdio_fnct_int_reg((void*)sdio_irq_callback);

  sdio_fnct_int_en(1, &err);
  SDIO->IEN |= SDIO_IFCR_CARDINT;
  SDIO->IFENC |= SDIO_IFENC_CARDINTEN;

  if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE) {
    return SL_STATUS_OK;
  }
  return SL_STATUS_FAIL;
}

/****************************************************************************************************//**
 *                                 sl_wfx_host_disable_platform_interrupt()
 *
 * @brief    Disable interrupts on the host side.
 *
 * @return   sl_status_t    Error code of the operation as defined in sl_status.h.
 *******************************************************************************************************/
sl_status_t sl_wfx_host_disable_platform_interrupt(void)
{
  RTOS_ERR err;
  SDIO->IEN &= ~(SDIO_IFCR_CARDINT);
  SDIO->IFENC &= ~(SDIO_IFENC_CARDINTEN);
  sdio_fnct_int_en(0, &err);

  if (RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE) {
    return SL_STATUS_OK;
  }
  return SL_STATUS_FAIL;
}

#endif
