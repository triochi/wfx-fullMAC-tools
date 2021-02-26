/***************************************************************************//**
 * @file
 * @brief Example main
 *******************************************************************************
 * # License
 * <b>Copyright 2019 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement.
 * The software is governed by the sections of the MSLA applicable to Micrium
 * Software.
 *
 ******************************************************************************/

#include <stdio.h>
#include <stdarg.h>

#include "em_cmu.h"
#include "em_emu.h"
#include "em_chip.h"
#include "sleep.h"
#include "bsp.h"
#include "bsp_trace.h"
#include "bspconfig.h"
#include "retargetserial.h"

#include "cmsis_os.h"

#include "demo_config.h"

#include "sl_wfx_task.h"
#include "sl_wfx_host.h"
#include "sl_wfx_host_cfg.h"
#include "sl_wfx_host_events.h"

#ifdef SL_WFX_USE_SECURE_LINK
#include  <mbedtls/threading.h>
extern void wfx_securelink_task_start(void);
#endif

extern osThreadId busCommTaskHandle;
void tcp_setup(void);
err_t tcpRecvCallback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
uint32_t tcp_send_packet(void);
err_t connectCallback(void *arg, struct tcp_pcb *tpcb, err_t err);

struct tcp_pcb *testpcb;
//shelly1-E098068D9B0F 10.42.0.50
//ip_addr_t ip = {.addr = 0x32002a0a};
//ip_addr_t ip = {.addr = 0xe0002a0a};
ip_addr_t ip = {.addr = 0x01002a0a};
uint8_t send_message;
uint8_t send_in_progress;
uint8_t send_complete;
int counter = 0;

#ifdef SLEEP_ENABLED
static bool sleepCallback(SLEEP_EnergyMode_t emode)
{
#ifdef SL_WFX_USE_SPI
  if (GPIO_PinInGet(SL_WFX_HOST_CFG_SPI_WIRQPORT,  SL_WFX_HOST_CFG_SPI_WIRQPIN))//wf200 messages pending
#else
    if (GPIO_PinInGet(SL_WFX_HOST_CFG_WIRQPORT,  SL_WFX_HOST_CFG_WIRQPIN)) //wf200 messages pending
#endif
    {
      return false;
    }

  return true;
}

static void wakeupCallback(SLEEP_EnergyMode_t emode)
{

}
#endif


static void gpio_setup(void);
/**************************************************************************//**
 * Main function
 *****************************************************************************/
int  main(void)
{
  CHIP_Init();       // Initialize CPU.

  // Set the HFRCO frequency.
  CMU_HFRCOFreqSet(cmuHFRCOFreq_72M0Hz);
  // Init DCDC regulator
  EMU_DCDCInit_TypeDef dcdcInit = EMU_DCDCINIT_STK_DEFAULT;
  // Initialize DCDC regulator
  dcdcInit.dcdcMode = emuDcdcMode_LowNoise;
  EMU_DCDCInit(&dcdcInit);

#ifdef SL_WFX_USE_SPI
  CMU_ClockPrescSet(cmuClock_HFPER, 0);
#endif

  RETARGET_SerialInit();
  RETARGET_SerialCrLf(1);
#ifdef SLEEP_ENABLED
  const SLEEP_Init_t sleepInit =
  {
    .sleepCallback = sleepCallback,
    .wakeupCallback = wakeupCallback,
    .restoreCallback = 0
  };
  SLEEP_InitEx(&sleepInit);
#endif

#ifdef SL_WFX_USE_SECURE_LINK
  // Enable mbedtls FreeRTOS support
#if defined ( MBEDTLS_THREADING_C )
  THREADING_setup();
#endif
#endif

#ifdef SLEEP_ENABLED
  // Don't allow EM3, since we use LF clocks.
  SLEEP_SleepBlockBegin(sleepEM3);
#endif

  gpio_setup();

  BSP_LedsInit();
  // Clear the console and buffer
  printf("\033\143");
  printf("\033[3J");
  printf("WF200 FreeRTOS LwIP Example\n");

  // Start wfx bus communication task.
  wfx_bus_start();
#ifdef SL_WFX_USE_SECURE_LINK
  wfx_securelink_task_start(); // start securelink key renegotiation task
#endif //SL_WFX_USE_SECURE_LINK

  wfx_events_task_start();
  lwip_start();

  // Start scheduler
  osKernelStart();

  // We should never get here as control is now taken by the scheduler

  while (1)
  {
    if(send_message || (counter%3000) == 0)
    {
     		 err_t err = 0;
    		 send_message = 0;
    		 send_in_progress = 1;
    		 tcp_setup();
    	 	 printf("Hello! %d err = %d\n\n", counter++, err);
    }
    if (send_complete)
    {
      send_complete = 0;
      sl_wfx_set_power_mode(WFM_PM_MODE_PS, WFM_PM_POLL_FAST_PS, WFM_PM_SKIP_CNT);
      sl_wfx_enable_device_power_save();
    }
  }

}


/**************************************************************************//**
 * Unified GPIO interrupt handler.
 *****************************************************************************/
static void GPIO_Unified_IRQ(void)
{
  BaseType_t xHigherPriorityTaskWoken;
  /* xHigherPriorityTaskWoken must be initialised to pdFALSE. */
  xHigherPriorityTaskWoken = pdFALSE;
  // Get and clear all pending GPIO interrupts
  uint32_t interrupt_mask = GPIO_IntGet();
  GPIO_IntClear(interrupt_mask);

  // Act on interrupts
  if (interrupt_mask & 0x400) {
    xSemaphoreGiveFromISR(wfx_wakeup_sem, &xHigherPriorityTaskWoken);
#ifdef  SL_WFX_USE_SPI
    vTaskNotifyGiveFromISR( busCommTaskHandle, &xHigherPriorityTaskWoken );
#endif
#ifdef  SL_WFX_USE_SDIO
#ifdef  SLEEP_ENABLED
    vTaskNotifyGiveFromISR( busCommTaskHandle, &xHigherPriorityTaskWoken );
#endif
#endif

  }
  if (interrupt_mask & (1 << BSP_GPIO_PB0_PIN)) {
    BSP_LedToggle(0);
  }

  if (interrupt_mask & (1 << BSP_GPIO_PB1_PIN)) {
    BSP_LedToggle(1);
  }
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**************************************************************************//**
 * GPIO even interrupt handler.
 *****************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

/**************************************************************************//**
 * GPIO odd interrupt handler.
 *****************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  GPIO_Unified_IRQ();
}

/**************************************************************************//**
 * Configure the GPIO pins.
 *****************************************************************************/
static void gpio_setup(void)
{
  // Enable GPIO clock.
  CMU_ClockEnable(cmuClock_GPIO, true);

  // Configure PB0 and PB1 as inputs (present on the Wireless Radio board in WGM160P case).
  GPIO_PinModeSet(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, gpioModeInput, 0);
  GPIO_PinModeSet(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, gpioModeInput, 0);
  // Enable interrupts.
  GPIO_IntConfig(BSP_GPIO_PB0_PORT, BSP_GPIO_PB0_PIN, false, true, true);
  GPIO_IntConfig(BSP_GPIO_PB1_PORT, BSP_GPIO_PB1_PIN, false, true, true);

  // Configure WF200 reset pin.
  GPIO_PinModeSet(SL_WFX_HOST_CFG_RESET_PORT, SL_WFX_HOST_CFG_RESET_PIN, gpioModePushPull, 0);
  // Configure WF200 WUP pin.
  GPIO_PinModeSet(SL_WFX_HOST_CFG_WUP_PORT, SL_WFX_HOST_CFG_WUP_PIN, gpioModePushPull, 0);
#ifdef  SL_WFX_USE_SPI
  // GPIO used as IRQ.
  GPIO_PinModeSet(SL_WFX_HOST_CFG_SPI_WIRQPORT, SL_WFX_HOST_CFG_SPI_WIRQPIN, gpioModeInputPull, 0);
#endif
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
#ifdef EFM32GG11B820F2048GM64 //WGM160PX22KGA2
  // GPIO used as IRQ
  GPIO_PinModeSet(SL_WFX_HOST_CFG_WIRQPORT,  SL_WFX_HOST_CFG_WIRQPIN,  gpioModeInputPull,  0);
  // SDIO Pull-ups
  GPIO_PinModeSet(gpioPortD,  0,  gpioModeDisabled,  1);
  GPIO_PinModeSet(gpioPortD,  1,  gpioModeDisabled,  1);
  GPIO_PinModeSet(gpioPortD,  2,  gpioModeDisabled,  1);
  GPIO_PinModeSet(gpioPortD,  3,  gpioModeDisabled,  1);
  GPIO_PinModeSet(gpioPortD,  5,  gpioModeDisabled,  1);
  //WF200 LF CLK
  CMU->CTRL      |= CMU_CTRL_CLKOUTSEL0_LFXO;
  CMU->ROUTEPEN  |= CMU_ROUTEPEN_CLKOUT0PEN;
  CMU->ROUTELOC0 |= CMU_ROUTELOC0_CLKOUT0LOC_LOC5;
  GPIO_PinModeSet(LP_CLK_PORT,  LP_CLK_PIN,  gpioModePushPull,  0);
#endif
  // Reset and enable associated CPU interrupt vector.
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_SetPriority(GPIO_EVEN_IRQn,1);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

void vApplicationStackOverflowHook (TaskHandle_t xTask, signed char *pcTaskName) {
  (void)xTask;
  (void)pcTaskName;
  configASSERT(0);
}

err_t tcpSendCallback(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  printf("Packet sent");
  //    sl_wfx_set_power_mode(WFX_POWER_MODE, 30);
  //    sl_wfx_enable_device_power_save();
  return 0;
}

void tcpErrorHandler(void *arg, err_t err)
{
  if (err == ERR_OK)
  {
    printf("Err OK");
  }
  else
  {
    printf("Err %d", err);
  }
  return;
}

void tcp_setup(void)
{
  uint32_t data = 0xdeadbeef;
  sl_wfx_set_power_mode(WFM_PM_MODE_ACTIVE, WFM_PM_POLL_FAST_PS, 0);
  sl_wfx_disable_device_power_save();
  /* create an ip */
  //    struct ip_addr ip;
  IP4_ADDR(&ip, 192, 168, 1, 100);
  //    IP4_ADDR(&ip, 10,42,0,224);    //IP of server

  /* create the control block */
  testpcb = tcp_new(); //testpcb is a global struct tcp_pcb
                       // as defined by lwIP

  /* dummy data to pass to callbacks*/

  tcp_arg(testpcb, &data);

  /* register callbacks with the pcb */

  tcp_err(testpcb, tcpErrorHandler);
  tcp_recv(testpcb, tcpRecvCallback);
  tcp_sent(testpcb, tcpSendCallback);

  /* now connect */
  tcp_connect(testpcb, &ip, 80, connectCallback);
  printf("connecting to destination\n");
}

/* connection established callback, err is unused and only return 0 */
err_t connectCallback(void *arg, struct tcp_pcb *tpcb, err_t err)
{
  printf("Connection Established.\n");
  printf("Now sending a packet\n");
  tcp_send_packet();
  return 0;
}

uint32_t tcp_send_packet(void)
{
  err_t error;
  static int counter3;
  char *string;
  //    if( GPIO_PinInGet(BSP_GPIO_LED0_PORT, BSP_GPIO_LED0_PIN))
  if (counter3++ & 1)
  {
    string = "GET /relay/0?turn=on HTTP/1.0\r\nHost: 10.42.0.224\r\nConnection: close\r\n\r\n ";
  }
  else
  {
    string = "GET /relay/0?turn=off HTTP/1.0\r\nHost: 10.42.0.224\r\nConnection: close\r\n\r\n ";
  }

  /* push to buffer */
  error = tcp_write(testpcb, string, strlen(string), TCP_WRITE_FLAG_COPY);

  if (error)
  {
    printf("ERROR: Code: %d (tcp_send_packet :: tcp_write)\n", error);
    return 1;
  }

  /* now send */
  error = tcp_output(testpcb);
  if (error)
  {
    printf("ERROR: Code: %d (tcp_send_packet :: tcp_output)\n", error);
    return 1;
  }
  return 0;
}

err_t tcpRecvCallback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  printf("Data recieved.\n");
  if (p == NULL)
  {
    printf("The remote host closed the connection.\n");
    printf("Now I'm closing the connection.\n");
    tcp_shutdown(testpcb, 1, 1);
    //        tcp_close_con();
    //        tcp_free(testpcb);
    return ERR_ABRT;
  }
  else
  {
    printf("Number of pbufs %d\n", pbuf_clen(p));
    printf("Contents of pbuf %s\n", (char *)p->payload);
  }
  tcp_shutdown(testpcb, 1, 1);
  send_in_progress = 0;
  //    sl_wfx_enable_device_power_save();
  send_complete = 1;
  return 0;
}
