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

#include "lwip_micriumos.h"
#include  <bsp_os.h>
#include  "bsp.h"
#include  <cpu/include/cpu.h>
#include  <kernel/include/os.h>
#include  <kernel/include/os_trace.h>
#include  <common/include/common.h>
#include  <common/include/lib_def.h>
#include  <common/include/rtos_utils.h>
#include  <common/include/toolchains.h>
#include  "retargetserial.h"
#include  "em_cmu.h"
#include  "em_emu.h"
#include  "em_chip.h"
#include  "sl_wfx_host_cfg.h"
#include  "sl_wfx_host_events.h"
#include "io.h"
#include "sl_wfx_task.h"
#include "sl_wfx_host.h"
#include <stdio.h>
#include <common/include/auth.h>
#include <common/include/shell.h>
#include <mbedtls/threading.h>
#include "sleep.h"

#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include <string.h>
#include "sl_wfx_constants.h"
#include "sl_wfx_host_api.h"
#include "sl_wfx.h"
#include "lwip_micriumos.h"

#include <kernel/include/os.h>
#include <common/include/rtos_utils.h>
#include <common/include/rtos_err.h>
#include <common/source/kal/kal_priv.h>
#include <common/include/rtos_err.h>
#include "sl_wfx_task.h"
#include "lwip/tcpbase.h"

#define  EX_MAIN_START_TASK_PRIO              30u
#define  EX_MAIN_START_TASK_STK_SIZE         512u

#ifdef SL_WFX_USE_SECURE_LINK
extern void wfx_securelink_task_start(void);
#endif

/// Start task stack.
static  CPU_STK  main_start_task_stk[EX_MAIN_START_TASK_STK_SIZE];
/// Start task TCB.
static  OS_TCB   main_start_task_tcb;
static  void     main_start_task (void  *p_arg);
void netif_config(void);

uint8_t send_message;
uint8_t send_in_progress;
uint8_t send_complete;
int counter = 0;
struct tcp_pcb *testpcb;
ip_addr_t ip = {.addr = 0x01002a0a};
extern uint8_t sleepBlockCnt[];
bool connect;

void tcp_setup(void);
err_t tcpRecvCallback(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
uint32_t tcp_send_packet(void);
err_t connectCallback(void *arg, struct tcp_pcb *tpcb, err_t err);


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

/***************************************************************************//**
 * @brief
 *   This is the idle hook.
 *
 * @detail
 *   This will be called by the Micrium OS idle task when there is no other
 *   task ready to run. We just enter the lowest possible energy mode.
 ******************************************************************************/
void OSIdleEnterHook(void)
{
  SLEEP_Sleep();
}
#endif

/**************************************************************************//**
 * Main function
 *****************************************************************************/
int  main(void)
{
  RTOS_ERR  err;
  // Set the HFRCO frequency.
  CMU_HFRCOFreqSet(cmuHFRCOFreq_72M0Hz);
  BSP_SystemInit(); // Initialize System.
  CPU_Init();       // Initialize CPU.
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
  // Clear the console and buffer
  printf("\033\143");
  printf("\033[3J");

  OS_TRACE_INIT(); // Initialize trace if enabled
  OSInit(&err);    // Initialize the Kernel.
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  // Enable mbedtls Micrium OS support
#if defined ( MBEDTLS_THREADING_C )
  THREADING_setup();
#endif

#ifdef SLEEP_ENABLED
  // Don't allow EM3, since we use LF clocks.
  SLEEP_SleepBlockBegin(sleepEM3);
#endif
  OSTaskCreate(&main_start_task_tcb, // Create the Start Task.
               "Ex Main Start Task",
               main_start_task,
               DEF_NULL,
               EX_MAIN_START_TASK_PRIO,
               &main_start_task_stk[0],
               (EX_MAIN_START_TASK_STK_SIZE / 10u),
               EX_MAIN_START_TASK_STK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);

  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  OSStart(&err); // Start the kernel.

  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);

  return (1);
}


/**************************************************************************//**
 * Unified GPIO interrupt handler.
 *****************************************************************************/
static void GPIO_Unified_IRQ(void)
{
  RTOS_ERR err;

  // Get and clear all pending GPIO interrupts
  uint32_t interrupt_mask = GPIO_IntGet();
  GPIO_IntClear(interrupt_mask);

  // Act on interrupts
  if (interrupt_mask & 0x400) {
    OSSemPost(&wfx_wakeup_sem, OS_OPT_POST_ALL, &err);
#ifdef SL_WFX_USE_SPI
    OSFlagPost(&wfx_bus_evts, SL_WFX_BUS_EVENT_FLAG_RX, OS_OPT_POST_FLAG_SET, &err);
#endif
#ifdef SL_WFX_USE_SDIO
#ifdef SLEEP_ENABLED
    OSFlagPost(&wfx_bus_evts, SL_WFX_BUS_EVENT_FLAG_RX,OS_OPT_POST_FLAG_SET,&err);
#endif
#endif
  }
  if (interrupt_mask & (1 << BSP_GPIO_PB0_PIN)) {
    BSP_LedToggle(0);
  }

  if (interrupt_mask & (1 << BSP_GPIO_PB1_PIN)) {
    BSP_LedToggle(1);
  }
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
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
}

/**************************************************************************//**
 * main_start_task()
 *
 * @param p_arg Argument passed from task creation. Unused.
 *
 *  This is the task that will be called by the startup when all services
 *  are initialized successfully.
 *
 *****************************************************************************/
static  void  main_start_task(void  *p_arg)
{
  RTOS_ERR  err;
  PP_UNUSED_PARAM(p_arg); // Prevent compiler warning.



#ifdef SL_WFX_USE_SDIO
#ifdef RTOS_MODULE_IO_AVAIL
  // Initialize the IO module.
  IO_Init(&err);
  APP_RTOS_ASSERT_DBG((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE), 1);
#endif
#endif

  // Enable GPIO clock.
  CMU_ClockEnable(cmuClock_GPIO, true);

  gpio_setup();

#ifdef CPU_CFG_INT_DIS_MEAS_EN
  // Initialize interrupts disabled measurement.
  CPU_IntDisMeasMaxCurReset();
#endif

  // Call common module initialization.
  Common_Init(&err);
  APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE,; );

#ifdef  RTOS_MODULE_COMMON_SHELL_AVAIL
  Shell_Init(&err);
  APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE,; );
#endif

  Auth_Init(&err);
  APP_RTOS_ASSERT_CRITICAL(err.Code == RTOS_ERR_NONE,; );

  // Initialize the BSP.
  BSP_OS_Init();
  BSP_LedsInit();
//  connect = true;

  printf("WF200 Micrium OS LwIP Example\n");

  wfx_events_start();
  wfx_bus_start();
#ifdef SL_WFX_USE_SECURE_LINK
  wfx_securelink_task_start(); // start securelink key renegotiation task
#endif //SL_WFX_USE_SECURE_LINK

  lwip_start();
  while (1)
  {
    if (send_complete)
    {
      send_complete = 0;
//      sl_wfx_set_power_mode(WFM_PM_MODE_PS, WFM_PM_POLL_FAST_PS, WFM_PM_SKIP_CNT);
      connect = false;
      sl_wfx_send_disconnect_command();
      OSTimeDly(50, OS_OPT_TIME_DLY, &err);
      sl_wfx_deinit();
      sl_wfx_context->state |= SL_WFX_SLEEPING;
      printf("--> Sleeping\r\n");
//      SLEEP_Sleep();
//      sleepBlockCnt[0] = 0; // Force enable sleep modes EM2 and EM3
//      sleepBlockCnt[1] = 0;
    }
    if(send_in_progress)
    {
    	OSTimeDly(50, OS_OPT_TIME_DLY, &err);
    }
    else
    {
    	sl_status_t status;
    	OSTimeDly(20000, OS_OPT_TIME_DLY, &err);
    	if((wifi.state & SL_WFX_STA_INTERFACE_CONNECTED) && !send_in_progress)
		{
			 err_t err = 0;
			 send_in_progress = 1;
			 tcp_setup();
			 printf("Hello! %d err = %d\n\n", counter++, err);
		}
    	else
    	{
    		printf("Will connect\n\n");
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
    		connect = (status == SL_STATUS_OK);
    	}
    }
  }
  // Delete the init thread.
  OSTaskDel(0, &err);

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
    string = "GET /relay/0?turn=on HTTP/1.0\r\nHost: 192.168.1.100\r\nConnection: close\r\n\r\n ";
  }
  else
  {
    string = "GET /relay/0?turn=off HTTP/1.0\r\nHost: 192.168.1.100\r\nConnection: close\r\n\r\n ";
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
