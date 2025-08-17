/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
 *task1 and task2 alternate printing
 */

#include "debug.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "tusb.h"
#include "vcom_serial.h"
#include "DAP.h"
#include "DAP_config.h"
#include "message_buffer.h"



/* Global define */
#define LED_TASK_PRIO      5+5
#define USB_TASK_PRIO      5+5
#define UART_TASK_PRIO     5+3
#define DAP_TASK_PRIO      5+4


#define LED_STK_SIZE      256
#define UART_STK_SIZE     256
#define USB_STK_SIZE      256
#define DAP_STK_SIZE      256

/* Global Variable */
TaskHandle_t LED_Task_Handler = NULL;
TaskHandle_t USB_Task_Handler = NULL;
TaskHandle_t DAP_Task_Handler = NULL;
TaskHandle_t UART_Task_Handler= NULL;
MessageBufferHandle_t cdc_rcv_queue = NULL;

MessageBufferHandle_t dap_rcv_queue = NULL;

static uint32_t led_delay_ms = 500;

#define LED_PIN_NUM   (GPIO_Pin_9 |GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12)
/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.0/1
 *
 * @return  none
 */
void GPIO_Toggle_INIT(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure={0};

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE); 
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

  GPIO_InitStructure.GPIO_Pin = LED_PIN_NUM;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}




/****************************************************************/ 
/****************************************************************/ 


/*********************************************************************
 * @fn      task2_task
 *
 * @brief   task2 program.
 *
 * @param  *pvParameters - Parameters point of task2
 *
 * @return  none
 *********************************************************************/

// #define CFG_DAP_RCV_BUFSIZE  (512*2*2)

// static uint8_t TxDataBuffer[CFG_DAP_RCV_BUFSIZE];
// static uint8_t RxDataBuffer[CFG_DAP_RCV_BUFSIZE];

static volatile int usbRcvIdx   = 0;

extern uint32_t DAP_ProcessCommand       (const uint8_t *request, uint8_t *response);
#if ENABLE_PRINTF
#define DAP_Printf printf
#else
#define DAP_Printf(...) 
#endif


char * dap_cmd_string[] = {
    [ID_DAP_Info               ] = "DAP_Info",
    [ID_DAP_HostStatus         ] = "DAP_HostStatus",
    [ID_DAP_Connect            ] = "DAP_Connect",
    [ID_DAP_Disconnect         ] = "DAP_Disconnect",
    [ID_DAP_TransferConfigure  ] = "DAP_TransferConfigure",
    [ID_DAP_Transfer           ] = "DAP_Transfer",
    [ID_DAP_TransferBlock      ] = "DAP_TransferBlock",
    [ID_DAP_TransferAbort      ] = "DAP_TransferAbort",
    [ID_DAP_WriteABORT         ] = "DAP_WriteABORT",
    [ID_DAP_Delay              ] = "DAP_Delay",
    [ID_DAP_ResetTarget        ] = "DAP_ResetTarget",
    [ID_DAP_SWJ_Pins           ] = "DAP_SWJ_Pins",
    [ID_DAP_SWJ_Clock          ] = "DAP_SWJ_Clock",
    [ID_DAP_SWJ_Sequence       ] = "DAP_SWJ_Sequence",
    [ID_DAP_SWD_Configure      ] = "DAP_SWD_Configure",
    [ID_DAP_SWD_Sequence       ] = "DAP_SWD_Sequence",
    [ID_DAP_JTAG_Sequence      ] = "DAP_JTAG_Sequence",
    [ID_DAP_JTAG_Configure     ] = "DAP_JTAG_Configure",
    [ID_DAP_JTAG_IDCODE        ] = "DAP_JTAG_IDCODE",
    [ID_DAP_SWO_Transport      ] = "DAP_SWO_Transport",
    [ID_DAP_SWO_Mode           ] = "DAP_SWO_Mode",
    [ID_DAP_SWO_Baudrate       ] = "DAP_SWO_Baudrate",
    [ID_DAP_SWO_Control        ] = "DAP_SWO_Control",
    [ID_DAP_SWO_Status         ] = "DAP_SWO_Status",
    [ID_DAP_SWO_ExtendedStatus ] = "DAP_SWO_ExtendedStatus",
    [ID_DAP_SWO_Data           ] = "DAP_SWO_Data",
    [ID_DAP_QueueCommands      ] = "DAP_QueueCommands",
    [ID_DAP_ExecuteCommands    ] = "DAP_ExecuteCommands",
};

#if 0 

void dump_memory(uint8_t *pbuff,uint32_t buff_size,int col){
    for(uint32_t i = 0;i<buff_size;i++){
        if(i&&i%col==0) printf("\r\n");printf(" %02x",pbuff[i]);
    }
    printf("\r\n");
}

OPT_CMSIS_DAPV2

void  handle_dap_message(const uint8_t* pbuffer, uint32_t buff_size ,uint32_t usb_recv) {
    uint32_t handled_len = 0;
    uint32_t resp_len    = 0;
    uint32_t pack_size   = 0;
    int loop = 0;
    RxDapMsgEndAt = (uint8_t*)pbuffer + buff_size;
    do{
        loop++;
        // uint32_t ret_len =  DAP_ProcessCommand(pbuffer + handled_len,TxDataBuffer);
        uint32_t ret_len = DAP_ExecuteCommand(pbuffer + handled_len,TxDataBuffer);
        pack_size    = ret_len >>16;
        resp_len     = ret_len & 0xffff; 
        int left     = buff_size - handled_len;

        if(ret_len){
            DAP_Printf("USB RCV[%3d-Left:%3d] CDM:[%02x:%-20s][%02x][%02x][%02x]-[%02x][%02x][%02x][%02x] PACK_SIZE:%3d",usb_recv,left,pbuffer[0+handled_len],dap_cmd_string[pbuffer[0+handled_len]],
                pbuffer[1+handled_len],pbuffer[2+handled_len],pbuffer[3+handled_len],
                pbuffer[4+handled_len],pbuffer[5+handled_len],pbuffer[6+handled_len],pbuffer[7+handled_len],pack_size);
            DAP_Printf("\t[RESP=%3d]:[%02x][%02x][%02x][%02x]-[%02x][%02x][%02x][%02x] \r\n",
                resp_len,TxDataBuffer[0],TxDataBuffer[1],TxDataBuffer[2],TxDataBuffer[3],TxDataBuffer[4],TxDataBuffer[5],TxDataBuffer[6],TxDataBuffer[7]);
        }else if(ret_len ==0 ){
            if(left >= sizeof(RxDataBuffer)){
                printf("==========================Error LEFT:%d\r\n",left);
                tud_vendor_read_flush();
                break;
            } 
            if(handled_len){
                memmove(RxDataBuffer,pbuffer + handled_len,left);
            }

            RxOffset = RxDataBuffer + left;
            DAP_Printf("\t [%d@%d - recv:%d]Lost: offset:%d   left:%d  PEEK[%x][%x][%x][%x]-[%x][%x][%x][%x]-[%x][%x][%x][%x]-[%x][%x][%x][%x]\r\n",usbRcvIdx,loop-1,usb_recv,handled_len,left,  
                RxDataBuffer[0],RxDataBuffer[1],RxDataBuffer[2],RxDataBuffer[3],RxDataBuffer[4],RxDataBuffer[5],RxDataBuffer[6],RxDataBuffer[7],
                RxDataBuffer[0+8],RxDataBuffer[1+8],RxDataBuffer[2+8],RxDataBuffer[3+8],RxDataBuffer[4+8],RxDataBuffer[5+8],RxDataBuffer[6+8],RxDataBuffer[7+8]);
            break;
        }



        if(resp_len){
            tud_vendor_write(TxDataBuffer, resp_len);
            tud_vendor_flush();
        }else{
            DAP_Printf("Resp Size is Zero\r\n");
        }

        handled_len += pack_size;
        if(handled_len >= buff_size){
            break;
        } 
       
    }while(buff_size>handled_len);
}


TimerHandle_t xClearBufTimer;
void dapBuf_ClearTimer( TimerHandle_t xTimer ){
    RxOffset = RxDataBuffer;
    // printf(">>>>>clear<<<<<<\r\n");
}

#endif

static  int  tinyusb_inited = false;
void cdc_thread(void*) {
    while(1){
        if(!tinyusb_inited) {
            vTaskDelay(pdMS_TO_TICKS(50)); 
            return;
        }
        // blackmagic_main(1,NULL);
        tud_cdc_n_connected(0)?GPIO_SetBits(GPIOC, GPIO_Pin_11):GPIO_ResetBits(GPIOC, GPIO_Pin_11);//USB connected connected
        for(uint8_t i = 0;i<CFG_TUD_CDC;i++){
            if (tud_cdc_n_connected(i)) {
                while (tud_cdc_n_available(i)) {

                    uint16_t bits =  GPIO_ReadOutputData(GPIOC);
                    (bits & 1<<10)?GPIO_ResetBits(GPIOC, GPIO_Pin_10):GPIO_SetBits(GPIOC, GPIO_Pin_10);//data active
                    led_delay_ms = 100;
                    
                    uint8_t buf[64];
                    uint32_t count =  0;
                    if(i == 0 ){
                        if(DMA_GetCurrDataCounter(DMA1_Channel2) == 0){
                            count = tud_cdc_n_read(i,buf, sizeof(buf));
                            if(count>0){
                                USART3_Send_DMA(buf,count);
                            }
                        }
                        continue;
                    }
                    count = tud_cdc_n_read(i,buf, sizeof(buf));
                    if(count>0){
                        tud_cdc_n_write(i,(const void*)buf,count);
                        tud_cdc_n_write_flush(i);
                    }
                    // echo back to both web serial and cdc
                }
            }
        }

        /***************************************************************/
        uint8_t ucRxData[ 256 ];
        size_t xReceivedBytes = 0;
        while(!xMessageBufferIsEmpty(cdc_rcv_queue)){

            xReceivedBytes = xMessageBufferReceive( cdc_rcv_queue,
                                ( void * ) ucRxData,
                                sizeof( ucRxData ),
                                pdMS_TO_TICKS(1) );
            if(xReceivedBytes > 0){
                uint8_t port = 0;
                if (tud_cdc_n_connected(port)) {
                    tud_cdc_n_write(port,(const void*)ucRxData,xReceivedBytes);
                    tud_cdc_n_write_flush(port);
                }
            }
        };
        /***************************************************************/
        // for(uint8_t i = 0;i<CFG_TUD_CDC;i++){
        //     if (tud_cdc_n_connected(i))  tud_cdc_n_write_flush(i);
        // }
    }

}


/*********************************************************************
 * @fn      led_task
 *
 * @brief   task1 program.
 *
 * @param  *pvParameters - Parameters point of task1
 *
 * @return  none
 */


void led_task(void *pvParameters)
{
    int max_delay = 800;
    while(1)
    {
        led_delay_ms +=50;
        // cdc_task();
        tud_connected()?GPIO_SetBits(GPIOC, GPIO_Pin_12):GPIO_ResetBits(GPIOC, GPIO_Pin_12);//USB connected
        // tud_cdc_n_connected(0)?GPIO_SetBits(GPIOC, GPIO_Pin_11):GPIO_ResetBits(GPIOC, GPIO_Pin_11);//USB connected connected

        GPIO_SetBits(GPIOC, GPIO_Pin_9);
        vTaskDelay(pdMS_TO_TICKS((led_delay_ms>max_delay)?max_delay:led_delay_ms));
        GPIO_ResetBits(GPIOC, GPIO_Pin_9);
        vTaskDelay(pdMS_TO_TICKS((led_delay_ms>max_delay)?max_delay:led_delay_ms));
    }
}


#define MSG_LENGHT (DAP_PACKET_SIZE*2)
static uint8_t dapRx_Cache[MSG_LENGHT]={0};
static uint8_t dapTx_Cache[MSG_LENGHT]={0};
uint32_t RxDapMsgLen = 0;
uint32_t recv_len = 0;

void dap_thread(void * p){
    printf("HERE in DAP thread\r\n");
    while(dap_rcv_queue){

        size_t nLen = xMessageBufferReceive(dap_rcv_queue,(void *)dapRx_Cache,MSG_LENGHT,portMAX_DELAY);
        if(nLen<=0) continue;
        RxDapMsgLen = (uint32_t)nLen;
        uint32_t ret_len = DAP_ExecuteCommand(dapRx_Cache,dapTx_Cache);
        // uint32_t pack_size    = ret_len >>16;
        uint32_t resp_len     = ret_len & 0xffff; 

        if(resp_len){
            tud_vendor_write(dapTx_Cache, resp_len);
            tud_vendor_flush();
        }else{
            DAP_Printf("Resp Size is Zero\r\n");
        }
        DAP_Printf("USB recv:(%d,%d) resp:%d\r\n",recv_len,nLen,resp_len);
    }

}
/*********************************************************************/
static uint8_t dapRx_Buff[MSG_LENGHT]={0};

void tusb_task(void *pvParameters)
{

    tusb_rhport_init_t dev_init = {.role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_AUTO};
    tusb_init (BOARD_TUD_RHPORT, &dev_init);
    tinyusb_inited = true;

    cdc_rcv_queue  = xMessageBufferCreate(512);
    dap_rcv_queue  = xMessageBufferCreate(1024*2);

    xTaskCreate(dap_thread, "DAP", DAP_STK_SIZE, NULL, DAP_TASK_PRIO , &DAP_Task_Handler);
    xTaskCreate(cdc_thread, "UART", UART_STK_SIZE, NULL, UART_TASK_PRIO , &UART_Task_Handler);

    TickType_t wake = xTaskGetTickCount();
    while (1) {
        tud_task();
        recv_len = tud_vendor_available();
        if (recv_len) {
            recv_len = tud_vendor_read((void*)dapRx_Buff, sizeof(dapRx_Buff));
            size_t ret = xMessageBufferSend(dap_rcv_queue,(const void *)dapRx_Buff,recv_len,portMAX_DELAY);
            // DAP_Printf("USB SEND MSG:%d : %d \r\n",recv_len,ret);
            continue;
        }
        if (!tud_task_event_ready()){
            xTaskDelayUntil(&wake,pdMS_TO_TICKS(2));
        } 
    }
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
extern void board_init();
int main(void)
{

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);
	board_init();
    
    VCOM_Init();

	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
	printf("FreeRTOS Kernel Version:%s\r\n",tskKERNEL_VERSION_NUMBER);

	GPIO_Toggle_INIT();

	/* create two task */
    xTaskCreate((TaskFunction_t )tusb_task,
                        (const char*    )"usbd",
                        (uint16_t       )USB_STK_SIZE,
                        (void*          )NULL,
                        (UBaseType_t    )USB_TASK_PRIO,
                        (TaskHandle_t*  )&USB_Task_Handler);

    xTaskCreate((TaskFunction_t )led_task,
                    (const char*    )"led",
                    (uint16_t       )LED_STK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )LED_TASK_PRIO,
                    (TaskHandle_t*  )&LED_Task_Handler);

    vTaskStartScheduler();

	while(1)
	{
	    printf("shouldn't run at here!!\n");
	}
}


#if OPT_CMSIS_DAPV1

    uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
    {
      // TODO not Implemented
      (void) itf;
      (void) report_id;
      (void) report_type;
      (void) buffer;
      (void) reqlen;

      return 0;
    }

    void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* RxDataBuffer, uint16_t bufsize)
    {
      uint32_t response_size = TU_MIN(CFG_TUD_HID_EP_BUFSIZE, bufsize);

      // This doesn't use multiple report and report ID
      (void) itf;
      (void) report_id;
      (void) report_type;

      DAP_ProcessCommand(RxDataBuffer, TxDataBuffer);

      tud_hid_report(0, TxDataBuffer, response_size);
    }

#endif


void vApplicationStackOverflowHook(TaskHandle_t Task, char *pcTaskName)
{
  printf("stack overflow (not the helpful kind) for %s\n", *pcTaskName);
  while(1);
}

void vApplicationMallocFailedHook(void)
{
  printf("Malloc Failed\n");
  while(1);
};
