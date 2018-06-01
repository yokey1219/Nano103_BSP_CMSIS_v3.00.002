/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/12/18 2:07p $
 * @brief    Transmit and receive data from PC terminal through RS232 interface.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "Nano103.h"

#include "uart.h"

#define RXBUFSIZE 512
#define PLLCON_SETTING      CLK_PLLCON_84MHz_HXT
#define PLL_CLOCK           84000000
#define ACKMAXSIZE 10

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
struct _send_cmd
{
  uint16_t msgid;
  uint16_t cmdid;
  uint16_t datalen;
  uint8_t* data;
  uint8_t waited;
  int32_t replyed;
  struct _send_cmd * next;
};
typedef struct _send_cmd SendCMD;


struct _gms_ack
{
  uint8_t cmdid;
  uint8_t stadata[3];
  struct _gms_ack * next;
};
typedef struct _gms_ack GmsAck;

GmsAck* pgmsackhead=NULL;
GmsAck* pgmsacktail=NULL;

uint8_t seriallen=0;
uint8_t* serialno;
SendCMD* phead=NULL;
SendCMD* ptail=NULL;

uint8_t* g_nbackqueue[ACKMAXSIZE]={0};
volatile uint32_t g_shead=0;
volatile uint32_t g_stail=0;
volatile uint32_t g_ackcnt=0;


const uint16_t CMDID_LOGIN=0;
const uint16_t CMDID_STATESYNC=1;
const uint16_t CMDID_REPORTCARPARKING=2;
const uint16_t CMDID_REPORTSTATUS=3;
const uint16_t CMDID_QUERYMODE=4;

uint8_t g_loginarg[1]={0};
volatile int32_t g_bwaitserverdata    =FALSE;
/*---------------------------------------------------------------------------------------------------------*/
/*For Uart0                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8Uart0SendData[RXBUFSIZE] = {0};
uint8_t g_u8Uart0RecData[RXBUFSIZE]  = {0};
uint8_t g_serialno[20]={0};
uint8_t g_seriallen=0;

uint8_t g_u8Uart0SendDataini[5]={0xaa,0x01,0x0b,0x00,0x0a};   //INFO查询
uint8_t g_u8Uart0SendDataque[4]={0xaa,0x00,0x04,0x04};//状态查询
uint8_t g_u8Uart0SendDataNoReport[]={0xAA,0x14,0x02,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0xE9 };//关闭报告周期
uint8_t g_u8Uart0SendDataReset[]={0xaa,0x01,0x06,0x00,0x07};
volatile int32_t g_uart0binit=TRUE;
volatile int32_t g_uart0que=TRUE;
volatile uint32_t g_uart0myidx=0;

volatile uint32_t g_uart0sendhead=0;
volatile uint32_t g_uart0sendtail=0;

volatile uint32_t g_u32com0Rbytes = 0;
volatile uint32_t g_u32com0Rhead  = 0;
volatile uint32_t g_u32com0Rtail  = 0;
 

volatile uint32_t g_u32testcnt=0;


/*---------------------------------------------------------------------------------------------------------*/
/*For Uart1                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8SendData[RXBUFSIZE] = {0};
uint8_t g_u8RecData[RXBUFSIZE]  = {0};

uint8_t g_u8SendDataini[12]={'A','T','+','N','B','A','N','D','?','\r','\n','\0'};
volatile int32_t g_binit=TRUE;

volatile uint32_t g_myidx=0;
volatile int32_t g_bsend=FALSE;
volatile uint32_t g_sendlen=0;
volatile uint32_t g_sendedidx=0;
volatile uint32_t g_sendhead=0;
volatile uint32_t g_sendtail=0;
volatile uint32_t g_step=0;
volatile uint32_t g_finishstep=100;

volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;
volatile int32_t g_bWait         = TRUE;
volatile int32_t g_i32pointer = 0;
volatile int32_t g_bInitaled    =FALSE;


volatile int32_t g_wakeuptimeout=TRUE;
volatile uint32_t g_transed=TRUE;
/*-------------------------------工作模式--------------------------------------*/
/*有三种工作模式：0初始模式；1工程模式；2工作模式                              */
/*0,初始模式：装置生产的模式，短定时初始化地磁模块，等得切换状态               */
/*1,工程模式：装置出厂安装前的模式，只有定时唤醒检测NB连接，关闭地磁唤醒。     */
/*2,工作模式：安装完毕后，切换到工作模式，在切换过程中，初始化地磁。           */
/*工作模式和工程模式可以相互切换。                                             */
/*-----------------------------------------------------------------------------*/
volatile int32_t g_workmode=0;
/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART1_TEST_HANDLE(void);
void UART0_TEST_HANDLE(void);
void UART_FunctionTest(void);
void NBFunctionTest(uint8_t*,uint32_t);
void NBSendTrans(uint8_t* strs,int len);
void NBTransImmedite(uint8_t* strs,int len);
int32_t NBSendCmd(uint16_t cmdid,uint8_t data[],uint16_t datalen);



volatile uint8_t u8ADF;  // ADC conversion finish flag

void ADC_IRQHandler(void);  //ADC interrupt service routine

void ADC_IRQHandler(void)  //ADC interrupt service routine
{
    uint32_t u32Flag;  //ADC interrupt status flag

    // Get ADC conversion finish interrupt flag
    u32Flag = ADC_GET_INT_FLAG(ADC, ADC_ADF_INT);

    // Check ADC conversion finish interrupt flag
    if(u32Flag & ADC_ADF_INT)
        u8ADF = 1;

    // Clear ADC conversion finish interrupt flag
    ADC_CLR_INT_FLAG(ADC, u32Flag);
}

/**
 *  @brief  Init system clock and I/O multi function .
 *  @param  None
 *  @return None
 */
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable External XTAL (4~24 MHz) */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk; // HXT Enabled
    
    /* Enable External LXT (32 kHz) */
    CLK->PWRCTL |= CLK_PWRCTL_LXTEN_Msk;

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 = (CLK->CLKSEL0 &~ CLK_CLKSEL0_HCLKSEL_Msk) | (CLK_CLKSEL0_HCLKSEL_HXT);

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable
    CLK->APBCLK |= CLK_APBCLK_UART1_EN; // UART1 Clock Enable
    CLK->APBCLK |= CLK_APBCLK_TMR0CKEN_Msk;

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART0SEL_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART0SEL_Pos);// Clock source from external 12 MHz crystal clock
    CLK->CLKSEL2 &= ~CLK_CLKSEL2_UART1SEL_Msk;
    CLK->CLKSEL2 |= (0x0 << CLK_CLKSEL2_UART1SEL_Pos);// Clock source from external 12 MHz crystal clock

    /* Select Timer0 clock source from LXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & ~CLK_CLKSEL1_TMR0SEL_Msk) | CLK_CLKSEL1_TMR0SEL_LXT;
    CLK_SetModuleClock(ADC_MODULE,CLK_CLKSEL1_ADCSEL_HIRC,CLK_ADC_CLK_DIVIDER(5));
    
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD and TXD  */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk|SYS_GPB_MFPL_PB1MFP_Msk|
                       SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
    SYS->GPB_MFPL |=  (SYS_GPB_MFPL_PB0MFP_UART0_RXD|SYS_GPB_MFPL_PB1MFP_UART0_TXD|
                      SYS_GPB_MFPL_PB2MFP_UART0_nRTS  | SYS_GPB_MFPL_PB3MFP_UART0_nCTS);

    /* Set PB multi-function pins for UART1 RXD, TXD, RTS, CTS  */
    SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB4MFP_Msk | SYS_GPB_MFPL_PB5MFP_Msk |
                       SYS_GPB_MFPL_PB6MFP_Msk | SYS_GPB_MFPL_PB7MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB4MFP_UART1_RXD | SYS_GPB_MFPL_PB5MFP_UART1_TXD |
                      SYS_GPB_MFPL_PB6MFP_UART1_nRTS  | SYS_GPB_MFPL_PB7MFP_UART1_nCTS);
    
    /* Set PA.0 multi-function pin for ADC channel 0 */
    SYS->GPA_MFPL = (SYS->GPA_MFPL & ~SYS_GPA_MFPL_PA0MFP_Msk) | SYS_GPA_MFPL_PA0MFP_ADC_CH0;

    /* Disable PA.0 digital input path */
    PA->DINOFF |= ((1 << 0) << GPIO_DINOFF_DINOFF0_Pos);

    /* Enable VBAT DIV 2 */
    SYS->BATDIVCTL |= 1;

    /* Lock protected registers */
    SYS_LockReg();

}




void TMR0_IRQHandler(void)
{
    g_wakeuptimeout=FALSE;
    TIMER_ClearWakeupFlag(TIMER0); /* Clear wakeup interrupt flag */
    TIMER_ClearIntFlag(TIMER0); /* Clear time-out interrupt flag */
    TIMER_Close(TIMER0);
}

/**
 *  @brief  Config UART0.
 *  @param  None
 *  @return None
 */
void UART0_Init()
{
    /* set uart baudrate is 115200 */
    UART_Open(UART0, 9600);
}

/**
 *  @brief  Config UART1.
 *  @param  None
 *  @return None
 */
void UART1_Init()
{
    /* set uart baudrate is 57600 */
    UART_Open(UART1, 9600);
}

/**
 *  @brief  main function.
 *  @param  None
 *  @return None
 */
int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART0 for printf */
    //UART0_Init();

    /* Init UART1 for test */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+---------------------+\n");
    printf("| UART function test  |\n");
    printf("+---------------------+\n");

    /* Configure PB.14 as Output mode */
    GPIO_SetMode(PA, BIT14, GPIO_PMD_OUTPUT);
    /*---------------------------------------------------------------------------------------------------------*/
    /* UART Test Sample                                                                                        */
    /* It sends the received data to HyperTerminal.                                                            */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_FunctionTest();

    while(!g_bInitaled);
    /* Infinite loop */
    while(1);
}



void GMS_DisableSleep()
{
  PC0=0;
}

void GMS_EnableSleep()
{
  PC0=1;
}


void MY_LEDON()
{
  PA14=1;
}

void MY_LEDOFF()
{
  PA14=0;
}

void NB_AddACKStr(uint8_t* str)
{
  if(g_ackcnt<ACKMAXSIZE)
  {
    g_nbackqueue[g_stail]=str;
    g_stail=(g_stail==(ACKMAXSIZE-1))?0:(g_stail+1);
    g_ackcnt++;
  }
}

uint8_t* NB_GetACKStr()
{
  if(g_ackcnt>0)
  {
    uint8_t* _t=g_nbackqueue[g_shead];
    g_shead=(g_shead==(ACKMAXSIZE-1))?0:(g_shead+1);
    g_ackcnt--;
    return _t;
  }
  else
    return NULL;
}

uint8_t* MY_GetNBReplyWithOK()
{
   uint8_t* str=NULL;
   uint8_t* okstr=NULL;
   uint32_t cnt=0;
   while(TRUE)
   {
         CLK_SysTickDelay(50000);
         str=NB_GetACKStr();
         if(str!=NULL&&strcmp(str,"OK\r\n")!=0)
         {
           while(TRUE)
           {
             okstr=NB_GetACKStr();
             if(okstr!=NULL)
               break;
             cnt++;
             if(cnt>60)
               break;
             CLK_SysTickDelay(50000);
           }
           if(okstr!=NULL)
           {
             if(strcmp(okstr,"OK\r\n")==0)
             {
               
                free(okstr);
                okstr=NULL;
                break;
             }
             else
             {
               free(okstr);
               free(str);
               okstr=NULL;
               str=NULL;
             }
           }
         }
         cnt++;
         if(cnt>60)
           break;
         
    }
   return str;
}

uint8_t* MY_GetNBReplyNoOK()
{
   uint8_t* str=NULL;
   uint32_t cnt=0;
   while(TRUE)
   {
         CLK_SysTickDelay(50000);
         str=NB_GetACKStr();
         if(str!=NULL)
         {
           break;
         }
         cnt++;
         if(cnt>240)
           break;
         
    }
   return str;
}

void MY_CHECKCGATT()
{
  NBSendTrans("AT+CGATT?\r\n",11);
  uint8_t* ackstr;
  while(TRUE)
  {
    CLK_SysTickDelay(50000);
    ackstr=NB_GetACKStr();
    if(ackstr!=NULL)
    {
      if(strcmp(ackstr,"+CGATT:1\r\n")==0)//匹配成功
      {
        free(ackstr);
        ackstr=MY_GetNBReplyNoOK();// while((ackstr=NB_GetACKStr())==NULL) CLK_SysTickDelay(50000);
        free(ackstr);
        break;
      }
      else if(strcmp(ackstr,"+CGATT:0\r\n")==0)//匹配情况2成功
      {
        free(ackstr);
        ackstr=MY_GetNBReplyNoOK();//while((ackstr=NB_GetACKStr())==NULL) CLK_SysTickDelay(50000);
        free(ackstr);
        CLK_SysTickDelay(50000);
        NBSendTrans("AT+CGATT?\r\n",11);
      }
    }
    else
    {
      NBSendTrans("AT+CGATT?\r\n",11);
    }
    CLK_SysTickDelay(50000);
  }
  ackstr=NULL;
}

void MY_GETCIMI()
{
       g_seriallen=0;
       NBSendTrans("AT+CIMI\r\n",9);
       uint8_t* str=NULL;
       uint32_t cnt=0;
       while(TRUE)
       {
         CLK_SysTickDelay(50000);
         str=NB_GetACKStr();
         if(str!=NULL)
           break;
         cnt++;
         if(cnt>30)
         {
           NBSendTrans("AT+CIMI\r\n",9);
           cnt=0;
         }
       }
   while(*(str+g_seriallen)!='\r'&&g_seriallen<20)
      {
        g_serialno[g_seriallen]=*(str+g_seriallen);
        g_seriallen+=1;
      }
      if(g_seriallen>=20) g_seriallen=19;
      g_serialno[g_seriallen]='\0';
      free(str);
      str=NULL;
}







void UART0_IRQHandler(void)
{
    UART0_TEST_HANDLE();
}


void UART0_TEST_HANDLE()
{
  
    uint8_t u8InChar=0xFF;
    uint32_t u32IntSts= UART0->INTSTS;

    
     /* Wake Up */
    if (u32IntSts & UART_INTSTS_WKUPIF_Msk) {
        //printf("UART_Wakeup. \n");
        UART0->INTSTS = UART_INTSTS_WKUPIF_Msk; //clear status

        if(UART0->WKUPSTS & UART_WKUPSTS_DATWKSTS_Msk)
            UART0->WKUPSTS = UART_WKUPSTS_DATWKSTS_Msk; //clear status
    }
    
    /* Check Receive Data */
    if(u32IntSts & UART_INTSTS_RDAIF_Msk) {
        printf("\nInput:");

        /* Get all the input characters */
        while(UART_IS_RX_READY(UART0)) {
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART0);           /* Rx trigger level is 1 byte*/

            printf("%c ", u8InChar);
            /* Check if buffer full */
                if(1){//if(g_step>=2){//if(g_u32com0Rbytes < RXBUFSIZE) {
                    /* Enqueue the character */
                    g_u8Uart0RecData[g_u32com0Rtail] = u8InChar;
                    g_u32com0Rtail = (g_u32com0Rtail == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rtail+1);
                    //g_u32com0Rbytes++;
                    //g_u32testcnt++;
                }
      }
       while(g_u8Uart0RecData[g_u32com0Rhead]!=0xaa&&g_u32com0Rhead!=g_u32com0Rtail)
         {
            g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
         }
         
          uint32_t len;
          if(g_u32com0Rhead<=g_u32com0Rtail)
              len=g_u32com0Rtail-g_u32com0Rhead;
          else
              len=RXBUFSIZE-g_u32com0Rhead+g_u32com0Rtail;
           
      
          if(len>4)
          {
                uint32_t temphead=g_u32com0Rhead;
                temphead=(temphead == (RXBUFSIZE-1)) ? 0 : (temphead+1);
                uint8_t cmdlen=g_u8Uart0RecData[temphead];
                //          g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                //          uint8_t cmdlen=g_u8Uart0RecData[g_u32com0Rhead];
                if(len>=(cmdlen+4))
                {
                          g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                          
                          g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                          uint8_t cmd=g_u8Uart0RecData[g_u32com0Rhead];
                          
                          if(cmd==0x03)
                          {
                               GmsAck* _ack=(GmsAck *)malloc(sizeof(GmsAck));
                               _ack->cmdid=cmd;
                               _ack->next=NULL;
                            
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                          
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                               uint8_t sta=g_u8Uart0RecData[g_u32com0Rhead];
                              
                               sprintf(_ack->stadata,"%X",sta);
                               if(_ack->stadata[1]==0)
                               {
                                 _ack->stadata[1]=_ack->stadata[0];
                                 _ack->stadata[0]='0';
                               }
                               _ack->stadata[2]=0;
                               //NBSendCmd(CMDID_REPORTCARPARKING,stadata,1);
                               
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);//电压
                               
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);//磁扰强度
                               
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);//校验码
                               
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                               
                               
                                if(pgmsackhead==NULL)
                               {
                                 pgmsackhead=_ack;
                                 pgmsacktail=_ack;
                               }
                               else
                               {
                                 if(pgmsacktail==NULL)
                                   pgmsacktail=pgmsackhead;
                                 pgmsacktail->next=_ack;
                                 pgmsacktail=_ack;
                               }
                               
                               _ack=NULL;
                               
                          }
                          else if(cmd==0x00)
                          {
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                               uint8_t sta=g_u8Uart0RecData[g_u32com0Rhead];
                               
                               if(sta==0x00)
                               {
                                 GMS_EnableSleep();//PC0=1;
                               }
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                               
                               g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                          }
                          else
                          {
                            uint8_t i=0;
                            for(;i<cmdlen;i++)
                            {
                              g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                            }
                            
                            g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);//校验码
                            
                            g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                          }
                }
          }
    }

     /* Check Tx empty */
    if(u32IntSts & UART_INTSTS_THREIF_Msk) {
           uint16_t tmp;
           tmp = g_uart0sendtail;
           if(g_uart0sendhead != tmp) {
               u8InChar = g_u8Uart0SendData[g_uart0sendhead];
            /* print the received char on screen */
               UART_WRITE(UART0,u8InChar);
               g_uart0sendhead = (g_uart0sendhead == (RXBUFSIZE-1)) ? 0 : (g_uart0sendhead+1);
            
          }
          else
          {
            UART_DISABLE_INT(UART0, ( UART_INTEN_THREIEN_Msk));
          }
        }
}

/**
 *  @brief ISR to handle UART Channel 1 interrupt event.
 *  @param[in] None
 *  @return None
 */
void UART1_IRQHandler(void)
{
    UART1_TEST_HANDLE();
}

/**
 *  @brief UART1 interrupt handle function.
 *  @param[in] None
 *  @return None
 */
void UART1_TEST_HANDLE()
{
    uint8_t u8InChar=0xFF;
    uint32_t u32IntSts= UART1->INTSTS;

    
     /* Wake Up */
    if (u32IntSts & UART_INTSTS_WKUPIF_Msk) {
        
        UART1->INTSTS = UART_INTSTS_WKUPIF_Msk; //clear status

        if(UART1->WKUPSTS & UART_WKUPSTS_DATWKSTS_Msk)
            UART1->WKUPSTS = UART_WKUPSTS_DATWKSTS_Msk; //clear status
    }
    
    /* Check Receive Data */
    if(u32IntSts & UART_INTSTS_RDAIF_Msk) {
        printf("\nInput:");

        /* Get all the input characters */
        while(UART_IS_RX_READY(UART1)) {
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART1);           /* Rx trigger level is 1 byte*/

            printf("%c ", u8InChar);
            
            {
                /* Check if buffer full */
                if(g_u32comRbytes < RXBUFSIZE) {
                    /* Enqueue the character */
                    g_u8RecData[g_u32comRtail] = u8InChar;
                    g_u32comRtail = (g_u32comRtail == (RXBUFSIZE-1)) ? 0 : (g_u32comRtail+1);
                    g_u32comRbytes++;
                }
                
                
            }
            if(u8InChar=='\n')
            {
                uint16_t tmp;
                uint8_t* p;
                uint32_t len;
                if(g_u32comRhead<g_u32comRtail)
                  len=g_u32comRtail-g_u32comRhead;
                else
                  len=RXBUFSIZE-g_u32comRhead+g_u32comRtail;
                if(len>4)
                {  
                    tmp = g_u32comRtail;
                    while(g_u32comRhead!=tmp)
                    {
                      if(g_u8RecData[g_u32comRhead]=='\r')
                        break;
                      g_u32comRhead = (g_u32comRhead == (RXBUFSIZE-1)) ? 0 : (g_u32comRhead+1);
                      g_u32comRbytes--;
                    }
                    
                   if(g_u32comRhead<g_u32comRtail)
                     len=g_u32comRtail-g_u32comRhead;
                   else
                     len=RXBUFSIZE-g_u32comRhead+g_u32comRtail;
                   
                   if(len>2)
                   {
                     
                     p=(uint8_t*)malloc((len-1)*sizeof(uint8_t));
                     uint32_t idx=0;
                     g_u32comRhead = (g_u32comRhead == (RXBUFSIZE-1)) ? 0 : (g_u32comRhead+1);
                     g_u32comRbytes--;
                     g_u32comRhead = (g_u32comRhead == (RXBUFSIZE-1)) ? 0 : (g_u32comRhead+1);
                     g_u32comRbytes--;
                     
                     while(g_u32comRhead != tmp) {
                       u8InChar = g_u8RecData[g_u32comRhead];
                       *(p+idx)=u8InChar;
                       g_u32comRhead = (g_u32comRhead == (RXBUFSIZE-1)) ? 0 : (g_u32comRhead+1);
                       g_u32comRbytes--;
                       idx++;
                    }
                    *(p+idx)='\0';
                    len=len-2;
                    /*NBFunctionTest(p,len);
                    free(p);*/
                    NB_AddACKStr(p);
                    p=NULL;
                   }
                }
            }
           
        }
        printf("\nTransmission Test:");
    }

    /* Check Tx empty */
    if(u32IntSts & UART_INTSTS_THREIF_Msk) {
           uint16_t tmp;
           tmp = g_sendtail;
           if(g_sendhead != tmp) {
               u8InChar = g_u8SendData[g_sendhead];
            /* print the received char on screen */
               UART_WRITE(UART1,u8InChar);
               g_sendhead = (g_sendhead == (RXBUFSIZE-1)) ? 0 : (g_sendhead+1);
            
          }
          else
          {
            UART_DISABLE_INT(UART1, ( UART_INTEN_THREIEN_Msk));
          }
        }
}




void NBFunctionTest(uint8_t* str,uint32_t len)
{
  if(g_bwaitserverdata)//如果处于接收服务器数据状态中
  {
    if((*str=='O'&&*(str+1)=='K')||(*(str+2)=='O'&&*(str+3)=='K'))
    {
      if(g_step==4)//已经发送登录消息。
        g_step=g_finishstep;
      g_bwaitserverdata=FALSE;
    }
    else
    {
      uint32_t sock_no,sock_port,datalen,remainlen;
      uint8_t ipaddr[20]={0};
      uint8_t data[100]={0};
      int _len=sscanf(str,"%u,%s,%u,%u,%s,%u",&sock_no,ipaddr,&sock_port,&datalen,data,&remainlen);
      {
        //计算信息，并去除相对应的命令
        //TODO
        printf("%u",sock_no);
        printf("%s",ipaddr);
        printf("%u",sock_port);
        printf("%u",datalen);
        printf("%s",data);
        printf("%u",remainlen);
        if(sock_no==0)
        {
          uint16_t _msgid;
          uint16_t _cmdid;
          sscanf(data,"%4x%4x",&_msgid,&_cmdid);
          printf("%x",_msgid);
          /*if(phead!=NULL)
          {
            SendCMD* _cmd=phead;
            if(_cmd->msgid==_msgid)
            {
              _cmd->replyed=TRUE;
            }
            else
              _cmd=_cmd->next;
          }*/
        }
      }
      
      
    }
    return;
  }
  
  if(len==2&&*str=='\r'&&*(str+1)=='\n')
  {
  }
  else
     if(len==4&&*str=='O'&&*(str+1)=='K')
    {
      
    }
    else
  if(len>=8)
  {
    if(*str=='+'&&*(str+1)=='N'&&*(str+2)=='B')
    {
      NBSendTrans("AT+CFUN?\r\n",10);
    }
    else if(*str=='+'&&*(str+1)=='C'&&*(str+2)=='F'&&*(str+3)=='U'&&*(str+4)=='N')
    {
      NBSendTrans("AT+CGATT?\r\n",11);
    }
    else if(*str=='+'&&*(str+1)=='C'&&*(str+2)=='G'&&*(str+3)=='A'&&*(str+4)=='T'&&*(str+5)=='T'&&*(str+6)==':'&&*(str+7)!='1')
    {
      
    }
    else if(*str=='+'&&*(str+1)=='C'&&*(str+2)=='G'&&*(str+3)=='A'&&*(str+4)=='T'&&*(str+5)=='T'&&*(str+6)==':'&&*(str+7)=='1')
    {
       g_seriallen=0;
       NBSendTrans("AT+CIMI\r\n",9);
       g_step=1;
    }
    else if (*str=='+'&&*(str+1)=='N'&&*(str+2)=='S'&&*(str+3)=='O'&&*(str+4)=='N'&&*(str+5)=='M'&&*(str+6)=='I'&&*(str+7)==':')  //+NSONMI:0,40
    {
        //AT+NSORF=0,4
        char s[100]={0};
        int ci=0;
        while(ci<99&&ci<(len-8))
        {
          s[ci]=*(str+8+ci);
          ci++;
        }
        s[ci<99?ci:99]='\0';
        
        char word[100]="AT+NSORF=";
        char *end="\r\n";
        strcat(word,s);
        strcat(word,end);
        
        NBSendTrans((uint8_t*)word,11+ci);
        g_bwaitserverdata=TRUE;
    }
    else
    if(g_step==1)
    {
      while(*(str+g_seriallen)!='\r'&&g_seriallen<20)
      {
        g_serialno[g_seriallen]=*(str+g_seriallen);
        g_seriallen+=1;
      }
      if(g_seriallen>=20) g_seriallen=19;
      g_serialno[g_seriallen]='\0';
      NBSendTrans("AT+NSOCR=DGRAM,17,5684,1\r\n",26);
      g_step=2;
    }
  }
  else
  {
    if(g_step==2&&*str=='0')
    { 
      g_step=3;
      g_transed=FALSE;
      NBSendCmd(CMDID_LOGIN,g_loginarg,0);
      g_step=4;
    }
    else
    if(len==7&&*str=='E'&&*(str+1)=='R'&&*(str+2)=='R')
     {
        
        g_step=1;
        g_transed=TRUE;
     }
  }
}

void NBSendTrans(uint8_t* strs,int len)
{
  int i=0;
  
  for(;i<len;i++)
  {
     g_u8SendData[g_sendtail] =*(strs+i);
     g_sendtail = (g_sendtail == (RXBUFSIZE-1)) ? 0 : (g_sendtail+1);
  }
  g_u8SendData[g_sendtail]=0;
  g_sendtail = (g_sendtail == (RXBUFSIZE-1)) ? 0 : (g_sendtail+1);
  UART_ENABLE_INT(UART1, ( UART_INTEN_THREIEN_Msk));
}


void NBTransImmedite(uint8_t* strs,int len)
{
  /*for(int i=0;i<len;i++)
    UART_WRITE(UART1,*(strs+i));
  UART_WRITE(UART1,'\0');*/
}


void DCSendTrans(uint8_t* strs,int len)
{
  int i=0;
  for(;i<len;i++)
  {
     g_u8Uart0SendData[g_uart0sendtail] =*(strs+i);
     g_uart0sendtail = (g_uart0sendtail == (RXBUFSIZE-1)) ? 0 : (g_uart0sendtail+1);
  }
  g_u8Uart0SendData[g_uart0sendtail]=0;
  g_uart0sendtail = (g_uart0sendtail == (RXBUFSIZE-1)) ? 0 : (g_uart0sendtail+1);
   UART_ENABLE_INT(UART0, ( UART_INTEN_THREIEN_Msk));
}

void DCSendTransByte(uint8_t bytes[],int len)
{
    int i=0;
    //int len=(sizeof(bytes) / sizeof(bytes[0]));
    for(;i<len;i++)
    {
        g_u8Uart0SendData[g_uart0sendtail] =bytes[i];
        g_uart0sendtail = (g_uart0sendtail == (RXBUFSIZE-1)) ? 0 : (g_uart0sendtail+1);
        UART_ENABLE_INT(UART0, ( UART_INTEN_THREIEN_Msk));
    }
}

volatile uint16_t g_totalmsgid=0;

int32_t NBSendCmd(uint16_t cmdid,uint8_t data[],uint16_t datalen)
{
  
  NBSendTrans("AT+NSOCR=DGRAM,17,5684,1\r\n",26);
  uint8_t* str=MY_GetNBReplyWithOK();
  if(str==NULL) return FALSE;
  uint32_t thisport;
  sscanf(str,"%u",&thisport);
  free(str);
    str=NULL;
  SendCMD* cmd=(SendCMD*)malloc(sizeof(SendCMD));
 /* if(phead==NULL)
  {
    cmd->msgid=0;
    phead=cmd;
    ptail=cmd;
  }
  else
  {
    uint16_t _msgid;
    _msgid=ptail->msgid>65535?0:ptail->msgid+1;
    cmd->msgid=_msgid;
    ptail->next=cmd;
    ptail=cmd;
  }*/
    cmd->msgid=g_totalmsgid++;
    cmd->cmdid=cmdid;
    cmd->data=data;
    cmd->datalen=datalen;
    //uint8_t* transdata=(uint8_t*)malloc(sizeof(uint8_t)*(2+2+1+g_seriallen+2+datalen));
    uint16_t transdatalen=2+2+1+g_seriallen+2+datalen;
    char transdata[100]={0};
    sprintf(transdata,"AT+NSOST=%u,119.23.12.86,8081,%d,",thisport,transdatalen);
    
    uint16_t totallen=strlen(transdata)+transdatalen*2+2;
    char tmpbyte[3]={0};
    sprintf(tmpbyte,"%X",cmd->msgid>>8);
    if(strlen(tmpbyte)==1)
    {
      strcat(transdata,"0");
    }
    strcat(transdata,tmpbyte);
    
    sprintf(tmpbyte,"%X",cmd->msgid&0x00ff);
    if(strlen(tmpbyte)==1)
    {
      strcat(transdata,"0");
    }
    strcat(transdata,tmpbyte);
    
    
    sprintf(tmpbyte,"%X",cmdid>>8);
    if(strlen(tmpbyte)==1)
    {
      strcat(transdata,"0");
    }
    strcat(transdata,tmpbyte);
    
    sprintf(tmpbyte,"%X",cmdid&0x00ff);
    if(strlen(tmpbyte)==1)
    {
      strcat(transdata,"0");
    }
    strcat(transdata,tmpbyte);
    
    sprintf(tmpbyte,"%X",g_seriallen);
    if(strlen(tmpbyte)==1)
    {
      strcat(transdata,"0");
    }
    strcat(transdata,tmpbyte);
    
    int i=0;
    for(;i<g_seriallen;i++)
    { 
      sprintf(tmpbyte,"%X",g_serialno[i]);
      if(strlen(tmpbyte)==1)
      {
        strcat(transdata,"0");
      }
      strcat(transdata,tmpbyte);
    }
   
     sprintf(tmpbyte,"%X",datalen>>8);
    if(strlen(tmpbyte)==1)
    {
      strcat(transdata,"0");
    }
    strcat(transdata,tmpbyte);
    
    sprintf(tmpbyte,"%X",datalen&0x00ff);
    if(strlen(tmpbyte)==1)
    {
      strcat(transdata,"0");
    }
    strcat(transdata,tmpbyte);
   
    strcat(transdata,(char*)data);
    strcat(transdata,"\r\n");
    while(TRUE)
    {
      NBSendTrans((uint8_t*)transdata,totallen);
      CLK_SysTickDelay(50000);
      str=MY_GetNBReplyWithOK();
      free(str);
      str=NULL;
      //CLK_SysTickDelay(1500000);
      str=MY_GetNBReplyNoOK();
      if(str==NULL)
      {
      }
      else
      if (*str=='+'&&*(str+1)=='N'&&*(str+2)=='S'&&*(str+3)=='O'&&*(str+4)=='N'&&*(str+5)=='M'&&*(str+6)=='I'&&*(str+7)==':')  //+NSONMI:0,40
      {
          char s[7]={0};
          
          sscanf(str,"+NSONMI:%s",s);
          
          char word[100]="AT+NSORF=";
          char *end="\r\n";
          strcat(word,s);
          strcat(word,end);
          int ci=strlen(s);
          NBSendTrans((uint8_t*)word,11+ci);
          free(str);
          str=NULL;
          str=MY_GetNBReplyWithOK();
          if(cmd->cmdid==CMDID_QUERYMODE)
          {
            uint32_t sock_no,sock_port,datalen,remainlen;
            uint32_t ipaddr1;
            uint32_t ipaddr2;
            uint32_t ipaddr3;
            uint32_t ipaddr4;
            uint8_t data[13]={0};
            int _len=sscanf(str,"%u,%u.%u.%u.%u,%u,%u,%12s,%u\r\n",&sock_no,&ipaddr1,&ipaddr2,&ipaddr3,&ipaddr4,&sock_port,&datalen,data,&remainlen);
          
            uint32_t _msgid;
            uint32_t _cmdid;
            uint32_t _modeid;
            uint32_t _intervalid;
            sscanf(data,"%4x%4x%4x%4x",&_msgid,&_cmdid,&_modeid,&_intervalid);
            //printf("%x",_msgid);
            g_workmode=_modeid;
              
            free(str);
            str=NULL;
            
            if(cmd->msgid==_msgid)
               break;
          }
          else
          {
            uint32_t sock_no,sock_port,datalen,remainlen;
            uint32_t ipaddr1;
            uint32_t ipaddr2;
            uint32_t ipaddr3;
            uint32_t ipaddr4;
            uint8_t data[9]={0};
            int _len=sscanf(str,"%u,%u.%u.%u.%u,%u,%u,%8s,%u\r\n",&sock_no,&ipaddr1,&ipaddr2,&ipaddr3,&ipaddr4,&sock_port,&datalen,data,&remainlen);
          
            uint32_t _msgid;
            uint32_t _cmdid;
            sscanf(data,"%4x%4x",&_msgid,&_cmdid);
            //printf("%x",_msgid);
              
            free(str);
            str=NULL;
            
            if(cmd->msgid==_msgid)
              break;
          }
      }
    }
    
    sprintf(transdata,"AT+NSOCL=%u\r\n",thisport);
    NBSendTrans((uint8_t*)transdata,strlen(transdata));
    str=MY_GetNBReplyNoOK();
    free(str);
    str=NULL;
          
          free(cmd);
          cmd=NULL;
    return TRUE;
}


void Enter_PowerDown()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable UART wake up interrupt */
    UART_EnableInt(UART0, UART_INTEN_WKUPIEN_Msk);
    UART_EnableInt(UART1, UART_INTEN_WKUPIEN_Msk);
    /* Enable UART Data Wake up function */
    UART0->WKUPEN |= UART_WKUPEN_WKDATEN_Msk;
    UART1->WKUPEN |= UART_WKUPEN_WKDATEN_Msk;

    NVIC_EnableIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART1_IRQn);

    /* Enable system wake up interrupt */
    //CLK->PWRCTL |= CLK_PWRCTL_WAKEINT_EN;
    //NVIC_EnableIRQ(PDWU_IRQn);
    
    TIMER_Open(TIMER0, TIMER_ONESHOT_MODE, 1); /* Initial Timer0 to periodic mode with 1Hz */
    TIMER_SET_PRESCALE_VALUE(TIMER0,255);
    TIMER_SET_CMP_VALUE(TIMER0, 125*60*60*2);
              
    TIMER_EnableWakeup(TIMER0); /* Enable timer wake up system */
    TIMER_EnableInt(TIMER0);    /* Enable Timer0 interrupt */
    NVIC_EnableIRQ(TMR0_IRQn);  /* Enable Timer0 IRQ */

    TIMER_Start(TIMER0); /* Start Timer0 counting */

    //SYS_UnlockReg(); /* Unlock protected registers */
              
    g_wakeuptimeout=TRUE;
                

    /* Enter Power Down mode */
    CLK_PowerDown();
}


/**
 *  @brief The function is UART demo code.
 *  @param[in] None
 *  @return None
 */
void UART_FunctionTest()
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Function Test                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART1 and PC.
        UART0 is set to debug port. UART1 is enable RDA and RLS interrupt.
        When inputting char to terminal screen, RDA interrupt will happen and
        UART1 will print the received char on screen.
    */

    /* Enable Interrupt */
    //UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    //UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    UART0_Init();
    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    NVIC_EnableIRQ(UART0_IRQn);
    
    
    //quit sleep地磁模块退出睡眠
    GMS_DisableSleep();//PC0=0;
    DCSendTransByte(g_u8Uart0SendDataNoReport,24);//关闭报告周期
    
    
    UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    
    //NBSendTrans("AT+CGATT?\r\n",11);
    NVIC_EnableIRQ(UART1_IRQn);
    //NBTransImmedite("AT+NBAND?\r\n",11);
    uint32_t cnt=0;
    
    MY_CHECKCGATT();
    cnt=0;
    MY_GETCIMI();
    
    //enter sleep
    GMS_EnableSleep();//PC0=1;//地磁模块进入睡眠
    
    NBSendCmd(CMDID_LOGIN,g_loginarg,0);
    
    g_step=g_finishstep;
    
    uint32_t sleepcnt=0;
    uint32_t retry=0;
    CLK_SysTickDelay(1000*1000*20);//等待20S，GSM初始化完成
    
    while(g_bWait)
    {
         MY_LEDOFF();//PA14 = 0;
         if(g_workmode==0)
         {
             MY_CHECKCGATT();
             
             if(pgmsackhead==NULL)
              {
                sleepcnt++;
              }
              else
              {
                GmsAck* thisack=pgmsackhead;
                pgmsackhead=pgmsackhead->next;
                NBSendCmd(CMDID_REPORTCARPARKING,thisack->stadata,1);
                free(thisack);
                thisack=NULL;
                if(thisack->stadata[1]=='0')
                  NBSendCmd(CMDID_QUERYMODE,g_loginarg,0);
                
              }
              if(g_workmode==0)
              {
                if(retry>3)
                {
                  NBSendCmd(CMDID_STATESYNC,g_loginarg,0);
                  Enter_PowerDown();
                  sleepcnt=0;
                  retry=0;
                }
                else if(sleepcnt>20)
                {
                  GMS_DisableSleep();
                  CLK_SysTickDelay(100*1000*1);
                  DCSendTransByte(g_u8Uart0SendDataReset,5);
                  GMS_EnableSleep();
                  retry++;
                  sleepcnt=0;
                  
                }
                else
                {
                  CLK_SysTickDelay(1000*1000*1);
                }
                  
              }
              else
              {
                  CLK_SysTickDelay(1000*1000*1);
              }
         }
         if(g_workmode==1)
         {
             MY_CHECKCGATT();
             NBSendCmd(CMDID_QUERYMODE,g_loginarg,0);
             if(g_workmode==1)
             {
                NBSendCmd(CMDID_STATESYNC,g_loginarg,0);
                Enter_PowerDown();
             }
             
         }
         else if(g_workmode==2)//正常工作模式
         {
              
              if(pgmsackhead==NULL)
              {
                sleepcnt++;
              }
              else
              {
                GmsAck* thisack=pgmsackhead;
                pgmsackhead=pgmsackhead->next;
                NBSendCmd(CMDID_REPORTCARPARKING,thisack->stadata,1);
                free(thisack);
                thisack=NULL;
                sleepcnt=0;
              }
           
              if(sleepcnt<90)
              {
                //CLK_SysTickDelay(500000);
                //MY_LEDON();
                //CLK_SysTickDelay(500000);
                CLK_SysTickDelay(1000000);
              }
              else
              {
               
                NBSendCmd(CMDID_STATESYNC,g_loginarg,0);
                Enter_PowerDown();
                
                if(g_wakeuptimeout)
                {
                  GMS_DisableSleep();
                  TIMER_ClearWakeupFlag(TIMER0); /* Clear wakeup interrupt flag */
                  TIMER_ClearIntFlag(TIMER0); /* Clear time-out interrupt flag */
                  TIMER_Close(TIMER0);
                }
                else
                {
                    NBSendCmd(CMDID_QUERYMODE,g_loginarg,0);
                }
                //NBSendCmd(CMDID_LOGIN,g_loginarg,0);
                
                DCSendTransByte(g_u8Uart0SendDataque,4);
                sleepcnt=0;
                CLK_SysTickDelay(500000);
                GMS_EnableSleep();   
              }
         }
                
    } // wait user press '0' to exit test

    /* Disable Interrupt */
    UART_DISABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    NVIC_DisableIRQ(UART0_IRQn);
    UART_DISABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    NVIC_DisableIRQ(UART1_IRQn);
    g_bWait =TRUE;
    printf("\nUART Sample Demo End.\n");

}


/*
void ClearReplyedCmd()
{
      if(phead==NULL) return;
      SendCMD* _tmpcmd=phead;
      SendCMD* newhead,newtail;
      newhead=NULL;
      newtail=NULL;
      while(_tmpcmd!=NULL)
      {
        if(_tmpcmd->replyed==TRUE)
        {
          free(_tmpcmd);
        }
        else
        {
          if(newhead==NULL)
          {
            newhead=_tmpcmd;
            newtail=_tmpcmd;
          }
          else
          {
            newtail->next=_tmpcmd;
            newtail=_tmpcmd;
          }
        }
        _tmpcmd=_tmpcmd->next;
          
      }
}*/


