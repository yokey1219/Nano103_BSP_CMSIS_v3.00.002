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

#define RXBUFSIZE 1024
#define PLLCON_SETTING      CLK_PLLCON_84MHz_HXT
#define PLL_CLOCK           84000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
struct _send_cmd
{
  uint16_t msgid;
  uint16_t cmdid;
  uint16_t datalen;
  uint8_t* data;
  struct _send_cmd * next;
};
typedef struct _send_cmd SendCMD;

uint8_t seriallen=0;
uint8_t* serialno;
SendCMD* phead=NULL;
SendCMD* ptail=NULL;


const uint16_t CMDID_LOGIN=0;
const uint16_t CMDID_BINDPARKING=1;
const uint16_t CMDID_REPORTCARPARKING=2;
const uint16_t CMDID_REPORTSTATUS=3;

uint8_t g_loginarg[1]={0};
volatile int32_t g_bwaitserverdata    =FALSE;
/*---------------------------------------------------------------------------------------------------------*/
/*For Uart0                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8Uart0SendData[RXBUFSIZE] = {0};
uint8_t g_u8Uart0RecData[RXBUFSIZE]  = {0};
uint8_t g_serialno[20]={0};
uint8_t g_seriallen=0;

uint8_t g_u8Uart0SendDataini[5]={0xaa,0x01,0x0b,0x00,0x0a};
uint8_t g_u8Uart0SendDataque[4]={0xaa,0x00,0x04,0x04};
uint8_t g_u8Uart0SendDataNoReport[]={0xAA,0x14,0x02,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0xE9 };
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



volatile uint32_t g_transed=TRUE;
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
void NBSendCmd(uint16_t cmdid,uint8_t data[],uint16_t datalen);
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

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady( CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to XTAL */
    CLK->CLKSEL0 = (CLK->CLKSEL0 &~ CLK_CLKSEL0_HCLKSEL_Msk) | (CLK_CLKSEL0_HCLKSEL_HXT);

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN; // UART0 Clock Enable
    CLK->APBCLK |= CLK_APBCLK_UART1_EN; // UART1 Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART0SEL_Msk;
    CLK->CLKSEL1 |= (0x0 << CLK_CLKSEL1_UART0SEL_Pos);// Clock source from external 12 MHz crystal clock
    CLK->CLKSEL2 &= ~CLK_CLKSEL2_UART1SEL_Msk;
    CLK->CLKSEL2 |= (0x0 << CLK_CLKSEL2_UART1SEL_Pos);// Clock source from external 12 MHz crystal clock

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

    /* Lock protected registers */
    SYS_LockReg();

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


void UART0_IRQHandler(void)
{
    UART0_TEST_HANDLE();
}


void UART0_TEST_HANDLE()
{
  
    uint8_t u8InChar=0xFF;
    uint32_t u32IntSts= UART0->INTSTS;

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
                p=(uint8_t*)malloc(len*sizeof(uint8_t));

                
                
                uint32_t idx=0;
                tmp = g_u32comRtail;
                while(g_u32comRhead != tmp) {
                   u8InChar = g_u8RecData[g_u32comRhead];
                   *(p+idx)=u8InChar;
                   g_u32comRhead = (g_u32comRhead == (RXBUFSIZE-1)) ? 0 : (g_u32comRhead+1);
                   g_u32comRbytes--;
                   idx++;
                }
                NBFunctionTest(p,len);
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
  if(g_bwaitserverdata)
  {
    if((*str=='O'&&*(str+1)=='K')||(*(str+2)='O'&&*(str+3)=='K'))
    {
      if(g_step==4)
        g_step=g_finishstep;
      g_bwaitserverdata=FALSE;
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
       //NBSendTrans("AT+NSOCR=DGRAM,17,5684,1\r\n",26);
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
  free(str);
}

void NBSendTrans(uint8_t* strs,int len)
{
  
 
  int i=0;
  //int len=sizeof(strs)/sizeof(uint8_t)-1;
  //while(g_bsend)
  //  ;
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

void NBSendCmd(uint16_t cmdid,uint8_t data[],uint16_t datalen)
{
  SendCMD* cmd=(SendCMD*)malloc(sizeof(SendCMD));
  if(phead==NULL)
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
  }
    cmd->cmdid=cmdid;
    cmd->data=data;
    cmd->datalen=datalen;
    //uint8_t* transdata=(uint8_t*)malloc(sizeof(uint8_t)*(2+2+1+g_seriallen+2+datalen));
    uint16_t transdatalen=2+2+1+g_seriallen+2+datalen;
    char transdata[100]={0};
    sprintf(transdata,"AT+NSOST=0,119.23.12.86,8081,%d,",transdatalen);
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
    NBSendTrans((uint8_t*)transdata,totallen);
    
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
    
    
    //quit sleep
    PC0=0;
    DCSendTransByte(g_u8Uart0SendDataNoReport,24);
    
    
    UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    
    NBSendTrans("AT+CGATT?\r\n",11);
    NVIC_EnableIRQ(UART1_IRQn);
    //NBTransImmedite("AT+NBAND?\r\n",11);
    uint32_t cnt=0;
    while(g_step<1)
    {
      PA14 = 0;
      CLK_SysTickDelay(50000);
      PA14 = 1;
      CLK_SysTickDelay(50000);
      cnt++;
      if(cnt>30)
      {
         NBSendTrans("AT+CGATT?\r\n",11);
         cnt=0;
      }
    }
    cnt=0;
    
    //enter sleep
    //PC0=1;
    
    while(g_step<3)
    {
      
      PA14 = 0;
      CLK_SysTickDelay(2000000);
      if(g_transed)
      {
          NBSendTrans("AT+NSOCR=DGRAM,17,5684,1\r\n",26);
          g_transed=FALSE;
      }
      PA14 = 1;
      CLK_SysTickDelay(2000000);
    }
   
    while(g_step!=g_finishstep)//wait for longin end
    {
      CLK_SysTickDelay(2000000);
    }
    
    uint8_t stadata[3]={0};
    
    while(g_bWait)
    {
          PA14 = 0;
          
               uint32_t len;
                if(g_u32com0Rhead<g_u32com0Rtail)
                  len=g_u32com0Rtail-g_u32com0Rhead;
                else
                  len=RXBUFSIZE-g_u32com0Rhead+g_u32com0Rtail;
           if(len>3)
              {
                
                
                while(g_u8Uart0RecData[g_u32com0Rhead]!=0xaa) 
                {
                  g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                  //if(g_u32com0Rhead>=g_u32com0Rtail) break;
                  if(g_u32com0Rhead<g_u32com0Rtail)
                    len=g_u32com0Rtail-g_u32com0Rhead;
                  else
                    len=RXBUFSIZE-g_u32com0Rhead+g_u32com0Rtail;
                  if(len==0) break;
                }
                if(len>0)
                {
                  
               
                      //if(len<3) continue;
                      //uint16_t tmp;
                      //uint32_t idx=0;
                      
                      g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                      uint8_t cmdlen=g_u8Uart0RecData[g_u32com0Rhead];
                      g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                      uint8_t cmd=g_u8Uart0RecData[g_u32com0Rhead];
                      if(cmd==0x03)
                      {
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           uint8_t sta=g_u8Uart0RecData[g_u32com0Rhead];
                           /*if(sta==0x00)
                           {
                             //NBSendTrans("AT+NSOST=0,119.23.12.86,8081,3,303030\r\n",39);
                             stadata[0]=0;
                             NBSendCmd(1,&stadata);
                           }
                            else if(sta==0x01)
                           {
                             //NBSendTrans("AT+NSOST=0,119.23.12.86,8081,3,313131\r\n",39);
                             stadata
                             NBSendCmd(1,{1});
                           }
                           else
                           {
                             NBSendTrans("AT+NSOST=0,119.23.12.86,8081,3,323232\r\n",39);
                             NBSendCmd(1,{sta});
                           }*/
                           sprintf(stadata,"%x",sta);
                           if(strlen(stadata)==1)
                           {
                             stadata[1]=stadata[0];
                             stadata[0]='0';
                           }
                           NBSendCmd(CMDID_REPORTCARPARKING,stadata,1);
                           
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           //g_uart0que=TRUE;
                      }
                      else if(cmd==0x00)
                      {
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           uint8_t sta=g_u8Uart0RecData[g_u32com0Rhead];
                           if(sta==0x00)
                           {
                             PC0=1;
                           }
                      }
                      else
                      {
                        uint8_t i=0;
                        for(;i<cmdlen;i++)
                        {
                          g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                        }
                        g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                      }
                }
              }   
              //else
              //{
                CLK_SysTickDelay(500000);
              //}
           PA14 = 1;
           CLK_SysTickDelay(500000);
    } // wait user press '0' to exit test

    /* Disable Interrupt */
    UART_DISABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    NVIC_DisableIRQ(UART0_IRQn);
    UART_DISABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    NVIC_DisableIRQ(UART1_IRQn);
    g_bWait =TRUE;
    printf("\nUART Sample Demo End.\n");

}


