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
#include "Nano103.h"

#include "uart.h"

#define RXBUFSIZE 1024
#define PLLCON_SETTING      CLK_PLLCON_84MHz_HXT
#define PLL_CLOCK           84000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/



/*---------------------------------------------------------------------------------------------------------*/
/*For Uart0                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8Uart0SendData[RXBUFSIZE] = {0};
uint8_t g_u8Uart0RecData[RXBUFSIZE]  = {0};

uint8_t g_u8Uart0SendDataini[5]={0xaa,0x01,0x0b,0x00,0x0a};
uint8_t g_u8Uart0SendDataque[4]={0xaa,0x00,0x04,0x04};
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
                if(g_u32com0Rbytes < RXBUFSIZE) {
                    /* Enqueue the character */
                    g_u8Uart0RecData[g_u32com0Rtail] = u8InChar;
                    g_u32com0Rtail = (g_u32com0Rtail == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rtail+1);
                    //g_u32com0Rbytes++;
                    //g_u32testcnt++;
                }
    }
      // if(g_u32testcnt>8)
      // {
      //    NBSendTrans("AT+NSOST=0,119.23.12.86,8081,3,313131\r\n",39);
      //    g_u32testcnt=0;
      // }
    }

    /* Check Tx empty */
    /*if(u32IntSts & UART_INTSTS_THREIF_Msk) {
      if(g_uart0binit)
        {
          
          for(g_uart0myidx=0;g_uart0myidx<5;g_uart0myidx++)
          {
            u8InChar=g_u8Uart0SendDataini[g_uart0myidx];
            UART_WRITE(UART0,u8InChar);
            
          }
          g_uart0binit=FALSE;
        }
         
        if(g_uart0que)
        {
          for(g_uart0myidx=0;g_uart0myidx<4;g_uart0myidx++)
          {
            u8InChar=g_u8Uart0SendDataque[g_uart0myidx];
            UART_WRITE(UART0,u8InChar);
            
          }
          g_uart0que=FALSE;
        }
      
        }*/
    
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

           // if(u8InChar == '0') {
           //     g_bWait = FALSE;
           // }
            //if(u8InChar=='\n'||u8InChar=='\0') continue;
            
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
                uint8_t* p;
                uint32_t len;
                if(g_u32comRhead<g_u32comRtail)
                  len=g_u32comRtail-g_u32comRhead;
                else
                  len=RXBUFSIZE-g_u32comRhead+g_u32comRtail;
                p=(uint8_t*)malloc(len*sizeof(uint8_t));
                
                uint16_t tmp;
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
        //uint16_t tmp;
        //tmp = g_u32comRtail;
       // if(g_u32comRhead != tmp) {
       //     u8InChar = g_u8RecData[g_u32comRhead];
            /* print the received char on screen */
            //UART_WRITE(UART1,u8InChar);
       //     g_u32comRhead = (g_u32comRhead == (RXBUFSIZE-1)) ? 0 : (g_u32comRhead+1);
       //     g_u32comRbytes--;
       // }
        
        /*if(g_binit)
        {
          
          for(g_myidx=0;g_myidx<12;g_myidx++)
          {
            u8InChar=g_u8SendDataini[g_myidx];
            UART_WRITE(UART1,u8InChar);
            
          }
          g_binit=FALSE;
        }*/
        //if(g_bsend)
        {
          //for(g_myidx=0;g_myidx<g_sendlen;g_myidx++)
          //{
          //  u8InChar=g_u8SendData[g_myidx];
          //  UART_WRITE(UART1,u8InChar);
          //}
          /*if(g_sendedidx<g_sendlen)
          {
             u8InChar=g_u8SendData[g_sendedidx];
             UART_WRITE(UART1,u8InChar);
             g_sendedidx++;
          }
          else
          {
            g_bsend=FALSE;
            g_sendedidx=0;
          }*/
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
}



void NBFunctionTest(uint8_t* str,uint32_t len)
{
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
      //NBSendTrans("AT+CGATT?\r\n",11);
    }
    else if(*str=='+'&&*(str+1)=='C'&&*(str+2)=='G'&&*(str+3)=='A'&&*(str+4)=='T'&&*(str+5)=='T'&&*(str+6)==':'&&*(str+7)=='1')
    {
       NBSendTrans("AT+NSOCR=DGRAM,17,5684,1\r\n",26);
       g_step=1;
    }
    else if (*str=='+'&&*(str+1)=='N'&&*(str+2)=='S'&&*(str+3)=='O'&&*(str+4)=='N'&&*(str+5)=='M'&&*(str+6)=='I')  //+NSONMI:0,40
    {
        //AT+NSORF=0,4
        NBSendTrans("AT+NSORF=0,40\r\n",15);
    }
  }
  else
  {
    if(g_step==1&&*str=='0')
    {
      //NBSendTrans("AT+NSOST=0,119.23.12.86,8081,3,313233\r\n",39);
      g_step=2;
      g_transed=FALSE;
    }
    //else if(g_step==1&&*str=='O'&&*(str+1)=='K')
    //{
    //  NBSendTrans("AT+NSOCL=0\r\n",12);
    //}
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
  //g_sendlen=i;
  //g_sendedidx=0;
  //g_bsend=TRUE;
   UART_ENABLE_INT(UART1, ( UART_INTEN_THREIEN_Msk));
  //NBTransImmedite(strs,len);
}


void NBTransImmedite(uint8_t* strs,int len)
{
  /*for(int i=0;i<len;i++)
    UART_WRITE(UART1,*(strs+i));
  UART_WRITE(UART1,'\0');*/
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
    
    UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    
    NBSendTrans("AT+CGATT?\r\n",11);
    NVIC_EnableIRQ(UART1_IRQn);
    //NBTransImmedite("AT+NBAND?\r\n",11);
    uint32_t cnt=0;
    while(g_step<2)
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
    
    while(g_step!=2)
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
   
    UART0_Init();
    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk));
    NVIC_EnableIRQ(UART0_IRQn);
    
    while(1)
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
                           if(sta==0x00)
                           {
                             NBSendTrans("AT+NSOST=0,119.23.12.86,8081,3,303030\r\n",39);
                           }
                            else if(sta==0x01)
                           {
                             NBSendTrans("AT+NSOST=0,119.23.12.86,8081,3,313131\r\n",39);
                           }
                           else
                           {
                             NBSendTrans("AT+NSOST=0,119.23.12.86,8081,3,323232\r\n",39);
                           }
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           g_u32com0Rhead = (g_u32com0Rhead == (RXBUFSIZE-1)) ? 0 : (g_u32com0Rhead+1);
                           //g_uart0que=TRUE;
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


