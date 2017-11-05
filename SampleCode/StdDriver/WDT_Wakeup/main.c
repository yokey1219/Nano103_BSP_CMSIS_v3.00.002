/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/12/29 3:06p $
 * @brief    Use WDT to wake up system from Power-down mode periodically.
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Nano103.h"

void WDT_IRQHandler(void)  // WDT interrupt service routine
{
    // Clear WDT interrupt flag
    WDT_CLEAR_TIMEOUT_INT_FLAG();

    // Check WDT wake up flag
    if(WDT_GET_TIMEOUT_WAKEUP_FLAG()) {
        printf("Wake up by WDT\n");
        // Clear WDT wake up flag
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /* Enable external 12MHz HXT, 32KHz LXT and HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk | CLK_PWRCTL_HIRC0EN_Msk | CLK_PWRCTL_HIRC1EN_Msk | CLK_PWRCTL_MIRCEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk | CLK_STATUS_HIRC0STB_Msk | CLK_STATUS_HIRC1STB_Msk | CLK_STATUS_MIRCSTB_Msk);

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(WDT_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_UART0_CLK_DIVIDER(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDTSEL_LXT, 0);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~( SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD );

    /* Lock protected registers */
    SYS_LockReg();
}

int32_t main (void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
    SYS_Init();

    UART_Open(UART0, 115200); /* Init UART to 115200-8n1 for print message */

    printf("\nThis sample code demonstrate using WDT to wake system up from power down mode\n");

    /*
      This sample code demonstrate using WDT to wake system up from power down mode
      Configure WDT timeout every 2^14 WDT clock, disable system reset, enable wake up system
      Enable WDT timeout interrupt.
      WDT timeout interrupt will wake up system.
     */

    // WDT register is locked, so it is necessary to unlock protect register before configure WDT
    SYS_UnlockReg();

    // WDT timeout every 2^14 WDT clock, disable system reset, enable wake up system
    WDT_Open(WDT_TIMEOUT_2POW14, 0, FALSE, TRUE);

    WDT_EnableInt();  // Enable WDT timeout interrupt
    NVIC_EnableIRQ(WDT_IRQn);  // Enable NVIC WDT interrupt

    while(1) {
        // Wait 'til UART FIFO empty to get a cleaner console out
        while(!UART_IS_TX_EMPTY(UART0));
        CLK_PowerDown();
    }
}

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/


