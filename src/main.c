/**
  ******************************************************************************
  * @file    main.c
  * @author  Microcontroller Division
  * @version V1.0.4
  * @date    Feb-2014
  * @brief   Main program body
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  */
 
/* Includes ------------------------------------------------------------------*/

#include "stm32l1xx.h"
#include "stdio.h"
#include "discover_board.h"
#include "stm32l_discovery_lcd.h"
#include "ds18b20.h"
#include <stdbool.h>
#include <math.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

#define DEBUG_SWD_PIN  1  /* needs to be set to 1 to enable SWD debug pins, set to 0 for power consumption measurement*/
#define DEBUG_DISABLE_AFTER_SLEEP 1 // same as DEBUG_SWD_PIN = 0 but wait one sleep
#define DEBUG_LED 0

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static volatile uint32_t TimingDelay;

/* Private function prototypes -----------------------------------------------*/
static void RCC_Configuration(void);
static void RTC_Configuration(void);
static void conf_analog_debug_pins(void);
static void Init_GPIOs (void);
static void configureWakeup (void);
static void clearUserButtonFlag(void);
/*******************************************************************************/


int main(void)
{ 
    int32_t temperature_C;
    RCC_ClocksTypeDef RCC_Clocks;
    bool firstRound = true;

    /*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32l1xx_md.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32l1xx.c file
     */ 

    /* Configure Clocks for Application need */
    RCC_Configuration();

    /* Configure RTC Clocks */
    RTC_Configuration();

    /* Set internal voltage regulator to 1.8V */
    PWR_VoltageScalingConfig(PWR_VoltageScaling_Range1);

    /* Wait Until the Voltage Regulator is ready */
    while (PWR_GetFlagStatus(PWR_FLAG_VOS) != RESET) ;

    /* Enable debug features in low power modes (Sleep, STOP and STANDBY) */
#ifdef  DEBUG_SWD_PIN
    DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, ENABLE);
#endif

    /* Configure SysTick IRQ and SysTick Timer to generate interrupts */
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(RCC_Clocks.HCLK_Frequency / 500);

    /* Init I/O ports */
    Init_GPIOs();

    /* Initializes the LCD glass */
    LCD_GLASS_Configure_GPIO();
    LCD_GLASS_Init();
      
    /* Display Welcome message */

    LCD_GLASS_ScrollSentence("       HELLO THERE - HAVE A NICE DAY", 1, 150);

    /* Configure Wakeup from sleep using RTC event*/
    configureWakeup();

    ds18b20_init();
    ds18b20_setResolution(DS18B20_RESOLUTION_11BIT);
    LCD_GLASS_Clear();

    while(1)
    {
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
        TimingDelay = 2000;
        while(!ds18b20_work())
        {
            if (TimingDelay == 0)
            {
                LCD_GLASS_DisplayString("ERR    ");
                ds18b20_init();
                break;
            }
        }
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, DISABLE);

        if (TimingDelay != 0)
        {
            /* Process measured Temperature data - calculate average temperature value in °C */
            temperature_C = round(ds18b20_getTemp(0) * 10);

            /* print average temperature value in °C  */
            //sprintf(strDisp, "%d.%d °C", temperature_C/10, temperature_C % 10 );
            uint8_t n = 1;
            LCD_GLASS_WriteChar((temperature_C < 0) ? '-' : ' ', POINT_OFF, COLUMN_OFF, n++);
            temperature_C = abs(temperature_C);
            LCD_GLASS_WriteChar((temperature_C > 100) ? '0' + (temperature_C/100)%10 : ' ', POINT_OFF, COLUMN_OFF, n++);
            LCD_GLASS_WriteChar('0' + (temperature_C/10) % 10, POINT_ON, COLUMN_OFF, n++);
            LCD_GLASS_WriteChar('0' + temperature_C%10, POINT_OFF, COLUMN_OFF, n++);
            LCD_GLASS_WriteChar('°', POINT_OFF, COLUMN_OFF, n++);
        }

#if DEBUG_LED
        GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);
        RCC_AHBPeriphClockCmd(LD_GPIO_PORT_CLK | USERBUTTON_GPIO_CLK, DISABLE);
#endif

        /* Disable SysTick */
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

        /* Enable RTC Wakeup */
        RTC_WakeUpCmd(ENABLE);

        /* Clear WakeUp flag */
        PWR_ClearFlag(PWR_FLAG_WU);
    
        /* Enter in wait for interrupt stop mode*/
        PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
    
        RCC_Configuration();  // reinitialize clock

        /* After Wake up : Disable Wake up from RTC*/
        RTC_WakeUpCmd(DISABLE);
    
        /* Enable SysTick */
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

#if DEBUG_LED
        //LED on for troubleshooting
        RCC_AHBPeriphClockCmd(LD_GPIO_PORT_CLK | USERBUTTON_GPIO_CLK, ENABLE);
        GPIO_HIGH(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);
#endif

#if DEBUG_DISABLE_AFTER_SLEEP
        if (firstRound)
        {
            DBGMCU_Config(DBGMCU_SLEEP | DBGMCU_STOP | DBGMCU_STANDBY, DISABLE);
            conf_analog_debug_pins();
            firstRound = false;
        }
#endif
    }
}

void configureWakeup (void)
{
  /* Declare initialisation structures for (NVIC) and external interupt (EXTI) */
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;

  /* Clear IT pending bit from external interrupt Line 20 */
  EXTI_ClearITPendingBit(EXTI_Line20);
  
  /* Initialise EXTI using its init structure */
  EXTI_InitStructure.EXTI_Line = EXTI_Line20;			 // interrupt generated on RTC Wakeup event (Line 20)
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;    // Use EXTI line as interrupt
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; // Trigg interrupt on rising edge detection
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;				 // Enable EXTI line
  EXTI_Init(&EXTI_InitStructure);						 
  
  /* Initialise the NVIC interrupts (IRQ) using its init structure */
  NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;        // set IRQ channel to RTC Wakeup Interrupt  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	 // set channel Preemption priority to 0
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;         // set channel sub priority to 0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	         // Enable channel
  NVIC_Init(&NVIC_InitStructure);
  
  /* Clear Wake-up flag */  
  PWR->CR |= PWR_CR_CWUF;

  /* Enable PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE); 

  /* Allow access to RTC */
  PWR_RTCAccessCmd(ENABLE);
#if 1
  /* Enable Low Speed External clock */
  RCC_LSEConfig(RCC_LSE_ON); 

  /* Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);

  /* Select LSE clock as RCC Clock source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
#else
  /* Enable Low Speed Internal clock */
  RCC_LSICmd(ENABLE);

  /* Wait till LSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);

  /* Select LSI clock as RCC Clock source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
#endif
  /* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();

  /* Select 1Hz clock for RTC wake up*/
  RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
  
  /* Set Wakeup auto-reload value to 60 sec */
  RTC_SetWakeUpCounter(60);

  /* Clear RTC Interrupt pending bit */
  RTC_ClearITPendingBit(RTC_IT_WUT);
  
  /* Clear EXTI line20 Interrupt pending bit */
  EXTI_ClearITPendingBit(EXTI_Line20);

  /* Enable the Wakeup Interrupt */
  RTC_ITConfig(RTC_IT_WUT, ENABLE);
}

void setUserButtonFlag(void)
{
}


/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{  
  /* Enable HSI Clock */
  RCC_HSICmd(ENABLE);
  
  /*!< Wait till HSI is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET)
  {}

  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
  
  RCC_MSIRangeConfig(RCC_MSIRange_6);

  RCC_HSEConfig(RCC_HSE_OFF);  
  if(RCC_GetFlagStatus(RCC_FLAG_HSERDY) != RESET )
  {
    while(1);
  }
 
  /* Enable  comparator clock LCD and PWR mngt */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_LCD | RCC_APB1Periph_PWR, ENABLE);
}


void RTC_Configuration(void)
{
  /* Allow access to the RTC */
  PWR_RTCAccessCmd(ENABLE);

  /* Reset Backup Domain */
  RCC_RTCResetCmd(ENABLE);
  RCC_RTCResetCmd(DISABLE);

  /* LSE Enable */
  RCC_LSEConfig(RCC_LSE_ON);

  /* Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {}
  
  RCC_RTCCLKCmd(ENABLE);
   
  /* LCD Clock Source Selection */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
}

static void conf_analog_debug_pins(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOs clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Disable GPIOs clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
}

static void conf_analog_all_GPIOS(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOs clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                          RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
                          RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, ENABLE);

    /* Configure all GPIO port pins in Analog Input mode (floating input trigger OFF) */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    GPIO_Init(GPIOH, &GPIO_InitStructure);

#if  DEBUG_SWD_PIN == 1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All & (~GPIO_Pin_13) & (~GPIO_Pin_14);
#endif

    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Disable GPIOs clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                          RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
                          RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, DISABLE);
}

static void  Init_GPIOs (void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    conf_analog_all_GPIOS();   /* configure all GPIOs as analog input */

    /* Enable GPIOs clock */
    //RCC_AHBPeriphClockCmd(LD_GPIO_PORT_CLK | USERBUTTON_GPIO_CLK, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                          RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
                          RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, ENABLE);

    /* USER button and WakeUP button init: GPIO set in input interrupt active mode */

    /* Configure User Button pin as input */
    GPIO_InitStructure.GPIO_Pin = USERBUTTON_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_Init(USERBUTTON_GPIO_PORT, &GPIO_InitStructure);

    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);

    /* Configure User Button and IDD_WakeUP EXTI line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0 ;  // PA0 for User button AND IDD_WakeUP
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set User Button and IDD_WakeUP EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);

    /* Configure the GPIO_LED pins  LD3 & LD4*/
    GPIO_InitStructure.GPIO_Pin = LD_GREEN_GPIO_PIN | LD_BLUE_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(LD_GPIO_PORT, &GPIO_InitStructure);
    GPIO_LOW(LD_GPIO_PORT, LD_GREEN_GPIO_PIN);
    GPIO_LOW(LD_GPIO_PORT, LD_BLUE_GPIO_PIN);

    /* Onewire data pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Onewire power pin */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    GPIO_HIGH(GPIOD, GPIO_Pin_2);

    /* Disable all GPIOs clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB |
                          RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD |
                          RCC_AHBPeriph_GPIOE | RCC_AHBPeriph_GPIOH, DISABLE);
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in 10 ms.
  * @retval None
  */
void Delay(uint32_t nTime)
{
  TimingDelay = nTime/2;

  while(TimingDelay != 0);
  
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}

#endif

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
