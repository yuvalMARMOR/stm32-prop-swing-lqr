
/* Private includes ----------------------------------------------------------*/
#include "stm32f10x_tim.h"
#include "uart.h"
#include "Motor.h"
#include "DAC.h"
#include "encoder.h"
#include "ADC.h"
#include "timeclock.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int uart_a=0, uart_b=0, uart_c=0, uart_d=0; // DO NOT DELETE THIS LINE

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
void controller_interrupt_init(uint16_t millisec);

int main(void)
{

/******************************************************************************/
/* Instructions to use this example:                                          */  
/*   Stabilize the swing horizontally and than push the user (blue) button    */
/*   The pinout description is as follows:                                    */  
/*       - PC6,7 are the motor PWM outputs                                    */
/*       - PA4,5 are the motor analog outputs                                 */
/*       - PB6,7 are encoder input channel A,B (yellow to PB6)                */
/*       - PA0 is a logic input (blue button)                                 */
/*       - PB0 is the potentiometer analog input                              */
/*       - PA2 is the UART RX pin to the UART convertion circuit              */
/******************************************************************************/
  
  GPIO_InitTypeDef GPIO_InitStructure;
  uint16_t timestep=20; //in millisec
  
  /* Configure blue button as input floating */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Initializing: */
  Motor_Init();
  ADCinit();
  DACinit();
  UART_Init();

  /* Initialize theta0: theta0 is updating when button is pushed*/
  while (GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)==0);

  Encoder_Init(); //encoder is initialized after button is pushed
  clock_init();
  controller_interrupt_init(timestep);

  
  /* Main loop: */
  while(1); // Infinite loop
}

void  controller_interrupt_init(uint16_t millisec)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable TIM7 peripherial */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = millisec;
  TIM_TimeBaseStructure.TIM_Prescaler = 23999;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
  TIM_ITConfig (TIM7, TIM_IT_Update, ENABLE);

  /* Enable the TIM7 global Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //level 0 is the highest interrupt priority.
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
  
  /* TIM7 enable counter */
  TIM_Cmd(TIM7, ENABLE);
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