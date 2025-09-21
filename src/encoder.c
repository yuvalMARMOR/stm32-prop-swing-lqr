#include "stm32f10x_tim.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Encoder_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

  /* clocks enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

  /* GPIOB Configuration:TIM3 Channel1, 2, as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_SetCounter(TIM4, 0x7FFF); // This is the initial value, to avoid crossing zero
  
  /* Encoder mode config */
  TIM_EncoderInterfaceConfig  ( TIM4,  
  TIM_EncoderMode_TI12,  
  TIM_ICPolarity_Rising,  
  TIM_ICPolarity_Rising ) ;
   
  /* TIM4 enable counter */
  TIM_Cmd(TIM4, ENABLE);
}

int Encoder_Pos(void)
{
  int pos;
  pos=TIM_GetCounter(TIM4)-0x7FFF;
  return pos;
}