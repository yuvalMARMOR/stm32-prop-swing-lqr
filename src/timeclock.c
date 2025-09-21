#include "stm32f10x_tim.h"

void clock_init(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
  
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
  TIM_TimeBaseStructure.TIM_Prescaler = 23999;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
  
  TIM_Cmd(TIM6, ENABLE);
}


float get_clock(void)
{
  int tt;
  float t;
  tt=TIM_GetCounter(TIM6);
  t=(float)tt/1000;
  return t;
}
