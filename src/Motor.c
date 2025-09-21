

#include "stm32f10x_tim.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
  unsigned int  resolution=300;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

void Motor_Init()
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  unsigned int  BasePrescale=239;


  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  /* GPIOC clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

  /* GPIOC Configuration:TIM3 Channel1, 2, as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /*pwm output from PC6,7*/
  GPIO_PinRemapConfig  ( GPIO_FullRemap_TIM3 ,ENABLE  ) ;

  
  /* Time base configuration */
  /* f_pwm= 24/(resolution*(Prescaler+1)) [kHz] resolution defined in privates*/
  TIM_TimeBaseStructure.TIM_Period = resolution;
  TIM_TimeBaseStructure.TIM_Prescaler = BasePrescale;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  /* PWM 1,2 Mode configuration: Channel 1,2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

  //TIM_ARRPreloadConfig(TIM3, ENABLE);
  
  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  
}

void PWM_Motor_Drive(int Duty1, int Duty2)
{
  /*Check Parameters*/
  while( (Duty1<0) | (Duty1>resolution) | (Duty2<0) | (Duty2>resolution) ) 
  {
    TIM_SetCompare1(TIM3, 0);
    TIM_SetCompare2(TIM3, 0);
    while(1);
  }   
  
    TIM_SetCompare1(TIM3, Duty1);
    TIM_SetCompare2(TIM3, Duty2);
}


