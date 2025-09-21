#include "stm32f10x_adc.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*void rsDelay(long long i)
{ 
  while(i!=0)
  i--; 
}
*/
  
void ADCinit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
 
  /* System clocks configuration ---------------------------------------------*/
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA, ENABLE);

  /* Analog input config */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  /* Voltage regulator check 
  while (!GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_12))
  {
  GPIO_WriteBit (GPIOC,GPIO_Pin_8, Bit_SET);
  rsDelay(300000);
  GPIO_WriteBit (GPIOC,GPIO_Pin_8, Bit_RESET);
  rsDelay(300000);
  } */

  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channels configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_1Cycles5);

 /* Enable and calibrate ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));

}

unsigned int ReadADC()
{
  unsigned int Ain=0;
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  while (!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC));
  Ain=ADC_GetConversionValue (ADC1);   
  return Ain;
}
