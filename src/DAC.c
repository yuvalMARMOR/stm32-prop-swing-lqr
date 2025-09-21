#include "stm32f10x_dac.h"
//#include "DAC.h"
#include "math.h" 

void DACinit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  DAC_InitTypeDef DAC_InitStructure;
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

  
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* DAC channel1 Configuration */
  DAC_InitStructure.DAC_Trigger = DAC_Trigger_Software;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = 0;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Enable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  /* Enable DAC Channel 1,2 */
  DAC_Cmd(DAC_Channel_1, ENABLE);
  DAC_Cmd(DAC_Channel_2, ENABLE);

  /* Reset DAC Channel 1 */
  DAC_SetChannel1Data(DAC_Align_12b_L, 0x0);
  DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);

  /* Reet DAC Channel 2 */
  DAC_SetChannel2Data(DAC_Align_12b_L, 0x0);
  DAC_SoftwareTriggerCmd(DAC_Channel_2, ENABLE);
   
}

void DAC_Motor_Drive(float a,float b)
{
  a=a/3*pow(2,16);
  b=b/3*pow(2,16);
  /*------------------------------*/
  
  /* Set DAC Channel1 DHR12L register */
  DAC_SetChannel1Data(DAC_Align_12b_L, (uint16_t)a);
  /* Start DAC Channel1 conversion by software */
  DAC_SoftwareTriggerCmd(DAC_Channel_1, ENABLE);
  /* Set DAC Channel1 DHR12L register */
  DAC_SetChannel2Data(DAC_Align_12b_L, (uint16_t)b);
  /* Start DAC Channel1 conversion by software */
  DAC_SoftwareTriggerCmd(DAC_Channel_2, ENABLE);
  
}