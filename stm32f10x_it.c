/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_tim.h"
#include "uart.h"
#include "motor.h"
#include "encoder.h"
#include "DAC.h"
#include "ADC.h"
#include "timeclock.h"
#include "math.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PI 3.14159

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int Enew, Eold,theta, theta_desired, pot, enc;
float u1,u2,u1analog,u2analog,t;

// --- Matching LQR Controller Variables ---
// Controlled system: G(s) = 83.01/(s^2 + 13.88s + 83.01)
// Reference model: F(s) = 400/(s^2 + 40s + 400) (מסנן מסדר שני, ריסון גבוה)
float Ar[2][2] = { {0.0f, 1.0f}, {-400.0f, -40.0f} };
float Br[2] = { 0.0f, 1.0f };
float Cr[2] = { 400.0f, 0.0f };

// Extended system (4 states: x1, x2, xr1, xr2)
float AA[4][4] = {
    {0.0f, 1.0f, 0.0f, 0.0f},
    {-83.01f, -13.88f, 0.0f, 0.0f},
    {0.0f, 0.0f, 0.0f, 1.0f},
    {0.0f, 0.0f, -400.0f, -40.0f}
};
float BB[4] = {0.0f, 1.0f, 0.0f, 0.0f};
float CC[4] = {83.01f, 0.0f, -400.0f, 0.0f};

// LQR gain מוקטן בחצי
float K[4] = {2.0f, 1.25f, -3.75f, -0.6f}; // ערכים מחושבים לדוגמה, יש לעדכן לפי חישוב LQR

// States
float x[2] = {0.0f, 0.0f};    // plant state: [x1, x2]
float xr[2] = {0.0f, 0.0f};   // reference model state: [xr1, xr2]

// LQR weights
float Q = 0.0001f;
float R = 0.0001f;

// Feedforward term
float feedforward = 0.0f;
float Kff = 4.0f; // feedforward gain מוגדל

// פונקציה לעדכון מצב המערכת המבוקרת מתוך encoder ו-ADC
void update_plant_state() {
    // x1 = מיקום (encoder), x2 = מהירות (הפרש בין צעדים)
    static int last_enc = 0;
    x[0] = (float)enc; // מיקום
    x[1] = (float)(enc - last_enc) / 0.02f; // מהירות (20ms)
    last_enc = enc;
}

// פונקציה לעדכון מצב מערכת הייחוס (סימולציה דיגיטלית)
void update_reference_state(float ref_input) {
    float xr_dot[2];
    xr_dot[0] = Ar[0][0]*xr[0] + Ar[0][1]*xr[1] + Br[0]*ref_input;
    xr_dot[1] = Ar[1][0]*xr[0] + Ar[1][1]*xr[1] + Br[1]*ref_input;
    xr[0] += 0.02f * xr_dot[0];
    xr[1] += 0.02f * xr_dot[1];
}

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

void TIM7_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    
    /* Sensor and time read */
    t=get_clock();
    pot=ReadADC();
    enc=Encoder_Pos();
    
    // מצב רצוי מהפוטנציומטר
    theta_desired = (int)(((float)pot-2048)/2048*450);
    
    // עדכון מצב המערכת המבוקרת
    update_plant_state();

    // עדכון מצב מערכת הייחוס
    update_reference_state((float)theta_desired);
    // יצירת וקטור מצב מורחב
    float x_ext[4] = { x[0], x[1], xr[0], xr[1] };
    // חישוב אות הבקרה: u = -K*[x;xr] + feedforward
    float feedforward = Kff * theta_desired;
    float u = -K[0]*x_ext[0] - K[1]*x_ext[1] - K[2]*x_ext[2] - K[3]*x_ext[3] + feedforward;

    // המרה לפקודות מנוע (u1, u2) - מכנה מוגדל ל-20
    u1 = 3.0f * (50.0f - u/20.0f);
    u2 = 3.0f * (50.0f + u/20.0f);
    
    // Saturation limit
    if (u1<0) u1=0;
    if (u2<0) u2=0;
    if (u1>300) u1=300;
    if (u2>300) u2=300;
    
    PWM_Motor_Drive((int)u1,(int)u2);
    u1 = 3.0f * (50.0f - u/27.0f);
    u2 = 3.0f * (50.0f + u/27.0f);
    u1analog=(u1)/100;
    u2analog=(u2)/100;
    // Saturation אנלוגי
    if (u1analog < 0) u1analog = 0;
    if (u1analog > 3) u1analog = 3;
    if (u2analog < 0) u2analog = 0;
    if (u2analog > 3) u2analog = 3;
    DAC_Motor_Drive(u1analog, u2analog);
    UART_transmit((int)u1,(int)u2, (int)x[0], theta_desired);
  }
}
void TIM2_IRQHandler(void)
/* This function sends data using USART protocol */
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    
    extern int uart_a, uart_b, uart_c, uart_d;
    int a_byte1, a_byte2, b_byte1, b_byte2, c_byte1, c_byte2, d_byte1, d_byte2;
  
    a_byte1=uart_a;
    a_byte2=(uart_a>>8);
    b_byte1=uart_b;
    b_byte2=(uart_b>>8);
    c_byte1=uart_c;
    c_byte2=(uart_c>>8);
    d_byte1=uart_d;
    d_byte2=(uart_d>>8);

    /*transmit start byte:*/
    USART_SendData(USART2,(uint8_t) '\n'); //start byte
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);          
    /*transmit a :*/
    USART_SendData(USART2,(uint8_t) a_byte1);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);          
    USART_SendData(USART2,(uint8_t) a_byte2);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);          
    /*transmit b :*/
    USART_SendData(USART2,(uint8_t) b_byte1);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);          
    USART_SendData(USART2,(uint8_t) b_byte2);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);          
    /*transmit c :*/
    USART_SendData(USART2,(uint8_t) c_byte1);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);          
    USART_SendData(USART2,(uint8_t) c_byte2);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);          
    /*transmit c :*/
    USART_SendData(USART2,(uint8_t) d_byte1);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);          
    USART_SendData(USART2,(uint8_t) d_byte2);
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);          
    /*transmit end byte:*/
    USART_SendData(USART2,(uint8_t) '\0'); //end byte
    while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);          
  }
}

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}