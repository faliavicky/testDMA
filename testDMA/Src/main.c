/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2022 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"   																													
#include "lcd_character.h"
#include "math.h"
#include "stdlib.h"
#define SampleData 3000 // maksimal sampel data
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */

/* INISIALISASI RELAY */
int l=0, j=0;
/* INISIALISASI ADC */
__IO uint16_t ADC_value[10];

/* INISISALISASI PWM */
float pwm=1050,pwm1;
float duty;
char bufferduty[200];

/* INISISALISASI SENSOR */
double A1, Vsq1[SampleData], Vsum1, Vadc1;
double A2, Vsq2[SampleData], Vsum2, Vadc2;
double A3, Vsq3[SampleData], Vsum3, Vadc3;
double A4, Vsq4[SampleData], Vsum4, Vadc4;
double A5, Vsq5[SampleData], Vsum5, Vadc5;

double B1, Isq1[SampleData], Isum1, Iadc1;
double B2, Isq2[SampleData], Isum2, Iadc2;
double B3, Isq3[SampleData], Isum3, Iadc3;
double B4, Isq4[SampleData], Isum4, Iadc4;
double B5, Isq5[SampleData], Isum5, Iadc5;

//double A5, POsq5[SampleData], POsum5, POadc5;
float V1, V2, V3, V4, V5, I1, I2, I3, I4, I5, Vin1, Vin2, Vin3, Vin4, Vin5, Iin1, Iin2, Iin3, Iin4, Iin5, Pin,Po, DC, PWM;
int 	k=0, Z=0, a=0;
char  buffer_ADC_V1[200], buffer_ADC_V2[200], buffer_ADC_V3[200], buffer_ADC_V4[200], buffer_ADC_V5[200];
char  buffer_ADC_I1[200], buffer_ADC_I2[200], buffer_ADC_I3[200], buffer_ADC_I4[200], buffer_ADC_I5[200];
char  buffer_V_Input1[200], buffer_V_Input2[200], buffer_V_Input3[200], buffer_V_Input4[200], buffer_V_Input5[200];
char  buffer_A_Input1[200], buffer_A_Input2[200], buffer_A_Input3[200], buffer_A_Input4[200], buffer_A_Input5[200];
char  buffer_ADC_PO[200], buffer_ADC_POT[200];


/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */


/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
__IO uint16_t Nilai_ADC[10];
int on,i;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
//////////// interrupt ADC (moving average) //////////// (Sampling Freq 5kHz)
 if (htim->Instance==TIM14) 																										
	   { 
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADC_value,10); //ADC DMA 5 channel (DMA Continous Circular Half Word)

//////////// Sesuaikan kebutuhan ADC masing masing ////////////	
			 
//			 //Sensor Tegangan 1 
//				Vadc1= ADC_value[9];
//				Vsum1-=Vsq1[k];
//				Vsq1[k]=Vadc1;
//				Vsum1+=Vsq1[k];
//				A1=Vsum1/SampleData;
////				Vin = (0.0123*A1) -0.1282 ;			 
//				//Vin = (0.0124*A1) - 0.0613;
//			 Vin1 = (0.00361880542627663 *A1) + 0.00292251531882836;
//			 //Vin=Sec;
//			 
//			 	//Sensor Tegangan 2
//				Vadc2= ADC_value[8];
//				Vsum2-=Vsq2[k];
//				Vsq2[k]=Vadc2;
//				Vsum2+=Vsq2[k];
//				A2=Vsum2/SampleData;
////				//Vo = (0.0123*A2) -0.1282 ;
//				Vin2 = (0.00360046269551241*A2) - 0.02327107067985370;
////				
//				//Sensor Tegangan 3
//				Vadc3= ADC_value[7];
//				Vsum3-=Vsq3[k];
//				Vsq3[k]=Vadc3;
//				Vsum3+=Vsq3[k];
//				A3=Vsum3/SampleData;
//				//Vo = (0.0123*A2) -0.1282 ;
//				Vin3 = (0.00360695990713917*A3) - 0.01562676963846250;	
//				//Vin3 = (0.00360046269551241*A3) - 0.02327107067985370;
////				//y = 0.00360695990713917x - 0.01562676963846250

////				
//				//Sensor Tegangan 4
//				Vadc4= ADC_value[6];
//				Vsum4-=Vsq4[k];
//				Vsq4[k]=Vadc4;
//				Vsum4+=Vsq4[k];
//				A4=Vsum4/SampleData;
//				//Vo = (0.0123*A2) -0.1282 ;
//				Vin4 = (0.00360062885621475*A4) - 0.05964763175110030;
////				
//				//Sensor Tegangan 5
//				Vadc5= ADC_value[5];
//				Vsum5-=Vsq5[k];
//				Vsq5[k]=Vadc5;
//				Vsum5+=Vsq5[k];
//				A5=Vsum5/SampleData;
//				//Vo = (0.0123*A2) -0.1282 ;
//				Vin5 = (0.00358939886026047*A5) - 0.05485033568183740;

	
			 //Sensor Arus 1
				Iadc1= ADC_value[5];
				Isum1-=Isq1[k];
				Isq1[k]=Iadc1;
				Isum1+=Isq1[k];
				B1=Isum1/SampleData;
//				//Iin1 =(0.0086*B1) - 17.674;
				Iin1 = (0.0147*B1) - 30.096;

//			 
			 	//Sensor Arus 2
				Iadc2= ADC_value[4];
				Isum2-=Isq2[k];
				Isq2[k]=Iadc2;
				Isum2+=Isq2[k];
				B2=Isum2/SampleData;
				//Iin1 = fabs((0.0086*B1) - 17.674);
				Iin2 = (0.0147*B2) - 30.096;
				//y = 0.00361208999745770x - 0.03304734834610130

////				
				//Sensor Arus 3
				Iadc3= ADC_value[3];
				Isum3-=Isq3[k];
				Isq3[k]=Iadc3;
				Isum3+=Isq3[k];
				B3=Isum3/SampleData;
				//Iin1 = fabs((0.0086*B1) - 17.674);
				Iin3 = (0.0147*B3) - 30.096;
//				
				//Sensor Arus 4
				Iadc4= ADC_value[2];
				Isum4-=Isq4[k];
				Isq4[k]=Iadc4;
				Isum4+=Isq4[k];
				B4=Isum4/SampleData;
				//Iin1 = fabs((0.0086*B1) - 17.674);
				Iin4 = ((0.0147*B4) - 28.096)*-1;
//				
				//Sensor Arus 5
				Iadc5= ADC_value[1];
				Isum5-=Isq5[k];
				Isq5[k]=Iadc5;
				Isum5+=Isq5[k];
				B5=Isum5/SampleData;
				//Iin1 = fabs((0.0086*B1) - 17.674);
				Iin5 = (0.0147*B5) - 31.096;
//				
//				Pin=Vin*Iin;
//				Po=Vo*Io;
				
//			  //ADC-PWM
//				POadc5= ADC_value[4];
//				POsum5-=POsq5[k];
//				POsq5[k]=POadc5;
//				POsum5+=POsq5[k];
//				A5=POsum5/SampleData;
//				DC = A5/4095.0*2099;
//				PWM = DC/2099*100;
				
				k++;
				if(k>=SampleData)
				k=0;					
 				//SENSOR TEGANGAN INPUT ==> Mikro ADC_Value[0] PIN PA0 PCB ADC4
				//SENSOR TEGANGAN OUTPUT ==> Mikro ADC_Value[1] PIN PA1 PCB ADC3
				//SENSOR ARUS INPUT  ==> Mikro ADC_Value[2] PIN PA2 PCB ADC2
				//SENSOR ARUS OUTPUT ==>  ==> Mikro ADC_Value[3] PIN PA3 PCB ADC1
			  //POT PWM ==>  ==> Mikro ADC_Value[4] PIN PA4 PCB ADC0
	  }

	}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();

  /* USER CODE BEGIN 2 */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &ADC_value,10);
	HAL_TIM_Base_Start_IT(&htim14); 																									// interrupt ADC
	HAL_TIM_Base_Start_IT(&htim13);  																									// Rencana interrupt PWM
//	HAL_TIM_Base_Start_IT(&htim9);   	
	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	
	lcd_init();
//	lcd_gotoxy(0,0);	
//	//lcd_puts("UJI ALAT DAN SENSOR");
	//lcd_puts("Uji 4 Beban");
	
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		if (HAL_GPIO_ReadPin(GPIOB, LDR_Pin)==0){
				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
				lcd_gotoxy(18,0);
				lcd_puts("ON");
				lcd_gotoxy(18,1);
				lcd_puts("ON ");
			}
			
			
			else if (HAL_GPIO_ReadPin(GPIOB, LDR_Pin)==1 ){
				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
				lcd_gotoxy(18,0);
				lcd_puts("OFF");
				lcd_gotoxy(18,1);
				lcd_puts("OFF");
			}
		
//		if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==0 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==0 ){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_RESET);
//				lcd_gotoxy(18,0);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,1);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,2);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,3);
//				lcd_puts("OFF");
//			}
//			
//			
//			else if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==1 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==0 ){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_RESET);
//				lcd_gotoxy(18,0);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,1);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,2);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,3);
//				lcd_puts("OFF");
//			}
//			else if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==0 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==1 ){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_SET);
//				lcd_gotoxy(18,0);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,1);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,2);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,3);
//				lcd_puts("ON ");
//			}
//			
//			else if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==1 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==1 ){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_SET);
//				lcd_gotoxy(18,0);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,1);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,2);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,3);
//				lcd_puts("ON ");
//			}
		
	
//			if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==0 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==0 && HAL_GPIO_ReadPin(GPIOB, LDR_Pin)==1){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_RESET);
//				lcd_gotoxy(18,0);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,1);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,2);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,3);
//				lcd_puts("OFF");
//			}
//			
//			
//			else if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==1 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==0 && HAL_GPIO_ReadPin(GPIOB, LDR_Pin)==1){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_RESET);
//				lcd_gotoxy(18,0);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,1);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,2);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,3);
//				lcd_puts("OFF");
//			}
//			else if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==0 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==1 && HAL_GPIO_ReadPin(GPIOB, LDR_Pin)==1){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_SET);
//				lcd_gotoxy(18,0);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,1);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,2);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,3);
//				lcd_puts("ON ");
//			}
//			
//			else if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==1 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==1 && HAL_GPIO_ReadPin(GPIOB, LDR_Pin)==1){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_SET);
//				lcd_gotoxy(18,0);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,1);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,2);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,3);
//				lcd_puts("ON ");
//			}
//		
		
		
//			else if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==0 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==0 && HAL_GPIO_ReadPin(GPIOB, LDR_Pin)==0){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_RESET);
//				lcd_gotoxy(18,0);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,1);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,2);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,3);
//				lcd_puts("OFF");}
//			
//				else if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==1 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==1 && HAL_GPIO_ReadPin(GPIOB, LDR_Pin)==0){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_SET);
//				lcd_gotoxy(18,0);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,1);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,2);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,3);
//				lcd_puts("ON ");}
//				
//				else if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==0 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==1 && HAL_GPIO_ReadPin(GPIOB, LDR_Pin)==0){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_RESET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_SET);
//				lcd_gotoxy(18,0);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,1);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,2);
//				lcd_puts("OFF");
//				lcd_gotoxy(18,3);
//				lcd_puts("ON ");}
//				
//				else if (HAL_GPIO_ReadPin(GPIOB, PIR3_Pin)==1 && HAL_GPIO_ReadPin(GPIOB, PIR4_Pin)==0 && HAL_GPIO_ReadPin(GPIOB, LDR_Pin)==0){
//				HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_SET);
//				HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_RESET);
//				lcd_gotoxy(18,0);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,1);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,2);
//				lcd_puts("ON ");
//				lcd_gotoxy(18,3);
//				lcd_puts("OFF");}
		
		
//		HAL_GPIO_TogglePin(GPIOE, LED1_Pin);
		
//		HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_RESET);
		
		


	//////// Tampilan V dan I ////////
//	lcd_gotoxy(0,0);																													
//	sprintf(buffer_V_Input1,"V1:%2.2f I1:%2.2f",Vin1,Iin1);
////	lcd_gotoxy(0,1);
////	sprintf(buffer_V_Input1,"V1:%2.2f",Vin1);
//	lcd_puts(buffer_V_Input1);
//	HAL_Delay(10);
		
	lcd_gotoxy(0,0);																													
	sprintf(buffer_V_Input2,"V2:%2.2f I2:%2.2f",Vin2,Iin2);
//		lcd_gotoxy(10,1);																													
//	sprintf(buffer_V_Input2,"V2:%2.2f",Vin2);
	lcd_puts(buffer_V_Input2);
	HAL_Delay(10);
		
	lcd_gotoxy(0,1);																													
	sprintf(buffer_V_Input3,"V3:%2.2f I3:%2.2f",Vin3,Iin3);
//	lcd_gotoxy(0,2);																													
//	sprintf(buffer_V_Input3,"V3:%2.2f",Vin3);
	lcd_puts(buffer_V_Input3);
	HAL_Delay(10);
		
	lcd_gotoxy(0,2);																													
	sprintf(buffer_V_Input4,"V4:%2.2f I4:%2.2f",Vin4,Iin4);
//	lcd_gotoxy(10,2);																													
//	sprintf(buffer_V_Input4,"V4:%2.2f",Vin4);
	lcd_puts(buffer_V_Input4);
	HAL_Delay(10);

	lcd_gotoxy(0,3);																													
	sprintf(buffer_V_Input5,"V5:%2.2f I5:%2.2f",Vin5,Iin5);
//	lcd_gotoxy(0,3);																													
//	sprintf(buffer_V_Input5,"V5:%2.2f",Vin5);
	lcd_puts(buffer_V_Input5);
	HAL_Delay(10);
		
	//////// Tampilan Konversi ADC ////////
//	lcd_gotoxy(0,1);																													
//	sprintf(buffer_V_Input1,"Vin:%4.0f Iin:%4.0f",A1,B1);
//	//sprintf(buffer_V_Input1,"V1:%4.0f",A1);
//	lcd_puts(buffer_V_Input1);
//	HAL_Delay(10);
//		
//	lcd_gotoxy(0,2);																													
////	sprintf(buffer_V_Input2,"V2 :%4.0f I2 :%4.0f",A2,B2);
//	sprintf(buffer_V_Input2,"V2:%4.0f",A2);
//	lcd_puts(buffer_V_Input2);
//	HAL_Delay(10);

//	lcd_gotoxy(9,1);																													
////	sprintf(buffer_V_Input3,"V3:%4.0f I3:%4.0f",A3,B3);
//	sprintf(buffer_V_Input3,"V3:%4.0f",A3);
//	lcd_puts(buffer_V_Input3);
//	HAL_Delay(10);
////		
//	lcd_gotoxy(9,2);																													
////	sprintf(buffer_V_Input4,"V4 :%4.0f I4 :%4.0f",A4,B4);
//	sprintf(buffer_V_Input4,"V4:%4.0f",A4);
//	lcd_puts(buffer_V_Input4);
//	HAL_Delay(10);

//	lcd_gotoxy(0,0);																													
////	sprintf(buffer_V_Input5,"V5 :%4.0f I5 :%4.0f",A5,B5);
//	sprintf(buffer_V_Input5,"V5:%4.0f",A5);
//	lcd_puts(buffer_V_Input5);
//	HAL_Delay(10);

	
	//////// PWM KONSTAN dan Tampilan //////////
	TIM4->CCR1 = 1470;
//	lcd_gotoxy(0,1);	
//	sprintf(buffer_ADC_PO,"DUTY :%2.0f   ", PWM);
//lcd_puts ("DUTY : 70");
//	HAL_Delay(10);

	//////// Relay ////////
//	
//	HAL_GPIO_WritePin(GPIOE, LED1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOE, LED2_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOE, FAN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOE, FAN2_Pin, GPIO_PIN_SET);

		
	
//	if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_7))
//		l++;
//		if(l==1)
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_SET);
//		else
//		{
//			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_14,GPIO_PIN_RESET);
//			l=0;
//		}
//	if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6))
//		j++;
//		if(j==1)
//		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_SET);
//		else
//		{
//			HAL_GPIO_WritePin(GPIOE,GPIO_PIN_13,GPIO_PIN_RESET);
//			j=0;
//		}			
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 10;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 9;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
    */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 10;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2099;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 5000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 42041;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM13 init function */
static void MX_TIM13_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 2099;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 16801;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED1_Pin|LED2_Pin|FAN1_Pin|FAN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PIR2_Pin PIR1_Pin */
  GPIO_InitStruct.Pin = PIR2_Pin|PIR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin FAN1_Pin FAN2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|FAN1_Pin|FAN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC7 PC8 PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PD0 PD1 PD2 PD3
                           PD4 PD5 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LDR_Pin PIR3_Pin PIR4_Pin */
  GPIO_InitStruct.Pin = LDR_Pin|PIR3_Pin|PIR4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
