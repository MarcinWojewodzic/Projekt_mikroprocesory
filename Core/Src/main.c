/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"keypad.h"
#include"MFRC522.h"
#include "ws2812b.h"
#include "ssd1331.h"
#include "stdio.h"
#include "GFX_Color.h"
#include "fonts/fonts.h"
#include "Lepsze.h"
#include "GFX_Lepsze.h"
#include "string.h"
#include "arm_math.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char keypad[4][4]={
		{'1','2','3','A'},
		{'4','5','6','B'},
		{'7','8','9','C'},
		{'*','0','#','D'}
};
uint32_t keypad_password_cnt=0;
keypad_t key;
volatile uint8_t UID[5];
char a;
typedef enum
{
	lock_keypad_ok=0,
	lock_ok,
	lock_idle,
	lock_error
}lock;
lock lock_state=lock_idle;
char password[]={'1','3','4','2','A'};
uint8_t RFID_password[]={170,96,10,120,184};
char keypad_password[5];
uint8_t Number_of_attemps=2;
uint16_t delay=10;
char oled_data[20];
uint32_t led_data[256];
uint16_t fft_samples[1024];
float fft_out_buffer[1024];
float fft_in_buffer[1024];
uint32_t fft_cnt=0;
uint16_t adc;
arm_rfft_fast_instance_f32 FFT;
uint8_t fft_is_ready=0;
uint8_t oled_fft[102];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
int8_t check_password(char* password,char* password_to_chech);
void lock_idle_task();
void lock_error_task();
void lock_keypad_ok_task();
void lock_ok_task();
void CalculateFFT();
float complexABS(float real, float compl);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_SPI5_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */

  MFRC522_Init();


  WS2812B_Init(&hspi2);

  for(int i=0;i<255;i++)
  {
	  led_data[i]=gamma8(sine8(i));
  }
  HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, led_data, 256);
  WS2812B_SetDiodeRGB(0, 0, gamma8(40), 0);
  WS2812B_SetDiodeRGB(1, 0, gamma8(40), 0);
  WS2812B_SetDiodeRGB(2, 0, gamma8(40), 0);
  WS2812B_SetDiodeRGB(11, 0, gamma8(40), 0);
  WS2812B_SetDiodeRGB(10, 0, gamma8(40), 0);
  ssd1331_init(&hspi5);
  for(int i=0;i<64;i++)
  {
	  for(int j=0;j<96;j++)
	  {
		  setPixel(j, i, RGB(gamma8(255-(i*4)),gamma8(0+(i*4)),0));
	  }
  }

  buffer_to_template();
  ssd1331_clear();
  GFX_SetFont(font_8x5);
  EF_SetFont(&timesNewRoman_14ptFontInfo);
  EF_PutString((uint8_t*)"password:",sizeof("password:"), 0, 0, GREEN, 0, BLACK);
  GFX_DrawString(0, 19, "*****", GREEN);
  ssd1331_display(0);

  WS2812B_Refresh();
  keypad_init( &key, (char*)keypad);
  uint32_t time=0;
  arm_rfft_fast_init_f32(&FFT, 1024);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {


	  if(HAL_GetTick()-time>delay)
	  {
		  switch(lock_state)
		  {
		  case lock_idle:
		  {
			  lock_idle_task();
			  break;
		  }
		  case lock_error:
		  {
			  lock_error_task();
			  break;
		  }
		  case lock_keypad_ok:
		  {
			  lock_keypad_ok_task();
			  break;
		  }
		  case lock_ok:
		  {
			  lock_ok_task();
			  break;
		  }
		  }
		  time=HAL_GetTick();
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* SPI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI2_IRQn);
  /* SPI5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SPI5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(SPI5_IRQn);
  /* TIM2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* USER CODE BEGIN 4 */
int8_t check_password(char* password,char* password_to_check)
{
	int8_t pass=1;
	for(int i=0;i<5;i++)
	{
		if(*password!=*password_to_check)
		{
			pass--;
			break;
		}
		password++;
		password_to_check++;
	}
	return pass;
}
void lock_idle_task()
{
	keypad_task(&key);
	uint8_t check_keypad=0;
	if(key.flag==1)
	 {
		keypad_password[keypad_password_cnt]= keypad_getKey(&key);
		keypad_password_cnt++;
		if(Number_of_attemps==2)
		{
			GFX_DrawString(45, 19, (char*)keypad_password, GREEN);
			ssd1331_display(0);
		}
		else if(Number_of_attemps==1)
		{
			GFX_DrawString(45, 29, (char*)keypad_password, GREEN);
			ssd1331_display(0);
		}
	 }
	if(keypad_password_cnt==5)
	{
		check_keypad=check_password((char*)password, (char*)keypad_password);
		if(!check_keypad)
		{
			Number_of_attemps--;
			if(Number_of_attemps==1)
			{
				  WS2812B_SetDiodeRGB(0, 0, gamma8(40), 0);
				  WS2812B_SetDiodeRGB(1, 0, gamma8(40), 0);
				  WS2812B_SetDiodeRGB(2, 0, 0, 0);
				  WS2812B_SetDiodeRGB(11, 0, gamma8(40), 0);
				  WS2812B_SetDiodeRGB(10, 0, 0, 0);
				  WS2812B_Refresh();
				  keypad_password_cnt=0;
				  GFX_DrawString(0, 19, "*****", RED);
				  GFX_DrawString(0, 29, "*****", GREEN);
				  GFX_DrawString(45, 19, (char*)keypad_password, RED);
				  for(int i=0;i<5;i++)
				  {
					  keypad_password[i]=0;
				  }
				  ssd1331_display(0);
			}
			else
			{
				delay=150;
				lock_state=lock_error;
				GFX_DrawString(0, 29, "*****", RED);
				GFX_DrawString(45, 29, (char*)keypad_password, RED);
				EF_PutString((uint8_t*)"ERROR!!", sizeof("ERROR!!"), 0, 49, RED, 0, BLACK);
				EF_PutString((uint8_t*)"password:",sizeof("password:"), 0, 0, RED, 0, BLACK);
				ssd1331_display(0);
				TIM2->PSC=999;
			}
		}
		else
		{
			WS2812B_SetDiodeRGB(9, 0, gamma8(40), 0);
			WS2812B_SetDiodeRGB(10, 0, gamma8(40), 0);
			WS2812B_SetDiodeRGB(3, 0, gamma8(40), 0);
			WS2812B_SetDiodeRGB(2, 0, gamma8(40), 0);
			WS2812B_Refresh();
			if(Number_of_attemps==2)
			{
				GFX_DrawString(0, 19, "*****", BLACK);
				GFX_DrawString(0, 19, (char*)password, GREEN);
			}
			else if(Number_of_attemps==1)
			{
				GFX_DrawString(0, 29, "*****", BLACK);
				GFX_DrawString(0, 29, (char*)password, GREEN);
			}
			for(int i=0;i<10;i++)
			{
				if(!(i%2))
				{
					if(Number_of_attemps==2)
					{
						GFX_DrawString(45, 19, (char*)keypad_password, BLACK);
					}
					else if(Number_of_attemps==1)
					{
						GFX_DrawString(45, 29, (char*)keypad_password, BLACK);
					}
				}
				else
				{
					if(Number_of_attemps==2)
					{
						GFX_DrawString(45, 19, (char*)keypad_password, GREEN);
					}
					else if(Number_of_attemps==1)
					{
						GFX_DrawString(45, 29, (char*)keypad_password, GREEN);
					}
				}
				ssd1331_display(0);
				HAL_Delay(100);
			}
			lock_state=lock_keypad_ok;
		}
	}
}
void lock_error_task()
{
	keypad_password_cnt++;
	if(keypad_password_cnt%2)
	{
		WS2812B_SetDiodeRGB(9, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(10, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(11, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(0, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(1, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(2, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(3, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(4, 0, 0, 0);
		WS2812B_SetDiodeRGB(5, 0, 0, 0);
		WS2812B_SetDiodeRGB(6, 0, 0, 0);
		WS2812B_SetDiodeRGB(7, 0, 0, 0);
		WS2812B_SetDiodeRGB(8, 0, 0, 0);
		WS2812B_Refresh();
	}
	else
	{
		WS2812B_SetDiodeRGB(4, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(5, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(6, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(7, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(8, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(9, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(3, gamma8(40), 0, 0);
		WS2812B_SetDiodeRGB(2, 0, 0, 0);
		WS2812B_SetDiodeRGB(1, 0, 0, 0);
		WS2812B_SetDiodeRGB(0, 0, 0, 0);
		WS2812B_SetDiodeRGB(11, 0, 0, 0);
		WS2812B_SetDiodeRGB(10, 0, 0, 0);
		WS2812B_Refresh();
	}
}
void lock_keypad_ok_task()
{
	uint8_t check_rfid=0;
	u_char status, cardstr[MAX_LEN+1];
	status = MFRC522_Request(PICC_REQIDL, cardstr);
	if(Number_of_attemps==2)
	{
		EF_PutString((uint8_t*)"rfid:",sizeof("rfid:"), 0, 27, GREEN, 0, BLACK);
		GFX_DrawString(0, 43, "*****", GREEN);
		ssd1331_display(0);
	}
	else if(Number_of_attemps)
	{
		EF_PutString((uint8_t*)"rfid:",sizeof("rfid:"), 0, 37, GREEN, 0, BLACK);
		GFX_DrawString(0, 53, "*****", GREEN);
		ssd1331_display(0);
	}
	if(status == MI_OK)
	{
		status = MFRC522_Anticoll(cardstr);
		if(status==MI_OK)
		{
			UID[0] = cardstr[0];
			UID[1] = cardstr[1];
			UID[2] = cardstr[2];
			UID[3] = cardstr[3];
			UID[4] = cardstr[4];
			check_rfid=check_password((char *)RFID_password, (char*)UID);
			if(!check_rfid)
			{
				delay=150;
				lock_state=lock_error;
				ssd1331_clear();
				EF_PutString((uint8_t*)"rfid:",sizeof("rfid:"), 0, 0, RED, 0, BLACK);
				sprintf(oled_data,"%x %x %x %x %x",UID[0],UID[1],UID[2],UID[3],UID[4]);
				GFX_DrawString(0, 20, oled_data, RED);
				EF_PutString((uint8_t*)"ERROR!!",sizeof("ERROR!!"), 0, 30, RED, 0, BLACK);
				ssd1331_display(0);
				TIM2->PSC=999;
			}
			else
			{
				if(Number_of_attemps==2)
				{
					GFX_DrawString(0, 43, "*****", BLACK);
					sprintf(oled_data,"%x %x %x %x %x",RFID_password[0],RFID_password[1],RFID_password[2],RFID_password[3],RFID_password[4]);
					GFX_DrawString(0, 43, oled_data, GREEN);

				}
				else if(Number_of_attemps==1)
				{
					GFX_DrawString(0, 53, "*****", BLACK);
					sprintf(oled_data,"%x %x %x %x %x",RFID_password[0],RFID_password[1],RFID_password[2],RFID_password[3],RFID_password[4]);
					GFX_DrawString(0, 53, oled_data, GREEN);
				}
				HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
				TIM2->CCR4=0;
				HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, led_data, 256);
				for(int i=0;i<10;i++)
				{
					if(!(i%2))
					{
						if(Number_of_attemps==2)
						{
							GFX_DrawString(0, 43, oled_data, BLACK);
						}
						else if(Number_of_attemps==1)
						{
							GFX_DrawString(0, 53, oled_data, BLACK);
						}
					}
					else
					{
						if(Number_of_attemps==2)
						{
							GFX_DrawString(0, 43, oled_data, GREEN);
						}
						else if(Number_of_attemps==1)
						{
							GFX_DrawString(0, 53, oled_data, GREEN);
						}
					}
					HAL_Delay(100);
					ssd1331_display(0);
				}
				delay=25;
				HAL_ADC_Start_DMA(&hadc1, (uint32_t*)fft_samples, 1024);
				HAL_TIM_Base_Start(&htim3);
				lock_state=lock_ok;
			}
		}
	}

}
void lock_ok_task()
{
	ws2812_clear();
	keypad_password_cnt++;
	WS2812B_SetDiodeRGB(keypad_password_cnt%12, 0, gamma8(100), 0);
	WS2812B_SetDiodeRGB((keypad_password_cnt+6)%12, 0, gamma8(100), 0);
	WS2812B_Refresh();
	if(fft_is_ready==1)
	{
		for(int i=0;i<1024;i++)
		{
			fft_in_buffer[i]=(float)fft_samples[i];
		}
		fft_is_ready=0;
		CalculateFFT();
		temlate_to_buffer();
	    analizator_GPIO_Port->ODR |= analizator_Pin;
		for(int i=0;i<96;i++)
		{
			int a=63-oled_fft[i];
			GFX_WriteLine(i, 0, i, a, BLACK);
		}
	    analizator_GPIO_Port->ODR ^=analizator_Pin ;
		ssd1331_display(0);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*)fft_samples, 1024);
	}

}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		fft_cnt++;
		fft_is_ready=1;
		int a=rand();
}
float complexABS(float real, float compl)
{
	return sqrtf(real*real+compl*compl);
}

void CalculateFFT(void)
{
	arm_rfft_fast_f32(&FFT, fft_in_buffer, fft_out_buffer, 0);

	int Freqs[512];
	int FreqPoint = 0;
	int Offset = 45; // variable noise floor offset

		// calculate abs values and linear-to-dB
	for (int i = 0; i < 1024; i = i+2)
	{
		Freqs[FreqPoint] = (int)(20*log10f(complexABS(fft_in_buffer[i], fft_in_buffer[i+1]))) - Offset;

		if(Freqs[FreqPoint] < 0)
		{
			Freqs[FreqPoint] = 0;
		}
		FreqPoint++;
	}

	for(int i=0;i<512;i+=5)
	{
		oled_fft[i/5]=Freqs[i];
	}
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
