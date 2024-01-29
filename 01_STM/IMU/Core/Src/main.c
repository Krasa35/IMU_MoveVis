/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "bmi160_wrapper.h"
#include "arm_math.h"
#include "stdbool.h"
#include "fatfs_sd.h"

//#include "stdint.h"
//#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
BMI160_t imu_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

FATFS fs;
FIL fil;
FRESULT fresult;
char parametry[64];
UINT bw;

float f = 250.0;
float T = 0.004;

int counter_led = 0;

int sample_number = 0;
const int max_sample_number = 4500;

float acc_data[3][4500];	//max 24s of moving//POPRAWIC 4500
float gyro_data[3][4500];	//max 24s of moving//POPRAWIC 4500

//FILTER LP RAW DATA
float32_t coeffs_low[]={0.145323883877042, 0.290647767754085, 0.145323883877042, 0.671029090774096,  -0.252324626282266};

float32_t iir_state_gyro_x_low[4]={0., 0., 0., 0.};
float32_t iir_state_gyro_y_low[4]={0., 0., 0., 0.};
float32_t iir_state_gyro_z_low[4]={0., 0., 0., 0.};
float32_t iir_state_acc_x_low[4]={0., 0., 0., 0.};
float32_t iir_state_acc_y_low[4]={0., 0., 0., 0.};
float32_t iir_state_acc_z_low[4]={0., 0., 0., 0.};

arm_biquad_casd_df1_inst_f32 iir_filter_gyro_x_low;
arm_biquad_casd_df1_inst_f32 iir_filter_gyro_y_low;
arm_biquad_casd_df1_inst_f32 iir_filter_gyro_z_low;
arm_biquad_casd_df1_inst_f32 iir_filter_acc_x_low;
arm_biquad_casd_df1_inst_f32 iir_filter_acc_y_low;
arm_biquad_casd_df1_inst_f32 iir_filter_acc_z_low;

//FILTER HP ACC TOTAL
float32_t coeffs_acc_high[]={0.999995557126931, -1.999991114253863, 0.999995557126931, 1.999991114234124, -0.999991114273602};

float32_t iir_state_acc_high[4]={0., 0., 0., 0.};
arm_biquad_casd_df1_inst_f32 iir_filter_acc_high;

//FILTER LP ORIENTATION Z
float32_t coeffs_orientation_low[]={0.0000035436071, 0.0000070872142, 0.0000035436071, 1.99466855305249, -0.994682727480893};

float32_t iir_state_orientation_low[4]={0., 0., 0., 0.};
arm_biquad_casd_df1_inst_f32 iir_filter_orientation_low;

void filtr_raw_data()
{
	for(int i=0; i<sample_number; i++)
	{
		float actual = gyro_data[0][i];
		arm_biquad_cascade_df1_f32(&iir_filter_gyro_x_low, &actual, &gyro_data[0][i], 1);
		actual = gyro_data[1][i];
		arm_biquad_cascade_df1_f32(&iir_filter_gyro_y_low, &actual, &gyro_data[1][i], 1);
		actual = gyro_data[2][i];
		arm_biquad_cascade_df1_f32(&iir_filter_gyro_z_low, &actual, &gyro_data[2][i], 1);
		actual = acc_data[0][i];
		arm_biquad_cascade_df1_f32(&iir_filter_acc_x_low, &actual, &acc_data[0][i], 1);
		actual = acc_data[1][i];
		arm_biquad_cascade_df1_f32(&iir_filter_acc_y_low, &actual, &acc_data[1][i], 1);
		actual = acc_data[2][i];
		arm_biquad_cascade_df1_f32(&iir_filter_acc_z_low, &actual, &acc_data[2][i], 1);
	}
}

void calc_total_acc(float acc_total[])
{
	for(int i=0; i<sample_number; i++)
	{
		acc_total[i] = sqrt(acc_data[0][i]*acc_data[0][i]+acc_data[1][i]*acc_data[1][i]+acc_data[2][i]*acc_data[2][i]);
	}
}

float calc_mean(float acc_total[], int number_of_samples)
{
	if(number_of_samples > sample_number)
	{
		number_of_samples = sample_number;
	}

	float mean = 0.0;

	for(int i=0; i<number_of_samples; i++)
	{
		mean += acc_total[i];
	}

	mean /= (float)number_of_samples;
	return mean;
}

void filtr_total_acc(float acc_total[])
{
	iir_state_acc_high[0] = calc_mean(acc_total, 100);	//set initial values x[n-1]
	iir_state_acc_high[1] = iir_state_acc_high[0];	//set initial values x[n-2]

	for(int i=0; i<sample_number; i++)
	{
		float actual = acc_total[i];
		arm_biquad_cascade_df1_f32(&iir_filter_acc_high, &actual, &acc_total[i], 1);
	}
}

void find_no_move(float acc_total[], bool no_move[])
{
	int counter_of_no_move = 0;

	for(int i=0; i<sample_number; i++)
	{
		counter_of_no_move = 0;
		while(fabs(acc_total[i]) < 0.07 && i<sample_number)
		{
			counter_of_no_move++;
			i++;
		}

		if(counter_of_no_move >= 60)
		{
			for(int j=0; j<counter_of_no_move; j++)
			{
				no_move[i-1-j] = 1;
			}
		}
	}

	for(int i=0; i<250; i++)
	{
		no_move[i] = 1;
	}
}

void calc_orientation(float orientation[3][max_sample_number], bool no_move[])
{
	orientation[0][0] = 0.0;
	orientation[1][0] = 0.0;
	orientation[2][0] = 0.0;
	for(int i=1; i<sample_number; i++)
	{
		if(!no_move[i])
		{
			orientation[0][i] = orientation[0][i-1] + gyro_data[0][i]*T;
			orientation[1][i] = orientation[1][i-1] + gyro_data[1][i]*T;
			orientation[2][i] = orientation[2][i-1] + gyro_data[2][i]*T;
		}
		else
		{
			orientation[0][i] = atan2(acc_data[1][i], acc_data[2][i])*180/M_PI;
			orientation[1][i] = -atan2(acc_data[0][i], acc_data[2][i])*180/M_PI;
			orientation[2][i] = orientation[2][i-1];
		}
	}
}

void filtr_orientation_z(float orientation[3][max_sample_number])
{
	for(int i=0; i<sample_number; i++)
	{
		float actual = orientation[2][i];
		arm_biquad_cascade_df1_f32(&iir_filter_orientation_low, &actual, &orientation[2][i], 1);
	}
}

void rotation_of_axis(float orientation[3][max_sample_number])
{
	for(int i=0; i<sample_number; i++)
	{
		float s1 = sin(-orientation[0][i]*M_PI/180);
		float c1 = cos(-orientation[0][i]*M_PI/180);
		float s2 = sin(-orientation[1][i]*M_PI/180);
		float c2 = cos(-orientation[1][i]*M_PI/180);
		float s3 = sin(-orientation[2][i]*M_PI/180);
		float c3 = cos(-orientation[2][i]*M_PI/180);

		float accX = acc_data[0][i];
		float accY = acc_data[1][i];
		float accZ = acc_data[2][i];

		//rotation matrix
		acc_data[0][i] = (accX*c2 - accZ*s2)*9.81;
		acc_data[1][i] = (accY*c1 + accX*s1*s2 + accZ*c2*s1)*9.81;
		acc_data[2][i] = (accX*c1*s2 - accY*s1 + accZ*c1*c2)*9.81 - 9.81;

		float total_ax_rotated = fabs(acc_data[0][i])*0.3;

		acc_data[0][i] = c3*total_ax_rotated;
		acc_data[1][i] = s3*total_ax_rotated;
		acc_data[2][i] *= 1.2;
	}
}

void calc_velocity(float velocity[3][max_sample_number], bool no_move[])
{
	velocity[0][0] = 0.0;
	velocity[1][0] = 0.0;
	velocity[2][0] = 0.0;
	for(int i=1; i<sample_number; i++)
	{
		if(!no_move[i])
		{
			velocity[0][i] = velocity[0][i-1] + acc_data[0][i]*T;
			velocity[1][i] = velocity[1][i-1] + acc_data[1][i]*T;
			velocity[2][i] = velocity[2][i-1] + acc_data[2][i]*T;
		}
		else
		{
			velocity[0][i] = 0;
			velocity[1][i] = 0;
			velocity[2][i] = 0;
		}
	}
}

void velocity_compensation(float velocity[3][max_sample_number], bool no_move[])
{
	int i = 0;
	int i_first_sample_with_move = 0;
	int i_last_sample_with_move = 0;
	float velZ_end_step = 0.0;
	int time_of_step = 0;

	while(i < sample_number-1)
	{
		while(no_move[i] && i < sample_number-1)
		{
			i++;
		}
		i_first_sample_with_move = i;
		while(!no_move[i] && i < sample_number-1)
		{
			i++;
		}
		i_last_sample_with_move = i-1;
		velZ_end_step = velocity[2][i_last_sample_with_move]*0.5;
		time_of_step = i_last_sample_with_move-i_first_sample_with_move;

		if(time_of_step > 50)
		{
			for(int j=i_first_sample_with_move; j<=i_last_sample_with_move; j++)
			{
				velocity[2][j] = velocity[2][j] - (velZ_end_step/time_of_step)*(j-i_first_sample_with_move);
			}
		}
	}
}

void calc_position(float position[3][max_sample_number], float velocity[3][max_sample_number])
{
	position[0][0] = 0.0;
	position[1][0] = 0.0;
	position[2][0] = 0.0;
	for(int i=1; i<sample_number; i++)
	{
		position[0][i] = position[0][i-1] + velocity[0][i]*T;
		position[1][i] = position[1][i-1] + velocity[1][i]*T;
		position[2][i] = position[2][i-1] + velocity[2][i]*T;
	}
}

void corection_in_flat_move(float position[3][max_sample_number], float velocity[3][max_sample_number], bool no_move[])
{
	int samples_in_step = 0;
	int number_of_steps = 0;
	int i=0;

	while(i < sample_number-1)
	{
		i++;
		while(!no_move[i] && i < sample_number-1)
		{
			i++;
			samples_in_step++;
		}
		if(samples_in_step > f*0.3)
		{
			number_of_steps++;
		}
		samples_in_step = 0;
	}

	int i_last_sample_without_move = 0;
	int i_next_sample_without_move = 0;

	float height = position[2][sample_number-1];
	if(fabs(height/number_of_steps) < 0.07)
	{
		i = 0;
		while(i < sample_number-1)
		{
			while(no_move[i] && i < sample_number-1)
			{
				i++;
			}
			i_last_sample_without_move = i-1;
			while(!no_move[i] && i < sample_number-1)
			{
				i++;
			}
			i_next_sample_without_move = i;

			float dH = position[2][i_next_sample_without_move]-position[2][i_last_sample_without_move];
			float all_Z_move = 0.0;
			for(int j=i_last_sample_without_move; j<i_next_sample_without_move+1;j++)
			{
				all_Z_move += fabs(velocity[2][j]*T);
			}
			float Z_move_down = (all_Z_move-dH)/2.0;
			float Z_move_up = all_Z_move - Z_move_down;

			if(fabs(Z_move_down) > 0.00001)
			{
				float vel_correction_ratio = Z_move_up/Z_move_down;

				for(int j=i_last_sample_without_move; j<i_next_sample_without_move+1;j++)
				{
					if(velocity[2][j] < 0.0)
					{
						velocity[2][j] = velocity[2][j]*vel_correction_ratio;
					}

				}
			}
		}

	}

	calc_position(position, velocity);
}

void print(float orientation[3][max_sample_number], float position[3][max_sample_number], float acc_total[max_sample_number], float velocity[3][max_sample_number])
{
	fresult = f_mount(&fs,"",0);
	fresult = f_open(&fil, "dane.csv", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
	for(int i=0; i<sample_number; i++)
	{
		  char text[200];
		  int length = sprintf(text, "%.2f;%.2f;%.2f;%.4f;%.4f;%.4f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;\r\n", orientation[0][i], orientation[1][i], orientation[2][i], position[0][i], position[1][i], position[2][i], acc_total[i], velocity[0][i], velocity[1][i], velocity[2][i], gyro_data[0][i], gyro_data[1][i], gyro_data[2][i], acc_data[0][i], acc_data[1][i], acc_data[2][i]);
			//int parametry_len = sprintf (parametry,"Text written from STM32");
			fresult = f_lseek(&fil,f_size(&fil));
			fresult = f_write(&fil,text,length,&bw);
		  //HAL_UART_Transmit(&huart3, (uint8_t*)text, length, 1000);
	}
	f_close(&fil);
	fresult = f_mount(NULL,"",1);
}

void print_raw()
{
	for(int i=0; i<sample_number; i++)
	{
		  char text[200];
		  int length = sprintf(text, "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;\r\n", gyro_data[0][i], gyro_data[1][i], gyro_data[2][i], acc_data[0][i], acc_data[1][i], acc_data[2][i]);

		  //HAL_UART_Transmit(&huart3, (uint8_t*)text, length, 1000);
	}
}

void print_tab(float tab[max_sample_number])
{
	for(int i=0; i<sample_number; i++)
	{
		  char text[200];
		  int length = sprintf(text, "%.2f;\r\n", tab[i]);

		  HAL_UART_Transmit(&huart3, (uint8_t*)text, length, 1000);
	}
}

void print_text(char text[4], int length)
{
	HAL_UART_Transmit(&huart3, (uint8_t*)text, length, 1000);
}

void calculate()
{
	float acc_total[max_sample_number];
	bool no_move[max_sample_number];

	float velocity[3][max_sample_number];

	float position[3][max_sample_number];
	float orientation[3][max_sample_number];


	for(int i=0; i<max_sample_number; i++)
	{
		acc_total[i] = 0.0;
		no_move[i] = 0;
		velocity[0][i] = 0.0;
		velocity[1][i] = 0.0;
		velocity[2][i] = 0.0;
		position[0][i] = 0.0;
		position[1][i] = 0.0;
		position[2][i] = 0.0;
		orientation[0][i] = 0.0;
		orientation[1][i] = 0.0;
		orientation[2][i] = 0.0;
	}

	filtr_raw_data();

	calc_total_acc(acc_total);

	filtr_total_acc(acc_total);

	find_no_move(acc_total, no_move);

	calc_orientation(orientation, no_move);

	filtr_orientation_z(orientation);

	rotation_of_axis(orientation);

	calc_velocity(velocity, no_move);

	velocity_compensation(velocity, no_move);

	calc_position(position, velocity);
	corection_in_flat_move(position, velocity, no_move);


	HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	print(orientation, position, acc_total, velocity);

}

void send_uart (char* string)
{
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart3, string, len, 2000);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	  if (htim->Instance == TIM1)
	  {
		  bmi160ReadAccelGyro(&imu_t);

		  if(sample_number < max_sample_number)
		  {
			  gyro_data[0][sample_number] = imu_t.BMI160_Gx_f32;
			  gyro_data[1][sample_number] = imu_t.BMI160_Gy_f32;
			  gyro_data[2][sample_number] = imu_t.BMI160_Gz_f32;

			  acc_data[0][sample_number] = imu_t.BMI160_Ax_f32;
			  acc_data[1][sample_number] = imu_t.BMI160_Ay_f32;
			  acc_data[2][sample_number] = imu_t.BMI160_Az_f32;

			  sample_number++;

			  if(counter_led >= f/2)
			  {
				  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
				  counter_led = 0;
			  }

			  counter_led++;
		  }
	  }
}
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  arm_biquad_cascade_df1_init_f32(&iir_filter_gyro_x_low, 1, coeffs_low, iir_state_gyro_x_low);
  arm_biquad_cascade_df1_init_f32(&iir_filter_gyro_y_low, 1, coeffs_low, iir_state_gyro_y_low);
  arm_biquad_cascade_df1_init_f32(&iir_filter_gyro_z_low, 1, coeffs_low, iir_state_gyro_z_low);
  arm_biquad_cascade_df1_init_f32(&iir_filter_acc_x_low, 1, coeffs_low, iir_state_acc_x_low);
  arm_biquad_cascade_df1_init_f32(&iir_filter_acc_y_low, 1, coeffs_low, iir_state_acc_y_low);
  arm_biquad_cascade_df1_init_f32(&iir_filter_acc_z_low, 1, coeffs_low, iir_state_acc_z_low);

  arm_biquad_cascade_df1_init_f32(&iir_filter_acc_high, 1, coeffs_acc_high, iir_state_acc_high);

  arm_biquad_cascade_df1_init_f32(&iir_filter_orientation_low, 1, coeffs_orientation_low, iir_state_orientation_low);

  while(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_RESET)
  {
	  HAL_Delay(100);
  }

  HAL_Delay(100);


  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_SET);
  HAL_Delay(500);

  //calibration
  while (BMI160_init(imu_t) == 1){;}


    HAL_Delay(5000);

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    //end of calibration

    while(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_RESET)
    {
	    HAL_Delay(100);
    }

    HAL_Delay(1000);

    HAL_TIM_Base_Start_IT(&htim1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET)
	  {
		  HAL_TIM_Base_Stop_IT(&htim1);
		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

		  HAL_Delay(100);


		  calculate();

		  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
	  }

	  HAL_Delay(500);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
