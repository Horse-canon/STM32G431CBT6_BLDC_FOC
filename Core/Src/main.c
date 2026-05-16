/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "stm32g4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mcu_uart.h"
#include "mcu_pwm.h"
#include "stdio.h"
#include "m_ctrl.h"
#include "m_parameter.h"
#include "m_foc.h"
#include "mcu_hall.h"
#include "mcu_adc.h"
#include "mcu_key.h"
#include "string.h"
#include "m_pid.h"
#include "m_rotor_angle.h"
#include "m_svpwm.h"

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void pid_vofa_debug(void);
void observer_vofa_debug(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
extern volatile uint32_t g_adc_irq;
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  drv_hall_init();
  drv_adc_init();
  drv_pwm_init();
  printf("foc driver board, hall svpwm project\r\n");	
       
  HAL_Delay(200);  // 等待系统电源稳定
  m_motor_phase_current_offset_calculate();  //相电流静态误差计算
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // m_us_radius_calculate();
    // m_motor_execute_ctrl();
    //drv_key_scan();
    //printf("%d %d %d\r\n", m_foc_unit.coordinate.q15_ud, m_id_pid_unit.q15_actual_value,m_id_pid_unit.q15_target_value);
    // m_hall_value_get();
    // switch (m_hall_unit.value) {
    //     case 1:
    //         m_svpwm_generate(MAX_DUTY_VALUE*0.5, EANGLE0);
    //         break;
    //     case 2:
    //         m_svpwm_generate(MAX_DUTY_VALUE*0.5, EANGLE240);
    //         break;
    //     case 3:
    //         m_svpwm_generate(MAX_DUTY_VALUE*0.5, EANGLE300);
    //         break;
    //     case 4:
    //          m_svpwm_generate(MAX_DUTY_VALUE*0.5, EANGLE120);
    //         break;
    //     case 5:
    //         m_svpwm_generate(MAX_DUTY_VALUE*0.5, EANGLE60);
    //         break;
    //     case 6:
    //          m_svpwm_generate(MAX_DUTY_VALUE*0.5, EANGLE180);
    //         break;
    //     default:
    //         printf("hall value: unknown\r\n");
    //         break;
    // }
    // printf("hall value: %d\r\n", m_hall_unit.value);

    observer_vofa_debug();
    // HAL_Delay(1000);
    // printf("%d\r\n", g_adc_irq);


    //pid_vofa_debug();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 ******************************************************************************
 * @brief  PID VOFA+调试  波特率1152000
 * @param  None.
 * @retval None.
 ******************************************************************************/
void pid_vofa_debug(void)
{
  char buf[256] = {0};

#if 1 
    sprintf(buf, "channels: %d,%d\r\n", \
            m_motor_ctrl.m_spd.set_spd_val, 
            m_motor_ctrl.m_spd.spd_val);
    drv_uart_send_data(DEBUG_COM, (uint8_t *)buf, (uint16_t)strlen(buf));
#endif

#if 0 
    sprintf(buf, "channels: %d,%d\r\n", \
            m_iq_pid_unit.q15_target_value, 
            m_iq_pid_unit.q15_actual_value);
    drv_uart_send_data(DEBUG_COM, (uint8_t *)buf, (uint16_t)strlen(buf));
#endif
}

/**
  ******************************************************************************
  * @brief  观测器 VOFA+调试   波特率1152000
  * @param  None.
  * @retval None.
  ******************************************************************************/
void observer_vofa_debug(void)
{
	char buf[256] = {0};

#if 0  //Iq		
    float angle_deg = (float) m_foc_unit.rotor_engle / 10922.0f;
		sprintf(buf, "channels: %d,%d,%d,%d,%f,%d,%d,%d\r\n", \
    m_iq_pid_unit.q15_target_value,
    m_iq_pid_unit.q15_actual_value,
    m_id_pid_unit.q15_actual_value,
    m_hall_unit.value,
    angle_deg,
    m_foc_unit.coordinate.q15_uq,
    m_motor_ctrl.m_spd.set_spd_val, 
    m_motor_ctrl.m_spd.spd_val
		);
#endif



#if 1  // 观察转子角度与线性化霍尔
    float angle_deg = (float) m_foc_unit.rotor_engle / 10922.0f;
    /* ==========================================================
    霍尔顺序映射表 (LUT)
    下标对应: 原始霍尔值 (0~6)
    数组值对应: 映射后的连续顺序 (1~6)
    映射关系: 1->1, 2->5, 3->6, 4->3, 5->2, 6->4
    ========================================================== */
    static const uint8_t HALL_SEQ_MAP[7] = {0, 1, 5, 6, 3, 2, 4};

    /* 1. 安全获取原始霍尔值（防止意外干扰导致数组越界）*/
    uint8_t raw_hall = m_hall_unit.value;
    if (raw_hall > 6) raw_hall = 0; 

    /* 2. 查表得出线性化的霍尔阶梯值 (1, 2, 3, 4, 5, 6) */
    uint8_t mapped_hall = HALL_SEQ_MAP[raw_hall];
    sprintf(buf, "channels: %f,%d\r\n", \
            angle_deg,       // 通道1：实时电角度 (0~65535)
            mapped_hall      // 通道2：放大后的霍尔阶梯波 (1~6)
    );
#endif


#if 0  // SVPWM 三相马鞍波观察      
    sprintf(buf, "channels: %d,%d,%d\r\n", \
            m_svpwm_unit.u_duty_value, \
            m_svpwm_unit.v_duty_value, \
            m_svpwm_unit.w_duty_value
    );
#endif

#if 0  // 相电流
    sprintf(buf, "channels: %d,%d,%d\r\n", \
            m_foc_unit.coordinate.q15_ia, \
            m_foc_unit.coordinate.q15_ib, \
            m_foc_unit.coordinate.q15_ic
    );
#endif

#if 0  // DQ轴电流
    sprintf(buf, "channels: %d,%d\r\n", \
            m_foc_unit.coordinate.q15_id, \
            m_foc_unit.coordinate.q15_iq
    );
#endif

#if 0  // DQ轴电压
    sprintf(buf, "channels: %d,%d\r\n", \
            m_foc_unit.coordinate.q15_ud, \
            m_foc_unit.coordinate.q15_uq
    );
#endif

#if 0  //δθ滤波前后角度变化比对		
		sprintf(buf, "channels: %d,%d\r\n", \
		m_obs_angle_unit.q15_delta_theta, \
		m_obs_angle_unit.q15_filter_delta_theta
		);
#endif
	
#if 0  //α电流估算跟踪		
		sprintf(buf, "channels: %d,%d\r\n", \
		m_obs_unit.q15_i_alpha_estimate, \
		m_foc_unit.coordinate.q15_i_alpha
		);
#endif
		
#if 0  //β电流估算跟踪		
		sprintf(buf, "channels: %d,%d\r\n", \
		m_obs_unit.q15_i_beta_estimate, \
		m_foc_unit.coordinate.q15_i_beta \
		);
#endif

#if 0	//估算θe eRPM跟踪
		sprintf(buf, "channels: %d,%d\r\n", \
		m_rotor_angle_unit.q15_theta_e,
		m_rotor_angle_unit.q15_erpm
		);
#endif

#if 0	//Eα Eβ反电动势估算跟踪
		sprintf(buf, "channels: %d,%d\r\n", \
		m_obs_unit.q15_e_alpha_final,
		m_obs_unit.q15_e_beta_final
		);
#endif
	drv_uart_send_data(DEBUG_COM, (uint8_t *)buf, strlen(buf));
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
#ifdef USE_FULL_ASSERT
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
