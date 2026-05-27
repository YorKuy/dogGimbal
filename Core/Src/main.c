/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RM_Lib.h"
#include "communication.h"
#include "CP_System.h"
#include "SMC.h"
#include "DM.h"
#include "RM.h"
#include "PID.h"
#include "YAW.h"
#include "Pitch.h"
#include "Shoot.h"
#include "hipnuc_dec.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ROBOT_ID_MASK (0x0001 << 0)
#define DOGHOLE_FLAG_MASK (0x0001 << 1)
#define SPEED_CUT_FLAG_MASK (0x0001 << 2)
#define XTL_FLAG_MASK (0x0001 << 3)
#define PI 3.1415926
#define JG_ON HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET)

#define PRINTF(x) INFO(#x "=%.1f\n", (x))
#define PRINTF_INFO(x) INFO(#x "=%d\n", (x));

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
USER_CAN CAN_1(&hcan1, 0),
    CAN_2(&hcan2, 1);
MOTOR_RM M6020_YAW(0x208, &CAN_2),
    M3508_MCL_Right(0x201, &CAN_1),
    M3508_MCL_Left(0x202, &CAN_1),
    M2006_BP(0x204, &CAN_2);

MOTOR_DM DM_PITCH(0x11, &CAN_1),
    DM_YAW(0x12, &CAN_2);
BMI088 GIMBAL_088(&hspi1, &htim7, GPIOA, GPIO_PIN_4, GPIOC, GPIO_PIN_4, 3000, 1.7f, BMI088_GYRO_RANGE_2000, BMI088_ACC_RANGE_24);
// RGB_UI RGB_UI(&htim3,TIM_CHANNEL_1,"put the Update function in 800kHz interrupt");
extern uint16_t zm_test_flag;
extern YAW *yaw;
extern PITCH *pitch;
extern MCL *mcl;
extern BP *bp;
extern uint8_t test_flag_1, test_flag_2;

Vision_LPF V_Pitch(30, 0.001, PI);
Vision_LPF V_Yaw(30, 0.001, PI);
Vision_LPF Yaw_PID_OUT(20, 0.001, PI);
Vision_LPF Pitch_PID_OUT(20, 0.001, PI);
Vision_LPF LPF_Deal_pitch(30, 0.001, PI);
Vision_LPF Pitch_MCU(10, 0.001, PI);
RC YK(&huart6, &huart1);
//-----------------------------------------------------------------------------------------------------//
uint8_t YK_Mode = PROTECT_MODE;
uint8_t GIMBAL_088_State = BMI088_ERROR;
uint8_t YAW_Mode = PROTECT_MODE, PITCH_Mode = PROTECT_MODE;
f Pitch_Pid_Out, Yaw_Pid_Out, BP_PID_OUT;
int remain_heat;
uint8_t jianshu_flag = 0, Prefabricate_Flag = 0;
uint16_t shooter_id1_17mm_barrel_cooling_value, shooter_id1_17mm_cooling_limit = 0, shooter_id1_17mm_cooling_rate = 0, shooter_id1_17mm_cooling_heat = 0;
uint16_t Communicate_Send_Flag_1, Communicate_Rx_Flag_1;
uint8_t Buff_Flag = 0;
float LPF_Pitch_out;
uint8_t DR16_Stop_Flag;
uint16_t barrel_cooling_value = 0, cooling_limit = 0, cooling_heat = 0;

float *MCL_PID_OUT = nullptr;
uint8_t Right_Flag = 0;
float pitch_PID_OUT, LPF_Deal_pitch_out;
//----------------------------------------INFO-----------------------------------------------//
static uint8_t free_FIFO;
static uint8_t test_flag;
static uint8_t Motor_test_flag = 0;
static f test_out = 0;
static f pitch_test;
uint16_t Arrmor_id, Last_Arrmor_id;
uint8_t Arrmor_event_seq, Last_Arrmor_event_seq;
static u8 start_flag;
static u8 buff_start_flag;
u8 buff_mode;
static uint8_t start_yaw = 0, start_pitch = 0, time_yaw = 0;
//----------------------------------------------UpDowm checking--------------------------------------------------------//
UpDown_check_class UD_E(0), UD_SpeedUp(0), UD_SpeedDown(0), UD_Buff(0), UD_YK_BoPan(0), UD_BoPan_lian(0), UD_GenSui(0), UD_l(0), UD_ch0_exceed_600(0), UD_Laser(0);
UpDown_check_class UD_GenSui_chance(0);
YKStateTransitionDetector YK_MODE_SW_C_N(0), YK_MODE_SW_C_S(0), YK_MODE_SW_N_C(0), YK_MODE_SW_S_C(0);

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//--------------------------------------------TASK1---------------------------------------------------------------------------//
void MODE_DEAL()
{
  if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = PROTECT_MODE;
  else if (YK.yaogan.s1 == YK_SW_UP && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = ONLY_GIMBAL;
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = ONLY_CHASSIC;
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = CONTROL_MODE;
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_MID)
    YK_Mode = XTL_MODE;
  else if (YK.yaogan.s1 == YK_SW_MID && YK.yaogan.s2 == YK_SW_DOWN)
    YK_Mode = SHOOT_MODE;
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_DOWN)
    YK_Mode = PLAYER_MODE;
  else if (YK.yaogan.s1 == YK_SW_DOWN && YK.yaogan.s2 == YK_SW_UP)
    YK_Mode = FAST_CHASSIC;
  else
    YK_Mode = PROTECT_MODE;

  if (GIMBAL_088_State == BMI088_OK && (YK_Mode == CONTROL_MODE || YK_Mode == ONLY_GIMBAL || (YK_Mode == SHOOT_MODE && !request.zimiao_status) || YK_Mode == PLAYER_MODE || YK_Mode == XTL_MODE) && !request.zimiao_status)
  {
    YAW_Mode = GYRO_MODE;
  }
  else if (request.zimiao_status)
  {
    YAW_Mode = AUTO_MODE;
  }
  else
  {
    YAW_Mode = PROTECT_MODE;
  }

  if (GIMBAL_088_State == BMI088_OK && (YK_Mode == CONTROL_MODE || YK_Mode == ONLY_GIMBAL || YK_Mode == SHOOT_MODE || YK_Mode == PLAYER_MODE || YK_Mode == XTL_MODE) && !request.zimiao_status)
  {
    PITCH_Mode = GYRO_MODE;
  }
  else if (request.zimiao_status)
  {
    PITCH_Mode = AUTO_MODE;
  }
  else
  {
    PITCH_Mode = PROTECT_MODE;
  }
}
void can2_communicate_deal()
{
  static uint8_t flag = 0;

  if (YK.VT13_Data.mode_sw)
  {
    if (YK.VT13_rx_buffer[0] == 0xA9 && YK.VT13_rx_buffer[1] == 0x53)
    {
      YK.VT13_rx_buffer[7] = (YK.VT13_rx_buffer[7] & 0x0F) | (0xA << 4);
      CAN_2.Send_RM(0x110,
                    YK.VT13_rx_buffer[3] | (YK.VT13_rx_buffer[2] << 8),
                    YK.VT13_rx_buffer[5] | (YK.VT13_rx_buffer[4] << 8),
                    YK.VT13_rx_buffer[7] | (YK.VT13_rx_buffer[6] << 8),
                    YK.VT13_rx_buffer[18] | (YK.VT13_rx_buffer[17] << 8));
    }
  }
  else
  {
    CAN_2.Send_RM(0x110,
                  YK.DR16_rx_buffer[1] | (YK.DR16_rx_buffer[0] << 8),
                  YK.DR16_rx_buffer[3] | (YK.DR16_rx_buffer[2] << 8),
                  YK.DR16_rx_buffer[5] | (YK.DR16_rx_buffer[4] << 8),
                  YK.DR16_rx_buffer[15] | (YK.DR16_rx_buffer[14] << 8));
  }

  if (flag)
  {
    CAN_2.Send_RM(0x113, YK.DR16_rx_buffer[17] | (YK.DR16_rx_buffer[16] << 8), Communicate_Send_Flag_1, 0, 0);
    flag = 0;
  }
  else
  {
    CAN_2.Send_RM(0x115, request.zimiao_status, 0, 0, 0);
    flag = 1;
  }

  if (CAN_2.RxHeader.StdId == 0x558)
  {
    Communicate_Rx_Flag_1 = CAN_2.rx_buf[1] | CAN_2.rx_buf[0] << 8;
    shooter_id1_17mm_barrel_cooling_value = CAN_2.rx_buf[3] | CAN_2.rx_buf[2] << 8;
    shooter_id1_17mm_cooling_limit = CAN_2.rx_buf[5] | CAN_2.rx_buf[4] << 8;
    shooter_id1_17mm_cooling_heat = CAN_2.rx_buf[7] | CAN_2.rx_buf[6] << 8;
  }
}
void jianshu_deal()
{
  if (YK_Mode == PLAYER_MODE)
  {
    jianshu_flag = GYRO_MODE;
  }
  else
    jianshu_flag = PROTECT_MODE;

  if (YK.shubiao.press_r || YK.yaogan.v < -600)
  {
    request.zimiao_status = 1;
    Right_Flag = 1;
  }
  else
  {
    request.zimiao_status = 0;
    Right_Flag = 0;
  }
  if (jianshu_flag)
  {
    if (YK.Pressed_Check(KEY_PRESSED_Z) && YK.Pressed_Check(KEY_PRESSED_CTRL))
    {
      uint8_t re_time;
      // RGB_UI.WS_WriteAll_RGB(RGB_goal,0,0);
      HAL_Delay(5);
      CAN_1.Send_RM(0x2FF, 0, 0, 0, 0);
      HAL_Delay(10);

      for (re_time = 0; re_time < 25; re_time++)
      {
        Yaw_Pid_Out = 0;
        CAN_2.Send_RM(0x1ff, 0, 0, 0, 0);
        HAL_Delay(10);
      }
      CAN_1.Send_RM(0x200, 0, 0, 0, 0);
      HAL_Delay(20);
      __set_FAULTMASK(1);
      NVIC_SystemReset();
    }
  }
  if (Communicate_Rx_Flag_1 & ROBOT_ID_MASK)
    AS.colour.typeMum = 0;
  else
    AS.colour.typeMum = 1;
  if (GIMBAL_088_State != 0)
    Communicate_Send_Flag_1 |= (0x0001 << 0);
  else
    Communicate_Send_Flag_1 &= ~(0x0001 << 0);
  if (Buff_Flag != 0)
    Communicate_Send_Flag_1 |= (0x0001 << 2);
  else
    Communicate_Send_Flag_1 &= ~(0x0001 << 2);
  if (UD_SpeedUp.updata(YK.Pressed_Check(KEY_PRESSED_F)) == UpDown_check_rising && (YK.jianpan & KEY_PRESSED_CTRL))
  {
    mcl->MCL_Change += 50;
  }

  if (UD_SpeedDown.updata(YK.Pressed_Check(KEY_PRESSED_F)) == UpDown_check_rising && !(YK.jianpan & KEY_PRESSED_CTRL))
  {
    mcl->MCL_Change -= 50;
  }
  if (UD_Buff.updata(YK.Pressed_Check(KEY_PRESSED_V)) == UpDown_check_rising || (YK_Mode == SHOOT_MODE && YK.yaogan.v > 600))
  {
    //  if(buff_start_flag == 0)
    //  {
    //     buff_start_flag = 1;
    //  }
    //  else
    //  {
    //     buff_start_flag = 0;
    //  }
    //  buff_mode = buff_start_flag;
    buff_mode = (buff_mode + 1) % 3;
  }
}
uint8_t start_deal()
{
  static u8 open = 0;
  CAN_1.Init(0, 0);
  HAL_Delay(5);
  CAN_2.Init(1, 1);
  HAL_Delay(5);
  YK.VT13_Init();
  YK.DT16_Init();
  HAL_Delay(5);
  Mini_PC_Init();
  HAL_Delay(5);
  DM_PITCH.DM_Start(0x01);
  HAL_Delay(10);
  DM_YAW.DM_Start(0x02);
  HAL_Delay(5);
  GIMBAL_088_State = GIMBAL_088.Init();
  HAL_Delay(10);
  HAL_TIM_Base_Start_IT(&htim1); // 500Hz
  HAL_TIM_Base_Start_IT(&htim8); // 1000Hz
  HAL_TIM_Base_Start_IT(&htim6); // 10Hz
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim2); // 2000Hz
  V_Yaw.Vision_Low_Pass_Filter_Init();
  V_Pitch.Vision_Low_Pass_Filter_Init();
  Yaw_PID_OUT.Vision_Low_Pass_Filter_Init();
  Pitch_PID_OUT.Vision_Low_Pass_Filter_Init();
  LPF_Deal_pitch.Vision_Low_Pass_Filter_Init();
  Pitch_MCU.Vision_Low_Pass_Filter_Init();
  HAL_Delay(5);
  open = 1;
  return open;
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_UART5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(500);
  // IMU_UART_Init();
  start_flag = start_deal();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    JG_ON;
    MODE_DEAL();
    mcl->MCL_while_layer(YK_Mode);
    bp->BP_while_layer();
    jianshu_deal();
    yaw->set_SMCref(yaw->Target_Angle);
    pitch->set_SMCref(pitch->Target_Angle);
    //------------------------------------------------------------INFO------------------------------------------------------------------//
    //	  INFO("%d\n",One_Target_Angle);
    //      PRINTF(Pitch_Pid_Out);
    //    INFO("%.f,%.f,%.f,%.f\n",Target_Yaw_Angle,GIMBAL_088.realAngle.yaw,M6020_Yaw_Speed.OUT_PID,GIMBAL_088.Anglespeed.Deal_yaw);
    //      INFO("%.f,%.f,%.f,%.f\n",Target_Pitch_Angle,GIMBAL_088.realAngle.roll,Pitch_Pid_Out,SuperPower.pitch.f);
    //      INFO("%.f,%.f,%.f,%.f\n",Target_Yaw_Angle,GIMBAL_088.realAngle.yaw,Target_Pitch_Angle,GIMBAL_088.realAngle.roll);
    //      INFO("%.2f,%.2f\n",pitch->Target_Angle,GIMBAL_088.realAngle.roll);
    //      INFO("%.2f\n", bp->PID_OUT);
    // INFO("%.2f,%.2f\n",hipnuc_raw.hi91.pitch,GIMBAL_088.realAngle.roll);
    // INFO("%.2f\n",Yaw_Pid_Out);
    // INFO("%.2f,%.2f\n", GIMBAL_088.realAngle.yaw, yaw->Target_Angle);
    //--------------------------------------------------TASK-----------------------------------------------------//

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &htim1) // 500Hz            //CAN2通讯
  {
    can2_communicate_deal();
  }
  if (htim == &htim6) // 10Hz
  {
    if (!DR16_Stop_Flag)
      YK.DT16_watchdog_run();
    YK.VT13_watchdog_run();
    static uint8_t re_start_flag = 0, motor_flag = 0;
    if (re_start_flag % 10 == 0)
    {
      if (motor_flag)
      {
        DM_YAW.DM_Start(0x02);
        test_flag++;
        re_start_flag = 0;
        motor_flag = 0;
      }
      else
      {
        DM_PITCH.DM_Start(0x01);
        motor_flag = 1;
        re_start_flag = 0;
      }
    }
    re_start_flag++;
  }
  if (htim == &htim7) // 1000Hz
  {
    GIMBAL_088.analyse();
    //bp->BP_time_out();
  }
  if (htim == &htim2) // 2000Hz
  {
    static uint8_t dm_send_flag = 0;
    if (dm_send_flag % 2 == 0)
    {
      if (DM_PITCH.ERR == 1)
      {
        if (PITCH_Mode == PROTECT_MODE)
          DM_PITCH.DM_MIT(0x01, 0, 0, 0, 0, 0); // Pitch_Pid_Out
        else
          DM_PITCH.DM_MIT(0x01, 0, 0, 0, 0.5, -Pitch_Pid_Out); //-Pitch_Pid_Out
      }
    }
    else
    {
      if (DM_YAW.ERR == 1)
      {
        if (YAW_Mode == PROTECT_MODE)
          DM_YAW.DM_MIT(0x02, 0, 0, 0, 0, 0); // Yaw_Pid_Out
        else
          DM_YAW.DM_MIT(0x02, 0, 0, 0, 0.5, Yaw_Pid_Out); // Yaw_Pid_Out
      }
    }
    dm_send_flag++;
  }

  if (htim == &htim8) // 1000Hz
  {
    AS.Q_info_0.f = GIMBAL_088.q0_t;
    AS.Q_info_1.f = GIMBAL_088.q1_t;
    AS.Q_info_2.f = GIMBAL_088.q2_t;
    AS.Q_info_3.f = GIMBAL_088.q3_t;
    AS.heat_speed.f = 24.4;
    Mini_PC_SendData();
    if (YK_Mode == PROTECT_MODE)
    {
      CAN_1.Send_RM(0x200, 0, 0, 0, 0);
    }
    else
    {
      CAN_1.Send_RM(0x200, mcl->PID_OUT[0], mcl->PID_OUT[1], 0, 0);
    }
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (CAN_1.Receive(&hcan1) == HAL_OK)
  {
    if (DM_PITCH.DM_update() == HAL_OK)
    {
      Pitch_Pid_Out = pitch->Pitch_Out_Interface(jianshu_flag);
    }
    MCL_PID_OUT = mcl->MCL_deal(YK_Mode);
  }
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (CAN_2.Receive(&hcan2) == HAL_OK)
  {
    if (DM_YAW.DM_update() == HAL_OK)
    {
      Yaw_Pid_Out = yaw->Yaw_Out_Interface(jianshu_flag);
      // CAN_2.Send_RM(0x1FE, 0, 0, 0, -Yaw_Pid_Out);
    }
    if (M2006_BP.update() == HAL_OK)
    {
      BP_PID_OUT = bp->BP_deal(YK_Mode, jianshu_flag);
      CAN_2.Send_RM(0x200, 0, 0, 0, BP_PID_OUT);
    }
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
