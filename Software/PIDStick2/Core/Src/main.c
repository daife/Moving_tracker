/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "stdio.h"
// #include "tgmath.h"
#include <math.h>
#include "motion/motion_types.h"
#include "motion/EKF.h"
#include "drv8833.h"
#include "mpu6500/mpu6500_fun.h"
#include "key.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

struct __pid
{
    float Dst;
    float kp, ki, kd;
    float duty_of_pid;
    float err0;    // �??????????????????????????????????????初误差，用来计算变积�??????????????????????????????????????
    float err[3];  // 储存�??????????????????????????????????????近三个误�??????????????????????????????????????
    uint8_t ep;    // 指针指向�??????????????????????????????????????近误�??????????????????????????????????????
    uint8_t state; // 状�?�量，判断是否有之前误差（为了处理前两次数据处理�??????????????????????????????????????0
    float integral;
} pid;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define duty_of_G 147 // 和重力平衡的功率绝对值，可以考虑初始化时候测�??????????????????????????????????????
#define outMax 256    // 总输出绝对�?�达到该值，只累加正/负积�??????????????????????????????????????
#define errorThres 16 // 角度误差在此以内积分系数�??????????????????????????????????????1，超出则变积�??????????????????????????????????????
#define DST 90        // 默认角度
#define KP 1.0     // 默认PID
#define KI 0.005
#define KD 165
#define ON 1 // 状�?�量
#define OFF 0
// #define WILLON 3/启动状�??
// #define WILLONDUTY 50//启动占空�??????????????????????????
// #define REPEATCOUNT 10//启动时段持续周期�??????????????????????????10ms*repeatcount
#define WILLOFF 2     // 准备躺下阶段
#define OFFANGLE1 30   // 第一躺下稳定角度
#define OFFANGLE2 5//第二躺下角度
#define WAITTIME 800 // 延时等待时间
#define Integral_limit 30

#define IN1_CH TIM_CHANNEL_1
// 变积分，防止积分超调
// 增量型，适用于指令变�??????????????????????????????????????
// 抗积分饱和？
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
sensor_accel_fifo_s accel_fifo;
sensor_gyro_fifo_s gyro_fifo;
float angle[3];
uint8_t controlNum = 0;       // 用于时序控制
float forwardduty = 0;        // 前馈
float sumduty = 0;            // 前馈与PID反馈的和
float param1, param2, param3; // 暂存�??????????????????????????�??????????????????????????增加的PID三�?�的�??????????????????????????
char received[12];
float g=duty_of_G;

// 积分限制
float integral_limit = Integral_limit;

uint8_t state = OFF; // 关闭状�??

uint8_t ifdelay = 0; // 是否延时

uint8_t KeyCode = 0; // 按键部分

float tempDst = DST; // 关闭时�?�暂存目标角�????????,在串口�?�讯中调用完uodateDst后赋值同样的�????????
float tempkd = KD;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// 打印
void printang()
{
    char angle_str[100];
    snprintf(angle_str, 100, "Roll:%d duty:%d\n P:%d I:%d D:%d im:%d", (int)angle[0], (int)sumduty, (int)(pid.kp * 100), (int)(pid.ki * 10000), (int)(pid.kd),(int)(pid.integral*10000));
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)angle_str, strlen(angle_str));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT);
}

void PID_init()
{
    pid.kp = KP;
    pid.ki = KI;
    pid.kd = KD;
    pid.Dst = DST;
    pid.duty_of_pid = 0;
    pid.err0 = 0; // 用于存储上一次的误差
    pid.integral = 0; // 初始化积分项
}

void PID_reinit() {
    pid.Dst = tempDst;
    pid.duty_of_pid = 0;
    pid.kp=KP;
    pid.kd = tempkd;
    pid.err0 = 0; // 重置上一次误�????????
    pid.integral = 0; // 重置积分�????????
    g=duty_of_G;
}

void PID_update()
{
    // 计算误差
    float err = pid.Dst - angle[0];

    // 位置式PID计算
    float p = pid.kp * err;

    pid.integral += err;
    // if(err<=errorThres&&err>=-errorThres) {
    //     integral_limit =Integral_limit;
    // }else {
    //     integral_limit = 30;
    // }
    if (pid.integral * pid.ki > integral_limit && pid.ki > 0)
    {
        pid.integral = integral_limit / pid.ki;
    }
    else if (pid.integral * pid.ki < -integral_limit && pid.ki > 0)
    {
        pid.integral = -integral_limit / pid.ki;
    }

    float i = pid.ki * pid.integral;
    float d = pid.kd * (err - pid.err0);
    pid.err0 = err; // 更新误差

    // 计算PID输出
    pid.duty_of_pid = p + i + d;
    // 前馈控制
    forwardduty = cos(angle[0] * M_PI / 180.0) * g;

    // 计算总输�????????
    sumduty = forwardduty + pid.duty_of_pid;
    // if (sumduty>outMax)
    // {
    //   sumduty = outMax;
    // }
    // else if (sumduty<-outMax)
    // {
    //     sumduty = -outMax;
    // }


    // if(pid.Dst<=90&&sumduty<=0) {
    //
    //     sumduty = 0;
    // }


}

void PID_shutdown()
{
    pid.duty_of_pid = 0;
    DRV8833_Control(0);
} // 刹停，测试用

void PID_updateDst(uint8_t dst)
{
    pid.Dst = dst;
} // 更新目标

// �??????????????????????????有的流程控制放在这，通过计数器控制，尽量多用封装函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim5)
    {
        if (controlNum >= 0 && controlNum <= 35)
        {
            mpu6500_run();
        }

        if (controlNum >= 15)
        {
            fusion_update(&accel_fifo, &gyro_fifo, angle);

            if (state == ON || state == WILLOFF)
            {
                PID_update();
                DRV8833_Control(sumduty);
                if (state == WILLOFF&&ifdelay==0)
                {
                    if (pid.err[pid.ep] >= -2)
                    {
                       // 准备延时在主循环关闭
                        ifdelay = 1;
                   }
                }
            }
        }
        else
        {
            fusion_cali(&gyro_fifo);
        }

        // 串口打印控制
        if (controlNum <= 34)
        {
            controlNum++;
        }
        else if (controlNum == 35)
        {
            controlNum = 15;
            printang();
        }
        KEY_Scan();
        KeyCode = KEY_FIFO_Get();
        if (KeyCode == KEY_UP_K1)
        {
            // WILL阶段不可�??????????????????????????
            if (state == OFF)
            {
                HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // 点亮示意
                state = ON;
            }
            else if (state == ON)
            {
                // HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);//关闭示意
                state = WILLOFF;
                pid.kd=160;
                PID_updateDst(OFFANGLE1);//不更新暂存目标�??
            }
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart == &huart1)
    {

        // 由于数组只有前四个元素可能有内容，所以这里直接读取前四个字符
        if (received[0] == '\0' || received[1] == '\0')
            return; // 确保至少有两个字�??????????

        char type = received[0]; // 获取第一个字符，用于判断类型
        char numberStr[5];       // 创建�??????????个大小为5的缓冲区来存储数字部分（包括null终止符）

        // 由于只有前四个字符可能有内容，直接复制第二个到第四个字符到numberStr
        // 第一个字符是类型标识，不包含在数字中
        int i;
        for (i = 1; i < 4 && received[i] != '\0'; ++i)
        {
            numberStr[i - 1] = received[i];
        }
        numberStr[i - 1] = '\0'; // 确保字符串以null结尾

        long value = strtol(numberStr, NULL, 10); // 转换字符串为长整�??????????

        switch (type)
        {
        case 'P':
        case 'p':
            pid.kp = (float)value / 100;
            // printf("Variable P updated to: %f\n", pid.kp);
            break;
        case 'I':
        case 'i':
            pid.ki = (float)value / 10000;
            // printf("Variable I updated to: %f\n", pid.ki);
            break;
        case 'D':
        case 'd':
            pid.kd = (float)value;
            tempkd = (float)value;
            // printf("Variable D updated to: %f\n", pid.kd);
            break;
        case 'W':
        case 'w':
            //sumduty = (float)value;

            PID_updateDst(value);
            tempDst=value;
            // printf("Variable W updated to: %f\n", sumduty);
            break;
        default:
            // 无效的输入类�??????????
            break;
        }

    }
    memset(received, 0, sizeof(received));
    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, received, sizeof(received));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT);
}
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
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // �??????????????????????????螺仪
    mpu6500_init(&hi2c1, &htim5, &accel_fifo, &gyro_fifo);

    fusion_init();
    HAL_Delay(400);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    // pwm
    DRV8833_Init();
    KEY_Init();
    HAL_TIM_Base_Start_IT(&htim5);
    //   //初始化PID
    PID_init();
    HAL_Delay(500);

     HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);



    // 测试
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, received, sizeof(received));
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT);
    HAL_Delay(300);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(500);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

        if (ifdelay == 1)
        {
            HAL_Delay(2000);
            PID_updateDst(OFFANGLE2);
            pid.kp=1.2;
            g=135;
            ifdelay = 2;
            HAL_Delay(2000);

            // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, received, sizeof(received));
            // __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
            // __HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT);
        }
        if (ifdelay == 2&&pid.err[pid.ep] >= 0) {

            state = OFF;
            motor1__Coast();
            //motor2__Coast();
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // 关闭示意
            PID_reinit();//恢复之前设置的目标�?�和部分初始�????????
            ifdelay = 0;
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
