
# 01_LED

![Untitled](01_LED/Untitled.png)

- **첫번째로 RCC (기준 클럭)을 세팅한다**

![image.png](01_LED/image.png)

- **MPU 내부에도 크리스탈이 있지만은 외부 크리스탈을 이용하여 클럭을 셋팅한다**
- **HSE (High Speed External) 이용한다**
- **HSE를 설정하면 우측의 화면 같이 PORT 가 자동으로 설정된다**

![Untitled](01_LED/Untitled%201.png)

- **Debug 및 프로그램 전송을 위한 SYS를 설정한다**

![Untitled](01_LED/Untitled%202.png)

- **Debug Mode를 Serial Wire로 설정한다**
- **설정되면 우측화면과 같이 PORT가 자동으로 선택된다**

![Untitled](01_LED/Untitled%203.png)

- **아래 화면과 같이 MPU 그림에서 PC8번 포트를 선택하면 설정할 수 있는 Function이 나옴**
- **Function은 언제든지 변경 가능함**
- **우선 출력을 설정함 → GPIO_Output**

![Untitled](01_LED/Untitled%204.png)

- **위에 그림에서 PORT를 Output으로 설정하면**
- **하기의 그림과 같이GPIO에 대한 Configuration을 할수 있음**
- **왼쪽 그림은 PC8 Port에 설정된 설명이고**
- **오른쪽 그림은 각각에 대해 설정할 수 있는 것임**

![Untitled](01_LED/Untitled%205.png)

- **GPIO output level**
    - **LOW, HIGH 가 있고 이것은 시작할 때 어떤것으로 시작할것인지 결정함**
- **GPIO mode**
    - **Output Push Pull**
        - **구성: 두 개의 트랜지스터(NPN 및 PNP)로 구성됩니다.**
        - **작동 방식:**
            - **HIGH 출력: NPN 트랜지스터가 ON되어 출력을 Vcc로 끌어올립니다.**
            - **LOW 출력: PNP 트랜지스터가 ON되어 출력을 GND로 끌어당깁니다.**
        - **장점:**
            - **빠른 상승 및 하강 시간**
            - **높은 구동력**
            - **명확한 출력 레벨 (HIGH 또는 LOW)**
        - **단점:**
            - **높은 전력 소비**
            - **더 복잡한 회로**
    - **Output Open Drain**
        - **구성: 하나의 NPN 트랜지스터로 구성됩니다.**
        - **작동 방식:**
            - **HIGH 출력: 트랜지스터가 OFF되어 출력이 High-Z(고 임피던스) 상태가 됩니다. 외부 pull-up 저항을 통해 Vcc로 연결됩니다.**
            - **LOW 출력: 트랜지스터가 ON되어 출력을 GND로 끌어당깁니다.**
        - **장점:**
            - **낮은 전력 소비**
            - **간단한 회로**
            - **여러 장치의 출력을 OR 연결할 수 있음**
        - **단점:**
            - **느린 상승 시간 (pull-up 저항 값에 따라 다름)**
            - **낮은 구동력**
            - **불확실한 출력 레벨 (HIGH-Z 상태 가능)**
    - **선택 가이드**
        - **빠른 속도와 높은 구동력이 필요한 경우: Push-Pull 출력**
        - **낮은 전력 소비가 중요한 경우: Open-Drain 출력**
        - **여러 장치의 출력을 OR 연결해야 하는 경우: Open-Drain 출력**
        - **불확실한 출력 레벨이 허용되는 경우: Open-Drain 출력**
    - **추가 정보**
        - **Open-Drain 출력은 종종 I2C, SPI, 1-Wire와 같은 통신 프로토콜에 사용됩니다.**
        - **Push-Pull 출력은 LED 구동, 모터 제어와 같은 애플리케이션에 사용됩니다.**
    - **예시**
        - **Push-Pull 출력 예시: LED를 켜고 끄는 회로**
        - **Open-Drain 출력 예시: I2C 버스에 여러 장치 연결**
    - **주의 사항**
        - **Open-Drain 출력을 사용할 때는 pull-up 저항의 값을 적절하게 선택해야 합니다.**
        - **Push-Pull 출력은 더 높은 전력 소비를 하기 때문에 전력 효율성이 중요한 경우 주의해야 합니다.**
- **GPIO Pull-up/Pull-down : 내장된 pull-up/pluu-down 을 사용 가능함**
    - **No pull-up and no pull-down**
    - **pull-up**
    - **pull-down**
- **Maximun output speed : 출력 스피드 설정**
    - **Low**
    - **Medium**
    - **High**
    - **Very High**
- **User Label**
    - **사용자가 이름을 설정할 수 있음**

---

- **GPIO 설정이 완료되면 화면 상단의 톱니바퀴를 눌러 Code 를 제네레이션 한다**

![Untitled](01_LED/Untitled%206.png)

---

- **코드 제네레이션이 완료되면 하단과 같이 각각의 코드를 분리해서 생성한다**

![Untitled](01_LED/Untitled%207.png)

---

- Intellisense 기능을 적극 활용한다

![Untitled](01_LED/Untitled%208.png)

- **HAL 함수를 활용하여 100ms 시간마다 LED가 깜빡이게 만든다**

![Untitled](01_LED/Untitled%209.png)

==========================

==========================

### ✅ **Non-Blocking**

```c
**/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
  LED_OFF,  // led off state
  LED_ON    // led of state
}LED_STATE;

// set struct for LED control
typedef struct
{
  GPIO_TypeDef  *port;
  uint16_t      number;
  uint32_t      interval;   // LED interval (ms)
  LED_STATE     state;
  uint32_t      lastTick;   // lastTick change LED blinking
}LED_CONTROL;

void updateLed(LED_CONTROL *led)
{
  uint32_t currentTick = HAL_GetTick(); // 현재의 시스템 틱(카운트)을 가져옴

  if(currentTick - led->lastTick >= led->interval)
  {
    led->lastTick = currentTick;

    if(led->state == LED_OFF)
    {
      HAL_GPIO_WritePin(led->port, led->number, 1);
      led->state = LED_ON;
    }
    else
    {
      HAL_GPIO_WritePin(led->port, led->number, 0);
      led->state = LED_OFF;
    }
  }
}

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
  /* USER CODE BEGIN 2 */

  LED_CONTROL led1 = {GPIOA, GPIO_PIN_5, 200, LED_OFF, 0};
  LED_CONTROL led2 = {GPIOA, GPIO_PIN_6, 1000, LED_OFF, 0};

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    updateLed(&led1);
    updateLed(&led2);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}**
```

==============================

==============================

### ✅ 파일 분리 형

```c
**#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"

typedef struct
{
  GPIO_TypeDef  *port;
  uint16_t      pinNumber;
  GPIO_PinState onState;
  GPIO_PinState offState;
}LED_CONTROL;

void ledOn(uint8_t num);
void ledOff(uint8_t num);
void ledToggle(uint8_t num);

void ledLeftShift(uint8_t num);
void ledRightShift(uint8_t num);
void ledFlower(uint8_t num);

#endif /* INC_LED_H_ */**
```

```c
**#include "led.h"

LED_CONTROL led[8]=
    {
        {GPIOA, GPIO_PIN_5, GPIO_PIN_SET, GPIO_PIN_RESET},
        {GPIOA, GPIO_PIN_6, GPIO_PIN_SET, GPIO_PIN_RESET},
        {GPIOA, GPIO_PIN_7, GPIO_PIN_SET, GPIO_PIN_RESET},
        {GPIOB, GPIO_PIN_6, GPIO_PIN_SET, GPIO_PIN_RESET},
        {GPIOC, GPIO_PIN_7, GPIO_PIN_SET, GPIO_PIN_RESET},
        {GPIOA, GPIO_PIN_9, GPIO_PIN_SET, GPIO_PIN_RESET},
        {GPIOA, GPIO_PIN_8, GPIO_PIN_SET, GPIO_PIN_RESET},
        {GPIOB, GPIO_PIN_10, GPIO_PIN_SET, GPIO_PIN_RESET}
    };

void ledOn(uint8_t num) // 8개 한꺼번에 켜는거
{
  for(uint8_t i = 0; i < num; i++)
  {
    HAL_GPIO_WritePin(led[i].port, led[i].pinNumber, led[i].onState);
  }
}

void ledOff(uint8_t num)  // 8개 한꺼번에 끄는거
{
  for(uint8_t i = 0; i < num; i++)
  {
    HAL_GPIO_WritePin(led[i].port, led[i].pinNumber, led[i].offState);
  }
}

void ledToggle(uint8_t num) // 지정된 핀만 토글
{
  HAL_GPIO_TogglePin(led[num].port, led[num].pinNumber);
}

void ledLeftShift(uint8_t num)
{
  for(uint8_t i = 0; i < num; i++)
  {
    ledOn(i);
    HAL_Delay(100);
  }
  HAL_Delay(500);
  for(uint8_t i = 0; i < num; i++)
  {
    ledOff(i);
  }
  HAL_Delay(500);
}

void ledRightShift(uint8_t num)
{

}

void ledFlower(uint8_t num)
{

}**
```

```c
**/* USER CODE BEGIN Header */
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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "led.h"

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    ledOn(8);
    HAL_Delay(500);
    ledOff(8);
    HAL_Delay(500);

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
#endif /* USE_FULL_ASSERT */**
```
