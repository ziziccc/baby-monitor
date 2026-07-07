/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : master
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi2_tx;
DMA_HandleTypeDef hdma_spi2_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// --- 전역 변수 (DMA 버퍼) ---
#define SLAVE1_FRAME_LEN 7

volatile uint8_t spi1_tx_buf[SLAVE1_FRAME_LEN] = {0};
volatile uint8_t spi1_rx_buf[SLAVE1_FRAME_LEN] = {0};

volatile uint8_t g_sim_mode = 0;
volatile uint32_t g_last_btn_tick = 0; // 버튼 디바운싱용

volatile uint32_t g_spi1_xfer_cycles = 0;   // SPI1 프레임 1번 보내는 데 걸린 사이클 수
volatile uint32_t g_spi2_xfer_cycles = 0;   // (원하면 SPI2도)

// Slave1 → Master 패킷 포맷
// Byte0 : 0xB1 (HDR_BPM_FSR)
// Byte1 : BPM_Q (0~240)
// Byte2 : STATUS (level + 플래그)
// Byte3 : FSR1_LVL (0~255)
// Byte4 : FSR2_LVL (0~255)
// Byte5 : FSR3_LVL (0~255)
// Byte6 : CHECKSUM (1~5번 XOR)

typedef struct {
    uint8_t bpm;
    uint8_t status;
    uint8_t fsr1;
    uint8_t fsr2;
    uint8_t fsr3;
    uint8_t level;
    uint8_t no_pulse;
    uint8_t high_bpm;
    uint8_t low_bpm;
    uint8_t checksum_ok;
} Slave1Frame_t;

#define SLAVE2_FRAME_LEN 6

volatile uint8_t spi2_tx_buf[SLAVE2_FRAME_LEN] = {0};
volatile uint8_t spi2_rx_buf[SLAVE2_FRAME_LEN] = {0};
volatile uint8_t s2_frame_fixed[SLAVE2_FRAME_LEN];

// Slave2 → Master 패킷 포맷 (예시)
// Byte0 : 0xB2 (HDR_CRY_DIST)
// Byte1 : CRY_FLAG (0 or 1)
// Byte2 : DIST1 (0~255)
// Byte3 : DIST2 (0~255)
// Byte4 : STATUS2 / reserved
// Byte5 : CHECKSUM (1~4번 XOR)

typedef struct {
    uint8_t hdr;
    uint8_t cry_flag;
    uint8_t dist1;
    uint8_t dist2;
    uint8_t status2;
    uint8_t checksum_ok;
} Slave2Frame_t;


typedef enum {
    BABY_EMPTY = 0,      // 상태 1: 비어 있음
    BABY_DEEP_SLEEP,     // 상태 2: 깊은 수면
    BABY_RESTLESS,       // 상태 3: 뒤척임
    BABY_ROLLOVER        // 상태 4: 뒤집기 / 쏠림 (Danger)
} BabyState_t;

typedef struct {
    uint8_t  center;        // 센서2 필터값
    uint8_t  left;          // 센서1 필터값
    uint8_t  right;         // 센서3 필터값
    uint16_t total;         // 전체 압력
    uint16_t side_sum;      // 좌우 합 (센서1+3)
    uint16_t activity;      // 활동성 점수
    int16_t  side_bias;     // 오른쪽-왼쪽 (양수=오른쪽, 음수=왼쪽)
    BabyState_t state;      // 최종 상태
} BabyStatus_t;


static uint8_t s_center_filt = 0;
static uint8_t s_left_filt   = 0;
static uint8_t s_right_filt  = 0;

static uint8_t s_prev_center = 0;
static uint8_t s_prev_left   = 0;
static uint8_t s_prev_right  = 0;

static uint16_t s_activity_score = 0;
static uint8_t  s_rollover_count = 0;

typedef enum {
    EVENT_NONE = 0,
    EVENT_CARDIAC_ARREST,   // 1. 심정지
    EVENT_BAD_POSTURE,      // 2. 위험 수면 자세 (뒤집힘)
    EVENT_FALL_RISK,        // 3. 낙상 위험
    EVENT_CRYING            // 4. 울음
} EventCode_t;

// Slave1 command bits
#define S1_CMD_LED_MASK      0x03
#define S1_CMD_LED_OFF       0x00
#define S1_CMD_LED_NORMAL    0x01  // 평상시 (녹색 or 미점등)
#define S1_CMD_LED_WARN      0x02  // 경고 (뒤집힘)
#define S1_CMD_LED_EMERG     0x03  // 긴급 (심정지/낙상)
#define S1_CMD_BUZZER_ON     0x80  // 부저 On 플래그

// Slave2 command bits
#define S2_CMD_MOTOR_OFF        0x00  // 모터 정지
#define S2_CMD_MOTOR_SOOTHE     0x01  // 진정 모션 (부드럽게 흔들기)
#define S2_CMD_MOTOR_EMERG_STOP 0x02  // 긴급 정지 (어떤 상황이든 강제 STOP)

// === 이벤트 검출용 카운터 & 커맨드 저장 ===
static EventCode_t g_event = EVENT_NONE;
static uint8_t     g_cmd_s1 = 0;
static uint8_t     g_cmd_s2 = 0;

static uint8_t s_no_pulse_cnt  = 0;
static uint8_t s_rollover_cnt  = 0;
static uint8_t s_fall_cnt      = 0;
static uint8_t s_cry_cnt       = 0;

// 튜닝 파라미터 (프레임 수 기준, 상황 보면서 조절)
#define TH_NO_PULSE_FRAMES   30   // 이 프레임 이상 no_pulse 유지 → 심정지 판단
#define TH_ROLLOVER_FRAMES    5   // ROLLOVER 연속 프레임
#define TH_FALL_FRAMES        3   // 낙상 위험 연속 프레임
#define TH_CRY_FRAMES         5   // 울음 연속 프레임

#define TH_FALL_NEAR         7   // dist1/2 < 30 이면 침대 가장자리 근처(가정)


static inline int16_t iabs16(int16_t x) { return (x < 0) ? -x : x; }

volatile BabyStatus_t g_baby;

uint8_t Parse_Slave1Frame(uint8_t rx[7], Slave1Frame_t *out)
{
    if (rx[0] != 0xB1) {
        out->checksum_ok = 0;
        return 0;
    }

    uint8_t cs_calc = 0;
    for (int i = 1; i <= 5; i++) cs_calc ^= rx[i];
    if (cs_calc != rx[6]) {
        out->checksum_ok = 0;
        return 0;
    }

    out->bpm    = rx[1];
    out->status = rx[2];
    out->fsr1   = rx[3];
    out->fsr2   = rx[4];
    out->fsr3   = rx[5];

    out->level    =  rx[2] & 0x03;
    out->no_pulse = (rx[2] >> 2) & 0x01;
    out->high_bpm = (rx[2] >> 3) & 0x01;
    out->low_bpm  = (rx[2] >> 4) & 0x01;

    out->checksum_ok = 1;
    return 1;
}

uint8_t Parse_Slave2Frame(uint8_t rx[SLAVE2_FRAME_LEN], Slave2Frame_t *out)
{
    if (rx[0] != 0xB2) {
        out->checksum_ok = 0;
        return 0;
    }

    uint8_t cs_calc = 0;
    for (int i = 1; i <= 4; i++) cs_calc ^= rx[i];  // 1~4 XOR
    if (cs_calc != rx[5]) {
        out->checksum_ok = 0;
        return 0;
    }

    out->hdr      = rx[0];
    out->cry_flag = rx[1];
    out->dist1    = rx[2];
    out->dist2    = rx[3];
    out->status2  = rx[4];

    out->checksum_ok = 1;
    return 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();              // ★ 반드시
  SystemClock_Config();
  MX_GPIO_Init();
  /* USER CODE BEGIN 1 */

  // 🔥 DWT 사이클 카운터 활성화 (Cortex-M3에서 사용 가능)
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Trace enable
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // Cycle counter enable

  SPI1_Master_Init();
  SPI2_Master_Init();
  MX_USART2_UART_Init();

  Slave1Frame_t frame;
  Slave2Frame_t frame2;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */


  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      // --- TX 패킷 채우기 (Master → Slave : 간단한 CTRL 패킷) ---
      spi1_tx_buf[0] = 0xA1;  // HDR_CTRL
      spi1_tx_buf[1] = g_cmd_s1; //
      spi1_tx_buf[2] = 0x00;
      spi1_tx_buf[3] = 0x00;
      spi1_tx_buf[4] = 0x00;
      spi1_tx_buf[5] = 0x00;
      spi1_tx_buf[6] = 0x00;

      // 7바이트 프레임 전송 + 수신
      //SPI1_Master_TransferFrame_DMA();
      SPI1_Master_TransferFrame_Polling();

      // --- RX 패킷 해석 (Slave → Master : [HDR, BPM, STATUS, FSR1, FSR2, FSR3, CS]) ---
      Parse_Slave1Frame(spi1_rx_buf, &frame);

      // ====================================================
            // 🔥 [추가된 로직] 버튼 상태에 따른 시뮬레이션 값 주입
            // ====================================================
            if (g_sim_mode == 1)
            {
                // [모드 1] 정상 시뮬레이션 (75 ~ 86 bpm)
                // HAL_GetTick()을 이용해 시간이 지남에 따라 숫자가 조금씩 변하도록 연출
                // 200ms마다 값이 바뀜
                uint8_t noise = (HAL_GetTick() / 200) % 12; // 0 ~ 11 사이 값 생성
                frame.bpm = 75 + noise;

                frame.no_pulse = 0;      // 심정지 아님
                frame.high_bpm = 0;
                frame.low_bpm = 0;
                frame.checksum_ok = 1;   // 강제로 유효 데이터 처리
            }
            else if (g_sim_mode == 2)
            {
                // [모드 2] 심정지 시뮬레이션 (0 bpm)
                frame.bpm = 0;
                frame.no_pulse = 1;      // 심정지 플래그 On

                frame.checksum_ok = 1;   // 강제로 유효 데이터 처리
            }

      if (frame.checksum_ok) {
          // FSR2 = center, FSR1 = left, FSR3 = right 라고 가정
          Baby_UpdateFromSensors(frame.fsr2, frame.fsr1, frame.fsr3, &g_baby);
      }

      // ===== Slave2: 울음 + 초음파 거리 =====
      // Master→Slave2
      spi2_tx_buf[0] = 0xA2;        // HDR_CTRL_S2
      spi2_tx_buf[1] = g_cmd_s2;    // 모터 명령
      spi2_tx_buf[2] = 0x00;
      spi2_tx_buf[3] = 0x00;
      spi2_tx_buf[4] = 0x00;
      spi2_tx_buf[5] = 0x00;

      SPI2_Master_Transfer_DMA((uint8_t*)spi2_tx_buf,
                               (uint8_t*)spi2_rx_buf,
                               SLAVE2_FRAME_LEN);
      // 여기서 한번 리맵핑
      Remap_Slave2Frame((uint8_t*)spi2_rx_buf, s2_frame_fixed);


      // 이제는 s2_frame_fixed[0] 이 0xB2 가 됨
      Parse_Slave2Frame(s2_frame_fixed, &frame2);

      // === (3) 센서값으로 다음 루프에 쓸 이벤트/커맨드 계산 ===
      if (frame.checksum_ok && frame2.checksum_ok) {
          Master_EvalEvents(&frame, &g_baby, &frame2);
      } else if (frame.checksum_ok) {
          Master_EvalEvents(&frame, &g_baby, NULL);
      } else if (frame2.checksum_ok) {
          Master_EvalEvents(NULL, &g_baby, &frame2);
      } else {
          Master_EvalEvents(NULL, &g_baby, NULL);
      }

      // TODO: GUI로 이벤트 전송 (USART2)
      GUI_SendEvent(&frame, &g_baby, &frame2);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      Simple_Delay(50000);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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


static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Simple_Delay(__IO uint32_t nCount) {
    for (; nCount != 0; nCount--);
}

// --- SPI1 및 DMA 초기화 함수 ---
void SPI1_Master_Init(void) {
    // 1. 클럭 활성화: GPIOA, SPI1, DMA1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;

    // 2. GPIO 설정 (PA4~PA7)
    // PA4 = NSS (소프트웨어 제어용 출력)
    // PA5 = SCK (AF-PP)
    // PA6 = MISO (Input floating)
    // PA7 = MOSI (AF-PP)
    GPIOA->CRL &= ~(0xFFFF0000);    // PA4~7 초기화

    GPIOA->CRL |= (0x3 << 16);      // PA4: Output 50MHz, GP push-pull
    GPIOA->CRL |= (0xB << 20);      // PA5: AF-PP, 50MHz (SCK)
    GPIOA->CRL |= (0x4 << 24);      // PA6: Input floating (MISO)
    GPIOA->CRL |= (0xB << 28);      // PA7: AF-PP, 50MHz (MOSI)

    GPIOA->BSRR = (1 << 4);         // CS High (Slave 비선택)

    // 3. SPI1 설정 (Master, Mode3: CPOL=1, CPHA=1 → Slave와 동일)
    SPI1->CR1 = 0;
    SPI1->CR1 |= SPI_CR1_MSTR;      // Master
    SPI1->CR1 |= SPI_CR1_CPOL;      // CPOL=1
    SPI1->CR1 |= SPI_CR1_CPHA;      // CPHA=1
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI; // Software NSS
    SPI1->CR1 |= (0x4 << 3);        // BaudRate = f_PCLK/32 정도 (적당히 느리게)

    // 4. SPI Enable
    SPI1->CR1 |= SPI_CR1_SPE;

    // 5. DMA 채널 기본 설정 (Normal 모드, 아직 Enable 안 함)
    // RX: DMA1 Channel2
    DMA1_Channel2->CCR  = 0;
    DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel2->CMAR = (uint32_t)spi1_rx_buf;
    DMA1_Channel2->CCR |= DMA_CCR_MINC;   // Memory increment, 8bit alignment

    // TX: DMA1 Channel3
    DMA1_Channel3->CCR  = 0;
    DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;
    DMA1_Channel3->CMAR = (uint32_t)spi1_tx_buf;
    DMA1_Channel3->CCR |= DMA_CCR_MINC;   // Memory increment
    DMA1_Channel3->CCR |= DMA_CCR_DIR;    // Memory → Peripheral

    // 6. SPI DMA 요청 허용
    SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN;
}

// === SPI2 / Slave2용 CS 핀 (PB12) ===
#define CS2_LOW()   (GPIOB->BRR  = (1U << 12))
#define CS2_HIGH()  (GPIOB->BSRR = (1U << 12))

void SPI2_Master_Init(void)
{
    // 1. 클럭 활성화: GPIOB, AFIO, SPI2, DMA1
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   // AFIO
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   // GPIOB
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;   // SPI2 (APB1)
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;    // DMA1

    // 2. GPIO 설정
    // PB13 = SCK2 (AF-PP 50MHz)
    // PB14 = MISO2 (Input floating)
    // PB15 = MOSI2 (AF-PP 50MHz)
    // PB12 = CS2  (GPIO Output push-pull 50MHz)

    // CRH: 핀 8~15
    GPIOB->CRH &= ~(
        (0xF << ((12 - 8) * 4)) |
        (0xF << ((13 - 8) * 4)) |
        (0xF << ((14 - 8) * 4)) |
        (0xF << ((15 - 8) * 4))
    );

    // PB12: Output 50MHz, Push-Pull → 0x3
    GPIOB->CRH |= (0x3 << ((12 - 8) * 4));

    // PB13: AF-PP 50MHz → 0xB
    GPIOB->CRH |= (0xB << ((13 - 8) * 4));

    // PB14: Input floating → 0x4
    GPIOB->CRH |= (0x4 << ((14 - 8) * 4));

    // PB15: AF-PP 50MHz → 0xB
    GPIOB->CRH |= (0xB << ((15 - 8) * 4));

    // CS2 High (비선택)
    CS2_HIGH();

    // 3. SPI2 설정 (Master, 모드는 Slave2와 동일하게)
    SPI2->CR1 = 0;
    SPI2->CR1 |= SPI_CR1_MSTR;      // Master

    // Slave1과 같은 Mode3(CPOL=1, CPHA=1) 쓴다고 가정
    SPI2->CR1 |= SPI_CR1_CPOL;
    SPI2->CR1 |= SPI_CR1_CPHA;

    // 소프트 NSS (마스터는 SSM=1, SSI=1, 실제 CS는 GPIO로 제어)
    SPI2->CR1 |= (SPI_CR1_SSM | SPI_CR1_SSI);

    // BaudRate: f_PCLK/32 (필요시 조절)
    SPI2->CR1 &= ~(7U << 3);    // BR 비트 클리어
    SPI2->CR1 |=  (0x6U << 3);
    // 4. DMA 기본 설정
    // RX: DMA1 Channel4
    DMA1_Channel4->CCR  = 0;
    DMA1_Channel4->CPAR = (uint32_t)&(SPI2->DR);
    DMA1_Channel4->CCR |= DMA_CCR_MINC;   // 메모리 인크리먼트

    // TX: DMA1 Channel5
    DMA1_Channel5->CCR  = 0;
    DMA1_Channel5->CPAR = (uint32_t)&(SPI2->DR);
    DMA1_Channel5->CCR |= DMA_CCR_MINC;
    DMA1_Channel5->CCR |= DMA_CCR_DIR;    // Memory → Peripheral

    // 5. SPI2 DMA 요청 활성화
    SPI2->CR2 |= (SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

    // 6. SPI2 Enable
    SPI2->CR1 |= SPI_CR1_SPE;
}

extern volatile uint32_t g_spi1_xfer_cycles;
void SPI1_Master_TransferFrame_Polling(void)
{
    uint32_t start = DWT->CYCCNT;   // ★ 시작 사이클 기록

    // 1) CS Low → Slave1 선택
    GPIOA->BRR = (1U << 4);

    // 2) 7바이트 프레임 전송
    for (int i = 0; i < SLAVE1_FRAME_LEN; i++)
    {
        // TXE(전송 가능) 기다림
        while (!(SPI1->SR & SPI_SR_TXE));

        // 전송 시작
        *((__IO uint8_t *)&SPI1->DR) = spi1_tx_buf[i];

        // RXNE(수신 완료) 기다림
        while (!(SPI1->SR & SPI_SR_RXNE));

        // 수신한 바이트 읽기
        spi1_rx_buf[i] = *((__IO uint8_t *)&SPI1->DR);
    }

    // 3) SPI 전송 완료(SPI_SR_BSY 클리어) 대기
    while (SPI1->SR & SPI_SR_BSY);

    // 4) CS High → 프레임 종료
    GPIOA->BSRR = (1U << 4);

    uint32_t end = DWT->CYCCNT;     // ★ 끝 사이클
    g_spi1_xfer_cycles = end - start;   // ★ Polling 방식 소요 사이클 저장
}

void SPI1_Master_TransferFrame_DMA(void)
{
    uint32_t start = DWT->CYCCNT;   // ★ 시작 사이클

    // 1) 혹시 켜져 있을 DMA 채널 Disable
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;

    // 2) 전송 길이 FRAME_LEN으로 재설정
    DMA1_Channel2->CNDTR = SLAVE1_FRAME_LEN;
    DMA1_Channel3->CNDTR = SLAVE1_FRAME_LEN;

    // 3) 이전 TC 플래그 클리어
    DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CTCIF3;

    // 4) CS Low → 프레임 시작
    GPIOA->BRR = (1 << 4);

    // 5) DMA 채널 Enable (RX → TX 순서 권장)
    DMA1_Channel2->CCR |= DMA_CCR_EN;   // RX
    DMA1_Channel3->CCR |= DMA_CCR_EN;   // TX

    // 6) RX/TX 완료 대기
    while (!(DMA1->ISR & DMA_ISR_TCIF2));  // RX TC
    while (!(DMA1->ISR & DMA_ISR_TCIF3));  // TX TC

    // 7) SPI BSY 클리어 대기 (마지막 비트 전송 끝날 때까지)
    while (SPI1->SR & SPI_SR_BSY);

    // 8) CS High → 프레임 종료
    GPIOA->BSRR = (1 << 4);

    // 9) 다음 프레임 대비 DMA 채널 Disable
    DMA1_Channel2->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;

    uint32_t end = DWT->CYCCNT;                 // ★ 끝 사이클
    g_spi1_xfer_cycles = end - start;           // ★ 한 프레임에 걸린 사이클 수
}


void SPI2_Master_Transfer_DMA(uint8_t *tx, uint8_t *rx, uint16_t len)
{
    if (!len) return;

    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    DMA1_Channel4->CMAR  = (uint32_t)rx;
    DMA1_Channel4->CNDTR = len;
    DMA1_Channel5->CMAR  = (uint32_t)tx;
    DMA1_Channel5->CNDTR = len;

    DMA1->IFCR = DMA_IFCR_CTCIF4 | DMA_IFCR_CTCIF5;

    CS2_LOW();

    DMA1_Channel4->CCR |= DMA_CCR_EN;
    DMA1_Channel5->CCR |= DMA_CCR_EN;

    while (!(DMA1->ISR & DMA_ISR_TCIF4));
    while (!(DMA1->ISR & DMA_ISR_TCIF5));

    while (SPI2->SR & SPI_SR_BSY);

    CS2_HIGH();

    DMA1_Channel4->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;

    volatile uint16_t tmp;
    tmp = SPI2->DR;
    tmp = SPI2->SR;
    (void)tmp;
}

void Remap_Slave2Frame(uint8_t in[SLAVE2_FRAME_LEN],
                       uint8_t out[SLAVE2_FRAME_LEN])
{
    // 2바이트 왼쪽 회전: out[i] = in[(i + 2) % 6]
    for (int i = 0; i < SLAVE2_FRAME_LEN; i++) {
        out[i] = in[(i + 2) % SLAVE2_FRAME_LEN];
    }
}

/**
 * @brief  센서 값 3개(센터/왼/오)를 받아서 상태를 업데이트
 * @param  fsr_center_raw  센터(센서2, Top Center) raw 값 (0~255)
 * @param  fsr_left_raw    왼쪽(센서1)
 * @param  fsr_right_raw   오른쪽(센서3)
 * @param  out             결과 저장용 구조체 포인터 (NULL 가능)
 */
void Baby_UpdateFromSensors(uint8_t fsr_center_raw,
                            uint8_t fsr_left_raw,
                            uint8_t fsr_right_raw,
                            BabyStatus_t *out)
{
    // 1) IIR 필터
    s_center_filt += (int8_t)(((int16_t)fsr_center_raw - (int16_t)s_center_filt) >> 2);
    s_left_filt   += (int8_t)(((int16_t)fsr_left_raw   - (int16_t)s_left_filt)   >> 2);
    s_right_filt  += (int8_t)(((int16_t)fsr_right_raw  - (int16_t)s_right_filt)  >> 2);

    // 2) 순간 움직임
    int16_t dC = (int16_t)s_center_filt - (int16_t)s_prev_center;
    int16_t dL = (int16_t)s_left_filt   - (int16_t)s_prev_left;
    int16_t dR = (int16_t)s_right_filt  - (int16_t)s_prev_right;

    uint16_t move_inst = (uint16_t)(iabs16(dC) + iabs16(dL) + iabs16(dR));

    s_prev_center = s_center_filt;
    s_prev_left   = s_left_filt;
    s_prev_right  = s_right_filt;

    // 3) Activity score (EMA)
    s_activity_score += (((int32_t)move_inst - (int32_t)s_activity_score) >> 3);

    uint8_t center = s_center_filt;
    uint8_t left   = s_left_filt;
    uint8_t right  = s_right_filt;

    uint16_t total = (uint16_t)center + left + right;

    BabyState_t st = BABY_EMPTY;

    // ------------------------------------------
    // 0) EMPTY 판단 수정 — 새 조건
    //    센서 값 하나라도 >0 이면 EMPTY 아님
    // ------------------------------------------
    if (center == 0 && left == 0 && right == 0) {
        st = BABY_EMPTY;
        s_rollover_count = 0;
    }
    else
    {
        // ------------------------------------------
        // 1) ROLLOVER 최우선 조건 (센서2 >= 80)
        // ------------------------------------------
        if (center >= 70) {
            st = BABY_ROLLOVER;
            s_rollover_count = 10;   // 디바운싱 + 상태 유지
        }

        else
        {
            // 뒤집힘이 아님 → 나머지 상태 계산

            // 2) DEEP SLEEP: activity 낮고, 센서2 중간 구간
            if (s_activity_score < 3 &&
                center >= 15 && center <= 60)
            {
               st = BABY_DEEP_SLEEP;
            }
            else
            {
               // 3) RESTLESS: activity 높음 또는 좌우 센서 반응
               if (s_activity_score >= 3 ||
                   left > 3 || right > 3)
               {
                   st = BABY_RESTLESS;
               }
               else {
                   st = BABY_DEEP_SLEEP;
               }
            }
        }
    }

    // 출력
    if (out) {
        out->center     = center;
        out->left       = left;
        out->right      = right;
        out->total      = total;
        out->activity   = s_activity_score;
        out->side_sum   = left + right;
        out->side_bias  = right - left;
        out->state      = st;
    }
}

void Master_EvalEvents(const Slave1Frame_t *f1,
                       const BabyStatus_t *baby,
                       const Slave2Frame_t *f2)
{
    // 1) 심정지 후보: no_pulse=1 && BABY_EMPTY 아니라면
    if (f1 && baby && (baby->state != BABY_EMPTY) && f1->no_pulse) {
        if (s_no_pulse_cnt < 255) s_no_pulse_cnt++;
    } else {
        s_no_pulse_cnt = 0;
    }

    // 2) 위험 수면자세(뒤집힘): BABY_ROLLOVER
    if (baby && (baby->state == BABY_ROLLOVER)) {
        if (s_rollover_cnt < 255) s_rollover_cnt++;
    } else {
        s_rollover_cnt = 0;
    }

    // 3) 낙상 위험: 뒤집힌 상태 + 초음파 거리 매우 가까움
    if (baby && f2 &&
        (baby->state != BABY_EMPTY) &&
        (f2->dist1 < TH_FALL_NEAR || f2->dist2 < TH_FALL_NEAR))
    {
        if (s_fall_cnt < 255) s_fall_cnt++;
    } else {
        s_fall_cnt = 0;
    }

    // 4) 울음
    if (f2 && f2->cry_flag) {
        if (s_cry_cnt < 255) s_cry_cnt++;
    } else {
        s_cry_cnt = 0;
    }

    // --- 우선순위: 심정지 > 낙상 > 위험 자세 > 울음 > 없음 ---
    EventCode_t ev = EVENT_NONE;

    if (s_no_pulse_cnt >= TH_NO_PULSE_FRAMES) {
        ev = EVENT_CARDIAC_ARREST;
    } else if (s_fall_cnt >= TH_FALL_FRAMES) {
        ev = EVENT_FALL_RISK;
    } else if (s_rollover_cnt >= TH_ROLLOVER_FRAMES) {
        ev = EVENT_BAD_POSTURE;
    } else if (s_cry_cnt >= TH_CRY_FRAMES) {
        ev = EVENT_CRYING;
    } else {
        ev = EVENT_NONE;
    }

    g_event = ev;

    // --- 이벤트에 따라 Slave1/Slave2 커맨드 생성 ---
    uint8_t cmd_s1 = S1_CMD_LED_NORMAL;
    uint8_t cmd_s2 = S2_CMD_MOTOR_OFF;

    switch (ev) {
        case EVENT_CARDIAC_ARREST:
        case EVENT_FALL_RISK:
            cmd_s1 = S1_CMD_LED_EMERG | S1_CMD_BUZZER_ON;  // 긴급 LED + 부저
            cmd_s2 = S2_CMD_MOTOR_EMERG_STOP;              // 모터 완전 정지
            break;

        case EVENT_BAD_POSTURE:
            cmd_s1 = S1_CMD_LED_WARN;     // 경고 LED
            cmd_s2 = S2_CMD_MOTOR_OFF;
            break;

        case EVENT_CRYING:
            cmd_s1 = S1_CMD_LED_NORMAL;   // 평상시
            cmd_s2 = S2_CMD_MOTOR_SOOTHE; // 진정 모션
            break;

        case EVENT_NONE:
        default:
            cmd_s1 = S1_CMD_LED_NORMAL;
            cmd_s2 = S2_CMD_MOTOR_OFF;
            break;
    }

    g_cmd_s1 = cmd_s1;
    g_cmd_s2 = cmd_s2;
}

// GUI용 8바이트 패킷 전송
// [0] P1, [1] P2, [2] P3, [3] HR
// [4] HR_ABN, [5] FALL, [6] STAND, [7] CRYING
void GUI_SendEvent(const Slave1Frame_t *s1,
                   const BabyStatus_t *baby,
                   const Slave2Frame_t *s2)
{
    uint8_t pkt[8] = {0};

    // ---------- 1) 압력 + 심박 ----------
    if (s1 && s1->checksum_ok) {
        // FSR: 0~255 압축된 값 그대로 사용
        pkt[0] = s1->fsr1;   // P1
        pkt[1] = s1->fsr2;   // P2
        pkt[2] = s1->fsr3;   // P3

        // HR
        pkt[3] = s1->bpm;    // HR (0~240 가정)

        // HR 이상 여부 (심정지/고혈압/저혈압 등)
        if (s1->no_pulse || s1->high_bpm || s1->low_bpm) {
            pkt[4] = 1;      // HR_ABN = 1
        } else {
            pkt[4] = 0;
        }
    }

    // ---------- 2) 침대 위 자세 이상 (뒤집힘 등) ----------
    // BABY_ROLLOVER → 자세 이상(STAND 플래그)로 매핑
    if (baby) {
        if (baby->state == BABY_ROLLOVER) {
            pkt[6] = 1;      // STAND = 1 (자세 이상)
        }
    }

    // ---------- 3) 낙상 위험 / 울음 (Slave2) ----------
    if (s2 && s2->checksum_ok) {
        // 울음 플래그 그대로 사용
        pkt[7] = (s2->cry_flag ? 1u : 0u);     // CRYING

        // 간단 기준으로 낙상 / 자세 위험 판단 (튜닝 가능)
        // dist1: 침대 밖/옆쪽 센서 (낙상 감지용이라고 가정)
        // dist2: 침대 발치/난간 근처 센서 (자세/기어 나감 감지용이라고 가정)
        const uint8_t FALL_TH_CM  = 10;   // 너무 가까우면 위험 (초음파 cm 기준)
        const uint8_t STAND_TH_CM = 20;

        if (s2->dist1 > 0 && s2->dist1 < FALL_TH_CM) {
            pkt[5] = 1;      // FALL = 1 (낙상 위험)
        }

        // 자세 이상은 BABY_ROLLOVER 와 OR 처리
        if (s2->dist2 > 0 && s2->dist2 < STAND_TH_CM) {
            pkt[6] = 1;      // STAND = 1
        }
    }

    // ---------- 4) UART로 8바이트 전송 ----------
    // Python 쪽에서 정확히 8바이트씩 끊어 읽음
    HAL_UART_Transmit(&huart2, pkt, sizeof(pkt), 10);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin) // PC13
    {
        uint32_t curr = HAL_GetTick();
        // 200ms 디바운싱 (너무 빠르게 두 번 눌리는 것 방지)
        if (curr - g_last_btn_tick > 200)
        {
            g_sim_mode++;
            if (g_sim_mode > 2) {
                g_sim_mode = 0; // 다시 실제 모드로 복귀
            }
            g_last_btn_tick = curr;
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
