/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Beep_ON  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14, GPIO_PIN_SET)
#define Beep_OFF HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14, GPIO_PIN_RESET)
#define Power_out_ON  HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_SET)
#define Power_out_OFF HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15, GPIO_PIN_RESET)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG				1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim1_up;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile uint16_t Sec_count = 0;


volatile uint16_t ADC[3] = {0,}; // у нас два канала поэтому массив из двух элементов

#if (DEBUG)
char Uart_str[64] = {0,};
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void setup(void);
void setModeRXuart2(void);
void sendDataFromPanel(uint8_t);
void running_lights(void);

uint8_t get_selector_position(uint8_t, uint16_t);
void start_wash(uint8_t, uint8_t, uint8_t);
void stop_wash(uint8_t);
void led_off(void);
void led_on(void);
void beep(uint16_t);
void check_errors(uint8_t);
void status_wash_led(uint8_t);
void error_display(void);

#define ControlMasterlID 1      //  номер контрол. master управления в сети
#define ControlPanelID   2      //  номер контрол. панели управления в сети
#define ControlWIFIlID   3      //  номер контрол. WIFI панели управления в сети

#define _get_Num_Prog   get_selector_position(selectorProg,ADC[selectorProg])
#define _get_Speed_Spin get_selector_position(selectorSpeed, ADC[selectorSpeed])
#define _get_Temp_Wash  get_selector_position(selectorTemp,  ADC[selectorTemp])

// длина звука биппера
#define Power_ON 		200
#define Btton_press 	70
#define Wash_end		1650
#define Err_beep		100

#define ON			  	1
#define OFF			  	0

//массив данных для панели управления получаемый по uart2
#define ID            0
#define Start         1
#define ProgNum       2
#define Temp          3
#define Spin          4
#define Status        5
#define END           6
#define Reserve       7
#define WIFIS         8
#define TempC         9
#define TempS        10
#define WatLev       11
#define ProgNow      12
#define DoorS        13
#define ProgTot      14
#define SpeedH       15
#define SpeedL       16
#define PauseH       17
#define PauseL       18
#define CRCdata      19

#define lenArr        20

volatile uint8_t     buf_cmd_mass[lenArr] = {0,};
volatile uint8_t  Command_Massive[lenArr] ={
                                   0, // 0  для кого сообщение                                                                                                             <-->
                                   0, // 1  Старт стирка -1 Стоп стрика - 0                                                                                            В Контроллер
                                   0, // 2  Программа стирки от 0 до 16                                                                                                В Контроллер
                                   0, // 3  Температура стирки от  0 до 95                                                                                             В Контроллер
                                   0, // 4  Обороты отжима от 0 до 100 (нужно будет умножить на 10)                                                                    В Контроллер
                                   0, // 5  Состояние стирки: 0 - Конец. 1 - Предваритель стирка. 2 - Стиирка. 3 - Полоскание. 4 - Отжим. от контрол. мастер          Из Контроллера
                                   0, // 6  END от контроллера мастер . остановка или конец стирки 1 - конец  0 - работаем                                            Из Контроллера
                                   0, // 7  reserve
                                   0, // 8  OFF == 0 WIFI, WIFI ON == 1, Reset password == 2                                                                          Из Контроллера
                                   0, // 9  tempWater температура воды                                                                                                Из Контроллера
                                   0, // 10 tempHeaterSet заданная температура воды                                                                                   Из Контроллера
                                   0, // 11 waterLevel() уровень воды 0 1 2                                                                                           Из Контроллера
                                   0, // 12 NumProg  номер исполняемой программы стирки                                                                               Из Контроллера
                                   0, // 14 getStatusDoor()  состояние люка                                                                                           Из Контроллера
                                   0, // 15 TotalProgSD  кол-во программ стирки на SD карте в каталоге STEP/                                                          Из Контроллера
                                   0, // 16 getValueArr(CurrentSpeedCell) >> 8;  int Сохраняем старший байт значения оборотов в  1 ячейку массив                      Из Контроллера
                                   0, // 17 getValueArr(CurrentSpeedCell);  Сохраняем младший байт значения оборотов во 2 ячейку массив                               Из Контроллера
                                   0, // 18 CountSecPause >> 8;   int Сохраняем старший байт значения паузы в  1 ячейку массив                                        Из Контроллера
                                   0, // 19 CountSecPause ;    Сохраняем младший байт значения паузы во 2 ячейку массив                                               Из Контроллера
                                   0  // 20 CRC                                                                                                                       <-->
                             };

#define selectorProg     	0
#define selectorSpeed    	1
#define selectorTemp     	2

#define totall_Prog 16
const uint16_t selectorProg_values [totall_Prog] =
                                        {150,350,600,900,1200,1500,1710,2000,2320,2666,2850,3099,3469,3747,4000,4500}; 	// положение регулятора считывание с аналог. входа
                                      //  {1,  2,  3,  4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15,  16}  	//  программы стирки
#define tempSpeedNum 8
const uint16_t selectorTempSpeed_values[3] [8] = {
                                                {400,1000,2000,2500,2900,3300,3800,4500}, 	// положение регулятора считывание с аналог. входа
                                                { 70, 80,  90,  100,   0,  40,  50,  60}, 	// обороты отжима
                                                {  0, 30,  40,   50,  60,  70,  80,  90}  	// температура стирки
                                                };

#define led_0 0x4018
#define led_1 0x2018
#define led_2 0x1018
#define led_3 0x4028
#define led_4 0x2028
#define led_5 0x1028
#define led_6 0x4030
#define led_7 0x2030
#define led_8 0x1030

#define led_power 8

/*										0		1		2		3		4		5		6		7		8	*/
const uint16_t Led_array_ON[10] = {0x4018, 0x2018, 0x1018, 0x4028, 0x2028, 0x1028, 0x4030, 0x2030, 0x1030, 0x0000 };
/*							0		1		2		3		4		5		6		7		8	*/
uint16_t Led_array[10] = {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };//{GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14};

volatile uint16_t Pin_button;
volatile uint8_t Flag_power_ON;
volatile uint8_t Flag_radio_ON;
volatile uint8_t Flag_start_wash;
uint8_t Flag_wash;
uint8_t Step_wash_session;

#define limit_Sec_ses_conn  50
volatile uint16_t Sec_count_sesion_connect;
volatile uint16_t Sec_delay_start_wash;
volatile uint8_t Count_mode_delay_wash; // 0 - off  0 < on
//									0h  2h 		4h 		6h 	  9h
const uint16_t Time_Delay_Arr[5] = {0, 05, 14400, 21600, 36000}; //0h 2h 4h 6h 9h
#define Tot_arr_err  16
uint8_t Error_Arr[Tot_arr_err] = {0,};																	// массив с ошибками полученный от главного контроллера

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
  MX_RTC_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, "\nI'm panel.\n\0", 12, 1000);
  setup();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
 // Beep_ON;
 // check_errors(200);check_errors(201); check_errors(202); check_errors(203);

  while (1)
  {

	         HAL_Delay(500);
/*	         Led_array[0] = led_0;
	         Led_array[1] = led_1;
	         Led_array[2] = led_2;
	         Led_array[7] = led_7;
	         Led_array[3] = led_3;
	         Led_array[4] = led_4;
	         Led_array[5] = led_5;
	         Led_array[6] = led_6;
	         Led_array[7] = led_7;
	         Led_array[8] = led_8;
 */
	                 //snprintf(Uart_str, 63, "ADC %d    %d    %d\n", (uint16_t)adc[0], (uint16_t)ADC[1], (uint16_t)adc[2]);
	                 //HAL_UART_Transmit(&huart1, (uint8_t*)Uart_str, strlen(Uart_str), 1000);


	                		 	 	 	 	 	 	 	 	 	 	 //get_selector_position(selectorSpeed, ADC[selectorSpeed]);
	         snprintf(Uart_str, 63, "Prog %d (%d) Speed  %d (%d) Temper  %d (%d)\r\n\0", get_selector_position(selectorProg,  ADC[selectorProg]),  ADC[selectorProg],
	         	                		 	 	 	 	 	 	 	 	 	 	      get_selector_position(selectorSpeed, ADC[selectorSpeed]), ADC[selectorSpeed],
	         																	      get_selector_position(selectorTemp,  ADC[selectorTemp]),  ADC[selectorTemp]);
	         //HAL_UART_Transmit(&huart1, (uint8_t*)Uart_str, strlen(Uart_str), 1000);

	         //snprintf(Uart_str, 63, "Radio %d Mashine %d Start %d Prog %d  Speed  %d  Temper  %d \n", Flag_radio_ON, Flag_power_ON, Command_Massive[Start], Command_Massive[ProgNum], Command_Massive[Spin], Command_Massive[Temp]);

	         //snprintf(Uart_str, 63, "Delay %d Flag_start_wash %d Status %d step %d\r\n\0",  Sec_delay_start_wash, Flag_start_wash, buf_cmd_mass[Status], Step_wash_session);
	         snprintf(Uart_str, 63, "cmdA[Start]: %d buf[Start]: %d Sess: %d buf[END]: %d\r\n\0",  Command_Massive[Start], buf_cmd_mass[Start], Sec_count_sesion_connect, buf_cmd_mass[END]);
	         HAL_UART_Transmit(&huart1, (uint8_t*)Uart_str, strlen(Uart_str), 1000);

	                if (!Flag_power_ON && !Flag_radio_ON)  // если питание выкл и радио выкл гасим индикаторы отключаем стирку
	                {
	                	stop_wash(0);
	                	Flag_start_wash = Flag_wash = Step_wash_session = buf_cmd_mass[Status] = buf_cmd_mass[Start] = 0; status_wash_led(0);
	                	led_off();
	                	HAL_Delay(500);
	                	Power_out_OFF;
	                	Sec_count_sesion_connect = 0;
	                }
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	                if (!Flag_power_ON && Flag_radio_ON)   // включено радио
	                {
	                	Power_out_OFF;
	                }
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------/
	                if (Flag_power_ON) 						// режим стирка
	                {
	                   if (Flag_Wash == ON) {start_wash(_get_Num_Prog, _get_Speed_Spin, _get_Temp_Wash); Flag_Wash = OFF;} 								// если включено питание  запуск стирки
	                   if (Flag_Wash == OFF && Flag_start_wash == OFF) stop_wash(OFF);						// остановка стирки
	                   if ((Sec_delay_start_wash == 0 && Flag_start_wash == ON) ) {
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   status_wash_led(OFF);	// гасим индикаторы статуса стирки - отсрочки
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   Led_array[3] = OFF;      // индикатор кнопки таймера отсрочки стирки
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   Flag_Wash = ON;}         // запуск стирки по таймеру отсрочки
	                   if (Flag_start_wash == ON) status_wash_led(buf_cmd_mass[Status]); 					// выдаем статус на панель если режим ON



	                }
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
	                if (Flag_power_ON) 						// режим стирка
	                {
	                   Power_out_ON;						// ВКЛ питание на контрол. машины
	                   if(Sec_count_sesion_connect == 10)setModeRXuart2();
	                   uint16_t sum_err = 0;
	                   for(uint8_t i = 0; i < Tot_arr_err; i++) sum_err += Error_Arr[i];
	                   if (sum_err > 1 && Flag_start_wash == OFF) error_display(); // если есть ощибки в памяти то выводим их на панель

	                   if (Sec_delay_start_wash == 0 && Flag_start_wash == ON && Flag_wash == 0 && Step_wash_session < 4)
	                   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	  {
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   status_wash_led(OFF);	// гасим индикаторы статуса стирки - отсрочки
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   Led_array[3] = OFF;      // индикатор кнопки таймера отсрочки стирки
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   Flag_wash = 1;
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	// супер стирка доп. прог                             здесь номер программы
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 if ( Led_array[6] && Step_wash_session == 0) {Step_wash_session = 1; start_wash(_get_Num_Prog, _get_Speed_Spin, _get_Temp_Wash); goto Exit;}
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 if (!Led_array[6] && Step_wash_session == 0)  Step_wash_session = 1;
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 	 	 	 	 	   // быстрая стирка доп. прог                            здесь номер программы
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 if ( Led_array[5] && Step_wash_session == 1) {Step_wash_session = 2; start_wash(_get_Num_Prog, _get_Speed_Spin, _get_Temp_Wash); goto Exit;}
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 if (!Led_array[5] && Step_wash_session == 1)  Step_wash_session = 2;
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 	 	 	 	 	 	 // обязательная стирка выбранная на селекторе
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 if (Step_wash_session == 2) {Step_wash_session = 3; start_wash(_get_Num_Prog, _get_Speed_Spin, _get_Temp_Wash);
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 	 	 	 	 if (!Led_array[4]) Step_wash_session = 4;
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 	 	 	 	 goto Exit;}
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 	 	 	 	 	 	 // дополнит полоскание доп. прог                   здесь номер программы
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 if ( Led_array[4] && Step_wash_session == 3) {Step_wash_session = 4; start_wash(_get_Num_Prog, _get_Speed_Spin, _get_Temp_Wash);}
	                	   	   	   	   	   	   	   	   	   	   	   	   	   	   		 if (!Led_array[4] && Step_wash_session == 3)  Step_wash_session = 4;


	                   	   	   	   	   	   	   	   	   	   	   	   	   	   	   	   }
	                   Exit:
	                   if (Flag_start_wash == ON) {status_wash_led(0); status_wash_led(buf_cmd_mass[Status]);} 					// выдаем статус на панель если режим ON
	                   if (Flag_wash == ON && Flag_start_wash == OFF) {Step_wash_session = 0; stop_wash(OFF);	}					// остановка стирки по кнопке


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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7200;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 22;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7200;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim4.Init.Period = 50;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
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
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA8 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB3
                           PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_CAN1_2();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void HAL_RTCEx_RTCEventCallback (RTC_HandleTypeDef *hrtc)				// прерывание 1 раз в секунду
{
 if(RTC_IT_SEC)
 {
	 Sec_count++;
	 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

	 if(Sec_count %2 == 0 && Flag_power_ON && !Flag_start_wash) 											// если только вкл питания
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 {
		 	 	 	 	 	 	 	 	 	 	 	 	 	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&ADC, 3); // стартуем АЦП  для преобразования
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	  return;
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 }
	 if(Sec_delay_start_wash != 0 && Flag_start_wash == ON) Sec_delay_start_wash--;								// если таймер включен и флаг старта стирки уменьшаем его на 1
	 HAL_ADC_Stop_DMA(&hadc1); 																				// стоп АЦП
	 if(!Flag_power_ON)  Led_array[led_power] = 0; else Sec_count_sesion_connect++;									// если флаг выкл питание гасим индикатор, а если ВКЛ счет сек между сенсами связи
	 if(Flag_power_ON && Flag_start_wash) Led_array[led_power] = led_8;												// если есть питание и старт стирки включаем индикатор
	 check_errors(0);																						// если от мастера пришли ошибки узлов то их обрабатываем \ выводим
	 //status_wash_led(buf_cmd_mass[Status]);
 }
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void setup(void)
{
	  Power_out_OFF;
	  setModeRXuart2();													// настройка послед порта 2 на прием

	  HAL_ADCEx_Calibration_Start(&hadc1);  							// калибровкa АЦП
	  //HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc, 3); 					// стартуем АЦП
	  //HAL_ADC_Stop_DMA(&hadc1); 										// стоп АЦП
	  HAL_RTCEx_SetSecond_IT(&hrtc);									// старт RTC прерывание 1 сек
	  HAL_DMA_Start_IT(htim1.hdma[TIM_DMA_ID_UPDATE], (uint32_t)Led_array, (uint32_t)&GPIOB->ODR, 9);  // запуск таймера 1 и ДМА для вывода   LED индикации
	  __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
	  __HAL_TIM_ENABLE(&htim1);

	  HAL_TIM_Base_Stop_IT(&htim4); 									// останавливаем таймер
	  HAL_TIM_Base_Start_IT(&htim2);
	  Flag_power_ON 	= 0;											// флаг панели управления Вкл Выкл ее
	  Flag_radio_ON 	= 0;											// флаг рвдио вкл выкл
	  Flag_start_wash 	= 0;											// флаг старта стирки от сост кнопки
	  Flag_wash 		= 0;    //<<<<<<<<<<<<
	  Step_wash_session = 0;
	  Sec_count_sesion_connect = 0;										// счетчик паузы между сенсами связи с мастером
	  Sec_delay_start_wash     = 0;										// таймер отсрочки старта стирки
	  Count_mode_delay_wash    = 0;
	  led_on();															// тест индикаторов
	  beep(Power_ON);													// бип при включении контроллера
	  HAL_Delay(500);													// задержка что бы увидеть индикаторы
	  led_off();														// гасим индикаторы
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) 				// По окончании отправки  в huart2 сработает прерывание и вызовет колбек
{
          if(huart == &huart2)
          { 															// можно установить какой-то флаг, сообщающий об окончании отправки
        	  setModeRXuart2();                                         // переходим в режим чтения порта
          }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)					// По окончании приема  в huart2 сработает прерывание и вызовет колбек
{
          if(huart == &huart2)
		  {																// что-то делаем -- возможно тут проверка CRC если не понадобится то избавимся от буф. массива
        	  uint8_t crc_l =0;
        	  for(uint8_t i =0; i<20-1; i++) crc_l ^= buf_cmd_mass[i];
        	  if (buf_cmd_mass[ID] == ControlPanelID && (20^crc_l+2 == buf_cmd_mass[CRCdata]))// если от мастера пришло сообщение и CRC совпадает
        	  {
        		  if (buf_cmd_mass[END] == 1) stop_wash(0);				// если 1 то пришел флаг конца стирки от мастера, значит переходим в режим конца стирки
        		  Sec_count_sesion_connect = 0;							// сброс счетчика контроля связи между платой панели и мастером
        		  check_errors(buf_cmd_mass[Status]);					// если от мастера пришли ошибки узлов то их обрабатываем \ выводим
        		  //status_wash_led(buf_cmd_mass[Status]);
        		  sendDataFromPanel(ControlMasterlID);					// отвечаем мастеру
        	  }
          }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)    				// колбек ошибки при приёме/отправке
{
#if(DEBUG)
	if(huart == &huart2)
        {																//  ошибки
        	uint32_t er = HAL_UART_GetError(&huart2);
            switch(er)
                {
                 case HAL_UART_ERROR_PE:
                 HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Uart - Parity error\n", 27, 1000);
                 __HAL_UART_CLEAR_PEFLAG(huart);
                 huart->ErrorCode = HAL_UART_ERROR_NONE;
                 break;
                 case HAL_UART_ERROR_NE:
                 HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Uart - Noise error\n", 26, 1000);
                 __HAL_UART_CLEAR_NEFLAG(huart);
                 huart->ErrorCode = HAL_UART_ERROR_NONE;
                 break;
                 case HAL_UART_ERROR_FE:
                 HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Uart - Frame error\n", 26, 1000);
                 __HAL_UART_CLEAR_FEFLAG(huart);
                 huart->ErrorCode = HAL_UART_ERROR_NONE;
                 break;
                 case HAL_UART_ERROR_ORE:
                 HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Uart - Overrun error\n", 28, 1000);
                 __HAL_UART_CLEAR_OREFLAG(huart);
                 huart->ErrorCode = HAL_UART_ERROR_NONE;
                 break;
                 case HAL_UART_ERROR_DMA:
                 HAL_UART_Transmit(&huart1, (uint8_t*)"ERR_Uart - DMA transfer error\n", 33, 1000);
                 huart->ErrorCode = HAL_UART_ERROR_NONE;
                 break;
                 default:
                 break;
                }
#endif
                __HAL_UART_FLUSH_DRREGISTER(huart);
                __HAL_UART_CLEAR_OREFLAG(huart);
                __HAL_UART_CLEAR_NEFLAG(huart);
                __HAL_UART_CLEAR_FEFLAG(huart);
                /* Disable the UART Error Interrupt: (Frame error, noise error, overrun error) */
                //__HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
                setModeRXuart2();													// настройка послед порта 2 на прием
        }
}
void setModeRXuart2(void)
{
	HAL_HalfDuplex_EnableReceiver(&huart2); 							//  перевод устройства в режиме приёма
	HAL_UART_Receive_IT(&huart2, (uint8_t*)buf_cmd_mass, lenArr);
}
void sendDataFromPanel(uint8_t idPanel)
{
	HAL_HalfDuplex_EnableTransmitter(&huart2); 							// перевод устройство в режиме отправки
	Command_Massive[ID] = idPanel;
	uint8_t crc_l = 0;
	for(uint8_t i = 0; i<20-1; i++) crc_l ^= Command_Massive[i];		// подсчет CRC
	Command_Massive[CRCdata]	= crc_l^20+2;
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)Command_Massive, lenArr);	// отпрпавляем данные в шину
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)							// прерывание от кнопок
{
	if(TIM4->CR1 == 1) return;											// если таймер 4 запущен (1) то выходим из ф-ции
	//HAL_NVIC_DisableIRQ(EXTI15_10_IRQn); 								// отключаем прерывания на этом пине
    Pin_button = GPIO_Pin;												// запом. номер пина кнопки
    HAL_TIM_Base_Start_IT(&htim4); 										// запускаем таймер 4
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)				// колбэк таймера 2, 4
{
	// 	 sprintf(StringCMD, "\nAutoRun: %d\r\n%c", AutoStart, '\0');
    //	HAL_UART_Transmit(&huart1, (uint8_t*)StringCMD, strlen(StringCMD), 30);
	 if(htim->Instance == TIM2)											// если прерывание от тамера 2
	  {
		 beep(0);
		 if (Flag_power_ON && !Flag_start_wash)
		 {
			 Led_array[led_power] ^= led_8; 						// мигаем индикатором
			 //running_lights();										// бегущие огни статуса индикаторов - шутиха
		}
		 if (Flag_power_ON && Flag_start_wash) Led_array[led_power] = led_8; 						// зажигаем индикатором
		 if (Flag_power_ON && Sec_delay_start_wash != 0) Led_array[3] ^= led_3;  // мигаем если таймер отсрочки включен
	}





     if(htim->Instance == TIM4 && HAL_GPIO_ReadPin(GPIOA, Pin_button) == 0) // если прерывание от тамера 4 и кнопка нажата
        {
        	static uint8_t count_press_bt;								// счетчик времени нажатия кнопки
        	count_press_bt++;											// инкримент его
        	if (count_press_bt > 30)									// если больше 30
            {
				switch (Pin_button)
				{
				case GPIO_PIN_4: //BT_3 Таймер отсрочки стирки
					if (Flag_power_ON || Flag_radio_ON)
														{
														 Led_array[3] = led_3;
														 Count_mode_delay_wash++;
														 if (Count_mode_delay_wash > 4) {
															 	 	 	 	 	 	 	 Count_mode_delay_wash = 0;
															 	 	 	 	 	 	 	 Led_array[3] = 0;
														 	 	 	 	 	 	 	 	 }
														 status_wash_led(0);
														 status_wash_led(Count_mode_delay_wash);
														 Sec_delay_start_wash = Time_Delay_Arr[Count_mode_delay_wash];
														} else {
															     Led_array[3] ^= Led_array[3];
																}
					count_press_bt = 0;
					break;

				case GPIO_PIN_5:  //BT_4  Супер стирка
					if (Flag_power_ON || Flag_radio_ON)Led_array[6] ^= led_6; else Led_array[6] ^= Led_array[6];
					count_press_bt = 0;
					break;

				case GPIO_PIN_8:  //BT_2 Быстрая Стирка
					if (Flag_power_ON || Flag_radio_ON)Led_array[5] ^= led_5; else Led_array[5] ^= Led_array[5];
					count_press_bt = 0;
					break;

				case GPIO_PIN_11: //BT_1 Дополнительное полоскание
					if (Flag_power_ON || Flag_radio_ON)Led_array[4] ^= led_4; else Led_array[4] ^= Led_array[4];
					count_press_bt = 0;
					break;

				case GPIO_PIN_15: // Button Start / Stop
					if (Flag_power_ON)
									{
									 Flag_start_wash ^= 1;
									}else{
										Led_array[led_power] ^= Led_array[led_power];  // led off
									}

					if (!Flag_power_ON) Flag_radio_ON ^= 1;								// radio ON/OFF
					count_press_bt = 0;
					break;

				case GPIO_PIN_12:                  // Button POWER
					if(!Flag_radio_ON)
					{
					 Led_array[led_power]  ^= led_8;
					 Flag_power_ON ^= count_press_bt;
					 //if (Flag_power_ON)Power_out_ON; else {Power_out_OFF;	led_off();}		// ВКЛ / Выкл питане мастера путем открытия закрытия транзистора
					}
					beep(70);
					count_press_bt = 0;
					break;

				default:
					count_press_bt = 0;
					break;
				}
				if (Flag_power_ON) beep(Btton_press);
            }
        	if (count_press_bt == 0) HAL_TIM_Base_Stop_IT(&htim4); 			// останавливаем таймер


        	//HAL_TIM_Base_Stop_IT(&htim4); 			// останавливаем таймер
             // __HAL_GPIO_EXTI_CLEAR_IT(Pin_button);   // очищаем бит EXTI_PR (бит прерывания)
            //  NVIC_ClearPendingIRQ(EXTI15_10_IRQn);   // очищаем бит NVIC_ICPRx (бит очереди)
            //  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);     // включаем внешнее прерывание
        }
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
uint8_t get_selector_position(uint8_t selector, uint16_t value)         // преобразов значен АЦП к номмеру программы стирки, темпер, скорости
{
 if (selector == selectorProg)
  {
   for (uint8_t i=0; i < totall_Prog; i++)
     if (value < selectorProg_values[i])
    	 	 	 	 	 	 	 	 	   return i+1;
  }


  for (uint8_t i=0; i < tempSpeedNum; i++)
  {
    if (value < selectorTempSpeed_values[0][i])
    										return selectorTempSpeed_values[selector][i];
   }

 return 0;
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void start_wash(uint8_t prog_num, uint8_t spin_set, uint8_t temp_set) //
{
		Command_Massive[Start]   = 1;
		Command_Massive[ProgNum] = prog_num;//get_selector_position(selectorProg,  ADC[selectorProg]);
		Command_Massive[Spin] 	 = spin_set;//get_selector_position(selectorSpeed, ADC[selectorSpeed]);
		Command_Massive[Temp] 	 = temp_set;//get_selector_position(selectorTemp,  ADC[selectorTemp]);
		//Command_Massive[END]     = 0;
		Command_Massive[ID] 	 = ControlMasterlID;
		//Flag_start_wash			 = 1;
		Count_mode_delay_wash = 0;
		//led_off();

}
void stop_wash(uint8_t mode) // 1 run 0 stop
{
	    Command_Massive[Start]   = 0;
		Flag_start_wash			 = 0;
		Flag_wash 				 = 0;
		Count_mode_delay_wash	 = 0;
		if (Step_wash_session > 3) {
									Step_wash_session = 0; beep(Wash_end);}
		if (Step_wash_session != 0 && Step_wash_session < 4) Flag_start_wash	= 1;
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void led_off(void)														// индикаторы  гасим
{
	for(uint8_t i = 0; i < 9; i++) Led_array[i] = 0;
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void led_on(void)														// индикаторы  все зажигаем например для теста индикаторов
{
	for(uint8_t i = 0; i < 9; i++) Led_array[i] = Led_array_ON[i];

}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void beep(uint16_t hold)												// вывод звука на активный излучатель, просто поднимаем ногу на опред время
{
static uint32_t holdTime, lastTime;
	    if (hold > 0)
	        {
	    	  Beep_ON;
	          holdTime = hold;
	          lastTime = HAL_GetTick();
	          }

	    if ((lastTime + holdTime) < HAL_GetTick())
	        {
	          Beep_OFF;
	          }
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void check_errors(uint8_t var)
{
	uint8_t Error = 0;
	if (Sec_count_sesion_connect > limit_Sec_ses_conn) Error = 1; 		//  нет связи с мастером
	if (Error) stop_wash(OFF);											//  если есть ошибки останавливаем стирку
	if (var < 200) return;						  						//  если меньше 200 значит нет ошибок у мастера

	for (uint8_t i=0; i < Tot_arr_err; i++)
	{
	 if(Error_Arr[i] == var - 199)	break;							// если код ошибки уже есть в массиве, то выходим
	 if(Error_Arr[i] == 0) {Error_Arr[i] = var - 199; break;}
	}

}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void status_wash_led(uint8_t status)									// зажигаем индикаторы статуса стирки. Всего 4 статуса у нас на панеле
{
	//Led_array[0] = Led_array[1] = Led_array[2] = Led_array[7] = 0;
	switch (status)
	{
	case 0:
		Led_array[0] = Led_array[1] = Led_array[2] = Led_array[7] = 0;
		break;
	case 1:
		Led_array[0] = led_7;
		break;
	case 2:
		Led_array[1] = led_2;
		break;
	case 3:
		Led_array[2] = led_1;
		break;
	case 4:
		Led_array[7] = led_0;
		break;

	}
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void running_lights()
{
	static uint8_t p;
	p++;
	if (p > 40) {p=0; }

	switch (p)
	{
		case 10:
		Led_array[0] ^= led_0;
		break;
		case 20:
		Led_array[1] ^= led_1;
		break;
		case 30:
		Led_array[2] ^= led_2;
		break;
		case 40:
		Led_array[7] ^= led_7;
		break;
	}
}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
void error_display()		//  процедура декодирования ошибки и вывод ее не панель в двоичном виде
{
	uint16_t i = 0;
	status_wash_led(0);		// тушим индикаторы
	while (i < 4)
	{
	 beep(Err_beep);		// зуммер для превлесения внимания
	 HAL_Delay(170);
	 i++;
	}
	i = 0;
	while (i < 500)
	{
	  running_lights();		// бегущие огни для превлесения внимания
	  HAL_Delay(5);
	  i++;
	}
	i = 0;
	status_wash_led(0);		// тушим индикаторы
	while (i < Tot_arr_err)
	{
		if (Error_Arr[i] !=0 )		// если ячейка имеет ошибку то индицируем
		{
			status_wash_led(0);
			HAL_Delay(1000);
			beep(Err_beep);
			if(Error_Arr[i]  & 1) status_wash_led(1);	// AND 0x00000001;
			if(Error_Arr[i]  & 2) status_wash_led(2);	// AND 0x00000010;
			if(Error_Arr[i]  & 4) status_wash_led(3);	// AND 0x00000100;
			if(Error_Arr[i]  & 8) status_wash_led(4);	// AND 0x00001000;
			HAL_Delay(1000*3);
		}
	 i++;
	}

}
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
