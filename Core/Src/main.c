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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../Library/Pressure/bmp280.h"
#include "../Library/Acceleration/mpu6050.h"
#include "../Library/GPS/gps.h"
#include "../Library/Communication/lora_e22.h"
#include "../Library/Communication/telemetry_packet.h"
#include "../Library/Servo/servo.h"
#include "../Library/TeleCommand/telecommand.h"
#include "../Library/Utility/utiltity.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 *  Event Group Flags
 */
#define EVENT_FLAG_LORA_WRITE_READY    			(1 << 0U)
#define EVENT_FLAG_LORA_READ_READY     			(1 << 1U)
#define EVENT_FLAG_GPS_DATA_PROCECSS_READY      (1 << 2U)
#define EVENT_FLAG_TELECOMMAND_READY         	(1 << 3U)
#define EVENT_FLAG_TELECOMMAND_FINISHED       	(1 << 4U)

#define EVENT_FLAG_SERVO_PROCESS_READY        	(1 << 5U)
#define EVENT_FLAG_SENSOR_PROCESS_READY      	(1 << 6U)
#define EVENT_FLAG_SD_WRITE_READY         		(1 << 7U)
#define EVENT_FLAG_CALIBRATION_DONE             (1 << 8U)
#define EVENT_FLAG_SEPERATION_STARTED         	(1 << 9U)
#define EVENT_FLAG_SEPERATION_COMPLETED        	(1 << 10U)
#define EVENT_FLAG_FIRST_SEPERATION				(1 << 11U)

#define EVENT_FLAG_ALL_READY              		(0xFFFFFFFF)
#define EVENT_FLAG_NONE               			(0x00000000)
#define EVENT_FLAG_ERROR              			(1 << 31U)
#define EVENT_FLAG_TIMEOUT            			(1 << 30U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Definitions for startingTask */
osThreadId_t startingTaskHandle;
const osThreadAttr_t startingTask_attributes = {
  .name = "startingTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for bmpProcessTask */
osThreadId_t bmpProcessTaskHandle;
const osThreadAttr_t bmpProcessTask_attributes = {
  .name = "bmpProcessTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow5,
};
/* Definitions for mpuProcessTask */
osThreadId_t mpuProcessTaskHandle;
const osThreadAttr_t mpuProcessTask_attributes = {
  .name = "mpuProcessTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow6,
};
/* Definitions for sdWriteTask */
osThreadId_t sdWriteTaskHandle;
const osThreadAttr_t sdWriteTask_attributes = {
  .name = "sdWriteTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal5,
};
/* Definitions for servoTask */
osThreadId_t servoTaskHandle;
const osThreadAttr_t servoTask_attributes = {
  .name = "servoTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for telecommandTask */
osThreadId_t telecommandTaskHandle;
const osThreadAttr_t telecommandTask_attributes = {
  .name = "telecommandTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for loraTask */
osThreadId_t loraTaskHandle;
const osThreadAttr_t loraTask_attributes = {
  .name = "loraTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh6,
};
/* Definitions for gpsProcess */
osThreadId_t gpsProcessHandle;
const osThreadAttr_t gpsProcess_attributes = {
  .name = "gpsProcess",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for seperationState */
osThreadId_t seperationStateHandle;
const osThreadAttr_t seperationState_attributes = {
  .name = "seperationState",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh6,
};
/* Definitions for loraParse */
osThreadId_t loraParseHandle;
const osThreadAttr_t loraParse_attributes = {
  .name = "loraParse",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for telecommandqueue */
osMessageQueueId_t telecommandqueueHandle;
const osMessageQueueAttr_t telecommandqueue_attributes = {
  .name = "telecommandqueue"
};
/* Definitions for uart2incomingbuffer */
osMessageQueueId_t uart2incomingbufferHandle;
const osMessageQueueAttr_t uart2incomingbuffer_attributes = {
  .name = "uart2incomingbuffer"
};
/* Definitions for uart2outcomequeue */
osMessageQueueId_t uart2outcomequeueHandle;
const osMessageQueueAttr_t uart2outcomequeue_attributes = {
  .name = "uart2outcomequeue"
};
/* Definitions for gpsBufferQueue */
osMessageQueueId_t gpsBufferQueueHandle;
const osMessageQueueAttr_t gpsBufferQueue_attributes = {
  .name = "gpsBufferQueue"
};
/* Definitions for loraWriteTimer */
osTimerId_t loraWriteTimerHandle;
const osTimerAttr_t loraWriteTimer_attributes = {
  .name = "loraWriteTimer"
};
/* Definitions for velocityCalcTimer */
osTimerId_t velocityCalcTimerHandle;
const osTimerAttr_t velocityCalcTimer_attributes = {
  .name = "velocityCalcTimer"
};
/* Definitions for seperationTimer */
osTimerId_t separationTimerHandle;
const osTimerAttr_t separationTimer_attributes = {
  .name = "separationTimer"
};
/* Definitions for sensorProcessMutex */
osMutexId_t sensorProcessMutexHandle;
const osMutexAttr_t sensorProcessMutex_attributes = {
  .name = "sensorProcessMutex"
};
/* Definitions for loraCommunicationMutex */
osMutexId_t loraCommunicationMutexHandle;
const osMutexAttr_t loraCommunicationMutex_attributes = {
  .name = "loraCommunicationMutex"
};
/* Definitions for systemEvents */
osEventFlagsId_t systemEventsHandle;
const osEventFlagsAttr_t systemEvents_attributes = {
  .name = "systemEvents"
};
/* USER CODE BEGIN PV */

GeneralControl_TypeDefInit controlStruct = {0};
BMP280_HandleTypedef bmp280;

//uint8_t pData[6] = {0xC0, 0x00, 0x03, 0x12, 0x34, 0x62};		//"selam1";
//uint8_t pDataRec[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM7_Init(void);
void StartDefaultTask(void *argument);
void BMPProcessTaskCallback(void *argument);
void MPUProcessTaskCallback(void *argument);
void SDWriteTaskCallback(void *argument);
void ServoTaskCallback(void *argument);
void TelecommandTask(void *argument);
void LoraTaskCallback(void *argument);
void GPSProcessCallback(void *argument);
void LoraParseCallback(void *argument);
void LoraWriteTimerCallBack(void *argument);
void VelocityCalculateTimerCallback(void *argument);
void SeparationTimerCallback(void *argument);
void SeperationStateCallback (void *argument);

/* USER CODE BEGIN PFP */

void BMP280Process(BMP280_TypeDefInit *bmpTypeDef);
void MPU6050Process(MPU6050_TypeDefInit *mpu);

void LoraWriteProcess(GeneralControl_TypeDefInit  *controlStruct);
void LoraReadProcess(uint8_t *rx_buffer, size_t buffer_size);

void GPSProcess(GeneralControl_TypeDefInit *controlStruct, char  *gps_buffer, size_t buffer_size);

void CalculateVelocityCallback(GeneralControl_TypeDefInit  *controlStruct);
void TelecommandProcess(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == KAAN_BUTTON_Pin) {
		// Butona basıldığında yapılacak işlemler
		controlStruct.button.button_state = HAL_GPIO_ReadPin(KAAN_BUTTON_GPIO_Port, KAAN_BUTTON_Pin);

		if (controlStruct.button.button_state == GPIO_PIN_SET && controlStruct.button.button_last_state == GPIO_PIN_RESET) {
			// Buton serbest bırakıldıysa
			controlStruct.button.separation_state = 1; // Kilitli
			osEventFlagsSet(systemEventsHandle, EVENT_FLAG_SEPERATION_STARTED);
			//osTimerStart(separationTimerHandle, 2000); // 2 saniye sonra ayrılma tamamlandı olarak işaretle
		}

		controlStruct.button.button_last_state = controlStruct.button.button_state;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if(huart == &huart6) {

        if(controlStruct.gps.rx_data != '\n' && controlStruct.gps.rx_index < sizeof(controlStruct.gps.rx_buffer) - 1) {
            controlStruct.gps.rx_buffer[controlStruct.gps.rx_index++] = controlStruct.gps.rx_data;
        } else {

        	controlStruct.gps.rx_buffer[controlStruct.gps.rx_index] = '\0';
        	osMessageQueuePut(gpsBufferQueueHandle, controlStruct.gps.rx_buffer, 0, 0);
        	osEventFlagsSet(systemEventsHandle, EVENT_FLAG_GPS_DATA_PROCECSS_READY);

            controlStruct.gps.rx_index = 0;
            controlStruct.gps.rx_data = 0;
        }
        HAL_UART_Receive_IT(&huart6, &controlStruct.gps.rx_data, 1);

    }
    if (huart == &huart2) {
    	if(controlStruct.uartData.rx_data != '\n' && controlStruct.uartData.rx_index < sizeof(controlStruct.uartData.rx_buffer) - 1)
			controlStruct.uartData.rx_buffer[controlStruct.uartData.rx_index++] = controlStruct.uartData.rx_data;
    	 else {

    		controlStruct.uartData.rx_buffer[controlStruct.uartData.rx_index] = '\0';
    		osMessageQueuePut(uart2incomingbufferHandle, controlStruct.uartData.rx_buffer, 0, 0);
			osEventFlagsSet(systemEventsHandle, EVENT_FLAG_LORA_READ_READY);

			controlStruct.uartData.rx_index = 0;
			controlStruct.uartData.rx_data = 0;
    	}
    	  HAL_UART_Receive_IT(&huart2,&controlStruct.uartData.rx_data, 1);
    }
}

void BMP280Process(BMP280_TypeDefInit *bmpTypeDef)
{
	osMutexAcquire(sensorProcessMutexHandle, osWaitForever);
	BME280_Calculate_Data(bmpTypeDef);
	osMutexRelease(sensorProcessMutexHandle);
}

void MPU6050Process(MPU6050_TypeDefInit *mpu)
{
	osMutexAcquire(sensorProcessMutexHandle, osWaitForever);
	mpu->AccX = MPU6050_Kalman_Accel_X();
	mpu->AccY = MPU6050_Kalman_Accel_Y();
	mpu->AccZ = MPU6050_Kalman_Accel_Z();
	mpu->GyroX = MPU6050_Kalman_Gyro_X();
	mpu->GyroY = MPU6050_Kalman_Gyro_Y();
	mpu->GyroZ = MPU6050_Kalman_Gyro_Z();
	mpu->RollAngle = MPU6050_Read_Roll_Angle();
	mpu->PitchAngle = MPU6050_Read_Pitch_Angle();
	mpu->YawAngle = MPU6050_Read_Yaw_Angle();
	osMutexRelease(sensorProcessMutexHandle);
}

void GPSProcess(GeneralControl_TypeDefInit *controlStruct, char  *gps_buffer, size_t buffer_size)
{
	lwgps_process(&controlStruct->gps.gps, gps_buffer, buffer_size);
	controlStruct->gps.Altitude = controlStruct->gps.gps.altitude;
	controlStruct->gps.Latitude = controlStruct->gps.gps.latitude;
	controlStruct->gps.Longitude = controlStruct->gps.gps.longitude;
	controlStruct->gps.Speed = controlStruct->gps.gps.speed;

}

void CalculateVelocityCallback(GeneralControl_TypeDefInit  *controlStruct)
{
	controlStruct->bmp.speed = (controlStruct->bmp.Altitude - controlStruct->bmp.previousAltitude) / 1.0f; // 1 saniyelik periyot varsayımı
	controlStruct->bmp.previousAltitude = controlStruct->bmp.Altitude;
}

float ReadOptimizedBatteryVoltage() {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK) {
        uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        float voltage = (adc_value * 3.3f) / 4095.0f;

        return voltage * 4.0f;
    }

    HAL_ADC_Stop(&hadc1);
    return 0.0f;
}

void LoraWriteProcess(GeneralControl_TypeDefInit  *controlStruct)
{

	static int package_counter = 0; // when variable is static in method, data keep inside ram during season hence, it will keep package counter data.
	controlStruct->telemetry_package.package_count = package_counter++;
	char error_state[7];
	memset(error_state,0,sizeof(error_state));

    CreateShortTimeString(controlStruct->telemetry_package.sending_time);

    controlStruct->telemetry_package.pressure1 = controlStruct->bmp.Pressure;
    controlStruct->telemetry_package.pressure2 = controlStruct->container.containerPressure;

    controlStruct->telemetry_package.altitude1 = controlStruct->bmp.Altitude;
    controlStruct->telemetry_package.altitude2 = controlStruct->container.containerAltitude;
    controlStruct->telemetry_package.diff_altitude = fabs(controlStruct->telemetry_package.altitude1 - controlStruct->telemetry_package.altitude2);

    controlStruct->telemetry_package.landing_speed = controlStruct->bmp.speed;
    controlStruct->telemetry_package.temperature = controlStruct->bmp.Temperature;

    osMutexAcquire(sensorProcessMutexHandle, osWaitForever);
    controlStruct->telemetry_package.battery_voltage = ReadOptimizedBatteryVoltage();
	osMutexRelease(sensorProcessMutexHandle);

	controlStruct->telemetry_package.gps1_latitude = controlStruct->gps.Latitude;
	controlStruct->telemetry_package.gps1_longitude = controlStruct->gps.Longitude;
	controlStruct->telemetry_package.gps1_altitude = controlStruct->gps.Altitude;

	controlStruct->telemetry_package.pitch = controlStruct->mpu.PitchAngle;
	controlStruct->telemetry_package.roll = controlStruct->mpu.RollAngle;
	controlStruct->telemetry_package.yaw = controlStruct->mpu.YawAngle;

    strcpy(controlStruct->telemetry_package.rhrh, "0000");
    controlStruct->telemetry_package.iot_s1_data = 25;
    controlStruct->telemetry_package.iot_s2_data = 25;
    controlStruct->telemetry_package.team_no = 655357;

    controlStruct->telemetry_package.satallite_status = UpdateOptimizedSatelliteStatus(&controlStruct->telemetry_package, controlStruct->bmp.previousAltitude);

    UpdateOptimizedErrorCode(&controlStruct->telemetry_package, controlStruct->telemetry_package.satallite_status, error_state);
    strncpy(controlStruct->telemetry_package.error_code, error_state,sizeof(controlStruct->telemetry_package.error_code) - 1);
    controlStruct->telemetry_package.error_code[6] = '\0';
    CreateBinaryTelemetryPacket(&controlStruct->binary_telemetry, &controlStruct->telemetry_package);

    memset(controlStruct->lora.TransmitPackage, 0, sizeof(controlStruct->lora.TransmitPackage));
    snprintf(controlStruct->lora.TransmitPackage,sizeof(controlStruct->lora.TransmitPackage),
        "S%lu,%u,%s,%s,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%s,%.1f,%.1f,%luE",
		 controlStruct->telemetry_package.package_count,
		 controlStruct->telemetry_package.satallite_status,
		 controlStruct->telemetry_package.error_code,
		 controlStruct->telemetry_package.sending_time,
		 controlStruct->telemetry_package.pressure1/1000.0f,   // Pascal -> kPa dönüşümü (2 basamaklı)
		 controlStruct->telemetry_package.pressure2/1000.0f,   // Pascal -> kPa dönüşümü (2 basamaklı)
		 controlStruct->telemetry_package.altitude1,
		 controlStruct->telemetry_package.altitude2,
		 controlStruct->telemetry_package.diff_altitude,
		 controlStruct->telemetry_package.landing_speed,
		 controlStruct->telemetry_package.temperature,
		 controlStruct->telemetry_package.battery_voltage,
		 controlStruct->telemetry_package.gps1_latitude,
		 controlStruct->telemetry_package.gps1_longitude,
		 controlStruct->telemetry_package.gps1_altitude,
		 controlStruct->telemetry_package.pitch,
		 controlStruct->telemetry_package.roll,
		 controlStruct->telemetry_package.yaw,
		 controlStruct->telemetry_package.rhrh,
		 controlStruct->telemetry_package.iot_s1_data,
		 controlStruct->telemetry_package.iot_s2_data,
		 controlStruct->telemetry_package.team_no
    );
    controlStruct->lora.TransmitPackage[sizeof(controlStruct->lora.TransmitPackage)-1] = '\0';

    osStatus_t st = osMessageQueuePut(uart2outcomequeueHandle,
                                          &controlStruct->binary_telemetry, 0, 0);
        if (st == osErrorResource) {
            BinaryTelemetryPacket_t drop_oldest;
            (void)osMessageQueueGet(uart2outcomequeueHandle, &drop_oldest, NULL, 0); // en eskisini at
            (void)osMessageQueuePut(uart2outcomequeueHandle,
                                    &controlStruct->binary_telemetry, 0, 0);
        }


    osEventFlagsSet(systemEventsHandle, EVENT_FLAG_LORA_WRITE_READY);
}

void LoraReadProcess(uint8_t *rx_buffer, size_t buffer_size)
{

 uint16_t size = strnlen((char *)rx_buffer,buffer_size);
 uint8_t cmd_buffer[buffer_size];
 static char previous_cmd[6];
 strncpy((char*)cmd_buffer, (char*)rx_buffer, size);
 cmd_buffer [size - 1] = '\0';

 if(size < 10)
 {
	 if(cmd_buffer[0] == 'K' && cmd_buffer[1] == 'A')
	 {
		 osEventFlagsSet(systemEventsHandle, EVENT_FLAG_SERVO_PROCESS_READY);
		 osThreadYield();
	 }
	 else if (isalpha(cmd_buffer[0]) && isdigit(cmd_buffer[1]) && isalpha(cmd_buffer[2]) &&
			 isdigit(cmd_buffer[3]) && (cmd_buffer[4] == '\n' || cmd_buffer[4] == '\0'))
	 {

		 //Telecommand_Execute((char*)cmd_buffer);
		 strncpy((char *)previous_cmd,(char *)cmd_buffer,sizeof(previous_cmd) - 1);
		 previous_cmd[5] = '\0';
		 osMessageQueuePut(telecommandqueueHandle, previous_cmd, 0, 0);
		 osEventFlagsSet(systemEventsHandle, EVENT_FLAG_TELECOMMAND_READY);
		 osThreadYield();
	 }

 } else
 {
	 // TODO: Added other container and IOT devices datas and added to general control structure inside
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
  Lora_Filling(&controlStruct.loraAdress);
  controlStruct.telemetry_package.package_count = 1;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM6_Init();
  MX_I2C3_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  ServoInit();
  Lora_Init(&controlStruct.loraAdress); //Addresing
  lwgps_init(&controlStruct.gps.gps);
  HAL_UART_Receive_IT(&huart6, &controlStruct.gps.rx_data, 1);
  HAL_UART_Receive_IT(&huart2, &controlStruct.uartData.rx_data, 1);
  BME280_Init(&controlStruct.bmp);
  MPU6050_Init();
  Servo1SetAngle(0);
  Servo2SetAngle(0);

  //Telecommand_Execute(cmd);
   //HAL_Delay(4000);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();
  /* Create the mutex(es) */
  /* creation of sensorProcessMutex */
  sensorProcessMutexHandle = osMutexNew(&sensorProcessMutex_attributes);

  /* creation of loraCommunicationMutex */
  loraCommunicationMutexHandle = osMutexNew(&loraCommunicationMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of loraWriteTimer */
  loraWriteTimerHandle = osTimerNew(LoraWriteTimerCallBack, osTimerPeriodic, &controlStruct, &loraWriteTimer_attributes);

  /* creation of velocityCalcTimer */
  velocityCalcTimerHandle = osTimerNew(VelocityCalculateTimerCallback, osTimerPeriodic, &controlStruct, &velocityCalcTimer_attributes);

  /* creation of seperationTimer */
  separationTimerHandle = osTimerNew(SeparationTimerCallback, osTimerOnce, &controlStruct, &separationTimer_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uart2incomingbuffer */
  uart2incomingbufferHandle = osMessageQueueNew (16, sizeof(controlStruct.uartData.rx_buffer), &uart2incomingbuffer_attributes);

  /* creation of uart2outcomequeue */
  uart2outcomequeueHandle = osMessageQueueNew (1, sizeof(controlStruct.binary_telemetry), &uart2outcomequeue_attributes);

  /* creation of gpsBufferQueue */
  gpsBufferQueueHandle = osMessageQueueNew (3, sizeof(controlStruct.gps.rx_buffer), &gpsBufferQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  telecommandqueueHandle = osMessageQueueNew (1, sizeof(char [6]), &telecommandqueue_attributes);


  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of startingTask */
  startingTaskHandle = osThreadNew(StartDefaultTask, &controlStruct, &startingTask_attributes);

  /* creation of bmpProcessTask */
  bmpProcessTaskHandle = osThreadNew(BMPProcessTaskCallback, &controlStruct, &bmpProcessTask_attributes);

  /* creation of mpuProcessTask */
  mpuProcessTaskHandle = osThreadNew(MPUProcessTaskCallback, &controlStruct, &mpuProcessTask_attributes);

  /* creation of sdWriteTask */
  sdWriteTaskHandle = osThreadNew(SDWriteTaskCallback, &controlStruct, &sdWriteTask_attributes);

  /* creation of servoTask */
  servoTaskHandle = osThreadNew(ServoTaskCallback, &controlStruct, &servoTask_attributes);

  /* creation of telecommandTask */
  telecommandTaskHandle = osThreadNew(TelecommandTask, &controlStruct, &telecommandTask_attributes);

  /* creation of loraTask */
  loraTaskHandle = osThreadNew(LoraTaskCallback, &controlStruct, &loraTask_attributes);

  /* creation of gpsProcess */
  gpsProcessHandle = osThreadNew(GPSProcessCallback, &controlStruct, &gpsProcess_attributes);

  /* creation of loraParse */
  loraParseHandle = osThreadNew(LoraParseCallback, &controlStruct, &loraParse_attributes);

  /* creation of seperationState */
  seperationStateHandle = osThreadNew(SeperationStateCallback, &controlStruct, &seperationState_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* creation of systemEvents */
  systemEventsHandle = osEventFlagsNew(&systemEvents_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
 //EMPTY
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
  RCC_OscInitStruct.PLL.PLLN = 170;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 85 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000 - 1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 85 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 180 - 1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 1000 - 1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 170 - 1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 1000 - 1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, STEP1_IN1_Pin|STEP1_IN2_Pin|STEP1_IN3_Pin|STEP1_IN4_Pin
                          |STEP2_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEP2_IN1_Pin|STEP2_IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LORA_M1_Pin|LORA_M0_Pin|STEP2_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : STEP1_IN1_Pin STEP1_IN2_Pin STEP1_IN3_Pin STEP1_IN4_Pin
                           STEP2_IN4_Pin */
  GPIO_InitStruct.Pin = STEP1_IN1_Pin|STEP1_IN2_Pin|STEP1_IN3_Pin|STEP1_IN4_Pin
                          |STEP2_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP2_IN1_Pin STEP2_IN3_Pin */
  GPIO_InitStruct.Pin = STEP2_IN1_Pin|STEP2_IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_M1_Pin LORA_M0_Pin */
  GPIO_InitStruct.Pin = LORA_M1_Pin|LORA_M0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : STEP2_IN2_Pin */
  GPIO_InitStruct.Pin = STEP2_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STEP2_IN2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KAAN_BUTTON_Pin */
  GPIO_InitStruct.Pin = KAAN_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KAAN_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the startingTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	osDelay(200);
	osMutexAcquire(sensorProcessMutexHandle, osWaitForever);
	GeneralControl_TypeDefInit *controlStruct = (GeneralControl_TypeDefInit *)argument;
	BME280_Calculate_Data(&controlStruct->bmp);
	controlStruct->bmp.previousAltitude = controlStruct->bmp.Altitude;
	osMutexRelease(sensorProcessMutexHandle);
	osTimerStart(loraWriteTimerHandle, 1000);
	osTimerStart(velocityCalcTimerHandle, 1000);

  /* Infinite loop */
	while(1)
	{
	 osDelay(1);
	};
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_BMPProcessTaskCallback */
/**
* @brief Function implementing the bmpProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BMPProcessTaskCallback */
void BMPProcessTaskCallback(void *argument)
{
  /* USER CODE BEGIN BMPProcessTaskCallback */

	GeneralControl_TypeDefInit *controlStruct = (GeneralControl_TypeDefInit *)argument;
  /* Infinite loop */
  for(;;)
  {
	BMP280Process(&controlStruct->bmp);
    osDelay(10);
  }
  /* USER CODE END BMPProcessTaskCallback */
}

/* USER CODE BEGIN Header_MPUProcessTaskCallback */
/**
* @brief Function implementing the mpuProcessTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MPUProcessTaskCallback */
void MPUProcessTaskCallback(void *argument)
{
  /* USER CODE BEGIN MPUProcessTaskCallback */
	GeneralControl_TypeDefInit *controlStruct = (GeneralControl_TypeDefInit *)argument;
  /* Infinite loop */
  while(1)
  {
	MPU6050Process(&controlStruct->mpu);
	osDelay(10);
  }
  /* USER CODE END MPUProcessTaskCallback */
}

/* USER CODE BEGIN Header_SDWriteTaskCallback */
/**
* @brief Function implementing the sdWriteTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SDWriteTaskCallback */
void SDWriteTaskCallback(void *argument)
{
  /* USER CODE BEGIN SDWriteTaskCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SDWriteTaskCallback */
}

/* USER CODE BEGIN Header_ServoTaskCallback */
/**
* @brief Function implementing the servoTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ServoTaskCallback */
void ServoTaskCallback(void *argument)
{
  /* USER CODE BEGIN ServoTaskCallback */
	//GeneralControl_TypeDefInit *controlStruct = (GeneralControl_TypeDefInit *)argument;
	uint32_t remoteSepFlags = 0;
	uint32_t algorithmSepFlags = 0;

  /* Infinite loop */
  while(1)
  {
	  remoteSepFlags = osEventFlagsWait(systemEventsHandle, EVENT_FLAG_SEPERATION_STARTED, osFlagsNoClear | osFlagsWaitAny, osWaitForever);
	  algorithmSepFlags = osEventFlagsWait(systemEventsHandle, EVENT_FLAG_FIRST_SEPERATION, osFlagsNoClear | osFlagsWaitAny, osWaitForever);

	  if((remoteSepFlags & EVENT_FLAG_SEPERATION_STARTED) || (algorithmSepFlags & EVENT_FLAG_FIRST_SEPERATION))
	  {
		  osEventFlagsClear(systemEventsHandle, EVENT_FLAG_SEPERATION_STARTED);
		  osEventFlagsClear(systemEventsHandle, EVENT_FLAG_FIRST_SEPERATION);
		  osTimerStart(separationTimerHandle,50);
	  }

    osDelay(10);
  }
  /* USER CODE END ServoTaskCallback */
}

/* USER CODE BEGIN Header_TelecommandTask */
/**
* @brief Function implementing the telecommandTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TelecommandTask */
void TelecommandTask(void *argument)
{
  /* USER CODE BEGIN TelecommandTask */

	uint32_t flags;
	static char tlc_com[6];
  /* Infinite loop */
  while(1)
  {
	  flags = osEventFlagsWait(systemEventsHandle, EVENT_FLAG_TELECOMMAND_READY, osFlagsNoClear| osFlagsWaitAny, osWaitForever);
	  if(flags & EVENT_FLAG_TELECOMMAND_READY)
	  {
		  osEventFlagsClear(systemEventsHandle, EVENT_FLAG_TELECOMMAND_READY);
		  if(osMessageQueueGet(telecommandqueueHandle, tlc_com, NULL, 0) == osOK)
		  {
			  Telecommand_Execute((char*)tlc_com);
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END TelecommandTask */
}

/* USER CODE BEGIN Header_LoraTaskCallback */
/**
* @brief Function implementing the loraTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LoraTaskCallback */
void LoraTaskCallback(void *argument)
{
  /* USER CODE BEGIN LoraTaskCallback */

	GeneralControl_TypeDefInit *controlStruct = (GeneralControl_TypeDefInit *)argument;

  /* Infinite loop */
  while(1)
  {
	LoraWriteProcess(controlStruct);
    osDelay(10);
  }
  /* USER CODE END LoraTaskCallback */
}

/* USER CODE BEGIN Header_GPSProcessCallback */
/**
* @brief Function implementing the gpsProcess thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_GPSProcessCallback */
void GPSProcessCallback(void *argument)
{
  /* USER CODE BEGIN GPSProcessCallback */

	GeneralControl_TypeDefInit *controlStruct = (GeneralControl_TypeDefInit *)argument;
	static uint32_t flags = 0;
	char gps_buffer[128];
  /* Infinite loop */
	while(1)
	{
		flags = osEventFlagsWait(systemEventsHandle, EVENT_FLAG_GPS_DATA_PROCECSS_READY, osFlagsWaitAny | osFlagsNoClear, osWaitForever);

		if(flags & EVENT_FLAG_GPS_DATA_PROCECSS_READY) // bit kontrolü
		{
			osEventFlagsClear(systemEventsHandle, EVENT_FLAG_GPS_DATA_PROCECSS_READY);

			if(osMessageQueueGet(gpsBufferQueueHandle, gps_buffer, NULL, 0) == osOK)
			{
				GPSProcess(controlStruct, gps_buffer,strlen(gps_buffer));
			}

		}

		osDelay(5);
  }
  /* USER CODE END GPSProcessCallback */
}

/* USER CODE BEGIN Header_LoraParseCallback */
/**
* @brief Function implementing the loraParse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LoraParseCallback */
void LoraParseCallback(void *argument)
{
  /* USER CODE BEGIN LoraParseCallback */

	static uint32_t flags;
	uint8_t rx_buffer[100];


  /* Infinite loop */
  while(1)
  {

	  flags = osEventFlagsWait(systemEventsHandle, EVENT_FLAG_LORA_READ_READY, osFlagsNoClear | osFlagsWaitAny, osWaitForever);

	  if(flags & EVENT_FLAG_LORA_READ_READY)
	  	{
		  osEventFlagsClear(systemEventsHandle, EVENT_FLAG_LORA_READ_READY);
	  		if(osMessageQueueGet(uart2incomingbufferHandle, rx_buffer, NULL, 0) == osOK)
	  		{
	  			LoraReadProcess(rx_buffer, sizeof(rx_buffer));
	  		}
	  	}
    osThreadYield();
  }
  /* USER CODE END LoraParseCallback */
}


/* USER CODE BEGIN Header_LoraParseCallback */
/**
* @brief Function implementing the loraParse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SeperationStateCallback */
void SeperationStateCallback(void *argument)
{
  /* USER CODE BEGIN SeperationStateCallback */

	GeneralControl_TypeDefInit *constrolStruct  = (GeneralControl_TypeDefInit *) argument;
	static uint16_t maxAltFlag = 0;
	static float maxAlt = 0.0f;

  /* Infinite loop */
  while(1)
  {

	if(constrolStruct->bmp.Altitude > maxAlt)
		maxAlt = constrolStruct->bmp.Altitude;
	else if(constrolStruct->bmp.Altitude < maxAlt)
		maxAltFlag = 1;

	if((constrolStruct->bmp.Altitude < 700.0f || constrolStruct->mpu.AccZ < 10.0f) && maxAltFlag)
	{
		osEventFlagsSet(systemEventsHandle, EVENT_FLAG_FIRST_SEPERATION);
		maxAltFlag = 0;
		osThreadExit();
	}

	osDelay(1);

  }
  /* USER CODE END SeperationStateCallback */
}

/* LoraWriteTimerCallBack function */
void LoraWriteTimerCallBack(void *argument)
{
  /* USER CODE BEGIN LoraWriteTimerCallBack */
	GeneralControl_TypeDefInit *controlStruct = (GeneralControl_TypeDefInit *)argument;
	static BinaryTelemetryPacket_t binary_telemetry;

	if(osMessageQueueGet(uart2outcomequeueHandle,&binary_telemetry, NULL, 0) == osOK)
	{
		osMutexAcquire(loraCommunicationMutexHandle, osWaitForever);
		Lora_Transmiter(&controlStruct->loraAdress, (uint8_t *)&binary_telemetry, sizeof(binary_telemetry));
		osMutexRelease(loraCommunicationMutexHandle);
	}

  /* USER CODE END LoraWriteTimerCallBack */
}

/* VelocityCalculateTimerCallback function */
void VelocityCalculateTimerCallback(void *argument)
{
  /* USER CODE BEGIN VelocityCalculateTimerCallback */
	GeneralControl_TypeDefInit *controlStruct = (GeneralControl_TypeDefInit *)argument;
	CalculateVelocityCallback(controlStruct);
  /* USER CODE END VelocityCalculateTimerCallback */
}

/* SeperationTimerCallback function */
void SeparationTimerCallback(void *argument)
{
  /* USER CODE BEGIN SeperationTimerCallback */
	Servo1SetAngle(0);
	Servo2SetAngle(0);
	osEventFlagsSet(systemEventsHandle,EVENT_FLAG_SEPERATION_COMPLETED);
  /* USER CODE END SeperationTimerCallback */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
