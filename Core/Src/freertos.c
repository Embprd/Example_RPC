/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern UART_HandleTypeDef huart1;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PACKET_SIZE 128

uint8_t crc8_calc(const uint8_t *data, uint16_t len);
bool uart_send(const uint8_t *data, uint16_t len);
bool link_layer_send(const uint8_t *payload, uint16_t payload_len);
bool transport_call(const char *name, const uint8_t *args, uint16_t args_len,
		uint8_t *resp, uint16_t *resp_len, uint32_t timeout);

void getTemp(const uint8_t* args, uint16_t args_len,uint8_t* resp, uint16_t* resp_len);
void getPressure(const uint8_t* args, uint16_t args_len,uint8_t* resp, uint16_t* resp_len);
void send_response(uint8_t seq,  uint8_t* data, uint16_t len) ;
void send_error(uint8_t seq, uint8_t error_code);

typedef void (*RPCFunction)(const uint8_t*, uint16_t, uint8_t*, uint16_t*);

typedef struct {
    uint8_t data[MAX_PACKET_SIZE];
    uint16_t length;
} Packet_t;

typedef struct {
    const char *name;
    RPCFunction func;
} FunctionEntry;

FunctionEntry functions[2] = {
		{"Get Temp",getTemp},
		{"Get Pressure",getPressure}
};
uint8_t func_count = 2;
uint8_t seq_counter = 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

void getTemp(const uint8_t* args, uint16_t args_len,uint8_t* resp, uint16_t* resp_len)
{
	// Функция получения температуры
}
void getPressure(const uint8_t* args, uint16_t args_len, uint8_t* resp, uint16_t* resp_len)
{
	// Функция установки температуры
}


void send_response(uint8_t seq, uint8_t* data, uint16_t len)
{
    uint8_t payload[2 + len];
    payload[0] = RESPONSE; // RESPONSE
    payload[1] = seq;
    memcpy(&payload[2], data, len);
    link_layer_send(payload, 2 + len);
}


void send_error(uint8_t seq, uint8_t error_code)
{
    uint8_t payload[3];
    payload[0] = ERR;
    payload[1] = seq;
    payload[2] = error_code;
    link_layer_send(payload, 3);
}

uint8_t crc8_calc(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    while (len--) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    }
    return crc;
}

bool uart_send(const uint8_t *data, uint16_t len)
{
    return HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 10) == HAL_OK;
}

bool link_layer_send(const uint8_t *payload, uint16_t payload_len)
{
    // Проверка максимального размера
    if (payload_len > MAX_PACKET_SIZE - 7) return false;

    uint8_t packet[MAX_PACKET_SIZE];
    uint16_t l = 6 + payload_len;  // Длина части после 0xFA

    // Формирование заголовка
    packet[0] = 0xFA;              // Стартовый байт 1
    packet[1] = l & 0xFF;           // Младший байт длины
    packet[2] = (l >> 8) & 0xFF;    // Старший байт длины
    packet[3] = crc8_calc(&packet[1], 2); // CRC заголовка (байты 1-2)
    packet[4] = 0xFB;              // Стартовый байт 2

    // Копирование полезной нагрузки
    memcpy(&packet[5], payload, payload_len);

    // Расчет CRC всего пакета (байты 0-4 + payload)
    uint8_t crc_full = crc8_calc(packet, 5 + payload_len);
    packet[5 + payload_len] = crc_full;   // CRC payload
    packet[6 + payload_len] = 0xFE;       // Стоповый байт

    // Отправка через физический уровень
    return uart_send(packet, 7 + payload_len);
}




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId linkLayerTaskHandle;
osThreadId transpRecTaskHandle;
osMessageQId rxQueueHandle;
osSemaphoreId GetPacketSemHandle;
osSemaphoreId SendPacketSemHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void LinkLayerReceive(void const * argument);
void TranspReceive(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of GetPacketSem */
  osSemaphoreDef(GetPacketSem);
  GetPacketSemHandle = osSemaphoreCreate(osSemaphore(GetPacketSem), 1);
  osSemaphoreWait(GetPacketSemHandle, osWaitForever);
  /* definition and creation of SendPacketSem */
  osSemaphoreDef(SendPacketSem);
  SendPacketSemHandle = osSemaphoreCreate(osSemaphore(SendPacketSem), 1);
  osSemaphoreWait(SendPacketSemHandle, osWaitForever);
  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of rxQueue */
  osMessageQDef(rxQueue, 16, uint16_t);
  rxQueueHandle = osMessageCreate(osMessageQ(rxQueue), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of linkLayerTask */
  osThreadDef(linkLayerTask, LinkLayerReceive, osPriorityHigh, 0, 512);
  linkLayerTaskHandle = osThreadCreate(osThread(linkLayerTask), NULL);

  /* definition and creation of transpRecTask */
  osThreadDef(transpRecTask, TranspReceive, osPriorityHigh, 0, 512);
  transpRecTaskHandle = osThreadCreate(osThread(transpRecTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_LinkLayerReceive */
/**
* @brief Function implementing the linkLayerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LinkLayerReceive */
void LinkLayerReceive(void const * argument)
{
  /* USER CODE BEGIN LinkLayerReceive */
    //uint8_t buffer[MAX_PACKET_SIZE];
    uint16_t length = 0, index = 0;
    uint8_t expected_crc;
    uint8_t eStatus;
    Packet_t packet;
  /* Infinite loop */
  for(;;)
  {
	osSemaphoreWait(GetPacketSemHandle, osWaitForever);

	if(Buffer[0] == 0xFA)
	{
		expected_crc = crc8_calc(&Buffer[1], 2);

		 if(expected_crc == Buffer[3])
		 {
			 length = Buffer[2]<<8 |Buffer[1];
			 index = 0;

			 if( Buffer[3] == 0xFB)
			 {
				 for(uint8_t i = 5 ;  i < length - 2 ; i++) // Учет CRC и 0xFE
				 {
					 packet.data[index++] = Buffer[i];
				 }

				 packet.length = index;

				 expected_crc = crc8_calc(&Buffer[0],length - 2);

				 if(expected_crc == Buffer[length - 2] && Buffer[length - 1] == 0xFE)
				 {
					 // Успешный прием пакета
					 eStatus = xQueueSend(rxQueueHandle, &packet, 0);
				 }
			 }

		 }
	}

	if(eStatus == 0)
	send_error(0,ERR_INVALID_ARGS); // Отправка ошибки

    osDelay(1000);
  }
  /* USER CODE END LinkLayerReceive */
}

/* USER CODE BEGIN Header_TranspReceive */
/**
* @brief Function implementing the transpRecTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TranspReceive */
void TranspReceive(void const * argument)
{
  /* USER CODE BEGIN TranspReceive */

  /* Infinite loop */
  for(;;)
  {
	  uint8_t packet[MAX_PACKET_SIZE];
	  uint8_t packet_len;

	  if(xQueueReceive(rxQueueHandle, &packet, portMAX_DELAY))
	  {
		 uint32_t start_time = xTaskGetTickCount();

		  Packet_t packet;
	      // Разбор пакета и вызов функции
		  uint16_t packet_len = packet.length;
		  uint8_t* data = packet.data;

		  uint8_t type = data[0];
		  uint8_t seq = data[1];

          if(seq != REQUEST  || seq != STREAM )
          {
        	  send_error(seq,ERR_FUNC_NOT_FOUND); // Отправка ошибки
          }
          else
          {
        	  // 3. Поиск терминатора имени функции
        	  uint16_t name_end_pos = 0;

        	  for(uint16_t i = 2; i < packet_len; i++)
        	  {
        		  if(data[i] == 0x00) {
        			  name_end_pos = i;
        			  break;
        		  }
        	  }
        	  // 4. Проверка наличия терминатора
        	  if(name_end_pos == 0)
        	  {
        		  send_error(seq,ERR_INVALID_ARGS); // Отправка ошибки
        		  continue;
        	  }

        	  // 5. Извлечение имени функции
        	  uint16_t name_len = name_end_pos - 2;
        	  char func_name[32];
        	  strncpy(func_name, &data[2], name_len);
        	  func_name[name_len] = '\0'; // Гарантированное завершение

        	  // 6. Извлечение аргументов
        	  uint16_t args_offset = name_end_pos + 1;
        	  uint16_t args_len = packet_len - args_offset;

        	  // Поиск функции
        	  RPCFunction func = NULL;
        	  for(int i = 0; i < func_count; i++)
        	  {
        		  if(strcmp(func_name, functions[i].name) == 0) {
        			  func = functions[i].func;
        			  break;
        		  }
        	  }
        	  // Вызов функции и отправка ответа
        	  if(func)
        	  {
        		  uint16_t resp_len = 0;
        		  uint8_t response[128];
        		  uint16_t arg_len;
        		  func(&data[args_offset], args_len ,response, &resp_len);
        	  	  // Формирование ответного пакета
        		  uint32_t elapsed = xTaskGetTickCount() - start_time;

        		  if (elapsed > FUNCTION_TIMEOUT_MS)
        		  {
        			  send_error(seq, ERR_TIMEOUT);
        		  }
        		  else
        		  {
            		  send_response(seq, response, resp_len);
        		  }

        	  }
        	  else {
        		  send_error(seq,ERR_FUNC_NOT_FOUND); // Отправка ошибки
        	  }
          }

	  }
	  osDelay(1000);
  }
  /* USER CODE END TranspReceive */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

