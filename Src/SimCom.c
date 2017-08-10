#include <string.h>

#include "FIFO.h"
#include "PhysicalLayer.h"
#include "ServiceLayer.h"
#include "SimCom.h"
#include "flash.h"
#include "motion.h"
#include "mpu6050.h"

#include "cmsis_os.h"


UART_HandleTypeDef *uart_device;


/*
 * Test
 */
void callback0(char from, char to, const char* data, SIMCOM_LENGTH_TYPE length)
{
  sl_send(to,from,data,length);
}

void callback5_motion(char from, char to, const char* data, SIMCOM_LENGTH_TYPE length)
{
	extern uint16_t pos_x, pos_y;
	extern uint16_t holes[];
	extern bool holes_available;

	if(length == 4) {
		pos_x = ((const uint16_t*)data)[0];
		pos_y = ((const uint16_t*)data)[1];

		if(debugFlag) {
			//sl_send(5, 0, &pos_x, 2);
			//sl_send(5, 0, &pos_y, 2);
		}
	}

	if(length == 36) {
		for(int i = 0; i < 18; i++) {
			holes[i] = ((const uint16_t*)data)[i];
		}
		holes_available = true;
		sl_send(to, from, 'A', 1);
	}

	if(length == 1) {
		switch(data[0]) {
		case 'S':
			sl_send(to, from, "Startting", 10);
			osDelay(1);
			motor_start();
			break;
		case 's':
			sl_send(to, from, "Stopping", 10);
			osDelay(1);
			motor_stop();
			break;
		case 'R':
			sl_send(to, from, "Resetting", 10);
			osDelay(1);
			motor_reset();
			break;
		case 'D':
			sl_send(to, from, "Debugging", 10);
			osDelay(1);
			debugFlag = true;
			break;
		case 'd':
			sl_send(to, from, "Not Debugging", 14);
			osDelay(1);
			debugFlag = false;
			break;
		default:
			break;
		}
	}
}

void callback9_flash(char from, char to, const char* data, SIMCOM_LENGTH_TYPE length)
{
	flash_write(0, data, length);

	sl_send(to, from, flash_get_addr(0), length);
}

void akm8963_calib();

#ifdef MPU6050_USE_MAG
void callback8_calib(char from, char to, const char* data, SIMCOM_LENGTH_TYPE length)
{
	akm8963_calib();
}
#endif


void StartSendTask(void const * argument)
{
  for(;;)
  {
	  ph_send_intr();
  }
}

void StartReceiveTask(void const * argument)
{
  for(;;)
  {
	  char c;

	  if(out_fifo(&ph_receive_fifo, &c)) {
		  in_char_queue(&ph_receive_queue, c);
		  sl_receive_intr();
		  osThreadYield();

	  } else {
		  extern char ph_receive_it_buf[];
		  HAL_UART_Receive_IT(uart_device, ph_receive_it_buf, 1);
		  osDelay(1);
	  }
  }
}

bool simcom_init(UART_HandleTypeDef *device)
{
	sl_config(0, callback0);

	sl_config(5, callback5_motion);

#ifdef MPU6050_USE_MAG
	sl_config(8, callback8_calib);
#endif

	sl_config(9, callback9_flash);

	uart_device = device;

	bool state = sl_init(device);

	return state;
}
