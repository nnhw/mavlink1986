/* 2017 Куклев Илья nnhw667@gmail.com
 * Программа управления сервоприводами для МК миландр 1986ве92у
 *
 * Комментарии к функциям содержатся в заголовочных файлах
 *
 *
 * */

#include "MDR32Fx.h"
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_rst_clk.h>
#include <MDR32F9Qx_adc.h>
#include <MDR32F9Qx_eeprom.h>
#include "MDR32F9Qx_config.h"
#include <MDR32F9Qx_iwdg.h>
#include <MDR32F9Qx_port.h>
#include <MDR32F9Qx_comp.h>
#include <MDR32F9Qx_timer.h>
#include <MDR32F9Qx_uart.h>

#include "information_exchange.h"
#include "error.h"
#include "drive.h"
#include "hardware.h"



int main(void){
MDR_IWDG->KR = KR_KEY_Reload; //сброс сторожевого таймера
peripheral_initialization(); //инициализация системы

system_default_state();

while(1)
  {
	  
	MDR_IWDG->KR = KR_KEY_Reload; //сброс сторожевого таймера

	inf_exhange();
		
	switch(mavlink_mode)
	{
		case MANUAL:
		manual_mode_control();
		break;

		case PROGRAM:
		PID_control();
		break;

		case STABILIZE:
		stabilization();
		break;

		default:

		break;
	}

 }

}
