
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
#include "hardware.h"
//#include <int_can.h>



//static volatile uint8_t voltage = 50;
static volatile uint16_t pwm_period = 15465; //экспериметанльно подобрано через reverse engineering
//static volatile uint8_t pwm_period = 100;
//volatile uint16_t sys_rotation_step_req = 0u;

/* Выбор периода таймера 1, считающего периоды ppm */
static volatile uint16_t ppm_time = 20000; 



static volatile uint32_t inductive_rotation_timer = 6250-1;
//uint32_t inductive_rotation_speed = 0;

//uint32_t flash_adress = START_EEPROM;

void pause(uint32_t _maxticks)
{

	uint32_t tick = 0;
	//SysTick->CTRL |= 1;
	SysTick_start();
	while(tick <= _maxticks)
		{

			if (SysTick->CTRL >= 0x8000)
				{

					tick++;
					SysTick->CTRL &= ~0x8000;
				}

		}
	SysTick_stop();

}

void ADC_init(void)
{
ADCx_InitTypeDef ADC1,ADC2;
PORT_InitTypeDef PADC;

RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD,ENABLE);

PORT_StructInit (&PADC);
PADC.PORT_Pin = PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2 | PORT_Pin_3 | PORT_Pin_4 | PORT_Pin_5;
PADC.PORT_OE = PORT_OE_IN;
PADC.PORT_FUNC = PORT_FUNC_PORT;
PADC.PORT_SPEED = PORT_SPEED_MAXFAST;
PADC.PORT_MODE = PORT_MODE_ANALOG;
PADC.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
PADC.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
PORT_Init(MDR_PORTD,&PADC);


RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC,ENABLE);
ADC_DeInit();
ADCx_StructInit(&ADC1);
ADC1.ADC_Prescaler = ADC_CLK_div_2;
ADC1.ADC_SamplingMode = ADC_SAMPLING_MODE_CICLIC_CONV;
ADC1.ADC_VRefSource = ADC_VREF_SOURCE_INTERNAL;
ADC1.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
ADC1.ADC_Channels = ADC_CH_ADC5_MSK;
ADC1.ADC_DelayGo = 0;
//ADC1.ADC_LevelControl=ADC_LEVEL_CONTROL_Enable;
//ADC1.ADC_HighLevel = 0x7FF;
//ADC1.ADC_LowLevel = 0x00;
ADC1_Init(&ADC1);

//RST_CLK_PCLKcmd(RST_CLK_PCLK_ADC,ENABLE);
//ADC_DeInit();
ADCx_StructInit(&ADC2);
ADC2.ADC_Prescaler = ADC_CLK_div_2;
ADC2.ADC_SamplingMode = ADC_SAMPLING_MODE_CICLIC_CONV;
ADC2.ADC_VRefSource = ADC_VREF_SOURCE_INTERNAL;
ADC2.ADC_ChannelSwitching = ADC_CH_SWITCHING_Disable;
ADC2.ADC_Channels = ADC_CH_ADC2_MSK | ADC_CH_ADC3_MSK | ADC_CH_ADC4_MSK ;
ADC2.ADC_DelayGo = 0;
ADC2.ADC_LevelControl=ADC_LEVEL_CONTROL_Disable;
//ADC2.ADC_HighLevel = 0x59E;
//ADC2.ADC_LowLevel = 0x00;
ADC2_Init(&ADC2);


ADC1_SetChannel(5);
ADC2_SetChannel(2);

ADC1_Cmd(ENABLE);
ADC2_Cmd(ENABLE);


//ADC1_ITConfig(ADC1_IT_OUT_OF_RANGE,ENABLE);
//ADC2_ITConfig(ADC2_IT_END_OF_CONVERSION,ENABLE);
NVIC_DisableIRQ(ADC_IRQn);



}

void EEPROM_write_32(uint32_t _adress ,uint32_t _data)
{
	NVIC_DisableIRQ(Timer1_IRQn);
	NVIC_DisableIRQ(Timer3_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);


	EEPROM_ProgramWord(_adress,EEPROM_Info_Bank_Select,_data);

	NVIC_EnableIRQ(Timer1_IRQn);
	NVIC_EnableIRQ(Timer3_IRQn);
	NVIC_EnableIRQ(CAN1_IRQn);


}
void EEPROM_write_16(uint32_t _adress ,uint16_t _data)
{
	NVIC_DisableIRQ(Timer1_IRQn);
	NVIC_DisableIRQ(Timer3_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);


	EEPROM_ProgramHalfWord(_adress,EEPROM_Info_Bank_Select,_data);

	NVIC_EnableIRQ(Timer1_IRQn);
	NVIC_EnableIRQ(Timer3_IRQn);
	NVIC_EnableIRQ(CAN1_IRQn);

}
uint32_t EEPROM_read_32(uint32_t _adress)
{
	uint32_t return_value = 0;
	NVIC_DisableIRQ(Timer1_IRQn);
	NVIC_DisableIRQ(Timer3_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);
	//__disable_irq();


	return_value = EEPROM_ReadWord(_adress,EEPROM_Info_Bank_Select);

	NVIC_EnableIRQ(Timer1_IRQn);
	NVIC_EnableIRQ(Timer3_IRQn);
	NVIC_EnableIRQ(CAN1_IRQn);


  	return return_value;
}
uint16_t EEPROM_read_16(uint32_t _adress)
{
	uint16_t return_value = 0;

	NVIC_DisableIRQ(Timer1_IRQn);
	NVIC_DisableIRQ(Timer3_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);

	return_value = EEPROM_ReadHalfWord(_adress,EEPROM_Info_Bank_Select);

	NVIC_EnableIRQ(Timer1_IRQn);
	NVIC_EnableIRQ(Timer3_IRQn);
	NVIC_EnableIRQ(CAN1_IRQn);

  	return return_value;
}
void EEPROM_erase_info_page(void)
{
	NVIC_DisableIRQ(Timer1_IRQn);
	NVIC_DisableIRQ(Timer3_IRQn);
	NVIC_DisableIRQ(CAN1_IRQn);


 	EEPROM_ErasePage(0x08000000,EEPROM_Info_Bank_Select);

	NVIC_EnableIRQ(Timer1_IRQn);
	NVIC_EnableIRQ(Timer3_IRQn);
	NVIC_EnableIRQ(CAN1_IRQn);


}


/*!
* одиночное чтение 32-битного слова из памяти по адресу _addr
*
* uint32_t eepromRead32bitWordOne(uint32_t _addr)
* \param uint32_t _addr - адрес ячейки памяти откуда читаем слово
* \return uint32_t - слово, прочитанное по адресу _addr
*/



__RAMFUNC uint32_t eepromRead32bitWordOne(uint32_t _addr)
{
	uint32_t tmp;
	uint32_t ret = 0;

	MDR_EEPROM->KEY = EEPROM_REG_ACCESS_KEY;

	tmp = MDR_EEPROM->CMD;
	tmp |= (EEPROM_CMD_CON | EEPROM_CMD_IFREN );	// режим программирования
	MDR_EEPROM->CMD = tmp;	// IFREN = 1

	MDR_EEPROM->ADR = _addr;

	tmp |= (EEPROM_CMD_XE | EEPROM_CMD_YE | EEPROM_CMD_SE);
	MDR_EEPROM->CMD = tmp;

	MDR_EEPROM->DO;   // Idle Reading for Delay //
  MDR_EEPROM->DO;   // Idle Reading for Delay //
  MDR_EEPROM->DO;   // Idle Reading for Delay //
  ret = MDR_EEPROM->DO;

	tmp &= EEPROM_CMD_DELAY_Msk;
	MDR_EEPROM->CMD = tmp;
	MDR_EEPROM->KEY = 0;
  return ret;
}
/*!
* одиночная запись 32-битного слова _data в память по адресу _addr
*
* void eepromWrite32bitWordOne(uint32_t _addr, uint32_t _data)
* \param uint32_t _addr - адрес ячейки памяти куда записываем слово
* \param uint32_t _data - слово, которое записываем в ячейку памяти
*/
__RAMFUNC void eepromWrite32bitWordOne(uint32_t _addr, uint32_t _data)
{
	uint32_t tmp;

	MDR_EEPROM->KEY = EEPROM_REG_ACCESS_KEY;

	tmp = MDR_EEPROM->CMD;
	tmp |= (EEPROM_CMD_CON | EEPROM_CMD_IFREN );	// режим программирования
	MDR_EEPROM->CMD = tmp;	// IFREN = 1

	MDR_EEPROM->ADR = _addr;
	MDR_EEPROM->DI = _data;

	tmp |= (EEPROM_CMD_XE | EEPROM_CMD_PROG);
	MDR_EEPROM->CMD = tmp;
	pause(5);	// 5us

	tmp |= (EEPROM_CMD_NVSTR);
	MDR_EEPROM->CMD = tmp;
	pause(10);	// 10us

	tmp |= (EEPROM_CMD_YE);
	MDR_EEPROM->CMD = tmp;
	pause(40);	// 40us запись в память

	tmp &= ~(EEPROM_CMD_YE);
	MDR_EEPROM->CMD = tmp;

	tmp &= ~(EEPROM_CMD_PROG);
	MDR_EEPROM->CMD = tmp;
	pause(5);	// 5us

	tmp &= ~(EEPROM_CMD_XE | EEPROM_CMD_NVSTR);
	MDR_EEPROM->CMD = tmp;
	pause(1);	// 1us

	tmp &= EEPROM_CMD_DELAY_Msk;
	MDR_EEPROM->CMD = tmp;
  MDR_EEPROM->KEY = 0;
}




/*!
* начальные установки регистров для процедуры записи 32-битного слова в память
*
* void eepromWriteStart(void)
*/
__RAMFUNC void eepromWriteStart(void){
	uint32_t tmp;

	tmp = MDR_EEPROM->CMD;

	tmp |= (EEPROM_CMD_XE | EEPROM_CMD_PROG);
	MDR_EEPROM->CMD = tmp;
	pause(5);	// 5us

	tmp |= (EEPROM_CMD_NVSTR);
	MDR_EEPROM->CMD = tmp;
	pause(10);	// 10us
}
/*!
* конечные установки регистров для процедуры записи 32-битного слова в память
*
* void eepromWriteStop(void)
*/
__RAMFUNC void eepromWriteStop(void){
	uint32_t tmp;

	tmp = MDR_EEPROM->CMD;

	tmp &= ~(EEPROM_CMD_PROG);
	MDR_EEPROM->CMD = tmp;
	pause(5);	// 5us

	tmp &= ~(EEPROM_CMD_XE | EEPROM_CMD_NVSTR);
	MDR_EEPROM->CMD = tmp;
	pause(1);	// 1us
}




void IWDG_init(void)
{
	IWDG_WriteAccessEnable();
	IWDG_SetPrescaler(IWDG_Prescaler_16);
	//IWDG_SetReload(0x7D0);
	IWDG_SetReload(0xFFF);

}
void IWDG_start(void)
{
 IWDG_Enable();
}
void IWDG_stop(void)
{
	IWDG_WriteAccessEnable();
}
void IWDG_reset(void)
{
	IWDG_ReloadCounter();
}


void SysTick_start(void) // 1 ticks ~1us
{
SysTick->CTRL = 5;
SysTick->LOAD = 16;
}
void SysTick_stop(void) // 1 ticks ~1us
{
SysTick->CTRL = 0;
SysTick->VAL = 0;
//SysTick->LOAD = 16000000;
}
/*
void SysTick_Handler(void)
{

}
*/

void tm1_ppm_init(void)
{
TIMER_CntInitTypeDef tm1;

RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER1,ENABLE);

TIMER_DeInit (MDR_TIMER1);
TIMER_CntStructInit(&tm1);

TIMER_BRGInit(MDR_TIMER1,TIMER_HCLKdiv8);

tm1.TIMER_IniCounter = 0x0;
tm1.TIMER_Prescaler = 0x0;
//tm1.TIMER_CounterMode = TIMER_CntMode_ClkFixedDir;
//tm1.TIMER_CounterDirection = TIMER_CntDir_Up;
//tm1.TIMER_EventSource = TIMER_EvSrc_TM1 ;
tm1.TIMER_Period = ppm_time;
//tm1.TIMER_ARR_UpdateMode = TIMER_ARR_Update_On_CNT_Overflow;
//tm1.TIMER_FilterSampling = TIMER_FDTS_TIMER_CLK_div_1;

TIMER_CntInit(MDR_TIMER1,&tm1);

//TIMER_ITConfig(MDR_TIMER1,TIMER_STATUS_CNT_ARR,ENABLE);
}
void tm1_ppm_start(void)
{
		//NVIC_EnableIRQ(Timer1_IRQn ); //включение прерывания
		TIMER_Cmd(MDR_TIMER1,ENABLE);	//Разрешение работы таймера
}
void tm1_ppm_stop(void)
{
		//NVIC_DisableIRQ(Timer1_IRQn ); //включение прерывания
		TIMER_Cmd(MDR_TIMER1,DISABLE);	//Разрешение работы таймера
}



void tm2_init_inductive(void)
{
TIMER_CntInitTypeDef tm2;

RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER2,ENABLE);

TIMER_DeInit (MDR_TIMER2);
TIMER_CntStructInit(&tm2);

TIMER_BRGInit(MDR_TIMER2,TIMER_HCLKdiv128);

tm2.TIMER_IniCounter = 0x0;
tm2.TIMER_Prescaler = 0x1;
tm2.TIMER_CounterMode = TIMER_CntMode_ClkFixedDir;
tm2.TIMER_CounterDirection = TIMER_CntDir_Up;
//tm1.TIMER_EventSource = TIMER_EvSrc_TM1 ;
tm2.TIMER_Period = inductive_rotation_timer;
//tm2.TIMER_ARR_UpdateMode = TIMER_ARR_Update_On_CNT_Overflow;
//tm1.TIMER_FilterSampling = TIMER_FDTS_TIMER_CLK_div_1;

TIMER_CntInit(MDR_TIMER2,&tm2);

TIMER_ITConfig(MDR_TIMER2,TIMER_STATUS_CNT_ZERO,ENABLE);
}
void tm2_start_inductive(void){
		NVIC_EnableIRQ(Timer2_IRQn); //включение прерывания
		TIMER_Cmd(MDR_TIMER2,ENABLE);	//Разрешение работы таймера
}
void tm2_stop_inductive(void)
{
		NVIC_DisableIRQ(Timer2_IRQn ); //включение прерывания
		TIMER_Cmd(MDR_TIMER2,DISABLE);	//Разрешение работы таймера
}



void tm3_init_pwm(void)

{
TIMER_CntInitTypeDef tm3;
TIMER_ChnInitTypeDef ch3;
TIMER_ChnOutInitTypeDef chout3;
PORT_InitTypeDef TMR3PORT;

RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB,ENABLE);

PORT_StructInit (&TMR3PORT);
TMR3PORT.PORT_Pin = PORT_Pin_0 | PORT_Pin_1 | PORT_Pin_2 | PORT_Pin_3 ;
TMR3PORT.PORT_FUNC = PORT_FUNC_ALTER;
TMR3PORT.PORT_OE = PORT_OE_OUT;
TMR3PORT.PORT_SPEED = PORT_SPEED_FAST;
TMR3PORT.PORT_MODE = PORT_MODE_DIGITAL;
PORT_Init(MDR_PORTB,&TMR3PORT);

PORT_StructInit (&TMR3PORT);
TMR3PORT.PORT_Pin = PORT_Pin_5 | PORT_Pin_6 | PORT_Pin_7  | PORT_Pin_8;
TMR3PORT.PORT_FUNC = PORT_FUNC_OVERRID;
TMR3PORT.PORT_OE = PORT_OE_OUT;
TMR3PORT.PORT_SPEED = PORT_SPEED_FAST;
TMR3PORT.PORT_MODE = PORT_MODE_DIGITAL;
PORT_Init(MDR_PORTB,&TMR3PORT);


RST_CLK_PCLKcmd(RST_CLK_PCLK_TIMER3,ENABLE);
TIMER_DeInit (MDR_TIMER3);
TIMER_CntStructInit(&tm3);
TIMER_BRGInit(MDR_TIMER3,TIMER_HCLKdiv8);

tm3.TIMER_IniCounter = 0x0;
tm3.TIMER_Prescaler = 0x0;
//tm1.TIMER_CounterMode = TIMER_CntMode_ClkFixedDir;
//tm1.TIMER_CounterDirection = TIMER_CntDir_Up;
//tm1.TIMER_EventSource = TIMER_EvSrc_TM1 ;
tm3.TIMER_Period = pwm_period;
//tm1.TIMER_ARR_UpdateMode = TIMER_ARR_Update_On_CNT_Overflow;
//tm1.TIMER_FilterSampling = TIMER_FDTS_TIMER_CLK_div_1;
TIMER_CntInit(MDR_TIMER3,&tm3);

TIMER_ChnStructInit(&ch3);
ch3.TIMER_CH_Number = TIMER_CHANNEL1;
ch3.TIMER_CH_Mode = TIMER_CH_MODE_PWM;
ch3.TIMER_CH_REF_Format= TIMER_CH_REF_Format6;
TIMER_ChnInit(MDR_TIMER3,&ch3);

TIMER_ChnStructInit(&ch3);
ch3.TIMER_CH_Number = TIMER_CHANNEL2;
ch3.TIMER_CH_Mode = TIMER_CH_MODE_PWM;
ch3.TIMER_CH_REF_Format= TIMER_CH_REF_Format6;
TIMER_ChnInit(MDR_TIMER3,&ch3);

TIMER_ChnStructInit(&ch3);
ch3.TIMER_CH_Number = TIMER_CHANNEL3;
ch3.TIMER_CH_Mode = TIMER_CH_MODE_PWM;
ch3.TIMER_CH_REF_Format= TIMER_CH_REF_Format6;
TIMER_ChnInit(MDR_TIMER3,&ch3);

TIMER_ChnStructInit(&ch3);
ch3.TIMER_CH_Number = TIMER_CHANNEL4;
ch3.TIMER_CH_Mode = TIMER_CH_MODE_PWM;
ch3.TIMER_CH_REF_Format= TIMER_CH_REF_Format6;
TIMER_ChnInit(MDR_TIMER3,&ch3);




TIMER_ChnOutStructInit(&chout3);
chout3.TIMER_CH_DirOut_Polarity          = TIMER_CHOPolarity_NonInverted;
chout3.TIMER_CH_DirOut_Source            = TIMER_CH_OutSrc_REF;
chout3.TIMER_CH_DirOut_Mode              = TIMER_CH_OutMode_Output;
chout3.TIMER_CH_NegOut_Polarity          = TIMER_CHOPolarity_NonInverted;
chout3.TIMER_CH_NegOut_Source            = TIMER_CH_OutSrc_REF;
chout3.TIMER_CH_NegOut_Mode              = TIMER_CH_OutMode_Output;
chout3.TIMER_CH_Number                   = TIMER_CHANNEL1;
TIMER_ChnOutInit(MDR_TIMER3, &chout3);

TIMER_ChnOutStructInit(&chout3);
chout3.TIMER_CH_DirOut_Polarity          = TIMER_CHOPolarity_NonInverted;
chout3.TIMER_CH_DirOut_Source            = TIMER_CH_OutSrc_REF;
chout3.TIMER_CH_DirOut_Mode              = TIMER_CH_OutMode_Output;
chout3.TIMER_CH_NegOut_Polarity          = TIMER_CHOPolarity_NonInverted;
chout3.TIMER_CH_NegOut_Source            = TIMER_CH_OutSrc_REF;
chout3.TIMER_CH_NegOut_Mode              = TIMER_CH_OutMode_Output;
chout3.TIMER_CH_Number                   = TIMER_CHANNEL2;
TIMER_ChnOutInit(MDR_TIMER3, &chout3);

TIMER_ChnOutStructInit(&chout3);
chout3.TIMER_CH_DirOut_Polarity          = TIMER_CHOPolarity_NonInverted;
chout3.TIMER_CH_DirOut_Source            = TIMER_CH_OutSrc_REF;
chout3.TIMER_CH_DirOut_Mode              = TIMER_CH_OutMode_Output;
chout3.TIMER_CH_NegOut_Polarity          = TIMER_CHOPolarity_NonInverted ;
chout3.TIMER_CH_NegOut_Source            = TIMER_CH_OutSrc_REF;
chout3.TIMER_CH_NegOut_Mode              = TIMER_CH_OutMode_Output;
chout3.TIMER_CH_Number                   = TIMER_CHANNEL3;
TIMER_ChnOutInit(MDR_TIMER3, &chout3);

TIMER_ChnOutStructInit(&chout3);
chout3.TIMER_CH_DirOut_Polarity          = TIMER_CHOPolarity_NonInverted;
chout3.TIMER_CH_DirOut_Source            = TIMER_CH_OutSrc_REF;
chout3.TIMER_CH_DirOut_Mode              = TIMER_CH_OutMode_Output;
chout3.TIMER_CH_NegOut_Polarity          = TIMER_CHOPolarity_NonInverted;
chout3.TIMER_CH_NegOut_Source            = TIMER_CH_OutSrc_REF;
chout3.TIMER_CH_NegOut_Mode              = TIMER_CH_OutMode_Output;
chout3.TIMER_CH_Number                   = TIMER_CHANNEL4;
TIMER_ChnOutInit(MDR_TIMER3, &chout3);

//TIMER_ChnOutDTGConfig(MDR_TIMER3,TIMER_CHANNEL1,0x000F,0,TIMER_CH_DTG_ClkSrc_TIMER_CLK);
//TIMER_ChnOutDTGConfig(MDR_TIMER3,TIMER_CHANNEL2,0x000F,0,TIMER_CH_DTG_ClkSrc_TIMER_CLK);
//TIMER_ChnOutDTGConfig(MDR_TIMER3,TIMER_CHANNEL3,0x000F,0,TIMER_CH_DTG_ClkSrc_TIMER_CLK);
//TIMER_ChnOutDTGConfig(MDR_TIMER3,TIMER_CHANNEL4,0x000F,0,TIMER_CH_DTG_ClkSrc_TIMER_CLK);

TIMER_SetChnCompare(MDR_TIMER3, TIMER_CHANNEL1, 1000);
TIMER_SetChnCompare(MDR_TIMER3, TIMER_CHANNEL2, 1000);
TIMER_SetChnCompare(MDR_TIMER3, TIMER_CHANNEL3, 1000);
TIMER_SetChnCompare(MDR_TIMER3, TIMER_CHANNEL4, 1000);



}
void tm3_start_pwm(void)
{
		//NVIC_DisableIRQ(Timer3_IRQn ); //выключение прерывания
		TIMER_Cmd(MDR_TIMER3,ENABLE);	//Разрешение работы таймера
}
void tm3_stop_pwm(void)
{
		NVIC_DisableIRQ(Timer3_IRQn ); //включение прерывания
		TIMER_Cmd(MDR_TIMER3,DISABLE);	//Разрешение работы таймера
}


void led_init(void){
PORT_InitTypeDef LED; //= {PORT_Pin_10,PORT_OE_OUT,PORT_PULL_UP_OFF,PORT_PULL_DOWN_OFF,PORT_PD_SHM_OFF,PORT_PD_DRIVER,PORT_GFEN_OFF,PORT_FUNC_PORT,PORT_SPEED_MAXFAST,PORT_MODE_DIGITAL};

RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA,ENABLE);

PORT_StructInit (&LED);
LED.PORT_Pin = PORT_Pin_5;
LED.PORT_OE = PORT_OE_OUT;
LED.PORT_FUNC = PORT_FUNC_PORT;
LED.PORT_SPEED = PORT_SPEED_MAXFAST;
LED.PORT_MODE = PORT_MODE_DIGITAL;
PORT_Init(MDR_PORTA,&LED);

//RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB,ENABLE);

//PORT_StructInit (&LED);
//LED.PORT_Pin = PORT_Pin_4;
//LED.PORT_OE = PORT_OE_OUT;
//LED.PORT_FUNC = PORT_FUNC_PORT;
//LED.PORT_SPEED = PORT_SPEED_MAXFAST;
//LED.PORT_MODE = PORT_MODE_DIGITAL;
//PORT_Init(MDR_PORTB,&LED);

//RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTF,ENABLE);

//PORT_StructInit (&LED);
//LED.PORT_Pin = PORT_Pin_3|PORT_Pin_2;
//LED.PORT_OE = PORT_OE_OUT;
//LED.PORT_FUNC = PORT_FUNC_PORT;
//LED.PORT_SPEED = PORT_SPEED_MAXFAST;
//LED.PORT_MODE = PORT_MODE_DIGITAL;
//PORT_Init(MDR_PORTF,&LED);

//RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTE,ENABLE);

//PORT_StructInit (&LED);
//LED.PORT_Pin = PORT_Pin_0;
//LED.PORT_OE = PORT_OE_OUT;
//LED.PORT_FUNC = PORT_FUNC_PORT;
//LED.PORT_SPEED = PORT_SPEED_MAXFAST;
//LED.PORT_MODE = PORT_MODE_DIGITAL;
//PORT_Init(MDR_PORTE,&LED);

//RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTD,ENABLE);

//PORT_StructInit (&LED);
//LED.PORT_Pin = PORT_Pin_6 | PORT_Pin_7;
//LED.PORT_OE = PORT_OE_OUT;
//LED.PORT_FUNC = PORT_FUNC_PORT;
//LED.PORT_SPEED = PORT_SPEED_MAXFAST;
//LED.PORT_MODE = PORT_MODE_DIGITAL;
//PORT_Init(MDR_PORTD,&LED);


}

void DAP_init(void){
PORT_InitTypeDef DAP; //= {PORT_Pin_10,PORT_OE_OUT,PORT_PULL_UP_OFF,PORT_PULL_DOWN_OFF,PORT_PD_SHM_OFF,PORT_PD_DRIVER,PORT_GFEN_OFF,PORT_FUNC_PORT,PORT_SPEED_MAXFAST,PORT_MODE_DIGITAL};

RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB,ENABLE);

PORT_StructInit (&DAP);
DAP.PORT_Pin = PORT_Pin_4;
DAP.PORT_OE = PORT_OE_OUT;
DAP.PORT_FUNC = PORT_FUNC_PORT;
DAP.PORT_SPEED = PORT_SPEED_MAXFAST;
DAP.PORT_MODE = PORT_MODE_DIGITAL;
DAP.PORT_PD	= PORT_PD_DRIVER;
DAP.PORT_PULL_UP = PORT_PULL_UP_OFF ;
DAP.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF ;
PORT_Init(MDR_PORTB,&DAP);
PORT_SetBits(MDR_PORTB,PORT_Pin_4);


RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTC,ENABLE);
PORT_StructInit (&DAP);
DAP.PORT_Pin = PORT_Pin_0| PORT_Pin_1| PORT_Pin_2;
DAP.PORT_OE = PORT_OE_OUT;
DAP.PORT_FUNC = PORT_FUNC_PORT;
DAP.PORT_SPEED = PORT_SPEED_MAXFAST;
DAP.PORT_MODE = PORT_MODE_DIGITAL;
DAP.PORT_PD	= PORT_PD_DRIVER;
DAP.PORT_PULL_UP = PORT_PULL_UP_OFF ;
DAP.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF ;
PORT_Init(MDR_PORTC,&DAP);

PORT_SetBits(MDR_PORTC,PORT_Pin_0|PORT_Pin_1|PORT_Pin_2);
}


void LS_init(void){
PORT_InitTypeDef LS; 

RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTB,ENABLE);
PORT_StructInit (&LS);
LS.PORT_Pin = PORT_Pin_9|PORT_Pin_10;
LS.PORT_OE = PORT_OE_IN;
LS.PORT_FUNC = PORT_FUNC_PORT;
LS.PORT_SPEED = PORT_SPEED_MAXFAST;
LS.PORT_MODE = PORT_MODE_DIGITAL;
LS.PORT_PD	= PORT_PD_DRIVER;
LS.PORT_PULL_UP = PORT_PULL_UP_OFF ;
LS.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF ;
PORT_Init(MDR_PORTB,&LS);

RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA,ENABLE);
PORT_StructInit (&LS);
LS.PORT_Pin = PORT_Pin_0;
LS.PORT_OE = PORT_OE_IN;
LS.PORT_FUNC = PORT_FUNC_PORT;
LS.PORT_SPEED = PORT_SPEED_MAXFAST;
LS.PORT_MODE = PORT_MODE_DIGITAL;
LS.PORT_PD	= PORT_PD_DRIVER;
LS.PORT_PULL_UP = PORT_PULL_UP_OFF ;
LS.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF ;
PORT_Init(MDR_PORTA,&LS);

}

/*!
* одиночное чтение 32-битного слова из памяти по адресу _addr
*
* uint32_t eepromRead32bitWordOne(uint32_t _addr)
* \param uint32_t _addr - адрес ячейки памяти откуда читаем слово
* \return uint32_t - слово, прочитанное по адресу _addr
*/

void COMP_init(void){
PORT_InitTypeDef PCOMP;
COMP_InitTypeDef COMP;

RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTE,ENABLE);

PORT_StructInit (&PCOMP);
PCOMP.PORT_Pin = PORT_Pin_2| PORT_Pin_3;
PCOMP.PORT_OE = PORT_OE_IN;
//COMP.PORT_FUNC = PORT_FUNC_PORT;
PCOMP.PORT_SPEED = PORT_SPEED_MAXFAST;
PCOMP.PORT_MODE = PORT_MODE_ANALOG;
//COMP.PORT_PD	= PORT_PD_DRIVER;
PCOMP.PORT_PULL_UP = PORT_PULL_UP_OFF ;
PCOMP.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF ;
PORT_Init(MDR_PORTE,&PCOMP);

RST_CLK_PCLKcmd(RST_CLK_PCLK_COMP,ENABLE);


COMP_DeInit();

COMP_StructInit(&COMP);
COMP.COMP_PlusInputSource = COMP_PlusInput_IN1;
COMP.COMP_MinusInputSource = COMP_MinusInput_IN2;
COMP.COMP_OutInversion	= COMP_OUT_INV_Disable ;

COMP_Init(&COMP);
COMP_ITConfig(ENABLE);
NVIC_EnableIRQ(COMPARATOR_IRQn);
COMP_Cmd(ENABLE);
while (!(COMP_GetCfgFlagStatus(COMP_CFG_FLAG_READY)));
}


void ADC_IRQHandler(void)
{

	//ADC1_ClearOutOfRangeFlag();
	//current_sens_count++;



/*	ADC1_ClearOutOfRangeFlag();
	if (rotate_ctrl.pwr_control_allow == 1)
		{

			if(current_sens_count == 1)
				change_pwr(limit_pwr_once,CURRENT);
			else{
					if (((get_ADC())&0xFFF)>=(rotate_ctrl.limit_pwr*(23)))
						{
						led1;
						//ADC1_ITConfig(ADC1_IT_OUT_OF_RANGE,DISABLE);
						limit_pwr_once--;
						change_pwr(limit_pwr_once,CURRENT);
						ADC1_SetLowLevel(rotate_ctrl.limit_pwr*(22));
						//change_pwr(limit_pwr_once,LIMIT);
						dled1;
						}
					else if (((get_ADC())&0xFFF)<=(rotate_ctrl.limit_pwr*(22)))
						{
							led2;
						limit_pwr_once++;
						change_pwr(limit_pwr_once,CURRENT);
						dled2;
						}
				}

		}*/


/*	if (rotate_ctrl.pwr_control_allow == 1)
		{

		led1;

		//if (((ADC1_GetResult())&0xFFF)>(rotate_ctrl.limit_pwr*(22.7)))
			//current_sens_count++;

		if (current_sens_count >= 5)
		{
			led2;
		change_pwr(rotate_ctrl.limit_pwr,CURRENT);
		current_sens_count = 0;
			dled2;
		}

		//ADC1_ITConfig(ADC1_IT_OUT_OF_RANGE,DISABLE);
		dled1;
		}*/

}

void en_HSE(void)
{
  RST_CLK_HSEconfig(RST_CLK_HSE_ON);
  while(RST_CLK_HSEstatus() != SUCCESS);
  MDR_RST_CLK->CPU_CLOCK = 0x0102; /// HSE w/o dividers
}


/*
void EXT_IRQ_init(void){
PORT_InitTypeDef EXT_IRQ; 

RST_CLK_PCLKcmd(RST_CLK_PCLK_PORTA,ENABLE);

PORT_StructInit (&EXT_IRQ);
EXT_IRQ.PORT_Pin = PORT_Pin_0;
EXT_IRQ.PORT_OE = PORT_OE_IN;
EXT_IRQ.PORT_FUNC = PORT_FUNC_ALTER; 
EXT_IRQ.PORT_SPEED = PORT_SPEED_MAXFAST;
EXT_IRQ.PORT_MODE = PORT_MODE_DIGITAL;
EXT_IRQ.PORT_PULL_DOWN = PORT_PULL_DOWN_ON;
PORT_Init(MDR_PORTA,&EXT_IRQ);

NVIC_EnableIRQ(EXT_INT1_IRQn);

}
*/

void UART1_init(void )
{
PORT_InitTypeDef UART_Port;
  
  UART_Port.PORT_Pin = PORT_Pin_6;
  UART_Port.PORT_PULL_UP = PORT_PULL_UP_OFF;
	UART_Port.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
  UART_Port.PORT_OE = PORT_OE_IN;
  UART_Port.PORT_FUNC = PORT_FUNC_OVERRID;
 	UART_Port.PORT_SPEED = PORT_SPEED_MAXFAST;
	UART_Port.PORT_MODE = PORT_MODE_DIGITAL; 
	PORT_Init(MDR_PORTA, &UART_Port);	
	
	UART_Port.PORT_Pin = PORT_Pin_7;
	UART_Port.PORT_OE = PORT_OE_OUT;
	PORT_Init(MDR_PORTA, &UART_Port);	
	


  UART_DeInit(MDR_UART1);
  
  RST_CLK_PCLKcmd(RST_CLK_PCLK_UART1, ENABLE);
  
  UART_BRGInit(MDR_UART1, UART_HCLKdiv1);
  
  UART_InitTypeDef UART_IS;
  UART_IS.UART_BaudRate = 57600;
  UART_IS.UART_StopBits = UART_StopBits1;
  UART_IS.UART_Parity = UART_Parity_No;
  UART_IS.UART_WordLength  = UART_WordLength8b;
  UART_IS.UART_FIFOMode = UART_FIFO_ON;
	UART_IS.UART_HardwareFlowControl = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
	
  UART_Init(MDR_UART1, &UART_IS);

	//UART_ITConfig(MDR_UART1,UART_IT_RX,ENABLE);

  UART_Cmd(MDR_UART1,ENABLE);
	//NVIC_EnableIRQ(UART1_IRQn);
  
}

void UART2_init(void )
{
PORT_InitTypeDef UART_Port;
  
  UART_Port.PORT_Pin = PORT_Pin_0;
  UART_Port.PORT_PULL_UP = PORT_PULL_UP_OFF;
	UART_Port.PORT_PULL_DOWN = PORT_PULL_DOWN_OFF;
  UART_Port.PORT_OE = PORT_OE_IN;
  UART_Port.PORT_FUNC = PORT_FUNC_OVERRID;
	UART_Port.PORT_SPEED = PORT_SPEED_MAXFAST;
	UART_Port.PORT_MODE = PORT_MODE_DIGITAL;
	PORT_Init(MDR_PORTF, &UART_Port);
	
	
	UART_Port.PORT_Pin = PORT_Pin_1;
	UART_Port.PORT_OE = PORT_OE_OUT;
	PORT_Init(MDR_PORTF, &UART_Port);
	
  UART_DeInit(MDR_UART2);
  
  RST_CLK_PCLKcmd(RST_CLK_PCLK_UART2, ENABLE);
  
  UART_BRGInit(MDR_UART2, UART_HCLKdiv1);
  
  UART_InitTypeDef UART_IS;
  UART_IS.UART_BaudRate = 19200;
  UART_IS.UART_StopBits = UART_StopBits1;
  UART_IS.UART_Parity = UART_Parity_No;
  UART_IS.UART_WordLength  = UART_WordLength8b;
  UART_IS.UART_FIFOMode = UART_FIFO_OFF;
  UART_IS.UART_HardwareFlowControl = UART_HardwareFlowControl_RXE | UART_HardwareFlowControl_TXE;
  
  UART_Init(MDR_UART2, &UART_IS);

  UART_Cmd(MDR_UART2,ENABLE);
  
}


void peripheral_initialization(void)
{
	//IWDG_stop();
	//en_HSE();
	//RST_CLK_PCLKcmd(RST_CLK_PCLK_BKP, ENABLE);
	//RST_CLK_PCLKcmd(RST_CLK_PCLK_IWDG, ENABLE);
	//IWDG_init();
	//IWDG_start();
	
	//RST_CLK_PCLKcmd(RST_CLK_PCLK_EEPROM,ENABLE);
	//rele_init();
	led_init();
	tm3_init_pwm();
	tm3_start_pwm();
		
	//tm1_ppm_init();
		
	//tm2_init_inductive(); //не работает в ОО
		//tm2_start();

	//UART2_init();
	
	UART1_init();
	
	//ADC_init();
	//COMP_init(); //не работает в ОО
	//DAP_init();
	//can1_up();
	//LS_init();
	
	//tm2_start_inductive();
	__enable_irq();
	//NVIC_SetPriority(Timer1_IRQn, 0);
	//NVIC_SetPriority(Timer2_IRQn, 2);
	//NVIC_SetPriority(COMPARATOR_IRQn, 1);
	//NVIC_SetPriority(ADC_IRQn, 1);  // максимальный приоритет для прерывания ацп,
										//во избежание опасного нарастания тока

	//EXT_IRQ_init();
	//SysTick_init();
 
	NVIC_EnableIRQ(Timer1_IRQn ); //включение прерывания	
}


