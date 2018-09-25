/*
 * peripheral.h
 *
 *  Created on: 23 янв. 2015 г.
 *      Author: nnhw
 */

#ifndef PERIPHERAL_H_
#define PERIPHERAL_H_



//void ADC_init(void);
void peripheral_initialization(void);
//void rele_switch_out(void);
//void rele_switch_in(void);
//void en_HSE(void);
//void COMP_init(void);
//void LS_init(void);
//void DAP_init(void);
//void led_init(void);
void tm3_stop_pwm(void);
void tm3_start_pwm(void);
//void tm3_init(void);
void tm2_stop_inductive(void);
void tm2_start_inductive(void);
//void tm2_init(void);
void tm1_ppm_stop(void);
void tm1_ppm_start(void);
//void tm1_init(void);
void SysTick_stop(void);
void SysTick_start(void);
void IWDG_reset(void);
void IWDG_stop(void);
void IWDG_start(void);
void IWDG_init(void);

//включение диодов
#define led1 MDR_PORTA->RXTX |= 1<<5
#define led2 MDR_PORTB->RXTX |= 1<<4
#define led3 MDR_PORTF->RXTX |= 1<<2
#define led4 MDR_PORTF->RXTX |= 1<<3
#define led5 MDR_PORTE->RXTX |= 1<<0
#define led_all  MDR_PORTB->RXTX |=  31

#define xled1 MDR_PORTA->RXTX ^= 1<<5
#define xled2 MDR_PORTB->RXTX ^= 1<<4
#define xled3 MDR_PORTF->RXTX ^= 1<<2
#define xled4 MDR_PORTF->RXTX ^= 1<<3

//выключение диодов
#define dled1 MDR_PORTA->RXTX &= ~(1<<5)
#define dled2 MDR_PORTB->RXTX &= ~(1<<4)
#define dled3 MDR_PORTF->RXTX &= ~(1<<2)
#define dled4 MDR_PORTF->RXTX &= ~(1<<3)
#define dled5 MDR_PORTE->RXTX &= ~(1<<0)
#define dled_all MDR_PORTB->RXTX &= ~(31)

#define COMP1 MDR_PORTD->RXTX |= (1<<6)
#define COMP2 MDR_PORTD->RXTX |= (1<<7)
#define DCOMP1 MDR_PORTD->RXTX &= ~(1<<6)
#define DCOMP2 MDR_PORTD->RXTX &= ~(1<<7)


#define START_EEPROM 0x08000000

#define EEPROM_REG_ACCESS_KEY ((uint32_t)0x8AAA5551)

//uint32_t flash_adress;

uint32_t eepromRead32bitWordOne(uint32_t _addr);
void eepromWrite32bitWordOne(uint32_t _addr, uint32_t _data);

void pause(uint32_t _maxticks);

#endif /* PERIPHERAL_H_ */
