/* STM Projet 2, Teamt 13: HD44780 Alfanumeryczny; Piotr Knutel*/

#include "stm32f10x.h"
#include <stdio.h>
#include <stdbool.h>

#define pinRS GPIO_Pin_7
#define pinRW GPIO_Pin_9
#define pinE  GPIO_Pin_8
#define pinD0 GPIO_Pin_6
#define pinD1 GPIO_Pin_10
#define pinD2 GPIO_Pin_10
#define pinD3 GPIO_Pin_13
#define pinD4 GPIO_Pin_14
#define pinD5 GPIO_Pin_15
#define pinD6 GPIO_Pin_11
#define pinD7 GPIO_Pin_12

#define portRS GPIOB
#define portRW GPIOB
#define portE  GPIOB
#define portD0 GPIOB
#define portD1 GPIOB
#define portD2 GPIOA
#define portD3 GPIOB
#define portD4 GPIOB
#define portD5 GPIOB
#define portD6 GPIOB
#define portD7 GPIOB

#define pinP1 GPIO_Pin_13
#define pinP2 GPIO_Pin_14
#define pinP3 GPIO_Pin_15

#define portP GPIOA

#define function_set 		0b00111000
#define	display_on_off 	0b00001100
#define	entry_mode_set	0b00000110
#define clear_display		0b00000001
#define return_home			0b00000010

#define kursor_prawo		0b00010100
#define kursor_lewo			0b00010100
#define tekst_prawo			0b00011100
#define tekst_lewo			0b00011000

#define adres_poczatku_dolnej_lini	0x40

unsigned int licznik_opoznienia;
unsigned char p1, p2, p3;
bool flaga_wlaczenia_obslugi_BF = 0;	//jesli 1 to sprawdza flage zajetosci BF, jesli 0 to czeka staly czas; zmienna uzywana w funkcji wyzwolenie()

void opoznienie(uint32_t ileRazy)
{
	for(uint32_t i=0; i < ileRazy*4000; i++);
}

void opoznienie_ms(unsigned int ile)
{
	licznik_opoznienia = ile*100;
	while(licznik_opoznienia != 0);
}

void opoznienie_10us(unsigned int ile)
{
	licznik_opoznienia = ile;
	while(licznik_opoznienia != 0); 
}

int SysTick_Konfiguracja(unsigned int Ticks) //Szumski M. "Mikrokontrolery STM32 w systemach sterowania i regulacji" str. 193
{
	unsigned int ctrl;
	if(Ticks > SysTick_LOAD_RELOAD_Msk) return 1;
	SysTick->LOAD = (Ticks & SysTick_LOAD_RELOAD_Msk) - 1; 			//LOAD = Ticks - 1
	NVIC_SetPriority(SysTick_IRQn, 0); 													//najwyzszy piorytet
	SysTick->VAL = 0; 																					//aktualna wartosc licznika
	ctrl = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk; 	//ustaw bity
	ctrl &= SysTick_CLKSource_HCLK_Div8; 												//podzial czestostliwsci AHB/8
	SysTick->CTRL = ctrl; 																			//start licznika SysTick
	return 0;
}

void SysTick_Handler(void)
{
	if(licznik_opoznienia != 0)
	{
		licznik_opoznienia--;
	}
}

void usart2_Konfiguracja(void)
{
	GPIO_InitTypeDef gpioU2;
 
	GPIO_StructInit(&gpioU2);
	gpioU2.GPIO_Pin = GPIO_Pin_2;
	gpioU2.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &gpioU2);
 
	gpioU2.GPIO_Pin = GPIO_Pin_3;
	gpioU2.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &gpioU2);

	USART_InitTypeDef uart2;
	USART_StructInit(&uart2);
	uart2.USART_BaudRate = 57600;
	USART_Init(USART2, &uart2);
	USART_Cmd(USART2, ENABLE);
}

void usart2WyslijBajt(char c)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
    USART_SendData(USART2, c);
}

void usart2WyslijCiag(char* s)
{
    while (*s)
        usart2WyslijBajt(*s++);
}

void przyciski_Konfiguracja(void)
{
	GPIO_InitTypeDef przyciski_structure ;
	przyciski_structure.GPIO_Pin = pinP1|pinP2|pinP3;
	przyciski_structure.GPIO_Mode = GPIO_Mode_AF_OD;
	przyciski_structure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portP, &przyciski_structure);
	p1 = 0;
	p2 = 0;
	p3 = 0;
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource13);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource14);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);
	
	EXTI_InitTypeDef exti;
	EXTI_StructInit(&exti);
	exti.EXTI_Line = EXTI_Line13|EXTI_Line14|EXTI_Line15;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;
	exti.EXTI_Trigger = EXTI_Trigger_Falling;
	exti.EXTI_LineCmd = ENABLE;
	EXTI_Init(&exti);
	
	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = EXTI15_10_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0x0F;
	nvic.NVIC_IRQChannelSubPriority = 0x0F;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
}

void EXTI15_10_IRQHandler()
{
	if (EXTI_GetITStatus(EXTI_Line13) != RESET)
	{
		p1 = 1;
		opoznienie(1000);
		EXTI_ClearITPendingBit(EXTI_Line13);
  }
	if (EXTI_GetITStatus(EXTI_Line14) != RESET)
	{
		p2 = 1;
    EXTI_ClearITPendingBit(EXTI_Line14);
  }
	if (EXTI_GetITStatus(EXTI_Line15) != RESET)
	{
		p3 = 1;
    EXTI_ClearITPendingBit(EXTI_Line15);
  }
}

void lcd_D_jako_wyjscia(void)
{
	GPIO_InitTypeDef d0_Struct;
	d0_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	d0_Struct.GPIO_Pin = pinD0;
	d0_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD0, &d0_Struct);
	
	GPIO_InitTypeDef d1_Struct;
	d1_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	d1_Struct.GPIO_Pin = pinD1;
	d1_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD1, &d1_Struct);

	GPIO_InitTypeDef d2_Struct;
	d2_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	d2_Struct.GPIO_Pin = pinD2;
	d2_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD2, &d2_Struct);

 	GPIO_InitTypeDef d3_Struct;
	d3_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	d3_Struct.GPIO_Pin = pinD3;
	d3_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD3, &d3_Struct);
	
 	GPIO_InitTypeDef d4_Struct;
	d4_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	d4_Struct.GPIO_Pin = pinD4;
	d4_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD4, &d4_Struct);
	
 	GPIO_InitTypeDef d5_Struct;
	d5_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	d5_Struct.GPIO_Pin = pinD5;
	d5_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD5, &d5_Struct);
	
 	GPIO_InitTypeDef d6_Struct;
	d6_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	d6_Struct.GPIO_Pin = pinD6;
	d6_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD6, &d6_Struct);
	
 	GPIO_InitTypeDef d7_Struct;
	d7_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	d7_Struct.GPIO_Pin = pinD7;
	d7_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD7, &d7_Struct);
}

void lcd_D_jako_wejscia(void)
{
	GPIO_InitTypeDef d0_Struct;
	d0_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	d0_Struct.GPIO_Pin = pinD0;
	d0_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD0, &d0_Struct);
	
	GPIO_InitTypeDef d1_Struct;
	d1_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	d1_Struct.GPIO_Pin = pinD1;
	d1_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD1, &d1_Struct);

	GPIO_InitTypeDef d2_Struct;
	d2_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	d2_Struct.GPIO_Pin = pinD2;
	d2_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD2, &d2_Struct);

 	GPIO_InitTypeDef d3_Struct;
	d3_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	d3_Struct.GPIO_Pin = pinD3;
	d3_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD3, &d3_Struct);
	
 	GPIO_InitTypeDef d4_Struct;
	d4_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	d4_Struct.GPIO_Pin = pinD4;
	d4_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD4, &d4_Struct);
	
 	GPIO_InitTypeDef d5_Struct;
	d5_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	d5_Struct.GPIO_Pin = pinD5;
	d5_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD5, &d5_Struct);
	
 	GPIO_InitTypeDef d6_Struct;
	d6_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	d6_Struct.GPIO_Pin = pinD6;
	d6_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD6, &d6_Struct);
	
 	GPIO_InitTypeDef d7_Struct;
	d7_Struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	d7_Struct.GPIO_Pin = pinD7;
	d7_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portD7, &d7_Struct);
}

void e(bool stan)
{
	if(stan)
		GPIO_SetBits(portE, pinE);
	else
		GPIO_ResetBits(portE, pinE);
}

void wyzwolenie(void)
{
	e(1);
	opoznienie_10us(10);
	e(0);
}


void lcd_ustaw_bajt(uint8_t bajt)
{
	bool d0, d1, d2, d3, d4, d5, d6, d7;
	d0 = (bool)(bajt&0b00000001);
	d1 = (bool)((bajt&0b00000010)>>1);
	d2 = (bool)((bajt&0b00000100)>>2);
	d3 = (bool)((bajt&0b00001000)>>3);
	d4 = (bool)((bajt&0b00010000)>>4);
	d5 = (bool)((bajt&0b00100000)>>5);
	d6 = (bool)((bajt&0b01000000)>>6);
	d7 = (bool)((bajt&0b10000000)>>7);
	
	if(d0)
		GPIO_SetBits(portD0, pinD0);
	else
		GPIO_ResetBits(portD0, pinD0);
	if(d1)
		GPIO_SetBits(portD1, pinD1);
	else
		GPIO_ResetBits(portD1, pinD1);
	if(d2)
		GPIO_SetBits(portD2, pinD2);
	else
		GPIO_ResetBits(portD2, pinD2);
	if(d3)
		GPIO_SetBits(portD3, pinD3);
	else
		GPIO_ResetBits(portD3, pinD3);
	if(d4)
		GPIO_SetBits(portD4, pinD4);
	else
		GPIO_ResetBits(portD4, pinD4);
	if(d5)
		GPIO_SetBits(portD5, pinD5);
	else
		GPIO_ResetBits(portD5, pinD5);
	if(d6)
		GPIO_SetBits(portD6, pinD6);
	else
		GPIO_ResetBits(portD6, pinD6);
	if(d7)
		GPIO_SetBits(portD7, pinD7);
	else
		GPIO_ResetBits(portD7, pinD7);
}

void lcd_rozkaz(uint8_t rozkaz)
{
	GPIO_ResetBits(portRW, pinRW);
	GPIO_ResetBits(portRS, pinRS);
	opoznienie_10us(10);
	lcd_ustaw_bajt(rozkaz); 
	wyzwolenie();
	opoznienie_10us(10);
}

void lcd_wyslij_dane(uint8_t dane)
{
	GPIO_ResetBits(portRW, pinRW);
	GPIO_SetBits(portRS, pinRS);
	opoznienie_10us(10);
	lcd_ustaw_bajt(dane);
	wyzwolenie();
	opoznienie_10us(10);
}

void lcd_Konfiguracja()
{
	GPIO_InitTypeDef rs_Struct;
	rs_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	rs_Struct.GPIO_Pin = pinRS;
	rs_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portRS, &rs_Struct); 
	
	GPIO_InitTypeDef rw_Struct;
	rw_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	rw_Struct.GPIO_Pin = pinRW;
	rw_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portRW, &rw_Struct);
	
	GPIO_InitTypeDef e_Struct;
	e_Struct.GPIO_Mode = GPIO_Mode_Out_PP;
	e_Struct.GPIO_Pin = pinE;
	e_Struct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(portE, &e_Struct);
	
	lcd_D_jako_wyjscia();
	opoznienie_ms(30);
	GPIO_SetBits(portE, pinE);
	
	lcd_rozkaz(function_set);
	lcd_rozkaz(display_on_off);
	lcd_rozkaz(entry_mode_set);
	lcd_rozkaz(clear_display);
	lcd_rozkaz(return_home);
	opoznienie_ms(2);
}
void lcd_wyslij_ciag(char *s)
{
	while (*s)
		lcd_wyslij_dane(*s++);
}

void lcd_ustaw_adres_DD(uint8_t adres)
{
	if(adres <= 0x27 || (adres >= 0x40 && adres <= 0x67))
		lcd_rozkaz(0b10000000 | adres);
}

int main()
{
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
	SysTick_Konfiguracja(90ul);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC|RCC_APB2Periph_GPIOD, ENABLE);
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	//przyciski_Konfiguracja();
	//usart2_Konfiguracja();
	lcd_Konfiguracja();
	lcd_wyslij_ciag("HD44780 jest ok!");
	lcd_ustaw_adres_DD(adres_poczatku_dolnej_lini);
	lcd_wyslij_dane(48); //wyswiela zero
	
		while(1)
	{
	
	}
}
