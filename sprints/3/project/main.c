/*
 * Controlador de Frequência de Respiração
 *
 * Created: 4/21/2021 11:14:53 AM
 * Author : Nicolas Pereira
 */ 

#define F_CPU 16000000UL

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "nokia5110.h"
#include "nokia5110.c"

static volatile uint8_t freq_resp;

void atualiza_leds(void);
void atualiza_lcd(void);


ISR(INT0_vect)
{
	if(freq_resp < 30) freq_resp++;
	atualiza_lcd();
}

ISR(INT1_vect)
{
	if(freq_resp > 5) freq_resp--;
	atualiza_lcd();
}

ISR(TIMER1_OVF_vect)
{	
	TCNT1 = 49540;
	// Array auxiliar de ticks para contabilizar periodos de respiração (em milisegundos)
	const uint16_t tick_ms_arr[26] = {750, 625, 535, 468, 416, 375, 340, 312, 288, 267, 250, 234, 220,
	                                  208, 197, 187, 178, 170, 163, 156, 150, 144, 138, 133, 129, 125};	
	static uint16_t tick_counter = 0;
	
	uint8_t norm_freq = freq_resp-5;	
	if (++tick_counter >= tick_ms_arr[norm_freq]){
		tick_counter = 0;
		atualiza_leds();
	}
	
}

void atualiza_leds(void){
	static uint8_t subindo = 1;
	if (subindo){
		if (PORTB==0xFF){
			subindo = 0;
			PORTB = PORTB>>1;
		}
		else
		PORTB = (PORTB<<1)|0x01;
	}
	else{
		if (PORTB==0x00){
			subindo = 1;
			PORTB = (PORTB<<1)|0x01;
		}
		else
		PORTB = PORTB>>1;
	}
}

void atualiza_lcd (void)
{
	char display_str[20];
	sprintf(display_str, "%02d resp/min", freq_resp);
	//nokia_lcd_clear();
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string(display_str, 1);
	nokia_lcd_render();
}

void setup(void)
{
	// Configuração de pinos
	DDRB = 0xFF;
	PORTB = 0x00;
	DDRC = 0xFF;
	PORTC = 0x00;
	DDRD = 0x00;
	PORTD = 0xFF;
	
	// Configuração de INT0 e INT1
	EICRA = 0x0A;
	EIMSK = 0x03;	
	
	// Configuração do timer1 para 1ms
	TCNT1 = 49540;
	TCCR1A = 0x00;
	TCCR1B = (1<<CS10);
	TIMSK1 |= (1<<TOIE1);
	
	// Ativação de interrupções
	sei();
	
	// Inicialização de váriaveis
	freq_resp = 15;
	
	// Configuração de LCD
	nokia_lcd_init();
	nokia_lcd_clear();	
	atualiza_lcd();
}
 
int main(void)
{
	setup();
	
    while (1){}
}

