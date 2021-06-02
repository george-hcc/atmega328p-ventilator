/*
 * Controlador de Frequência de Respiração
 *
 * Created: 4/21/2021 6:05:21 AM
 * Author : George Camboim
 */ 

#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

static volatile uint8_t freq_resp;

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

ISR(TIMER1_OVF_vect)
{
	// Array auxiliar de ticks para contabilizar periodos de respiração (em milisegundos)
	const uint16_t tick_ms_arr[26] = {750, 625, 535, 468, 416, 375, 340, 312, 288, 267, 250, 234, 220,
	                                  208, 197, 187, 178, 170, 163, 156, 150, 144, 138, 133, 129, 125};	
	static uint16_t tick_counter = 0;
	
	uint8_t norm_freq = freq_resp-5;	
	if (++tick_counter >= tick_ms_arr[norm_freq]){
		tick_counter = 0;
		atualiza_leds();
	}
	TCNT1 = 49555;
}

void setup(void)
{
	// Configuração de pinos
	DDRB = 0xFF;
	PORTB = 0x00;
	DDRC = 0xFF;
	DDRD = 0x00;
	PORTD = 0xFF;
	
	// Configuração do timer1 para 1ms
	TCNT1 = 49555;
	TCCR1A = 0x00;
	TCCR1B = (1<<CS10);
	TIMSK1 |= (1<<TOIE1);
	
	sei();
}
 
int main(void)
{
	setup();
	// Declaração de variáveis
	freq_resp = 15;
	uint8_t aumenta_freq, diminui_freq;
	uint8_t aumenta_anterior = (~PIND) & 0x01;
	uint8_t diminui_anterior = (~PIND) & 0x02;
	
	// Loop principal
    while (1){
		// Lê entrada
		aumenta_freq = (~PIND) & 0x01;
		diminui_freq = (~PIND) & 0x02;
		// Aumentar Frequência		
		if (aumenta_freq && !aumenta_anterior){
			if(freq_resp < 30) freq_resp++;
		}
		// Diminui Frequência
		else if (diminui_freq && !diminui_anterior){
			if(freq_resp > 5) freq_resp--;
		}
		// Atualiza valores anteriores
		aumenta_anterior = aumenta_freq;
		diminui_anterior = diminui_freq;
		// Atualiza saída e espera
		PORTC = freq_resp;
    }
}

