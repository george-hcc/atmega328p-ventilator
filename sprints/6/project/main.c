/*
 * Controlador de Frequência de Respiração
 *
 * Created: 4/22/2021 06:52:57 AM
 * Author : Nicolas Pereira
 */ 

#define F_CPU 16000000UL
#define TIMER1_ZERO 49540
#define TIMER0_ZERO 0
#define BAUD 9600
#define MYUBBR F_CPU/16/BAUD-1

#define TRUE 1
#define FALSE 0

// Valores calculados previamente para converter leituras do ADC
#define TEMP_PARAM_A 15.0/307.0
#define TEMP_PARAM_B 3060.0/307.0
#define SPO2_PARAM_A 100.0/819.0

#include <stdio.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "nokia5110.h"
#include "nokia5110.c"

static volatile uint8_t freq_resp;
static volatile uint8_t bpm;
static volatile uint16_t temp_corporal_t10; // Temperatura corporal multiplicada por 10
static volatile uint8_t spO2;
static volatile char pressao_sis_dia [7];
static volatile uint32_t tempo_ms; //32 bits são suficientes para medir 49 dias de funcionamento

void atualiza_leds(void);
void atualiza_lcd(void);
void atualiza_pressao(char*);
uint8_t pressao_eh_valida(char*);

// Aumenta frequência do respirador
ISR(INT0_vect)
{
	if(freq_resp < 30) freq_resp++;
}

// Diminui frequência do respirador
ISR(INT1_vect)
{
	if(freq_resp > 5) freq_resp--;
}

// Interrupção do oscilador de batimentos
ISR(PCINT2_vect)
{
	// Lógica de atualização e calculo do valor de bpm
	static uint32_t bat_ms_anterior = 0;
	if (PIND & 0x40){
		bpm = (uint8_t) (60000/(tempo_ms - bat_ms_anterior));
		bat_ms_anterior = tempo_ms;
	}
}

// Evento de timer a cada 1ms - Contador de tempo e controle do ventilador
ISR(TIMER1_OVF_vect)
{	
	TCNT1 = TIMER1_ZERO;
	tempo_ms++;
	
	// Array auxiliar de ticks para contabilizar periodos de respiração (em milisegundos)
	const uint16_t tick_ms_arr[26] = {750, 625, 535, 468, 416, 375, 340, 312, 288, 267, 250, 234, 220,
	                                  208, 197, 187, 178, 170, 163, 156, 150, 144, 138, 133, 129, 125};
	// Armazena tempo em ms do ultimo tick do led
	static uint32_t tick_ms_anterior = 0;
	
	// Lógica de atualização dos leds
	uint8_t norm_freq = freq_resp-5;
	uint32_t ms_diff = tempo_ms - tick_ms_anterior;
	if (ms_diff >= tick_ms_arr[norm_freq]){
		tick_ms_anterior = tempo_ms;
		atualiza_leds();
	}	
}

// Evento de timer a cada 15ms - Usado para iniciar medições do ADC
ISR (TIMER0_OVF_vect)
{
	TCNT0 = TIMER0_ZERO;
	static uint8_t ten_counter = 0;
	static uint8_t measure_C0 = TRUE;
	
	
	if (++ten_counter == 10){
		ten_counter = 0;
		//Lógica de inicialização do conversor ADC alternando dois canais
		if (measure_C0){
			measure_C0 = FALSE;
			ADMUX = (ADMUX & 0xF0) | 0x00;
			ADCSRA |= 0x40;
			
		}
		else{
			measure_C0 = TRUE;
			ADMUX = (ADMUX & 0xF0) | 0x01;
			ADCSRA |= 0x40;
		}
		
	}
}

// Interrupção de conclusão de leitura do ADC
ISR(ADC_vect)
{
	uint16_t adc_result = (ADCH<<8) | ADCL;
	uint8_t canal_c0 = ((ADMUX & 0x0F) == 0x00);
	if (canal_c0){
		if (adc_result < 410) adc_result = 410;
		if (adc_result > 717) adc_result = 717;
		temp_corporal_t10 = (int) 10*(TEMP_PARAM_A*adc_result + TEMP_PARAM_B);
	}
	else{
		if (adc_result > 819) adc_result = 819;
		spO2 = (int) (SPO2_PARAM_A*adc_result);
	}
}

// Leitura de pressão através da UART
ISR(USART_RX_vect)
{
	static uint8_t wait_start = TRUE;
	static char msg [7];
	static uint8_t msg_ptr = 0;
	
	char rx_byte = UDR0;
	// Esperando inicio da mensagem
	if (wait_start){
		if (rx_byte == ';') wait_start=FALSE;
	}
	// Protocolando o fim da mensagem
	else if (msg_ptr == 7){
		if (rx_byte == ':' && pressao_eh_valida(msg)) 
			atualiza_pressao(msg);
		else                
			atualiza_pressao("#ERROR#");
		wait_start=TRUE;
		msg_ptr = 0;
	}
	// Construindo mensagem
	else{
		if (rx_byte == ':') {
			atualiza_pressao("#ERROR#");
			wait_start=TRUE;
			msg_ptr = 0;
		}
		else if (rx_byte == ';'){
			atualiza_pressao("#ERROR#");
			msg_ptr = 0;
		}
		else{
			msg[msg_ptr] = rx_byte;
			msg_ptr++;
		}
	}
}

// Lógica do ventilador
void atualiza_leds(void){
	static uint8_t subindo = TRUE;
	if (subindo){
		if (PORTB==0xFF){
			subindo = FALSE;
			PORTB = PORTB>>1;
		}
		else
		PORTB = (PORTB<<1)|0x01;
	}
	else{
		if (PORTB==0x00){
			subindo = TRUE;
			PORTB = (PORTB<<1)|0x01;
		}
		else
		PORTB = PORTB>>1;
	}
}

void atualiza_lcd (void)
{
	char display_str[15];
	sprintf(display_str, "%02d      resp/min", freq_resp);
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%03d     bpm", bpm);
	nokia_lcd_set_cursor(0, 10);
	nokia_lcd_write_string(display_str, 1);
	nokia_lcd_render();
	
	sprintf(display_str, "%d.%d    *C", temp_corporal_t10 / 10, temp_corporal_t10 % 10);
	nokia_lcd_set_cursor(0, 20);
	nokia_lcd_write_string(display_str, 1);
	nokia_lcd_render();
	
	sprintf(display_str, "%03d     %%SpO2", spO2);
	nokia_lcd_set_cursor(0, 30);
	nokia_lcd_write_string(display_str, 1);
	nokia_lcd_render();
	
	sprintf(display_str, "%.7s %%mmHg", pressao_sis_dia);
	nokia_lcd_set_cursor(0, 40);
	nokia_lcd_write_string(display_str, 1);
	nokia_lcd_render();
}

void atualiza_pressao(char* str_sete){
	for(int i = 0; i < 7; i++){
		pressao_sis_dia[i] = str_sete[i];
	}	
}

uint8_t pressao_eh_valida(char* str_sete){
	if (str_sete[3] != 'x') return FALSE;
	for(int i = 0; i < 3; i++){
		if (!isdigit(str_sete[i])) return FALSE;
		if (!isdigit(str_sete[i+4])) return FALSE;
	}
	return TRUE;
}

void setup(void)
{
	// Configuração de pinos
	DDRB = 0xFF;
	PORTB = 0x00;
	DDRC = 0xFC;
	PORTC = 0x00;
	DDRD = 0x80;
	PORTD = 0x7F;
	
	// Configuração de INT0 e INT1
	EICRA = 0x0A;
	EIMSK = 0x03;
	
	// Configuração de PCINT16 (porta D4)
	PCICR = (1<<PCIE2);
	PCMSK2 = (1<<PCINT22);
	
	// Configuração do timer1 para 1ms
	TCNT1 = TIMER1_ZERO;
	TCCR1A = 0x00;
	TCCR1B = (1<<CS10);
	TIMSK1 = (1<<TOIE1);
	
	// Configuração do timer0 para 15ms
	TCNT0 = TIMER0_ZERO;
	TCCR0A = 0x00;
	TCCR0B = (1<<CS02)|(1<<CS00);
	TIMSK0 = (1<<TOIE0);
	
	// Configuração do ADC
	ADMUX = 0x40;
	ADCSRA = 0xCF;
	ADCSRB = 0x00;
	DIDR0 = 0b00111100;
	
	// Configuração da UART
	UBRR0L = (MYUBBR) & 0xFF;
	UBRR0H = (MYUBBR) >> 8;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0); // Enable de RX com interrupção
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); // Assíncrono de 8bits sem paridade
	
	// Ativação de interrupções
	sei();
	
	// Inicialização de váriaveis
	freq_resp = 15;
	tempo_ms = 0;
	temp_corporal_t10 = 350;
	spO2 = 50;
	atualiza_pressao("000x000");
	
	// Configuração de LCD
	nokia_lcd_init();
	nokia_lcd_clear();	
	atualiza_lcd();
}
 
int main(void)
{
	setup();
	
    while (1){
		_delay_ms(200);
		atualiza_lcd();
		if ((temp_corporal_t10 < 410 && temp_corporal_t10 > 350) | (spO2 < 60)){
			PORTD = (PORTD & 0x7F) | 0x80;
		}
		else{
			PORTD = (PORTD & 0x7F);
		}
	}
}