/*
 * Controlador de Frequência de Respiração
 *
 * Created: 5/22/2021 01:13:13 AM
 * Author : George Camboim
 */ 

// Defines de configuração de valores auxiliares
#define F_CPU 16000000UL
#define TIMER0_ZERO 193
#define TIMER1_ZERO 0
#define TIMER2_ZERO 0
#define BAUD 9600
#define MYUBBR F_CPU/16/BAUD-1

#define TRUE 1
#define FALSE 0
#define STATE_FREQ_RESP 0
#define STATE_VAL_O2 1
#define STATE_VOLUME 2
#define STATE_SETTINGS 3
#define STATE_SETT_FREQ 4
#define STATE_SETT_O2 5
#define STATE_SETT_VOL 6
#define STATE_MONITOR 7

// Valores calculados previamente para converter leituras do ADC
#define TEMP_PARAM_A 15.0/307.0
#define TEMP_PARAM_B 3060.0/307.0
#define SPO2_PARAM_A 100.0/819.0

// Include de bibliotecas
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "nokia5110.h"
#include "nokia5110.c"

// Declaração de váriaveis globais
static volatile uint8_t freq_resp;
static volatile uint8_t percent_O2;
static volatile uint8_t volume;

static volatile uint8_t bpm;
static volatile uint16_t temp_corporal_t10; // Temperatura corporal multiplicada por 10
static volatile uint8_t spO2;
static volatile char pressao_sis_dia [7];

static volatile uint8_t estado_display;
static volatile uint8_t limpar_display;

static volatile uint32_t tempo_ms; //32 bits são suficientes para medir 49 dias de funcionamento

// Variaveis relacionadas a transmissão UART
static volatile char tx_msg[22];
static volatile uint8_t tx_ptr;

// Variaveis relacionadas a configuração de parametros
static volatile uint8_t uart_freq_resp;
static volatile uint8_t uart_o2;
static volatile uint8_t uart_volume;

// Declaração de funções auxiliares
void inicia_tx(void);
void atua_ventilador (void);
void atua_valvula (void);

void atualiza_lcd (void);
void atualiza_lcd_0 (void);
void atualiza_lcd_1 (void);
void atualiza_lcd_2 (void);
void atualiza_lcd_3 (void);
void atualiza_lcd_4 (void);
void atualiza_lcd_5 (void);
void atualiza_lcd_6 (void);
void atualiza_lcd_7 (void);

void atualiza_pressao (char*);
uint8_t pressao_eh_valida (char*);
void atualiza_comando (char*);
uint8_t comando_eh_valido (char*);

// Interrupções por botões de controle
ISR (PCINT0_vect)
{
	// Caso botão "-" tenha sido pressionado
	if ((PINB & 0x40) == 0x00){
		if (estado_display == STATE_FREQ_RESP && !uart_freq_resp){
			if(freq_resp > 5) freq_resp--;
		}
		else if (estado_display ==  STATE_VAL_O2 && !uart_o2){
			if (percent_O2 > 0) percent_O2 -= 10;
			atua_valvula();
		}
		else if (estado_display ==  STATE_VOLUME && !uart_volume){
			if (volume > 0) volume -= 1;
		}
		else if (estado_display == STATE_SETTINGS){
			estado_display = STATE_SETT_FREQ;
			limpar_display = TRUE;
		}
		else if (estado_display == STATE_SETT_FREQ){
			uart_freq_resp = !uart_freq_resp;
			limpar_display = TRUE;
		}
		else if (estado_display == STATE_SETT_O2){
			uart_o2 = !uart_o2;
			limpar_display = TRUE;
		}
		else if (estado_display == STATE_SETT_VOL){
			uart_volume = !uart_volume;
			limpar_display = TRUE;
		}
	}
	// Caso botão "+" tenha sido pressionado
	if ((PINB & 0x80) == 0x00){
		if (estado_display == STATE_FREQ_RESP && !uart_freq_resp){
			if(freq_resp < 30) freq_resp++;
		}
		else if (estado_display ==  STATE_VAL_O2 && !uart_o2){
			if (percent_O2 < 100) percent_O2 += 10;
			atua_valvula();
		}
		else if (estado_display ==  STATE_VOLUME && !uart_volume){
			if (volume < 8) volume += 1;
		}
		else if (estado_display == STATE_SETTINGS){
			estado_display = STATE_SETT_FREQ;
			limpar_display = TRUE;
		}
		else if (estado_display == STATE_SETTINGS){
			estado_display = STATE_SETT_FREQ;
			limpar_display = TRUE;
		}
		else if (estado_display == STATE_SETT_FREQ){
			uart_freq_resp = !uart_freq_resp;
			limpar_display = TRUE;
		}
		else if (estado_display == STATE_SETT_O2){
			uart_o2 = !uart_o2;
			limpar_display = TRUE;
		}
		else if (estado_display == STATE_SETT_VOL){
			uart_volume = !uart_volume;
			limpar_display = TRUE;
		}
	}
	// Caso botão "Sel" tenha sido pressionado
	if ((PINB & 0x01) == 0x00){
		if (estado_display == STATE_MONITOR)
			estado_display = STATE_FREQ_RESP;
		else if (estado_display == STATE_FREQ_RESP)
			estado_display = STATE_VAL_O2;
		else if (estado_display ==  STATE_VAL_O2)
			estado_display = STATE_VOLUME;
		else if (estado_display ==  STATE_VOLUME || estado_display == STATE_SETT_VOL)
			estado_display = STATE_SETTINGS;
		else if (estado_display ==  STATE_SETT_FREQ)
			estado_display = STATE_SETT_O2;
		else if (estado_display ==  STATE_SETT_O2)
			estado_display = STATE_SETT_VOL;
		else if (estado_display ==  STATE_SETTINGS)
			estado_display = STATE_MONITOR;
		
		limpar_display = TRUE;
	}
}

// Interrupção do oscilador de batimentos
ISR(PCINT2_vect)
{
	// Lógica de atualização e calculo do valor de bpm
	static uint32_t bat_ms_anterior = 0;
	if (PIND & 0x04){
		bpm = (uint8_t) (60000/(tempo_ms - bat_ms_anterior));
		bat_ms_anterior = tempo_ms;
	}
}

// Evento de timer a cada 1ms - Contador de tempo e controle do ventilador
ISR(TIMER0_OVF_vect)
{	
	TCNT0 = TIMER0_ZERO;
	tempo_ms++;
	
	// Armazena tempo em ms do ultimo tick do led
	static uint8_t ms_desde_atuacao = 0;
	
	// Lógica de atualização dos leds
	ms_desde_atuacao++;
	if (ms_desde_atuacao >= 100){
		ms_desde_atuacao = 0;
		atua_ventilador();
	}	
}

// Evento de timer a cada 15ms - Usado para iniciar medições do ADC
ISR (TIMER2_OVF_vect)
{
	TCNT2 = TIMER2_ZERO;
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

// Leitura de pressão e comandos através da UART
ISR(USART_RX_vect)
{
	static uint8_t wait_start = TRUE;
	static char msg [7];
	static uint8_t msg_ptr = 0;
	static uint8_t flag_comando = FALSE;
	
	char rx_byte = UDR0;
	// Esperando inicio da mensagem
	if (wait_start){
		if (rx_byte == ';') wait_start=FALSE;
	}
	// Checando se é comando ou pressão
	else if (msg_ptr == 0){
		if(rx_byte == 'p')
			flag_comando = TRUE;
		else
			flag_comando = FALSE;
		msg[msg_ptr] = rx_byte;
		msg_ptr++;
	}
	// Protocolando o fim da mensagem de comando
	else if (flag_comando && msg_ptr == 5){		
		if (rx_byte == ':' && comando_eh_valido(msg))
			atualiza_comando(msg);
		wait_start=TRUE;
		msg_ptr = 0;
	}
	// Protocolando o fim da mensagem de pressao
	else if (!flag_comando && msg_ptr == 7){
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
			if(!flag_comando)
				atualiza_pressao("#ERROR#");
			wait_start=TRUE;
			msg_ptr = 0;
		}
		else if (rx_byte == ';'){
			if(!flag_comando)
				atualiza_pressao("#ERROR#");
			msg_ptr = 0;
		}
		else{
			msg[msg_ptr] = rx_byte;
			msg_ptr++;
		}
	}
}

// Interrupção de transmissão UART
ISR(USART_TX_vect)
{
	if(tx_ptr==22){
		tx_ptr=0;
	}
	else{
		while(!(UCSR0A & (1<<UDRE0))){} // Espera registrador de UART ficar vazio
		UDR0 = tx_msg[tx_ptr];
		tx_ptr++;				
	}
}

// Inicia transmissão UART de leituras
void inicia_tx(){
	char temp_tx_msg[22];
	sprintf(temp_tx_msg, ";%03d,%d.%d,%03d,%.7s:", 
			bpm, temp_corporal_t10 / 10, temp_corporal_t10 % 10, spO2, pressao_sis_dia);
			
	for(int i = 0; i < 22; i++){
		tx_msg[i] = temp_tx_msg[i];
	}
	
	while(!(UCSR0A & (1<<UDRE0))){} // Espera registrador de UART ficar vazio
	UDR0 = tx_msg[0];
	tx_ptr = 1;
}

// Atuação da bomba de ar do respirador
void atua_ventilador (){
	const uint16_t max_volume[9] = {2000, 2250, 2500, 2750, 3000, 3250, 3500, 3750, 4000};
	const uint8_t tick_size_arr[26] = {4, 5, 6, 7, 7, 8, 9, 10, 11, 12, 12, 13, 14, 15, 
		                               16, 17, 18, 18, 19, 20, 21, 22, 22, 23, 24, 25};
	static uint8_t subindo = TRUE;
	
	uint8_t norm_freq = freq_resp-5;
	
	uint8_t diff_OCR1B = tick_size_arr[norm_freq]*volume;
	uint16_t novo_OCR1B;
	if (subindo) novo_OCR1B = OCR1B + diff_OCR1B;
	else         novo_OCR1B = OCR1B - diff_OCR1B;
	
	if (novo_OCR1B >= max_volume[volume]){
		subindo = FALSE;
		OCR1B = max_volume[volume];
	}
	else if (novo_OCR1B <= 2000){
		subindo = TRUE;
		OCR1B = 2000;
		PORTB = (PORTB ^ 0x10); //Beep de buzzer invertendo bit		
	}
	else{
		OCR1B = novo_OCR1B;
	}
}

// Atuação da válvula de oxigenio
void atua_valvula (void)
{
	OCR1A = 2000 + 20*percent_O2;	
}

// Função de atualização de LCD
void atualiza_lcd (void)
{
	if (limpar_display){
		nokia_lcd_clear();
		limpar_display = FALSE;
	}
	
	if (estado_display == STATE_FREQ_RESP)
		atualiza_lcd_0();
	else if (estado_display == STATE_VAL_O2)
		atualiza_lcd_1();
	else if (estado_display == STATE_VOLUME)
		atualiza_lcd_2();
	else if (estado_display == STATE_SETTINGS)
		atualiza_lcd_3();
	else if (estado_display == STATE_SETT_FREQ)
		atualiza_lcd_4();
	else if (estado_display == STATE_SETT_O2)
		atualiza_lcd_5();
	else if (estado_display == STATE_SETT_VOL)
		atualiza_lcd_6();
	else
		atualiza_lcd_7();
}

// Display de Estado 0 - Configuração de Freq_Resp
void atualiza_lcd_0 (void)
{
	char display_str[15];
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string("Parametros", 1);
	
	sprintf(display_str, "%02d    * resp/min", freq_resp);
	nokia_lcd_set_cursor(0, 10);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%03d     %%O2", percent_O2);
	nokia_lcd_set_cursor(0, 20);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%03d     volume", volume);
	nokia_lcd_set_cursor(0, 30);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "Settings");
	nokia_lcd_set_cursor(0, 40);
	nokia_lcd_write_string(display_str, 1);
	
	nokia_lcd_render();
}

// Display de Estado 1 - Configuração de porcentagem de oxigênio
void atualiza_lcd_1 (void)
{
	char display_str[15];
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string("Parametros", 1);
	
	sprintf(display_str, "%02d      resp/min", freq_resp);
	nokia_lcd_set_cursor(0, 10);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%03d   * %%O2", percent_O2);
	nokia_lcd_set_cursor(0, 20);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%03d     volume", volume);
	nokia_lcd_set_cursor(0, 30);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "Settings");
	nokia_lcd_set_cursor(0, 40);
	nokia_lcd_write_string(display_str, 1);
	
	nokia_lcd_render();
}

// Display de Estado 2 - Configuração de volume da bomba
void atualiza_lcd_2 (void)
{
	char display_str[15];
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string("Parametros", 1);
	
	sprintf(display_str, "%02d      resp/min", freq_resp);
	nokia_lcd_set_cursor(0, 10);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%03d     %%O2", percent_O2);
	nokia_lcd_set_cursor(0, 20);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%03d   * volume", volume);
	nokia_lcd_set_cursor(0, 30);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "Settings");
	nokia_lcd_set_cursor(0, 40);
	nokia_lcd_write_string(display_str, 1);
	
	nokia_lcd_render();
}

// Display de Estado 3 - Opção de Configuração
void atualiza_lcd_3 (void)
{
	char display_str[15];
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string("Parametros", 1);
	
	sprintf(display_str, "%02d      resp/min", freq_resp);
	nokia_lcd_set_cursor(0, 10);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%03d     %%O2", percent_O2);
	nokia_lcd_set_cursor(0, 20);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%03d     volume", volume);
	nokia_lcd_set_cursor(0, 30);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "Settings *");
	nokia_lcd_set_cursor(0, 40);
	nokia_lcd_write_string(display_str, 1);
	
	nokia_lcd_render();
}

// Display de Estado 4 - Configuração UART de frequência
void atualiza_lcd_4 (void)
{
	char display_str[15];
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string("Param. UART", 1);
	
	if (uart_freq_resp)
		sprintf(display_str, "freq_resp * ON");
	else
		sprintf(display_str, "freq_resp *OFF");
	
	nokia_lcd_set_cursor(0, 10);
	nokia_lcd_write_string(display_str, 1);
	
	if (uart_o2)
		sprintf(display_str, "%%O2         ON");
	else
		sprintf(display_str, "%%O2        OFF");
	nokia_lcd_set_cursor(0, 20);
	nokia_lcd_write_string(display_str, 1);
	
	if (uart_volume)
		sprintf(display_str, "volume      ON");
	else
		sprintf(display_str, "volume     OFF");
	nokia_lcd_set_cursor(0, 30);
	nokia_lcd_write_string(display_str, 1);
	
	nokia_lcd_render();	
}

// Display de Estado 5 - Configuração UART de oxigenação
void atualiza_lcd_5 (void)
{
	char display_str[15];
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string("Param. UART", 1);
	
	if (uart_freq_resp)
	sprintf(display_str, "freq_resp   ON");
	else
	sprintf(display_str, "freq_resp  OFF");
	
	nokia_lcd_set_cursor(0, 10);
	nokia_lcd_write_string(display_str, 1);
	
	if (uart_o2)
	sprintf(display_str, "%%O2       * ON");
	else
	sprintf(display_str, "%%O2       *OFF");
	nokia_lcd_set_cursor(0, 20);
	nokia_lcd_write_string(display_str, 1);
	
	if (uart_volume)
	sprintf(display_str, "volume      ON");
	else
	sprintf(display_str, "volume     OFF");
	nokia_lcd_set_cursor(0, 30);
	nokia_lcd_write_string(display_str, 1);
	
	nokia_lcd_render();
	
}

// Display de Estado 6 - Configuração UART de volume
void atualiza_lcd_6 (void)
{
	char display_str[15];
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string("Param. UART", 1);
	
	if (uart_freq_resp)
	sprintf(display_str, "freq_resp   ON");
	else
	sprintf(display_str, "freq_resp  OFF");
	
	nokia_lcd_set_cursor(0, 10);
	nokia_lcd_write_string(display_str, 1);
	
	if (uart_o2)
	sprintf(display_str, "%%O2         ON");
	else
	sprintf(display_str, "%%O2        OFF");
	nokia_lcd_set_cursor(0, 20);
	nokia_lcd_write_string(display_str, 1);
	
	if (uart_volume)
	sprintf(display_str, "volume    * ON");
	else
	sprintf(display_str, "volume    *OFF");
	nokia_lcd_set_cursor(0, 30);
	nokia_lcd_write_string(display_str, 1);
	
	nokia_lcd_render();
	
}

// Display de Estado 7 - Exibição de Sinais Vitais
void atualiza_lcd_7 (void)
{	
	char display_str[15];
	nokia_lcd_set_cursor(0, 0);
	nokia_lcd_write_string("Sinais Vitais", 1);	
	
	sprintf(display_str, "%03d     bpm", bpm);
	nokia_lcd_set_cursor(0, 10);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%d.%d    *C", temp_corporal_t10 / 10, temp_corporal_t10 % 10);
	nokia_lcd_set_cursor(0, 20);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%03d     %%SpO2", spO2);
	nokia_lcd_set_cursor(0, 30);
	nokia_lcd_write_string(display_str, 1);
	
	sprintf(display_str, "%.7s mmHg", pressao_sis_dia);
	nokia_lcd_set_cursor(0, 40);
	nokia_lcd_write_string(display_str, 1);
	
	nokia_lcd_render();
}

// Função para realizar assign de string (função da stdlib buga com variaveis volatiles)
void atualiza_pressao(char* str_sete)
{
	for(int i = 0; i < 7; i++){
		pressao_sis_dia[i] = str_sete[i];
	}	
}

// Teste de validade da pressão recebida
uint8_t pressao_eh_valida(char* str_sete)
{
	if (str_sete[3] != 'x') return FALSE;
	for(int i = 0; i < 3; i++){
		if (!isdigit(str_sete[i])) return FALSE;
		if (!isdigit(str_sete[i+4])) return FALSE;
	}
	return TRUE;
}

// Atualiza parametros através da UART
void atualiza_comando(char* str_cinco)
{
	char valor_str[3];
	uint8_t valor_int;
	
	// Casting de string para int
	for (int i = 0; i < 3; i++)
		valor_str[i] = str_cinco[i+2];
	valor_int= atoi(valor_str);
	 
	if (str_cinco[1] == 'f' && uart_freq_resp){
		if(valor_int >= 5 && valor_int <= 30)
			freq_resp = valor_int;		
	}
	else if (str_cinco[1] == 'o' && uart_o2){
		if(valor_int >= 0 && valor_int <= 100)
			percent_O2 = valor_int;
		
	}
	else if (str_cinco[1] == 'v' && uart_volume){
		if(valor_int >= 0 && valor_int <= 8)
			volume = valor_int;		
	}
}

// Teste se o comando realmente é um comando
uint8_t comando_eh_valido (char* str_cinco)
{
	if (str_cinco[0] != 'p') return FALSE;
	for(int i = 0; i < 3; i++){
		if (!isdigit(str_cinco[i+2])) return FALSE;
	}
	return TRUE;
}

// Configuração inicial do microcontrolador
void setup(void)
{
	// Configuração de pinos
	DDRB = 0x3E;
	PORTB |= 0xC1;
	DDRC = 0xFC;
	PORTC = 0x03;
	DDRD = 0x00;
	PORTD = 0xFF;
	
	// Configuração de PCINT (portas B0, B6, B7, D2)
	PCICR = (1<<PCIE0)|(1<<PCIE2);
	PCMSK0 = (1<<PCINT0)|(1<<PCINT6)|(1<<PCINT7);
	PCMSK2 = (1<<PCINT18);
	
	// Configuração do timer0 para 1ms
	TCNT0 = TIMER0_ZERO;
	TCCR0A = 0x00;
	TCCR0B = (1<<CS02);
	TIMSK0 = (1<<TOIE0);
	
	// Configuração de timer1 como PWM duplo de 50Hz
	TCNT1 = TIMER1_ZERO;
	ICR1 = 39999;
	TCCR1A = 0b10100010;
	TCCR1B = 0b00011010;
	OCR1A = 2000;
	OCR1B = 2000;
	
	// Configuração do timer2 para 15ms
	TCNT2 = TIMER2_ZERO;
	TCCR2A = 0x00;
	TCCR2B = (1<<CS22)|(1<<CS20);
	TIMSK2 = (1<<TOIE2);
	
	// Configuração do ADC
	ADMUX = 0x40;
	ADCSRA = 0xCF;
	ADCSRB = 0x00;
	DIDR0 = 0b00111100;
	
	// Configuração da UART
	UBRR0L = (MYUBBR) & 0xFF;
	UBRR0H = (MYUBBR) >> 8;
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXCIE0)|(1<<TXEN0); // Enable de RXTX com interrupção
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00); // Assíncrono de 8bits sem paridade
	
	// Ativação de interrupções
	sei();
	
	// Inicialização de váriaveis
	freq_resp = 15;
	percent_O2 = 0;
	volume = 8;
	temp_corporal_t10 = 350;
	spO2 = 50;
	atualiza_pressao("000x000");
	estado_display = STATE_MONITOR;	
	uart_freq_resp = FALSE;
	uart_o2 = FALSE;
	uart_volume = FALSE;
	tempo_ms = 0;
	
	// Configuração de LCD
	nokia_lcd_init();
	nokia_lcd_clear();	
	atualiza_lcd();
	
	// Atuação Inicial
	atua_ventilador();
	atua_valvula();
}
 
int main(void)
{
	setup();
	
    while (1){
		_delay_ms(200);
		atualiza_lcd();
		// Buzzer
		if ((temp_corporal_t10 < 410 && temp_corporal_t10 > 350) || (spO2 < 60)){
			PORTB = (PORTB & 0xEF) | 0x10;
		}
		else{
			PORTB = (PORTB & 0xEF);
		}
		
		inicia_tx();
	}
}