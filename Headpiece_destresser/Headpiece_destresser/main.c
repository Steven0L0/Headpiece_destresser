#define F_CPU 16000000UL
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "i2cmaster.h"

#define BAUDRATE 9600
#define BAUD_PRESCALER ((F_CPU/(16UL*BAUDRATE))-1)


//#define ADC_PIN 7   //what is the ADC channel we ’ll use for the temperature sens
//#define ADC_PIN2 6  //what is the ADC channel we ’ll use for the humidity sens


#define BH1790GLC_write  0b10110110
#define BH1790GLC_read  0b10110111
#define MEAS_CONTROL_1 0x41
#define MEAS_CONTROL_2 0x42
#define MEAS_START 0x43

float k,y;
float voltage,hum;

uint16_t adc_result;
char byte1, byte2, byte3, byte4;
int u,q;



void USART_init(void);
void sendbyte(char);
void sendstr(unsigned char *);
unsigned char receivebyte(void);
void receivestr(unsigned char*);

unsigned char hr_msg[] = "\nYour HR is: ";
unsigned char instr_msg[] = "\n\nInstructions:\n(1)Make sure you have your finger pressing the HR sensor\n(2)To measure your heart rate press the *HR* button\n(3)For vibration or music press *Motors* or *Music* buttons\n(4)To clean the screen press the 3 dots on the top-right of the app\n";
unsigned char lmsg[] = "\n";
unsigned char pl_msg[] = "\n";
unsigned char rxdata;
char string[3];
unsigned char a[]="1";
unsigned char b[]="2";

void heart()
{
	
	i2c_start(BH1790GLC_write);
	i2c_write(MEAS_CONTROL_1);
	i2c_write(0b10000110);
	i2c_write(0b00101100);
	i2c_stop();
	
	i2c_start(BH1790GLC_write);
	i2c_write(MEAS_START);
	i2c_write(0b00000001);
	i2c_stop();
	_delay_us(31250);
	i2c_start(BH1790GLC_write);
	i2c_write(0x54);
	i2c_stop();
	i2c_start(BH1790GLC_read);
	byte1 = i2c_readAck();
	byte2 = i2c_readAck();
	byte3 = i2c_readAck();
	byte4 = i2c_readNak();
	i2c_stop();
	
	if (byte2==0)
	 u=120-byte4;
	else
	 u=0;
	
}

int main(void)
{
    USART_init();
	
	DDRC = 0b11000000; //all input
	PORTC = 0b00111111; // defines pullup
	
	DDRB = 0b11111111; //all output
	PORTB=0b11111110;
	
	double bpm = 50;
	int o=0;
	int motor_state=0;
	
	
    while (1) 
    {
		heart();
		rxdata = receivebyte();
		if(rxdata == '1'){
		sendstr(instr_msg);}
		
		if(rxdata == '2')
		{
			sendstr(hr_msg);
			itoa(u,string,10);
			for (int i=0;i<3; i++)
			sendbyte(string[i]);
		}
		
		if  ((rxdata == '3') && (motor_state != 1))
			motor_state = 1;
	
			
		else if((rxdata == '3') && (motor_state == 1))
			motor_state = 2;
			
			
		if(motor_state == 1){
			PORTB &= 0b11111101;
			for(int i=0; i < 30000/bpm;i++)
			{
				_delay_ms(1);
			}
			
			PORTB |= 0b00000010;
			for(int i=0; i < 30000/bpm;i++)
			{
				_delay_ms(1);
			}
		}
				
		if  ((rxdata == '4') && (o==1))
		{
			PORTB &= 0b11111110;
			o=0;
		}
		
		else if ((rxdata == '4') && (o==0)){
			PORTB |= 0b00000001;
			o=1;
		}
		
    }
}

void USART_init(void){
	UBRR0H = (uint8_t)(BAUD_PRESCALER>>8);
	UBRR0L = (uint8_t)(BAUD_PRESCALER);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0); //enable rx and tx
	UCSR0C = ((1<<UCSZ00)|(1<<UCSZ01));
	UCSR0B |= (1<<RXCIE0); //enable rx interrupt
}
	
void sendbyte( char u){
	while((UCSR0A&(1<<UDRE0)) == 0);     // Wait if a byte is being transmitted
	UDR0 = u;
}

void sendstr(unsigned char *s){
	unsigned char i = 0;
	while(s[i] != '\0'){
		sendbyte(s[i]);
		i++;
	}
}

unsigned char receivebyte(void){
	//while(!(UCSR0A & (1<<RXC0)));
	return UDR0;
}
