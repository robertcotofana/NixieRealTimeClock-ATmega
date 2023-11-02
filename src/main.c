#define F_CPU 			6000000UL
#define F_SCL 			10000UL	

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define DATA_PORT 		PORTD
#define DATA_DDR  		DDRD

#define BTN_Right		PC5
#define BTN_Middle		PC6
#define BTN_Left		PC7	
	
#define LED_PORT		PORTC
#define LED_DDR			DDRC

#define LEDL			PC2
#define LEDR			PC3

#define DATA_PIN 		PD2
#define SH_RegClk		PD5
#define SH_RegClear		PD6 		
#define LATCH 			PD4
#define EN 				PD3 		

#define DS1307_ADDR 	0xD0
#define SECONDS_REG 	0x00
#define MINUTES_REG 	0x01
#define HOURS_REG 		0x02
#define DAY_REG			0x03
#define DATE_REG		0x04
#define MONTH_REG		0x05
#define YEAR_REG		0x06
#define CNTRL_REG		0x07

#define SERIAL_BAUD 	19200
#define BAUD_PRESCALE 	(((F_CPU / (SERIAL_BAUD * 16UL))) - 1)	


ISR (TIMER1_OVF_vect);

void UPDATE_Clk(uint8_t hour, uint8_t minute, uint8_t second);
void SHIFT_Data(uint8_t value);
void UART_Init();
void UART_TxChar(uint8_t ch);
void UART_TxStr(uint8_t *s);
void I2C_Init();
void I2C_Stop();
void HW_Init();
void SET_Clk(uint8_t hours, uint8_t minutes, uint8_t seconds);
void CLOCK_loop();
void Timer1_Init();
void PWM_Init();

uint8_t I2C_Start(uint8_t address);
uint8_t I2C_Repeated_Start(uint8_t address);
uint8_t I2C_Read_Ack();
uint8_t I2C_Read_Nack();
uint8_t I2C_Write(uint8_t data);

uint8_t DECtoBCD(uint8_t val);
uint8_t BCDtoDEC(uint8_t val);

uint8_t UART_RxChar();

/*init variables*/
uint8_t seconds 	= 0;
uint8_t minutes 	= 0;
uint8_t hours 		= 0;
uint8_t btnLeft 	= 0;
uint8_t btnMiddle 	= 0;
uint8_t btnRight 	= 0;

void main() {
	
	/*init peripherals*/
	char c;
	UART_Init();
	/*
	while(1) {
		c = UART_RxChar();
		if (c == 'P') {
			UART_TxStr("P\n");
		}
	}
	*/
	
	I2C_Init();
	HW_Init();
	PWM_Init();
	Timer1_Init();
	
	/*set brightness*/
	OCR0=32;
	
	/*update digits*/
	UPDATE_Clk((hours), (minutes), (seconds));
	
	/*rtc settle*/
	_delay_ms(1000);
	
	/*read time at startup*/
	I2C_Start(0xA0);
	I2C_Write(0x02);
	I2C_Repeated_Start(0xA1);
	
	/*BCD to DEC format conversion*/
	seconds = BCDtoDEC(I2C_Read_Ack());
	minutes = BCDtoDEC(I2C_Read_Ack());
	hours	= BCDtoDEC(I2C_Read_Nack());
	I2C_Stop();
	
	while(1) {		
		CLOCK_loop();
	}
}

void CLOCK_loop() {
	/*update the time*/
	UPDATE_Clk(DECtoBCD(hours), DECtoBCD(minutes), DECtoBCD(seconds));
	
	btnLeft 	= PINC & (1 << BTN_Left);
	btnMiddle 	= PINC & (1 << BTN_Middle);
	btnRight 	= PINC & (1 << BTN_Right);
	
	/*buttons pressed*/
	if (!btnRight) {
		seconds++;
		if (seconds==60) seconds = 0;
		
		SET_Clk(hours, minutes, seconds);
	}
	if (!btnMiddle) {
		minutes++;
		if (minutes==60) minutes = 0;

	}
	if (!btnLeft) {
		hours++;
		if (hours==24) hours = 0;
	}

	/*hard coded delay*/
	_delay_ms(100);
}

uint8_t DECtoBCD(uint8_t val) {
	return ((val / 10) << 4) | (val % 10);
}

uint8_t BCDtoDEC(uint8_t val) {
	return (((val >> 4) * 10) + (val & 0xF));
}

void UPDATE_Clk(uint8_t hours, uint8_t minutes, uint8_t seconds) {
	
	SHIFT_Data(hours);
	SHIFT_Data(minutes);
	SHIFT_Data(seconds);
	
	/*shifting complete*/
	/*clock the LATCH pin*/
	DATA_PORT |= (1 << LATCH);
	_delay_us(100);
	DATA_PORT &= (~(1 << LATCH));
	_delay_us(100);
}

void SET_Clk(uint8_t hours, uint8_t minutes, uint8_t seconds) {
	
	I2C_Start(0xA0);
	I2C_Write(0x02);
	I2C_Write(DECtoBCD(seconds)); 
	I2C_Write(DECtoBCD(minutes)); 
	I2C_Write(DECtoBCD(hours)); 
	I2C_Stop();
	
}

void SHIFT_Data(uint8_t value) {
	for (uint8_t i = 0; i < 8; i++) {
		if (value & 0b10000000) {
			/*output high*/
			DATA_PORT |= 1 << DATA_PIN;
		} else {
			/*output low*/
			DATA_PORT &= (~(1 << DATA_PIN));
		}
		/*clock the SHIFT pin*/
		DATA_PORT |= (1 << SH_RegClk);
		_delay_us(100);
		DATA_PORT &= (~(1 << SH_RegClk));
		_delay_us(100);
		value = value << 1;
	}
}

void HW_Init() {

	/*LED output pins*/
	
	//LED_DDR 	= 0x0C;
	//LED_PORT 	= 0xE0;
	LED_DDR		= LED_DDR | (1 << LEDL)|(1 << LEDR);
	LED_DDR		= LED_DDR & (~(1<<BTN_Left));
	LED_DDR		= LED_DDR & (~(1<<BTN_Middle));
	LED_DDR		= LED_DDR & (~(1<<BTN_Right));
	LED_PORT	|= (0 << LEDL)|(0 < LEDR)|(1<<BTN_Left)|(1<<BTN_Middle)|(1<<BTN_Right);
	
	/*74HC595 output pins*/
	DATA_DDR 	= (1 << DATA_PIN)|(1 << SH_RegClk)|(1 << SH_RegClear)|(1 << LATCH)|(1 << EN);
	DATA_PORT 	|= 1 << SH_RegClear;
	DATA_PORT 	&= (~(1 << EN));
}

void PWM_Init() {
	TCCR0 = (1<<WGM00) | (1<<COM01) | (1<<CS00);
	DDRB 	|=(1<<PB3);
}

ISR (TIMER1_OVF_vect) {
	/*blink the LEDs*/
	LED_PORT ^= (1 << LEDL) | (1 << LEDR);	
	seconds++;
	/*simple clock logic*/
	if (seconds == 60) {
		seconds = 0;
		minutes++;
	}
	if (minutes == 60) {
		minutes = 0;
		hours++;
	}	
	if (hours == 24) {
		hours = 0;
	}
	/*reload the timer*/
	TCNT1 = 59674; 
}

void Timer1_Init() {
	TCNT1 = 59674; 
	TCCR1A = 0x00;
	TCCR1B = (1<<CS10) | (1<<CS12);  
	TIMSK = (1 << TOIE1); 
	sei();	
}

void UART_Init() {
	UCSRB |= (1 << RXEN) | (1 << TXEN);	
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
	UBRRL = BAUD_PRESCALE;			
	UBRRH = (BAUD_PRESCALE >> 8);		
}

void UART_TxChar(uint8_t ch) {
	while (!(UCSRA & (1<<UDRE))); 
	UDR = ch ;
}

uint8_t UART_RxChar() {
	while ((UCSRA & (1 << RXC)) == 0);
	return(UDR);			
}

void UART_TxStr(uint8_t *s) {
	uint8_t c = 0;
	while(s[c] != '\0') {
		UART_TxChar(s[c]);
		c++;
	}
}

void I2C_Init() {
	/*scale down CPU frequency to 100 KHz for the i2c clock*/
	TWSR 	= 0x00;
	TWBR 	= 0x16;
}

uint8_t I2C_Start(uint8_t address) {
	
    uint8_t status;	
	
	/*send the start signal and wait for the flag*/
    TWCR 	= (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);
    while (!(TWCR&(1<<TWINT)));	
    
	/*mask the prescaler bits and get the TWI status*/
	status 	= TWSR&0xF8;
	
    if (status != 0x08)		
		return 0;		//start signal failed, abort		
    
	/*start transmission of address*/
	TWDR	= address;		
    TWCR	= (1<<TWEN)|(1<<TWINT);	
	
    while (!(TWCR&(1<<TWINT)));	
    
	status	= TWSR&0xF8;
	
    if (status == 0x18)	//slave ack		
		return 1;			
    if (status == 0x20)	//slave nack
		return 2;			
    else
		return 3;		//unknown error	
}

uint8_t I2C_Repeated_Start(uint8_t address) {
	
    uint8_t status;	
	
	/*send the start signal and wait for the flag*/
    TWCR	= (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);
	while (!(TWCR&(1<<TWINT)));
	
    status	= TWSR&0xF8;		
    
	if (status != 0x10)		
		return 0;		//repeated start signal failed, abort			
    
	/*start transmission of address*/
	TWDR	= address;		
    TWCR	= (1<<TWEN)|(1<<TWINT);	
    
	while (!(TWCR&(1<<TWINT)));	
    
	status	= TWSR&0xF8;		
    if (status == 0x40)	//slave ack	
		return 1;			
    if (status == 0x48)	//slave nack	
		return 2;			
    else
		return 3;		//unknown error			
}

uint8_t I2C_Write(uint8_t data) {
	
    uint8_t status;	
	
	/*start transmission of data*/
    TWDR	= data;			
    TWCR	= (1<<TWEN)|(1<<TWINT);	
    while (!(TWCR&(1<<TWINT)));	
    
	status	= TWSR&0xF8;	
	
    if (status == 0x28)	//data ack	
		return 0;			
    if (status == 0x30)	//data nack		
		return 1;			
    else
		return 2;		//unknown error		
}

uint8_t I2C_Read_Ack() {
	
    TWCR	= (1<<TWEN)|(1<<TWINT)|(1<<TWEA); 
    while (!(TWCR&(1<<TWINT)));	
    return TWDR;			
}

uint8_t I2C_Read_Nack() {
	
    TWCR	= (1<<TWEN)|(1<<TWINT);	
    while(!(TWCR&(1<<TWINT)));	
    return TWDR;		
}

void I2C_Stop()	{
	
    TWCR	= (1<<TWSTO)|(1<<TWINT)|(1<<TWEN);
    while(TWCR&(1<<TWSTO));	
}
