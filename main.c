#define F_CPU 8000000UL  // 8MHz clock frequency

#include <xc.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <math.h>

// ??nh ngh?a ch�n k?t n?i
#define TRIG_PIN PC0        // Ch�n Trigger k?t n?i v?i PC0
#define TRIG_PORT PORTC
#define TRIG_DDR DDRC

#define ECHO_PIN PD4        // Ch�n Echo k?t n?i v?i PD4 (ICP1)

// ??nh ngh?a LCD 20x4
#define LCD_RS_PIN PB0      // RS pin
#define LCD_RW_PIN PB1      // RW pin
#define LCD_EN_PIN PB2      // Enable pin
#define LCD_CTRL_PORT PORTB
#define LCD_CTRL_DDR DDRB

#define LCD_DATA_PORT PORTC // D4-D7 pins (PC4-PC7)
#define LCD_DATA_DDR DDRC

// Bi?n to�n c?c
volatile uint16_t timer_start = 0;
volatile uint16_t timer_end = 0;
volatile uint16_t pulse_width = 0;
volatile uint8_t measurement_complete = 0;
volatile uint8_t capture_state = 0;  // 0: waiting for rising edge, 1: waiting for falling edge

// Khai b�o h�m tr??c
void lcd_init(void);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_string(const char* str);
void lcd_goto(uint8_t row, uint8_t col);
void lcd_enable_pulse(void);
void timer1_init(void);
void input_capture_init(void);
void send_trigger(void);
void format_distance_udm(uint16_t distance_cm, char* buffer);

// Kh?i t?o Timer1 cho Input Capture
void timer1_init() {
	// Timer1 normal mode, prescaler 1
	TCCR1A = 0;
	TCCR1B = (1 << CS10);  // Prescaler 1
	TCNT1 = 0;
}

// Kh?i t?o Input Capture
void input_capture_init() {
	// C?u h�nh PD4 (ICP1) l� input
	DDRD &= ~(1 << PD4);
	
	// Kh?i t?o Timer1
	timer1_init();
	
	// C?u h�nh Input Capture ?? k�ch ho?t tr�n c?nh l�n tr??c
	TCCR1B |= (1 << ICES1);  // Input Capture Edge Select - rising edge
	
	// X�a c? Input Capture
	TIFR |= (1 << ICF1);
	
	// Cho ph�p Input Capture Interrupt
	TIMSK |= (1 << TICIE1);
	
	// Cho ph�p global interrupt
	sei();
}

// Interrupt Service Routine cho Input Capture
ISR(TIMER1_CAPT_vect) {
	if (capture_state == 0) {
		// C?nh l�n - l?u th?i ?i?m b?t ??u
		timer_start = ICR1;
		// Chuy?n sang ch? ?? k�ch ho?t c?nh xu?ng
		TCCR1B &= ~(1 << ICES1);  // Falling edge
		capture_state = 1;
		} else {
		// C?nh xu?ng - l?u th?i ?i?m k?t th�c
		timer_end = ICR1;
		
		// T�nh ?? r?ng xung
		if (timer_end > timer_start) {
			pulse_width = timer_end - timer_start;
			} else {
			// X? l� tr??ng h?p timer overflow
			pulse_width = (0xFFFF - timer_start) + timer_end + 1;
		}
		
		measurement_complete = 1;
		capture_state = 0;
		
		// Chuy?n l?i v? ch? ?? k�ch ho?t c?nh l�n cho l?n ?o ti?p theo
		TCCR1B |= (1 << ICES1);  // Rising edge
	}
	
	// X�a c? Input Capture
	TIFR |= (1 << ICF1);
}

// Kh?i t?o LCD 20x4
void lcd_init() {
	// C?u h�nh ch�n ?i?u khi?n
	LCD_CTRL_DDR |= (1 << LCD_RS_PIN) | (1 << LCD_RW_PIN) | (1 << LCD_EN_PIN);
	// C?u h�nh ch�n data D4-D7 (PC4-PC7)
	LCD_DATA_DDR |= 0xF0;  // Set PC4-PC7 as output
	
	_delay_ms(50);  // Ch? LCD kh?i t?o
	
	// K�o RW xu?ng LOW (ch? ?? write)
	LCD_CTRL_PORT &= ~(1 << LCD_RW_PIN);
	
	// Reset sequence
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | 0x30;  // Function set 8-bit
	lcd_enable_pulse();
	_delay_ms(5);
	
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | 0x30;  // Function set 8-bit
	lcd_enable_pulse();
	_delay_us(100);
	
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | 0x30;  // Function set 8-bit
	lcd_enable_pulse();
	_delay_us(100);
	
	// Chuy?n sang 4-bit mode
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | 0x20;
	lcd_enable_pulse();
	_delay_us(100);
	
	// Function set: 4-bit, 2 lines, 5x7 font
	lcd_command(0x28);
	
	// Display off
	lcd_command(0x08);
	
	// Clear display
	lcd_command(0x01);
	_delay_ms(2);
	
	// Entry mode set: increment cursor, no shift
	lcd_command(0x06);
	
	// Display on, cursor off, blink off
	lcd_command(0x0C);
}

void lcd_enable_pulse() {
	LCD_CTRL_PORT |= (1 << LCD_EN_PIN);   // EN = 1
	_delay_us(1);
	LCD_CTRL_PORT &= ~(1 << LCD_EN_PIN);  // EN = 0
	_delay_us(50);
}

void lcd_command(uint8_t cmd) {
	// K�o RS xu?ng LOW (command mode)
	LCD_CTRL_PORT &= ~(1 << LCD_RS_PIN);
	
	// Send upper nibble (D7-D4)
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (cmd & 0xF0);
	lcd_enable_pulse();
	
	// Send lower nibble (D3-D0)
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | ((cmd << 4) & 0xF0);
	lcd_enable_pulse();
	
	_delay_us(50);
}

void lcd_data(uint8_t data) {
	// K�o RS l�n HIGH (data mode)
	LCD_CTRL_PORT |= (1 << LCD_RS_PIN);
	
	// Send upper nibble (D7-D4)
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (data & 0xF0);
	lcd_enable_pulse();
	
	// Send lower nibble (D3-D0)
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | ((data << 4) & 0xF0);
	lcd_enable_pulse();
	
	_delay_us(50);
}

void lcd_string(const char* str) {
	while(*str) {
		lcd_data(*str++);
	}
}

void lcd_goto(uint8_t row, uint8_t col) {
	uint8_t address;
	switch(row) {
		case 0: address = 0x80 + col; break;  // Line 1
		case 1: address = 0xC0 + col; break;  // Line 2
		case 2: address = 0x94 + col; break;  // Line 3
		case 3: address = 0xD4 + col; break;  // Line 4
		default: address = 0x80; break;
	}
	lcd_command(address);
}

// G?i xung trigger
void send_trigger() {
	TRIG_PORT |= (1 << TRIG_PIN);   // K�o trigger l�n HIGH
	_delay_us(10);                   // Gi? trong 10us
	TRIG_PORT &= ~(1 << TRIG_PIN);  // K�o trigger xu?ng LOW
}

// Chuy?n ??i s? th�nh chu?i theo ??nh d?ng UDM
void format_distance_udm(uint16_t distance_cm, char* buffer) {
	// Ki?m tra ngo�i ph?m vi ?o
	if (distance_cm < 2 || distance_cm > 400) {
		sprintf(buffer, "outrange");
		return;
	}
	
	if (distance_cm > 999) {
		// N?u > 999cm, hi?n th? l� "999cm"
		sprintf(buffer, "999cm");
		return;
	}
	
	if (distance_cm >= 100) {
		// H�ng tr?m: ABC.Xcm
		uint8_t hundreds = distance_cm / 100;
		uint8_t remainder = distance_cm % 100;
		uint8_t tens = remainder / 10;
		uint8_t ones = remainder % 10;
		sprintf(buffer, "%d%d%d.%dcm", hundreds, tens, ones, 0);
	}
	else if (distance_cm >= 10) {
		// H�ng ch?c: xBC.Xcm
		uint8_t tens = distance_cm / 10;
		uint8_t ones = distance_cm % 10;
		sprintf(buffer, "x%d%d.%dcm", tens, ones, 0);
	}
	else {
		// H�ng ??n v?: xxC.Xcm
		sprintf(buffer, "xx%d.%dcm", distance_cm, 0);
	}
}

int main() {
	// Kh?i t?o c�c module
	TRIG_DDR |= (1 << TRIG_PIN);    // C?u h�nh trigger pin l� output
	TRIG_PORT &= ~(1 << TRIG_PIN);  // K�o trigger xu?ng LOW ban ??u
	
	lcd_init();
	input_capture_init();
	
	char distance_str[21];  // T?ng buffer size cho LCD 20x4
	char timer_str[21];
	uint16_t distance_cm;
	float distance_float;
	
	// Hi?n th? th�ng b�o kh?i t?o
	char ready_msg[] = "HC-SR04 UDM Sensor";
	char init_msg[] = "Initializing...";
	
	lcd_goto(0, 1);  // D�ng 1, c?t 2 (gi?a m�n h�nh)
	lcd_string(ready_msg);
	lcd_goto(1, 3);  // D�ng 2, c?t 4
	lcd_string(init_msg);
	_delay_ms(2000);
	
	while(1) {
		// Reset tr?ng th�i ?o
		measurement_complete = 0;
		capture_state = 0;
		pulse_width = 0;
		
		// ??m b?o Input Capture ???c c?u h�nh cho c?nh l�n
		TCCR1B |= (1 << ICES1);  // Rising edge
		
		// Reset Timer1
		TCNT1 = 0;
		
		// G?i xung trigger
		send_trigger();
		
		// Ch? ?o xong (timeout sau 50ms)
		uint16_t timeout = 0;
		while (!measurement_complete && timeout < 5000) {
			_delay_us(10);
			timeout++;
		}
		
		lcd_command(0x01);  // Clear display
		
		if (measurement_complete) {
			// Hi?n th? ti�u ??
			lcd_goto(0, 6);  // D�ng 1, gi?a m�n h�nh
			lcd_string("UDM SENSOR");
			
			// Hi?n th? gi� tr? pulse width
			lcd_goto(1, 0);  // D�ng 2
			sprintf(timer_str, "Pulse: %u counts", pulse_width);
			lcd_string(timer_str);
			
			// T�nh kho?ng c�ch theo c�ng th?c: distance = 17150 * pulse_width * 10^-6
			distance_float = 17150.0 * pulse_width * 1e-6;
			distance_cm = (uint16_t)round(distance_float);
			
			// Hi?n th? kho?ng c�ch theo ??nh d?ng UDM
			lcd_goto(2, 0);  // D�ng 3
			lcd_string("Distance: ");
			format_distance_udm(distance_cm, distance_str);
			lcd_string(distance_str);
			
			// Hi?n th? ??n v? v� th�ng tin th�m
			lcd_goto(3, 4);  // D�ng 4
			sprintf(distance_str, "= %u cm", distance_cm);
			lcd_string(distance_str);
		}
		else {
			// Timeout - kh�ng nh?n ???c t�n hi?u
			lcd_goto(0, 6);
			lcd_string("UDM ERROR");
			lcd_goto(1, 2);
			lcd_string("Timeout occurred");
			lcd_goto(2, 1);
			lcd_string("Check connections");
			lcd_goto(3, 3);
			lcd_string("and try again");
		}
		
		_delay_ms(1000);  // ?o m?i 1 gi�y
	}
	
	return 0;
}