#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

// Định nghĩa chân kết nối
// HC-SR04
#define TRIG_PIN PC0
#define TRIG_PORT PORTC
#define TRIG_DDR DDRC

// LCD
#define LCD_RS PB0
#define LCD_RW PB1
#define LCD_E PB2
#define LCD_PORT PORTB
#define LCD_DDR DDRB
#define LCD_DATA_PORT PORTC
#define LCD_DATA_DDR DDRC

// Biến toàn cục
volatile uint16_t timer_value = 0;
volatile uint8_t echo_detected = 0;

// Khai báo trước các hàm
void lcd_init(void);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_string(const char* str);
void lcd_gotoxy(uint8_t x, uint8_t y);
void timer1_init(void);
void hc_sr04_init(void);
void hc_sr04_trigger(void);
uint32_t calculate_distance(void);
void delay_us(uint16_t us);

// Hàm delay microsecond
void delay_us(uint16_t us) {
	while(us--) {
		_delay_us(1);
	}
}

// Khởi tạo LCD
void lcd_init(void) {
	// Cấu hình chân điều khiển LCD
	LCD_DDR |= (1 << LCD_RS) | (1 << LCD_RW) | (1 << LCD_E);
	LCD_DATA_DDR |= 0xF0; // PC4-PC7 là output
	
	_delay_ms(20);
	
	// Khởi tạo LCD ở chế độ 4-bit
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | 0x30;
	LCD_PORT &= ~(1 << LCD_RS);
	LCD_PORT &= ~(1 << LCD_RW);
	LCD_PORT |= (1 << LCD_E);
	_delay_ms(1);
	LCD_PORT &= ~(1 << LCD_E);
	_delay_ms(5);
	
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | 0x30;
	LCD_PORT |= (1 << LCD_E);
	_delay_ms(1);
	LCD_PORT &= ~(1 << LCD_E);
	_delay_ms(1);
	
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | 0x30;
	LCD_PORT |= (1 << LCD_E);
	_delay_ms(1);
	LCD_PORT &= ~(1 << LCD_E);
	_delay_ms(1);
	
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | 0x20; // 4-bit mode
	LCD_PORT |= (1 << LCD_E);
	_delay_ms(1);
	LCD_PORT &= ~(1 << LCD_E);
	_delay_ms(1);
	
	// Function set: 4-bit, 2 lines, 5x8 font
	lcd_command(0x28);
	// Display on, cursor off
	lcd_command(0x0C);
	// Clear display
	lcd_command(0x01);
	// Entry mode set
	lcd_command(0x06);
}

// Gửi lệnh đến LCD
void lcd_command(uint8_t cmd) {
	// Gửi 4 bit cao
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (cmd & 0xF0);
	LCD_PORT &= ~(1 << LCD_RS);
	LCD_PORT &= ~(1 << LCD_RW);
	LCD_PORT |= (1 << LCD_E);
	delay_us(1);
	LCD_PORT &= ~(1 << LCD_E);
	delay_us(100);
	
	// Gửi 4 bit thấp
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | ((cmd << 4) & 0xF0);
	LCD_PORT |= (1 << LCD_E);
	delay_us(1);
	LCD_PORT &= ~(1 << LCD_E);
	_delay_ms(2);
}

// Gửi dữ liệu đến LCD
void lcd_data(uint8_t data) {
	// Gửi 4 bit cao
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | (data & 0xF0);
	LCD_PORT |= (1 << LCD_RS);
	LCD_PORT &= ~(1 << LCD_RW);
	LCD_PORT |= (1 << LCD_E);
	delay_us(1);
	LCD_PORT &= ~(1 << LCD_E);
	delay_us(100);
	
	// Gửi 4 bit thấp
	LCD_DATA_PORT = (LCD_DATA_PORT & 0x0F) | ((data << 4) & 0xF0);
	LCD_PORT |= (1 << LCD_E);
	delay_us(1);
	LCD_PORT &= ~(1 << LCD_E);
	delay_us(100);
}

// Hiển thị chuỗi lên LCD
void lcd_string(const char* str) {
	while(*str) {
		lcd_data(*str);
		str++;
	}
}

// Đặt vị trí con trỏ trên LCD
void lcd_gotoxy(uint8_t x, uint8_t y) {
	uint8_t address;
	switch(y) {
		case 0: address = 0x00 + x; break;
		case 1: address = 0x40 + x; break;
		case 2: address = 0x14 + x; break;
		case 3: address = 0x54 + x; break;
		default: address = 0x00; break;
	}
	lcd_command(0x80 | address);
}

// Khởi tạo Timer1 cho Input Capture
void timer1_init(void) {
	// Normal mode, no prescaler
	TCCR1A = 0;
	TCCR1B = (1 << ICES1) | (1 << CS10); // Rising edge, no prescaler
	
	// Enable Input Capture interrupt
	TIMSK |= (1 << TICIE1);
	
	sei(); // Enable global interrupts
}

// Interrupt Service Routine cho Input Capture
ISR(TIMER1_CAPT_vect) {
	static uint8_t edge_count = 0;
	static uint16_t start_time = 0;
	
	if(edge_count == 0) {
		// Rising edge - bắt đầu đo
		start_time = ICR1;
		TCCR1B &= ~(1 << ICES1); // Đổi sang falling edge
		edge_count = 1;
		} else {
		// Falling edge - kết thúc đo
		timer_value = ICR1 - start_time;
		TCCR1B |= (1 << ICES1); // Đổi lại rising edge
		edge_count = 0;
		echo_detected = 1;
	}
}

// Khởi tạo HC-SR04
void hc_sr04_init(void) {
	TRIG_DDR |= (1 << TRIG_PIN); // Trigger pin là output
	DDRD &= ~(1 << PD4); // Echo pin (ICP1) là input
	TRIG_PORT &= ~(1 << TRIG_PIN); // Trigger ban đầu ở mức thấp
}

// Gửi xung trigger
void hc_sr04_trigger(void) {
	TRIG_PORT |= (1 << TRIG_PIN);
	delay_us(10);
	TRIG_PORT &= ~(1 << TRIG_PIN);
}

// Tính khoảng cách
uint32_t calculate_distance(void) {
	if(timer_value == 0) return 0;
	
	// Công thức: distance = 17150 * TIMER / 2 (đơn vị: 0.1mm)
	uint32_t distance = ((uint32_t)17150 * timer_value) / 2;
	
	return distance; // Trả về khoảng cách tính bằng 0.1mm
}

int main(void) {
	char buffer[21];
	uint32_t distance;
	uint16_t distance_cm, distance_mm;
	
	// Khởi tạo các thiết bị
	lcd_init();
	hc_sr04_init();
	timer1_init();
	
	// Hiển thị tiêu đề
	lcd_gotoxy(0, 0);
	lcd_string("  HC-SR04 Distance  ");
	lcd_gotoxy(0, 1);
	lcd_string("      Sensor       ");
	lcd_gotoxy(0, 2);
	lcd_string("--------------------");
	
	_delay_ms(2000);
	
	while(1) {
		// Gửi xung trigger
		hc_sr04_trigger();
		echo_detected = 0;
		timer_value = 0;
		
		// Đợi echo được phát hiện (timeout 100ms)
		uint16_t timeout = 10000;
		while(!echo_detected && timeout--) {
			delay_us(10);
		}
		
		if(echo_detected) {
			// Tính khoảng cách
			distance = calculate_distance();
			distance_cm = distance / 10;     // Chuyển sang cm
			distance_mm = distance % 10;     // Phần lẻ mm
			
			// Hiển thị kết quả
			lcd_gotoxy(0, 3);
			if(distance_cm > 400) {
				lcd_string("Distance: Out Range ");
				} else {
				sprintf(buffer, "Distance: %3d.%1d cm  ", distance_cm, distance_mm);
				lcd_string(buffer);
			}
			
			// Hiển thị thông tin debug
			lcd_gotoxy(0, 2);
			sprintf(buffer, "Timer: %5d      ", timer_value);
			lcd_string(buffer);
			
			} else {
			// Timeout - không nhận được echo
			lcd_gotoxy(0, 3);
			lcd_string("Distance: No Signal ");
			lcd_gotoxy(0, 2);
			lcd_string("Timer: -----       ");
		}
		
		_delay_ms(500); // Đo mỗi 500ms
	}
	
	return 0;
}
