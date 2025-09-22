#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

// Kết nối mới cho HC-SR04
// Trigger -> PE0 (có thể điều khiển được)
// Echo -> PE4 (INT4 - External Interrupt 4)

// LCD connections (giữ nguyên)
#define LCD_RS PB0
#define LCD_RW PB1
#define LCD_E PB2
#define LCD_PORT PORTB
#define LCD_DDR DDRB
#define LCD_DATA_PORT PORTC
#define LCD_DATA_DDR DDRC

// HC-SR04 connections
#define TRIG_PIN PE0
#define TRIG_PORT PORTE
#define TRIG_DDR DDRE
#define ECHO_PIN PE4  // INT4

#define F_CPU 16000000UL

// Biến toàn cục
volatile uint16_t timer_start = 0;
volatile uint16_t timer_end = 0;
volatile uint8_t echo_received = 0;
volatile uint8_t measuring = 0;

// Khai báo trước các hàm
void lcd_init(void);
void lcd_command(uint8_t cmd);
void lcd_data(uint8_t data);
void lcd_string(const char* str);
void lcd_gotoxy(uint8_t x, uint8_t y);
void hc_sr04_init(void);
void timer1_init(void);
void hc_sr04_trigger(void);
uint32_t calculate_distance(void);
void delay_us(uint16_t us);

// Hàm delay microsecond
void delay_us(uint16_t us) {
	while(us--) {
		_delay_us(1);
	}
}

// Khởi tạo Timer1 (chạy tự do để đếm thời gian)
void timer1_init(void) {
	// Timer1 Normal mode, prescaler = 1 (16MHz)
	TCCR1A = 0;
	TCCR1B = (1 << CS10); // No prescaler
	TCNT1 = 0;
}

// Khởi tạo HC-SR04 và External Interrupt
void hc_sr04_init(void) {
	// Cấu hình Trigger pin
	TRIG_DDR |= (1 << TRIG_PIN);  // PE0 output
	TRIG_PORT &= ~(1 << TRIG_PIN); // Low initially
	
	// Cấu hình Echo pin và External Interrupt 4
	DDRE &= ~(1 << ECHO_PIN);     // PE4 input
	PORTE &= ~(1 << ECHO_PIN);    // No pull-up
	
	// Cấu hình External Interrupt 4 (INT4)
	// INT4 trigger on any logic change
	EICRB |= (1 << ISC40);        // Any logical change on INT4
	EICRB &= ~(1 << ISC41);
	
	// Enable INT4
	EIMSK |= (1 << INT4);
	
	// Enable global interrupts
	sei();
}

// External Interrupt 4 Service Routine (Echo pin)
ISR(INT4_vect) {
	if(measuring) {
		if(PINE & (1 << ECHO_PIN)) {
			// Rising edge - Echo start
			timer_start = TCNT1;
			} else {
			// Falling edge - Echo end
			timer_end = TCNT1;
			echo_received = 1;
			measuring = 0;
		}
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

// Gửi xung trigger
void hc_sr04_trigger(void) {
	// Reset các biến
	echo_received = 0;
	measuring = 1;
	timer_start = 0;
	timer_end = 0;
	
	// Gửi xung trigger 10us
	TRIG_PORT |= (1 << TRIG_PIN);
	delay_us(10);
	TRIG_PORT &= ~(1 << TRIG_PIN);
}

// Tính khoảng cách
uint32_t calculate_distance(void) {
	uint16_t timer_diff;
	uint32_t distance;
	
	if(!echo_received) return 0;
	
	// Tính thời gian echo
	if(timer_end >= timer_start) {
		timer_diff = timer_end - timer_start;
		} else {
		// Timer overflow case
		timer_diff = (0xFFFF - timer_start) + timer_end + 1;
	}
	
	// Công thức: distance = 17150 * TIMER / 2 (đơn vị: 0.1mm)
	// Với prescaler = 1, mỗi tick = 1/16MHz = 62.5ns
	// Để có kết quả chính xác, ta sẽ điều chỉnh công thức
	distance = ((uint32_t)timer_diff * 17150UL) / (2 * 16);  // Chia 16 vì 16MHz
	
	return distance; // Trả về khoảng cách tính bằng 0.1mm
}

int main(void) {
	char buffer[21];
	uint32_t distance;
	uint16_t distance_cm, distance_mm;
	uint16_t timer_count;
	
	// Khởi tạo các thiết bị
	lcd_init();
	timer1_init();
	hc_sr04_init();
	
	// Hiển thị tiêu đề
	lcd_gotoxy(0, 0);
	lcd_string("  HC-SR04 Distance  ");
	lcd_gotoxy(0, 1);
	lcd_string("   (Ext Interrupt)  ");
	lcd_gotoxy(0, 2);
	lcd_string("--------------------");
	
	_delay_ms(2000);
	
	while(1) {
		// Gửi xung trigger
		hc_sr04_trigger();
		
		// Đợi echo được phát hiện (timeout 50ms)
		uint16_t timeout = 5000;
		while(!echo_received && timeout-- && measuring) {
			delay_us(10);
		}
		
		if(echo_received) {
			// Tính khoảng cách
			distance = calculate_distance();
			distance_cm = distance / 10;     // Chuyển sang cm
			distance_mm = distance % 10;     // Phần lẻ mm
			
			// Tính timer count để hiển thị
			if(timer_end >= timer_start) {
				timer_count = timer_end - timer_start;
				} else {
				timer_count = (0xFFFF - timer_start) + timer_end + 1;
			}
			
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
			sprintf(buffer, "Timer: %5d      ", timer_count);
			lcd_string(buffer);
			
			} else {
			// Timeout - không nhận được echo
			lcd_gotoxy(0, 3);
			lcd_string("Distance: No Signal ");
			lcd_gotoxy(0, 2);
			lcd_string("Timer: -----       ");
			measuring = 0; // Reset measuring flag
		}
		
		_delay_ms(500); // Đo mỗi 500ms
	}
	
	return 0;
}
