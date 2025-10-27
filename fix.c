// Bare Metal C - Smart Wildfire Monitor
// Step 12: Basic Setup and LCD Initialization

#include <avr/io.h>
#include <util/delay.h>

// Define which AVR Ports and Pins connect to the LCD
// We are using PORTD for Data pins D4-D7, and PORTB for control pins RS and E.
#define LCD_RS_PORT     PORTB
#define LCD_RS_DDR      DDRB
#define LCD_RS_PIN      PB0      // Arduino Digital Pin 8 is PORTB Bit 0

#define LCD_E_PORT      PORTB
#define LCD_E_DDR       DDRB
#define LCD_E_PIN       PB1      // Arduino Digital Pin 9 is PORTB Bit 1

#define LCD_D4_PORT     PORTD
#define LCD_D4_DDR      DDRD
#define LCD_D4_PIN      PD4      // Arduino Digital Pin 4 is PORTD Bit 4? Let's check mapping. CORRECTION: Digital Pin 4 is actually PD4. Our assignment D4->9 is PORTB Bit 1 which is E. Let's reassign correctly.

// Let's correctly reassign LCD pins to AVR Ports based on our wiring:
// RS (Pin4) -> Arduino 7 -> PD7
// E  (Pin6) -> Arduino 8 -> PB0
// D4 (Pin11)-> Arduino 9 -> PB1
// D5 (Pin12)-> Arduino 10-> PB2
// D6 (Pin13)-> Arduino 11-> PB3
// D7 (Pin14)-> Arduino 12-> PB4

// Corrected Definitions for our specific wiring:
#define LCD_RS_PORT     PORTD
#define LCD_RS_DDR      DDRD
#define LCD_RS_PIN      PD7

#define LCD_E_PORT      PORTB
#define LCD_E_DDR       DDRB
#define LCD_E_PIN       PB0

#define LCD_D4_PORT     PORTB
#define LCD_D4_DDR      DDRB
#define LCD_D4_PIN      PB1

#define LCD_D5_PORT     PORTB
#define LCD_D5_DDR      DDRB
#define LCD_D5_PIN      PB2

#define LCD_D6_PORT     PORTB
#define LCD_D6_DDR      DDRB
#define LCD_D6_PIN      PB3

#define LCD_D7_PORT     PORTB
#define LCD_D7_DDR      DDRB
#define LCD_D7_PIN      PB4

// Function to send a pulse to the Enable (E) pin
void lcd_enable_pulse(void) {
    LCD_E_PORT |= (1 << LCD_E_PIN);
    _delay_us(1);
    LCD_E_PORT &= ~(1 << LCD_E_PIN);
    _delay_us(100);
}

// Function to send 4 bits of data to the LCD
void lcd_send_nibble(uint8_t data) {
    // Set data pins based on the nibble
    if (data & 0x01) LCD_D4_PORT |= (1 << LCD_D4_PIN); else LCD_D4_PORT &= ~(1 << LCD_D4_PIN);
    if (data & 0x02) LCD_D5_PORT |= (1 << LCD_D5_PIN); else LCD_D5_PORT &= ~(1 << LCD_D5_PIN);
    if (data & 0x04) LCD_D6_PORT |= (1 << LCD_D6_PIN); else LCD_D6_PORT &= ~(1 << LCD_D6_PIN);
    if (data & 0x08) LCD_D7_PORT |= (1 << LCD_D7_PIN); else LCD_D7_PORT &= ~(1 << LCD_D7_PIN);
    
    lcd_enable_pulse();
}

// Function to send a command to the LCD
void lcd_send_command(uint8_t command) {
    LCD_RS_PORT &= ~(1 << LCD_RS_PIN); // RS low for command mode
    
    lcd_send_nibble(command >> 4);  // Send high nibble
    lcd_send_nibble(command & 0x0F); // Send low nibble
    
    _delay_ms(2); // Wait for command to execute
}

// Function to initialize the LCD
void lcd_init(void) {
    // Set all LCD pins as outputs
    LCD_RS_DDR |= (1 << LCD_RS_PIN);
    LCD_E_DDR |= (1 << LCD_E_PIN);
    LCD_D4_DDR |= (1 << LCD_D4_PIN);
    LCD_D5_DDR |= (1 << LCD_D5_PIN);
    LCD_D6_DDR |= (1 << LCD_D6_PIN);
    LCD_D7_DDR |= (1 << LCD_D7_PIN);
    
    _delay_ms(50); // Wait for LCD to power up
    
    // Initialization sequence for 4-bit mode
    lcd_send_nibble(0x03);
    _delay_ms(5);
    lcd_send_nibble(0x03);
    _delay_us(150);
    lcd_send_nibble(0x03);
    lcd_send_nibble(0x02); // Function Set: 4-bit mode
    
    lcd_send_command(0x28); // Function Set: 4-bit, 2 lines, 5x8 font
    lcd_send_command(0x0C); // Display ON, Cursor OFF
    lcd_send_command(0x06); // Entry Mode: Increment cursor, no shift
    lcd_send_command(0x01); // Clear Display
    _delay_ms(2);
}

// Function to send a character to the LCD
void lcd_send_data(uint8_t data) {
    LCD_RS_PORT |= (1 << LCD_RS_PIN); // RS high for data mode
    
    lcd_send_nibble(data >> 4);  // Send high nibble
    lcd_send_nibble(data & 0x0F); // Send low nibble
    
    _delay_us(100);
}

// Function to display a string on the LCD
void lcd_print_string(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

// Function to set cursor position
void lcd_set_cursor(uint8_t col, uint8_t row) {
    uint8_t address = col;
    if (row == 1) {
        address += 0x40; // Second line starts at address 0x40
    }
    lcd_send_command(0x80 | address); // Set DDRAM address command
}

// DS18B20 Temperature Sensor Functions

// Function to send a bit to the DS18B20
void onewire_write_bit(uint8_t bit) {
    DDRD |= (1 << PD2);  // Set DATA pin as OUTPUT
    PORTD &= ~(1 << PD2); // Pull line LOW
    _delay_us(5);        // Wait 5μs
    
    if (bit) {
        PORTD |= (1 << PD2); // If writing '1', release line
    }
    
    _delay_us(80);       // Wait 80μs
    PORTD |= (1 << PD2); // Release line
    _delay_us(2);        // Recovery time
}

// Function to read a bit from the DS18B20
uint8_t onewire_read_bit(void) {
    uint8_t bit = 0;
    DDRD |= (1 << PD2);  // Set DATA pin as OUTPUT
    PORTD &= ~(1 << PD2); // Pull line LOW
    _delay_us(2);        // Wait 2μs
    
    DDRD &= ~(1 << PD2); // Set DATA pin as INPUT (release line)
    _delay_us(12);       // Wait 12μs
    
    if (PIND & (1 << PD2)) {
        bit = 1;         // Read the value
    }
    
    _delay_us(50);       // Wait 50μs
    return bit;
}

// Function to write a byte to the DS18B20
void onewire_write_byte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        onewire_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

// Function to read a byte from the DS18B20
uint8_t onewire_read_byte(void) {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        byte >>= 1;
        if (onewire_read_bit()) {
            byte |= 0x80;
        }
    }
    return byte;
}

// Function to initialize the DS18B20
uint8_t ds18b20_init(void) {
    uint8_t response = 0;
    DDRD |= (1 << PD2);  // Set as OUTPUT
    PORTD &= ~(1 << PD2); // Pull LOW
    _delay_us(480);      // Wait 480μs
    
    DDRD &= ~(1 << PD2); // Set as INPUT (release line)
    _delay_us(80);       // Wait 80μs
    
    if (!(PIND & (1 << PD2))) {
        response = 1;    // Device present
    }
    
    _delay_us(400);      // Wait 400μs
    return response;
}

// Function to read temperature from DS18B20
float ds18b20_read_temp(void) {
    if (!ds18b20_init()) {
        return -1000; // Error value
    }
    
    onewire_write_byte(0xCC); // Skip ROM
    onewire_write_byte(0x44); // Start temperature conversion
    _delay_ms(750);           // Wait for conversion
    
    if (!ds18b20_init()) {
        return -1000; // Error value
    }
    
    onewire_write_byte(0xCC); // Skip ROM
    onewire_write_byte(0xBE); // Read scratchpad
    
    uint8_t temp_low = onewire_read_byte();
    uint8_t temp_high = onewire_read_byte();
    
    int16_t temp_raw = (temp_high << 8) | temp_low;
    float temperature = temp_raw / 16.0;
    
    return temperature;
}

// Function to convert float to string for display
void float_to_string(float value, char* buffer) {
    int integer_part = (int)value;
    int decimal_part = (int)((value - integer_part) * 100);
    
    if (decimal_part < 0) decimal_part = -decimal_part;
    
    buffer[0] = (integer_part / 10) + '0';
    buffer[1] = (integer_part % 10) + '0';
    buffer[2] = '.';
    buffer[3] = (decimal_part / 10) + '0';
    buffer[4] = (decimal_part % 10) + '0';
    buffer[5] = 'C';
    buffer[6] = '\0';
}

// Flame Sensor Functions

// Initialize ADC for flame sensor (Analog Pin A0)
void adc_init(void) {
    // Set reference voltage to AVCC (5V)
    ADMUX = (1 << REFS0);
    // Enable ADC and set prescaler to 128 (16MHz/128 = 125kHz)
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// Read analog value from flame sensor (0-1023)
uint16_t read_flame_sensor(void) {
    // Select ADC0 (A0) channel
    ADMUX = (ADMUX & 0xF0) | (0 & 0x0F);
    
    // Start conversion
    ADCSRA |= (1 << ADSC);
    
    // Wait for conversion to complete
    while (ADCSRA & (1 << ADSC));
    
    // Return ADC value
    return ADC;
}

// Keypad Functions

// Initialize keypad pins as inputs with pull-up resistors
void keypad_init(void) {
    // Set pins as INPUT and enable pull-up resistors
    DDRD &= ~((1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6));
    PORTD |= (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6);
}

// Check if a button is pressed (returns 1-4, or 0 if no press)
uint8_t check_keypad(void) {
    if (!(PIND & (1 << PD3))) return 1; // Button 1
    if (!(PIND & (1 << PD4))) return 2; // Button 2
    if (!(PIND & (1 << PD5))) return 3; // Button 3
    if (!(PIND & (1 << PD6))) return 4; // Button 4
    return 0; // No button pressed
}

int main(void) {
    lcd_init();
    adc_init();
    keypad_init();
    
    char temp_str[8];
    char flame_str[6];
    float temperature;
    uint16_t flame_value;
    uint8_t display_mode = 1; // 1=welcome, 2=temperature, 3=flame, 4=contacts, 5=emergency
    uint8_t last_button = 0;

    uint8_t button = 0;
    
    while(1) {
        // Read sensors
        temperature = ds18b20_read_temp();
        flame_value = read_flame_sensor();
        
        // Check for flame emergency (has priority over everything)
        if (flame_value < 400) {
            display_mode = 5; // Emergency mode - locks display
        }
        
        // Check keypad (only if not in emergency mode)
        // if (display_mode != 5) {
        //     // uint8_t button = check_keypad();
        //     if (button != 0 && button != last_button) {
        //         display_mode = button;
        //         last_button = button;
        //     }
        //     if (button == 0) {
        //         last_button = 0;
        //     }
        // }
        
        // Display based on mode
        lcd_send_command(0x01); // Clear display
        _delay_ms(2);
        
        switch(display_mode) {
            case 1: // Welcome screen
                lcd_set_cursor(0, 0);
                lcd_print_string("Wildfire Monitor");
                lcd_set_cursor(0, 1);
                lcd_print_string("Press 2,3,4");
                break;
                
            case 2: // Temperature mode
                lcd_set_cursor(0, 0);
                lcd_print_string("Soil Temperature");
                lcd_set_cursor(0, 1);
                if (temperature == -1000) {
                    lcd_print_string("Sensor Error");
                } else {
                    float_to_string(temperature, temp_str);
                    lcd_print_string(temp_str);
                }
                break;
                
            case 3: // Flame sensor mode
                lcd_set_cursor(0, 0);
                lcd_print_string("Flame Sensor");
                lcd_set_cursor(0, 1);
                flame_str[0] = (flame_value / 1000) + '0';
                flame_str[1] = ((flame_value % 1000) / 100) + '0';
                flame_str[2] = ((flame_value % 100) / 10) + '0';
                flame_str[3] = (flame_value % 10) + '0';
                flame_str[4] = '\0';
                lcd_print_string(flame_str);
                break;
                
            case 4: // Emergency contacts
                lcd_set_cursor(0, 0);
                lcd_print_string("Emergency: 911");
                lcd_set_cursor(0, 1);
                lcd_print_string("Fire Dept: 000");
                break;
                
            case 5: // Emergency - flame detected
                lcd_set_cursor(0, 0);
                lcd_print_string("! FIRE DETECTED !");
                lcd_set_cursor(0, 1);
                lcd_print_string("EVACUATE AREA!");
                break;
        }
        
        while((button = check_keypad()) == 0);
        display_mode = button;
    }
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}