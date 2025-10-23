/*
 * embedded.c
 * Created: 20/10/2025
 * Author : ogenr
 */

#define F_CPU 16000000L

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// lcd -1602A pin definitions
#define LCD_1602A_RS PB1
#define LCD_1602A_EN PB0

#define LCD_1602A_D4 PD4
#define LCD_1602A_D5 PD5
#define LCD_1602A_D6 PD6
#define LCD_1602A_D7 PD7

#define LCD_1602A_DATA_PORT PORTD
#define LCD_1602A_DATA_DDR DDRD

#define LCD_1602A_CTRL_PORT PORTB
#define LCD_1602A_CTRL_DDR DDRB


// sensor -DS18B20 pin definitions
#define DS18B20_DQ PC0
#define DS18B20_PIN PINC
#define DS18B20_PORT PORTC
#define DS18B20_DDR DDRC

#define DS18B20_OUTPUT() (DS18B20_DDR |= (1 << DS18B20_DQ))
#define DS18B20_INPUT() (DS18B20_DDR &= ~(1 << DS18B20_DQ))
#define DS18B20_LOW() (DS18B20_PORT &= ~(1 << DS18B20_DQ))
#define DS18B20_HIGH() (DS18B20_PORT |= (1 << DS18B20_DQ))

// sensor -SONAR HC-SR04 pin definitions
#define HCSR04_TRIG_PIN PD2
#define HCSR04_ECHO_PIN PD3

#define HCSR04_TRIG_DDR DDRD
#define HCSR04_TRIG_PORT PORTD
#define HCSR04_ECHO_DDR DDRD
#define HCSR04_ECHO_PORT PORTD
#define HCSR04_ECHO_DEF_PIN PIND

// keypad -1x4 matrix pin definitions
#define KEYPAD_KEY_1 PB2
#define KEYPAD_KEY_2 PB3
#define KEYPAD_KEY_3 PB4
#define KEYPAD_KEY_4 PB5

#define KEYPAD_NO_KEY 5
#define KEYPAD_PORT PORTB
#define KEYPAD_PIN PINB
#define KEYPAD_DDR DDRB

// lcd -1602A firmware functions
void LCD_1602A_latch()
{
    LCD_1602A_CTRL_PORT |= (1 << LCD_1602A_EN);
    _delay_us(1);
    LCD_1602A_CTRL_PORT &= ~(1 << LCD_1602A_EN);
    _delay_us(100);
}

void LCD_1602A_send_nibble(uint8_t nibble)
{
    LCD_1602A_DATA_PORT &= ~((1 << LCD_1602A_D4) | (1 << LCD_1602A_D5) | (1 << LCD_1602A_D6) | (1 << LCD_1602A_D7));

    if (nibble & 0x01) LCD_1602A_DATA_PORT |= (1 << LCD_1602A_D4);
    if (nibble & 0x02) LCD_1602A_DATA_PORT |= (1 << LCD_1602A_D5);
    if (nibble & 0x04) LCD_1602A_DATA_PORT |= (1 << LCD_1602A_D6);
    if (nibble & 0x08) LCD_1602A_DATA_PORT |= (1 << LCD_1602A_D7);

    LCD_1602A_latch();
}

void LCD_1602A_load_command(uint8_t cmd)
{
    LCD_1602A_CTRL_PORT &= ~(1 << LCD_1602A_RS); // Command mode
    LCD_1602A_send_nibble(cmd >> 4);
    LCD_1602A_send_nibble(cmd & 0x0F);
}

void LCD_1602A_load_data(uint8_t data)
{
    LCD_1602A_CTRL_PORT |= (1 << LCD_1602A_RS); // Data mode
    LCD_1602A_send_nibble(data >> 4);
    LCD_1602A_send_nibble(data & 0x0F);
}

void LCD_1602A_print(const char* str)
{
    while (*str)
    {
        LCD_1602A_load_data(*str++);
    }
}

void LCD_1602A_init()
{
    LCD_1602A_CTRL_DDR |= (1 << LCD_1602A_RS) | (1 << LCD_1602A_EN);
    LCD_1602A_DATA_DDR |= (1 << LCD_1602A_D4) | (1 << LCD_1602A_D5) | (1 << LCD_1602A_D6) | (1 << LCD_1602A_D7);

    _delay_ms(20); // Wait for LCD power up
    LCD_1602A_send_nibble(0x03);
    _delay_ms(5);
    LCD_1602A_send_nibble(0x03);
    _delay_us(150);
    LCD_1602A_send_nibble(0x03);
    LCD_1602A_send_nibble(0x02); // Switch to 4-bit mode

    LCD_1602A_load_command(0x28); // 4-bit, 2 lines, 5x8 font
    LCD_1602A_load_command(0x0C); // Display ON, Cursor OFF
    LCD_1602A_load_command(0x06); // Entry mode: auto increment cursor
    LCD_1602A_load_command(0x01); // Clear display
    _delay_ms(2);
}


// temperature sensor -DS18B20 firmware functions
uint8_t DS18B20_reset(void)
{
    DS18B20_OUTPUT();
    DS18B20_LOW();
    _delay_us(480);
    DS18B20_INPUT();
    _delay_us(60);

    uint8_t presence = !(DS18B20_PIN & (1 << DS18B20_DQ));
    _delay_us(420);
    return presence;
}

void DS18B20_write_bit(uint8_t bit)
{
    DS18B20_OUTPUT();
    DS18B20_LOW();
    if (bit)
    {
        _delay_us(1);
        DS18B20_INPUT();
        _delay_us(60);
    }
    else
    {
        _delay_us(60);
        DS18B20_INPUT();
    }
}

void DS18B20_write_byte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        DS18B20_write_bit(data & 0x01);
        data >>= 1;
    }
}

uint8_t DS18B20_read_bit(void)
{
    uint8_t bit;
    DS18B20_OUTPUT();
    DS18B20_LOW();
    _delay_us(2);
    DS18B20_INPUT();
    _delay_us(10);
    bit = (DS18B20_PIN & (1 << DS18B20_DQ)) ? 1 : 0;
    _delay_us(50);
    return bit;
}

uint8_t DS18B20_read_byte(void)
{
    uint8_t data = 0;
    for (uint8_t i = 0; i < 8; i++)
    {
        data >>= 1;
        if (DS18B20_read_bit())
            data |= 0x80;
    }
    return data;
}

float DS18B20_read_temperature(void)
{
    uint8_t temp_l, temp_h;
    int16_t temp;

    DS18B20_reset();
    DS18B20_write_byte(0xCC);
    DS18B20_write_byte(0x44);
    _delay_ms(750);

    DS18B20_reset();
    DS18B20_write_byte(0xCC);
    DS18B20_write_byte(0xBE);

    temp_l = DS18B20_read_byte();
    temp_h = DS18B20_read_byte();

    temp = (temp_h << 8) | temp_l;

    return (float)temp / 16.0;
}


// sonar sensor -HC-SR04 firmware functions
void HCSR04_init(void)
{
    HCSR04_TRIG_DDR |= (1 << HCSR04_TRIG_PIN);
    HCSR04_ECHO_DDR &= ~(1 << HCSR04_ECHO_PIN);
}

void HCSR04_trigger(void)
{
    HCSR04_TRIG_PORT &= ~(1 << HCSR04_TRIG_PIN);
    _delay_ms(200);
    HCSR04_TRIG_PORT |= (1 << HCSR04_TRIG_PIN);
    _delay_ms(200);
    HCSR04_TRIG_PORT &= ~(1 << HCSR04_TRIG_PIN);
}

uint16_t HCSR04_read(void)
{
    uint32_t count = 0;

    while (!(HCSR04_ECHO_DEF_PIN & (1 << HCSR04_ECHO_PIN)));

    while (HCSR04_ECHO_DEF_PIN & (1 << HCSR04_ECHO_PIN))
    {
        _delay_us(1);
        count++;
    }

    return (uint16_t)(count / 58.0);
}

uint16_t HCSR04_get_distance(void)
{
    HCSR04_trigger();
    return HCSR04_read();
}


// keypad -1x4 matrix firmware functions

void KEYPAD_init()
{
    KEYPAD_DDR &= ~(1 << KEYPAD_KEY_1) & ~(1 << KEYPAD_KEY_2) & ~(1 << KEYPAD_KEY_3) & ~(1 << KEYPAD_KEY_4);
    KEYPAD_PORT |= (1 << KEYPAD_KEY_1) | (1 << KEYPAD_KEY_2) | (1 << KEYPAD_KEY_3) | (1 << KEYPAD_KEY_4);
}

uint8_t KEYPAD_read(void)
{
    if (!(KEYPAD_PIN & (1 << KEYPAD_KEY_1))) return 1;
    if (!(KEYPAD_PIN & (1 << KEYPAD_KEY_2))) return 2;
    if (!(KEYPAD_PIN & (1 << KEYPAD_KEY_3))) return 3;
    if (!(KEYPAD_PIN & (1 << KEYPAD_KEY_4))) return 4;
    return KEYPAD_NO_KEY;
}




int main(void)
{
    LCD_1602A_init();
    HCSR04_init();
    KEYPAD_init();

    LCD_1602A_print("Booting...");
    _delay_ms(200);

    float temperature;
    uint16_t distance;
    uint8_t key;

    LCD_1602A_load_command(0x01);
    LCD_1602A_print("Ready");
    while (1)
    {
        while ((key = KEYPAD_read()) == KEYPAD_NO_KEY);
        _delay_ms(200);

        _delay_ms(5000);
        temperature = DS18B20_read_temperature();
        distance = HCSR04_get_distance();

        char buffer[16];
        snprintf(buffer, sizeof(buffer), "Dist: %d cm", distance);

        LCD_1602A_load_command(0x01); // clear
        _delay_ms(2);
        LCD_1602A_print(buffer);
        _delay_ms(1000);
    }
}