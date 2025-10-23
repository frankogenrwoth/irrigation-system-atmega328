/*
 * embedded.c
 * Created: 20/10/2025
 * Author : ogenr
 */

#define F_CPU 16000000L

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
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


// display os definitions
#define DISPLAY_CLEAR_SCREEN 0x01
#define DISPLAY_RETURN_HOME 0x02
#define DISPLAY_ENTRY_MODE_SET 0x06
#define DISPLAY_DISPLAY_ON_CURSOR_OFF 0x0C
#define DISPLAY_FUNCTION_SET_4BIT_2LINE 0x28
#define DISPLAY_SET_CURSOR_LINE_1 0x80
#define DISPLAY_SET_CURSOR_LINE_2 0xC0
#define DISPLAY_WAIT_POWER_UP 0x03
#define DISPLAY_4_BIT_MODE 0x02

// display user interface functions
unsigned char buffer[16];

char *MENU_BUFFER[] = {
    "Tank capacity",
    "Refilling rate",
    "Leakage rate",
    "Soil temperature",
    "Triggers",
    "Messages",
    "Config"};

char *TRIGGERS_BUFFER[] = {
    "MIN CAPACITY",
    "MAX TEMP"};

char *CONFIG_BUFFER[] = {
    "Step size",
    "Pumping threshold",
    "Spraying threshold",
    "Enable triggers",
    "Enable alerts"};

char *MESSAGES_BUFFER[10] = {0};

int active_menu_index = -1;
int active_trigger_index = -1;
int active_config_index = -1;
int active_message_index = -1;

int menu_hover_index = 0;
int trigger_hover_index = 0;
int config_hover_index = 0;
int message_hover_index = 0;


// user application variables
float MINIMUM_CAPACITY_BEFORE_REFILLING_TRIGGER = 1.0;
float MAXIMUM_TEMPERATURE_BEFORE_PUMPING = 28.5;
float STEP_SIZE_FOR_INCREMENTS = 0.5;
float PUMP_THRESHOLD = 20.0;
float SPRAY_THRESHOLD = 24.0;
char ENABLE_TRIGGER_VALUE = 1;
char ENABLE_ALERT_VALUE = 1;


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
    LCD_1602A_send_nibble(DISPLAY_WAIT_POWER_UP);
    _delay_ms(5);
    LCD_1602A_send_nibble(DISPLAY_WAIT_POWER_UP);
    _delay_us(150);
    LCD_1602A_send_nibble(DISPLAY_WAIT_POWER_UP);

    LCD_1602A_send_nibble(DISPLAY_4_BIT_MODE); // Switch to 4-bit mode

    LCD_1602A_load_command(DISPLAY_FUNCTION_SET_4BIT_2LINE);
    LCD_1602A_load_command(DISPLAY_DISPLAY_ON_CURSOR_OFF);
    LCD_1602A_load_command(DISPLAY_ENTRY_MODE_SET);
    LCD_1602A_load_command(DISPLAY_CLEAR_SCREEN);
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

// display os firmaware functions
static void format_float(char *dest, size_t dest_size, float value, uint8_t precision, const char *suffix)
{
    char num[24];

    dtostrf(value, 0, precision, num);
    if (suffix && *suffix)
    {
        snprintf(dest, dest_size, "%s %s", num, suffix);
    }
    else
    {
        snprintf(dest, dest_size, "%s", num);
    }
}

void display_set(const unsigned char *title, const unsigned char *data)
{
    // LCD_1602A_load_command(DISPLAY_CLEAR_SCREEN);
    _delay_ms(2);

    unsigned char line1[16];
    unsigned char line2[16];

    // prepare line1: copy up to 12 chars from title, pad with spaces
    int i = 0;
    for (; i < 12 && *title; ++i)
        line1[i] = *title++;
    for (; i < 12; ++i)
        line1[i] = ' ';

    // add control commands at the end of line1
    line1[12] = 0x7F;
    line1[13] = '-';
    line1[14] = '+';
    line1[15] = 0x7E;

    // prepare line2: copy up to 16 chars from data, pad with spaces
    i = 0;
    for (; i < 16 && *data; ++i)
        line2[i] = *data++;
    for (; i < 16; ++i)
        line2[i] = ' ';

    // reset cursor to first line and write 16 chars
    LCD_1602A_load_command(DISPLAY_SET_CURSOR_LINE_1);
    for (i = 0; i < 16; ++i)
    {
        LCD_1602A_load_data(line1[i]);
    }

    // reset cursor to second line and write 16 chars
    LCD_1602A_load_command(DISPLAY_SET_CURSOR_LINE_2);
    for (i = 0; i < 16; ++i)
    {
        LCD_1602A_load_data(line2[i]);
    }
}


// user application
float get_tank_capacity()
{
    return 75.5;
}

float get_refill_rate()
{
    _delay_ms(2000);
    return 4.0;
}

float get_leak_rate()
{
    return 1.5;
}

float get_soil_temperature()
{
    return DS18B20_read_temperature();
}

// user interface functions
void ui_show_display()
{
    if (active_menu_index == -1)
    {
        display_set("MAIN MENU", MENU_BUFFER[menu_hover_index]);
    }
    else
    {
        switch (active_menu_index)
        {
        case 0:
            format_float(buffer, sizeof(buffer), get_tank_capacity(), 1, "Ltrs");
            display_set("TANK CAPACITY", buffer);
            break;
        case 1:
            format_float(buffer, sizeof(buffer), get_refill_rate(), 1, "Ltrs per min");
            display_set("REFILL RATE", buffer);
            break;
        case 2:
            format_float(buffer, sizeof(buffer), get_leak_rate(), 1, "Ltrs per min");
            display_set("LEAKAGE RATE", buffer);
            break;
        case 3:
            format_float(buffer, sizeof(buffer), get_soil_temperature(), 1, "degrees");
            display_set("SOIL TEMPERATURE", buffer);
            break;
        case 4:
            if (active_trigger_index == -1)
            {
                display_set("TRIGGERS", TRIGGERS_BUFFER[trigger_hover_index]);
            }
            else
            {
                switch (active_trigger_index)
                {
                case 0:
                    format_float(buffer, sizeof(buffer), MINIMUM_CAPACITY_BEFORE_REFILLING_TRIGGER, 1, "Ltrs");
                    display_set("MIN CAPACITY", buffer);
                    break;
                case 1:
                    format_float(buffer, sizeof(buffer), MAXIMUM_TEMPERATURE_BEFORE_PUMPING, 2, "degrees");
                    display_set("MAX TEMP", buffer);
                    break;
                default:
                    display_set("ERROR", "Invalid trigger");
                    break;
                }
            }
            break;

        case 5:
            display_set("MESSAGES", MESSAGES_BUFFER[message_hover_index]);
            break;
        case 6:
            if (active_config_index == -1)
            {
                display_set("CONFIG", CONFIG_BUFFER[config_hover_index]);
            }
            else
            {
                switch (active_config_index)
                {
                case 0:
                    format_float(buffer, sizeof(buffer), STEP_SIZE_FOR_INCREMENTS, 1, "Ltrs");
                    display_set("STEP SIZE", buffer);
                    break;
                case 1:
                    format_float(buffer, sizeof(buffer), PUMP_THRESHOLD, 1, "degrees");
                    display_set("PUMP THRESHOLD", buffer);
                    break;
                case 2:
                    format_float(buffer, sizeof(buffer), SPRAY_THRESHOLD, 1, "degrees");
                    display_set("SPRAY THRESHOLD", buffer);
                    break;
                case 3:
                    snprintf(buffer, sizeof(buffer), "%s", ENABLE_TRIGGER_VALUE ? "Enabled" : "Disabled");
                    display_set("ENABLE TRIGGERS", buffer);
                    break;
                case 4:
                    snprintf(buffer, sizeof(buffer), "%s", ENABLE_ALERT_VALUE ? "Enabled" : "Disabled");
                    display_set("ENABLE ALERTS", buffer);
                    break;
                default:
                    display_set("ERROR", "Invalid config");
                    break;
                }
            }
            break;

        default:
            display_set("ERROR", "Invalid menu");
            break;
        }
    }
}

int main(void)
{
    LCD_1602A_init();
    HCSR04_init();
    KEYPAD_init();

    float temperature;
    uint16_t distance;
    uint8_t key;

    while (1)
    {
        ui_show_display();
        _delay_ms(20);
    }
}