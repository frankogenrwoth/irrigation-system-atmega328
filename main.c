#define F_CPU 16000000L

#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>

// interrupt definitions
volatile uint8_t tick = 0;
volatile char one_second_event = 0;

// led port definition
#define SYSTEM_ACTIVE_LED_PIN PC1
#define SYSTEM_ACTIVE_LED_PORT PORTC
#define SYSTEM_ACTIVE_LED_DDR DDRC


// lcd -1602A pin definitions
#define LCD_1602A_RS PB0
#define LCD_1602A_EN PB1

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
char buffer[16];

#define MSG_BUFFER_SIZE 4 // starting from 0

char *MENU_BUFFER[] = {
    "Tank capacity",
    "Refilling rate",
    "Leakage rate",
    "Soil temperature",
    "Live View",
    "Triggers",
    "Messages",
    "Config",
};


char *LIVE_VIEW_BUFFER[] = {
    "Capacity",
    "Refill Rate",
    "Leak Rate",
    "Soil Temp"
};

char *TRIGGERS_BUFFER[] = {
    "MIN CAPACITY",
    "MAX TEMP"
};

char *CONFIG_BUFFER[] = {
    "Step size",
    "Pumping threshold",
    "Spraying threshold",
    "Enable triggers",
    "Enable alerts"
};


int active_menu_index = -1;
int active_trigger_index = -1;
int active_config_index = -1;
int active_message_index = -1;
int active_live_view_index = -1;

int menu_hover_index = 0;
int trigger_hover_index = 0;
int config_hover_index = 0;
int message_hover_index = 0;
int live_view_hover_index = 0;


// user application variables
float MINIMUM_CAPACITY_BEFORE_REFILLING_TRIGGER = 1.0;
float MAXIMUM_TEMPERATURE_BEFORE_PUMPING = 28.5;
float STEP_SIZE_FOR_INCREMENTS = 0.5;
float STEP_INCREMENT = 0.25;
float PUMP_THRESHOLD = 20.0;
float SPRAY_THRESHOLD = 24.0;
char ENABLE_TRIGGER_VALUE = 1;
char ENABLE_ALERT_VALUE = 1;


// ENVIRONMENT DEFINITIONS
#define TANK_HEIGHT_IN_CM 15.6
#define TANK_RADIUS_IN_CM 8.0
#define ROOM_TEMPERATURE_IN_CELCIUS 25
#define PI 3.14159

// live view variables
float current_tank_capacity = 0.0;
float current_refill_rate = 0.0;
float current_leak_rate = 0.0;
float current_soil_temperature = 0.0;


char SHOW_LOADING_WIDGET = 0;


// queue for keeping 10 distances implemented in the padded second heights
#define BUFFER_SIZE 10 // max queue size

typedef struct {
    int data[BUFFER_SIZE];
    int front;
    int rear;
} Queue;

// Initialize the queue
void initQueue(Queue *q) {
    q->front = -1;
    q->rear = -1;
}

// Check if empty
int isEmpty(Queue *q) {
    return (q->front == -1);
}

// Check if full
int isFull(Queue *q) {
    return ((q->rear + 1) % BUFFER_SIZE == q->front);
}

// Remove (dequeue)
int dequeue(Queue *q) {
    if (isEmpty(q)) {
        printf("Queue is empty!\n");
        return -1;
    }

    int value = q->data[q->front];

    if (q->front == q->rear) {
        // only one element left
        q->front = q->rear = -1;
    } else {
        q->front = (q->front + 1) % BUFFER_SIZE;
    }

    return value;
}


// Add (enqueue)
void enqueue(Queue *q, int value) {
    // If queue is full, overwrite the oldest (drop the first uploaded)
    if (isFull(q)) {
        // advance front and rear to make room for the new value (overwrite oldest)
        q->front = (q->front + 1) % BUFFER_SIZE;
        q->rear = (q->rear + 1) % BUFFER_SIZE;
        q->data[q->rear] = value;
        // optional debug print
        printf("Queue full, overwrote oldest with: %d\n", value);
        return;
    }

    if (isEmpty(q)) {
        q->front = 0;
        q->rear = 0;
        q->data[q->rear] = value;
        return;
    }

    q->rear = (q->rear + 1) % BUFFER_SIZE;
    q->data[q->rear] = value;
}

// Peek at the first (oldest) element without removing it.
// Returns 0 on success and writes value to *out, -1 if empty.
int peekFront(Queue *q, int *out) {
    if (isEmpty(q)) return -1;
    *out = q->data[q->front];
    return 0;
}

// Peek at the last (newest) element without removing it.
// Returns 0 on success and writes value to *out, -1 if empty.
int peekRear(Queue *q, int *out) {
    if (isEmpty(q)) return -1;
    *out = q->data[q->rear];
    return 0;
}

// Get number of elements currently in the queue
int queueSize(Queue *q) {
    if (isEmpty(q)) return 0;
    if (q->rear >= q->front) return (q->rear - q->front + 1);
    return (BUFFER_SIZE - q->front + q->rear + 1);
}


Queue height_per_second;
// String message queue (separate from integer queue)
#define MESSAGE_QUEUE_SIZE 10

typedef struct {
    char *data[MESSAGE_QUEUE_SIZE];
    int front;
    int rear;
} StrQueue;

// Initialize the string queue
void initStrQueue(StrQueue *q) {
    q->front = -1;
    q->rear = -1;
    for (int i = 0; i < MESSAGE_QUEUE_SIZE; ++i) q->data[i] = NULL;
}

// Check if string queue is empty
int isStrQueueEmpty(StrQueue *q) {
    return (q->front == -1);
}

// Check if string queue is full
int isStrQueueFull(StrQueue *q) {
    return ((q->rear + 1) % MESSAGE_QUEUE_SIZE == q->front);
}

// Remove (dequeue) from string queue. Returns pointer to malloc'd string (caller must free), or NULL if empty.
char *dequeueStr(StrQueue *q) {
    if (isStrQueueEmpty(q)) {
        printf("String queue is empty!\n");
        return NULL;
    }

    char *value = q->data[q->front];
    q->data[q->front] = NULL;

    if (q->front == q->rear) {
        // only one element left
        q->front = q->rear = -1;
    } else {
        q->front = (q->front + 1) % MESSAGE_QUEUE_SIZE;
    }

    return value;
}

// Add (enqueue) a copy of the string to the queue. On overwrite, the oldest string is freed.
void enqueueStr(StrQueue *q, const char *value) {
    if (!value) return; // ignore NULL

    size_t len = strlen(value) + 1;
    char *copy = (char *)malloc(len);
    if (!copy) {
        printf("enqueueStr: malloc failed\n");
        return;
    }
    strcpy(copy, value);

    // If queue is full, overwrite the oldest (drop the first uploaded) and free it
    if (isStrQueueFull(q)) {
        free(q->data[q->front]);
        q->front = (q->front + 1) % MESSAGE_QUEUE_SIZE;
        q->rear = (q->rear + 1) % MESSAGE_QUEUE_SIZE;
        q->data[q->rear] = copy;
        return;
    }

    if (isStrQueueEmpty(q)) {
        q->front = 0;
        q->rear = 0;
        q->data[q->rear] = copy;
        return;
    }

    q->rear = (q->rear + 1) % MESSAGE_QUEUE_SIZE;
    q->data[q->rear] = copy;
}

// Peek at the first (oldest) string without removing it. Returns 0 on success and sets *out, -1 if empty.
int peekFrontStr(StrQueue *q, char **out) {
    if (isStrQueueEmpty(q)) return -1;
    *out = q->data[q->front];
    return 0;
}

// Peek at the last (newest) string without removing it. Returns 0 on success and sets *out, -1 if empty.
int peekRearStr(StrQueue *q, char **out) {
    if (isStrQueueEmpty(q)) return -1;
    *out = q->data[q->rear];
    return 0;
}

// Get number of elements currently in the string queue
int strQueueSize(StrQueue *q) {
    if (isStrQueueEmpty(q)) return 0;
    if (q->rear >= q->front) return (q->rear - q->front + 1);
    return (MESSAGE_QUEUE_SIZE - q->front + q->rear + 1);
}

StrQueue MESSAGES_Q;
char SYSTEM_SIGNAL = 0;

// timer0 for calculating in the background
void TIMER2_INIT(void)
{
    TCCR2A = 0x00;                                    // normal mode
    TCCR2B = (1 << CS22) | (1 << CS21) | (1 << CS20); // prescaler = 1024
    TIMSK2 = (1 << TOIE2);                            // enable overflow interrupt
    sei();                                            // enable global interrupts
}

// led firmware functions


void LED_SYSTEM_ACTIVE_INIT(void)
{
    SYSTEM_ACTIVE_LED_DDR |= (1 << SYSTEM_ACTIVE_LED_PIN);
    SYSTEM_ACTIVE_LED_PORT &= ~(1 << SYSTEM_ACTIVE_LED_PIN);

}

/*
    turn off the system active LED
    params: void
    returns: void
*/
void LED_system_active_off(void)
{
    SYSTEM_ACTIVE_LED_PORT &= ~(1 << SYSTEM_ACTIVE_LED_PIN);
}

/* 
    turn on the system active LED
    params: void
    returns: void
*/
void LED_system_active_on(void)
{
    SYSTEM_ACTIVE_LED_PORT |= (1 << SYSTEM_ACTIVE_LED_PIN);
}
// lcd -1602A firmware api

/*
    latch api enable pin by setting it high for a brief moment and then setting it low
    params: void
    returns: void
*/
void LCD_1602A_latch(void)
{
    LCD_1602A_CTRL_PORT |= (1 << LCD_1602A_EN);
    _delay_us(1);
    LCD_1602A_CTRL_PORT &= ~(1 << LCD_1602A_EN);
    _delay_us(100);
}

/*
    send a nibble (4 bits) to the lcd data port
    params: uint8_t nibble - the 4 bits to send (lower nibble is used)
    returns: void
*/
void LCD_1602A_send_nibble(uint8_t nibble)
{
    LCD_1602A_DATA_PORT &= ~((1 << LCD_1602A_D4) | (1 << LCD_1602A_D5) | (1 << LCD_1602A_D6) | (1 << LCD_1602A_D7));

    if (nibble & 0x01) LCD_1602A_DATA_PORT |= (1 << LCD_1602A_D4);
    if (nibble & 0x02) LCD_1602A_DATA_PORT |= (1 << LCD_1602A_D5);
    if (nibble & 0x04) LCD_1602A_DATA_PORT |= (1 << LCD_1602A_D6);
    if (nibble & 0x08) LCD_1602A_DATA_PORT |= (1 << LCD_1602A_D7);

    LCD_1602A_latch();
}

/*
    load a command byte to the lcd
    params: uint8_t cmd - the command byte to send
    returns: void
*/
void LCD_1602A_load_command(uint8_t cmd)
{
    LCD_1602A_CTRL_PORT &= ~(1 << LCD_1602A_RS); // Command mode
    LCD_1602A_send_nibble(cmd >> 4);
    LCD_1602A_send_nibble(cmd & 0x0F);
}

/*
    load a data byte to the lcd
    params: uint8_t data - the data byte to send
    returns: void
*/
void LCD_1602A_load_data(uint8_t data)
{
    LCD_1602A_CTRL_PORT |= (1 << LCD_1602A_RS); // Data mode
    LCD_1602A_send_nibble(data >> 4);
    LCD_1602A_send_nibble(data & 0x0F);
}

/*
    print a string to the lcd
    params: const char* str - the null-terminated string to print
    returns: void
*/
void LCD_1602A_print(const char* str)
{
    while (*str)
    {
        LCD_1602A_load_data(*str++);
    }
}

void LCD_1602A_create_char(uint8_t location, uint8_t charmap[])
{
    location &= 0x07;                               // only 0–7 valid
    LCD_1602A_load_command(0x40 | (location << 3)); // Set CGRAM address

    for (uint8_t i = 0; i < 8; i++)
    {
        LCD_1602A_load_data(charmap[i]); // write pixel pattern
    }

    // Return to DDRAM (display memory)
    LCD_1602A_load_command(0x80);
}

/*
    initialize the lcd in 4-bit mode
    params: void
    returns: void
*/
void LCD_1602A_init(void)
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

    /* create arrow glyphs once during init (used by display_set)
       location 0..3 reserved for arrows */
    uint8_t arrow_down[8] = {
        0x00,
        0x04,
        0x04,
        0x04,
        0x1F,
        0x0E,
        0x04,
        0x00
    };

    uint8_t arrow_up[8] = {
        0x00,
        0x04,
        0x0E,
        0x1F,
        0x04,
        0x04,
        0x04,
        0x00
    };

    uint8_t arrow_right[8] = {
        0x00,
        0x04,
        0x06,
        0x1F,
        0x06,
        0x04,
        0x00,
        0x00
    };

    uint8_t arrow_left[8] = {
        0x00,
        0x04,
        0x0C,
        0x1F,
        0x0C,
        0x04,
        0x00,
        0x00
    };

    LCD_1602A_create_char(0, arrow_down); // store at location 0
    LCD_1602A_create_char(1, arrow_up);   // store at location 1
    LCD_1602A_create_char(2, arrow_left);   // store at location 2
    LCD_1602A_create_char(3, arrow_right);   // store at location 3
}


// temperature sensor -DS18B20 firmware functions
/* 
    reset the DS18B20 sensor and check for presence pulse
    params: void
    returns: uint8_t - 1 if presence pulse detected, 0 otherwise
*/
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

/* 
    write a single bit to the DS18B20 sensor
    params: uint8_t bit - the bit to write (0 or 1)
    returns: void
*/
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

/* 
    write a byte to the DS18B20 sensor
    params: uint8_t data - the byte to write
    returns: void
*/
void DS18B20_write_byte(uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        DS18B20_write_bit(data & 0x01);
        data >>= 1;
    }
}

/* 
    read a single bit from the DS18B20 sensor
    params: void
    returns: uint8_t - the bit read (0 or 1)
*/
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

/* 
    read a byte from the DS18B20 sensor
    params: void
    returns: uint8_t - the byte read
*/
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

/* 
    read temperature from the DS18B20 sensor
    params: void
    returns: float - the temperature in degrees Celsius
*/
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
/* 
    initialize the HC-SR04 sensor pins
    params: void
    returns: void
*/
void HCSR04_init(void)
{
    HCSR04_TRIG_DDR |= (1 << HCSR04_TRIG_PIN);
    HCSR04_ECHO_DDR &= ~(1 << HCSR04_ECHO_PIN);
}

/* 
    trigger the HC-SR04 sensor to start measurement
    params: void
    returns: void
*/
void HCSR04_trigger(void)
{
    /* Proper trigger: low for a short time, then high for 10us pulse */
    HCSR04_TRIG_PORT &= ~(1 << HCSR04_TRIG_PIN);
    _delay_us(2);
    HCSR04_TRIG_PORT |= (1 << HCSR04_TRIG_PIN);
    _delay_us(10);
    HCSR04_TRIG_PORT &= ~(1 << HCSR04_TRIG_PIN);
}

/* 
    read the distance measurement from the HC-SR04 sensor
    params: void
    returns: uint16_t - the distance in centimeters
*/
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

/* 
    get the distance measurement from the HC-SR04 sensor
    params: void
    returns: uint16_t - the distance in centimeters
*/
uint16_t HCSR04_get_distance(void)
{
    HCSR04_trigger();
    return HCSR04_read();
}


// keypad -1x4 matrix firmware functions
/* 
    initialize the keypad pins
    params: void
    returns: void
*/
void KEYPAD_init(void)
{
    KEYPAD_DDR &= ~((1 << KEYPAD_KEY_1) | (1 << KEYPAD_KEY_2) | (1 << KEYPAD_KEY_3) | (1 << KEYPAD_KEY_4));
    KEYPAD_PORT |= (1 << KEYPAD_KEY_1) | (1 << KEYPAD_KEY_2) | (1 << KEYPAD_KEY_3) | (1 << KEYPAD_KEY_4);
}

/* 
    read the pressed key from the keypad
    params: void
    returns: uint8_t - the key number (1-4) or KEYPAD_NO_KEY if no key is pressed
*/
uint8_t KEYPAD_read(void)
{
    if (!(KEYPAD_PIN & (1 << KEYPAD_KEY_1))) return 1;
    if (!(KEYPAD_PIN & (1 << KEYPAD_KEY_2))) return 2;
    if (!(KEYPAD_PIN & (1 << KEYPAD_KEY_3))) return 3;
    if (!(KEYPAD_PIN & (1 << KEYPAD_KEY_4))) return 4;
    return KEYPAD_NO_KEY;
}


// display os firmaware functions
/* 
    format a float value into a string with specified precision and optional suffix
    params: char* dest - destination buffer to store the formatted string
            size_t dest_size - size of the destination buffer
            float value - the float value to format
            uint8_t precision - number of decimal places
            const char* suffix - optional suffix to append
    returns: void
*/
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

/* 
    set the display with title and data strings
    params: const unsigned char* title - title string (max 12 chars)
            const unsigned char* data - data string (max 16 chars)
    returns: void
*/
void display_set(const unsigned char *title, const unsigned char *data)
{
    // LCD_1602A_load_command(DISPLAY_CLEAR_SCREEN);
    _delay_ms(2);

    unsigned char line1[12];
    unsigned char line2[16];

    // prepare line1: copy up to 12 chars from title, pad with spaces
    int i = 0;
    for (; i < 12 && *title; ++i)
        line1[i] = *title++;

    for (; i < 12; ++i)
        line1[i] = ' ';

    /* custom characters created at init; just write the codes */

    // prepare line2: copy up to 16 chars from data, pad with spaces
    i = 0;
    for (; i < 16 && *data; ++i)
        line2[i] = *data++;

    // LCD_1602A_load_data(0);

    for (; i < 16; ++i)
        line2[i] = ' ';

    // reset cursor to first line and write 12 chars
    LCD_1602A_load_command(DISPLAY_SET_CURSOR_LINE_1);
    for (i = 0; i < 12; ++i)
    {
        LCD_1602A_load_data(line1[i]);
    }

    LCD_1602A_load_data(2);
    LCD_1602A_load_data(0);
    LCD_1602A_load_data(1);
    LCD_1602A_load_data(3);

    // reset cursor to second line and write 16 chars
    LCD_1602A_load_command(DISPLAY_SET_CURSOR_LINE_2);
    for (i = 0; i < 16; ++i)
    {
        LCD_1602A_load_data(line2[i]);
    }
}


float get_tank_capacity_at_height(int water_depth)
{
    float water_height = (float)TANK_HEIGHT_IN_CM - (float)water_depth;

    // // The following formula assumes the tank is a perfect cylinder: V = π * r^2 * h
    float volume = (PI * ((float)TANK_RADIUS_IN_CM * (float)TANK_RADIUS_IN_CM) * water_height) / 1000.0; // convert cm^3 to liters

    if (volume < 0.0)
    {
        volume = 0.0;
    }

    return volume;
}

// user application
/* 
    get the tank capacity based on sonar distance measurement
    params: void
    returns: float - the tank capacity in liters
*/
float get_tank_capacity()
{

    uint16_t water_depth = HCSR04_get_distance();
    water_depth *= 1.5; // convert to float

    return get_tank_capacity_at_height(water_depth);
}


/* 
    get the refill rate based on tank capacity change over time
    params: void
    returns: float - the refill rate in liters per minute
*/
float get_refill_rate()
{
    int height_1;
    peekRear(&height_per_second, &height_1);

    int height_2;
    peekFront(&height_per_second, &height_2);

    float capacity_at_1 = get_tank_capacity_at_height(height_1 * 1.5);
    float capacity_at_2 = get_tank_capacity_at_height(height_2 * 1.5);

    if (capacity_at_1 > capacity_at_2)
    {
        return 0.0; // no leak detected
    }

    float leak_rate_per_second = (capacity_at_2 - capacity_at_1) / 2.0;

    return leak_rate_per_second * 60.0; // convert to liters per minute
}

/*
    get the leak rate based on tank capacity change over time
    params: void
    returns: float - the leak rate in liters per minute
*/
float get_leak_rate()
{
    int height_1;
    peekRear(&height_per_second, &height_1);

    int height_2;
    peekFront(&height_per_second, &height_2);

    float capacity_at_1 = get_tank_capacity_at_height(height_1 * 1.5);
    float capacity_at_2 = get_tank_capacity_at_height(height_2 * 1.5);

    if (capacity_at_1 < capacity_at_2) {
        return 0.0; // no leak detected
    }
    float leak_rate_per_second = (capacity_at_1 - capacity_at_2) / queueSize(&height_per_second);

    return leak_rate_per_second * 60.0; // convert to liters per minute
}

float get_soil_temperature()
{
    return DS18B20_read_temperature();
}

// user interface functions
/* 
    update the display based on the current active menu and hover indices
    params: void
    returns: void
*/
void ui_show_display(void)
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
            display_set("TANK CAPACITY", "Loading...");
            format_float(buffer, sizeof(buffer), get_tank_capacity(), 1, "Ltrs");
            display_set("TANK CAPACITY", buffer);
            break;
        case 1:
            display_set("REFILL RATE", "Loading...");
            format_float(buffer, sizeof(buffer), get_refill_rate(), 1, "Ltrs per hour");
            display_set("REFILL RATE", buffer);
            break;
        case 2:
            display_set("LEAKAGE RATE", "Loading...");
            format_float(buffer, sizeof(buffer), get_leak_rate(), 1, "Ltrs per min");
            display_set("LEAKAGE RATE", buffer);
            break;
        case 3:
            display_set("SOIL TEMPERATURE", "Loading...");
            format_float(buffer, sizeof(buffer), get_soil_temperature(), 1, "degrees");
            display_set("SOIL TEMPERATURE", buffer);
            break;
        case 4:
            if (active_live_view_index == -1)
            {
                display_set("LIVE VIEW", LIVE_VIEW_BUFFER[live_view_hover_index]);
            }
            else
            {
                switch (live_view_hover_index)
                {
                case 0:
                    format_float(buffer, sizeof(buffer), current_tank_capacity, 1, "Ltrs");
                    display_set("CAPACITY", buffer);
                    break;
                case 1:
                    format_float(buffer, sizeof(buffer), current_refill_rate, 1, "Ltrs per min");
                    display_set("REFILL RATE", buffer);
                    break;
                case 2:
                    format_float(buffer, sizeof(buffer), current_leak_rate, 1, "Ltrs per min");
                    display_set("LEAK RATE", buffer);
                    break;
                case 3:
                    format_float(buffer, sizeof(buffer), current_soil_temperature, 1, "degrees");
                    display_set("SOIL TEMP", buffer);
                    break;
                default:
                    display_set("ERROR", "Invalid live view");
                    break;
                }
            }
            break;
        case 5:
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

        case 6:
            // if the messages queue is not empty, show the first (oldest) message
			{
                char *msgptr = NULL;
                if (!isStrQueueEmpty(&MESSAGES_Q))
                {
                    if (peekFrontStr(&MESSAGES_Q, &msgptr) == 0 && msgptr)
                    {
                        display_set("MESSAGES", msgptr);
                    }
                } else 
                {
                    display_set("MESSAGES", "No messages");
				}
            }
            break;
        case 7:
            if (active_config_index == -1)
            {
                display_set("CONFIG", CONFIG_BUFFER[config_hover_index]);
            }
            else
            {
                switch (active_config_index)
                {
                case 0:
                    format_float(buffer, sizeof(buffer), STEP_SIZE_FOR_INCREMENTS, 1, "Units");
                    display_set("STEP SIZE", buffer);
                    break;
                case 1:
                    format_float(buffer, sizeof(buffer), PUMP_THRESHOLD, 1, "Ltrs");
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

// user process command
/* 
    process a key command from the keypad and update the UI state
    params: uint8_t key - the key number (1-4)
    returns: void
*/
void ui_process_key_command (uint8_t key) {
    if (key == 1)
    {
        if (active_menu_index == -1)
        {
            return;
        }
        else
        {
            if (active_trigger_index != -1)
            {
                active_trigger_index = -1;
            }
            else if (active_config_index != -1)
            {
                active_config_index = -1;
            }
            else if (active_live_view_index != -1)
            {
                active_live_view_index = -1;
            }
            else
            {
                active_menu_index = -1;
            }
        }
    }
    else if (key == 2)
    {
        if (active_menu_index == -1)
        {
            if (menu_hover_index < (sizeof(MENU_BUFFER)/sizeof(MENU_BUFFER[0]) - 1))
                menu_hover_index++; // navigate downwards on the main menu
        }
        else if (active_menu_index == 4)
        {
            // user is in the live view menu
            if (live_view_hover_index < (sizeof(LIVE_VIEW_BUFFER)/sizeof(LIVE_VIEW_BUFFER[0]) - 1))
                live_view_hover_index++; // navigate downwards on the live view menu
        }
        else if (active_menu_index == 5) 
        {
            // user is in the triggers menu
            if (active_trigger_index == -1)
            {
                if (trigger_hover_index < (sizeof(TRIGGERS_BUFFER)/sizeof(TRIGGERS_BUFFER[0]) - 1))
                    trigger_hover_index++; // navigate downwards on the triggers menu
            } else {
                switch (active_trigger_index) {
                    case 0:
                        // adjust MINIMUM_CAPACITY_BEFORE_REFILLING_TRIGGER - decrease
                        MINIMUM_CAPACITY_BEFORE_REFILLING_TRIGGER -= STEP_SIZE_FOR_INCREMENTS;
                        break;
                    case 1:
                        // adjust MAXIMUM_TEMPERATURE_BEFORE_PUMPING - decrease
                        MAXIMUM_TEMPERATURE_BEFORE_PUMPING -= STEP_SIZE_FOR_INCREMENTS;
                        break;
                    default:
                        break;
                }
            }
        }
        else if (active_menu_index == 6) {
            if (message_hover_index < MSG_BUFFER_SIZE - 1)
                message_hover_index++;
        }
        else if (active_menu_index == 7)
        {
            if (active_config_index == -1)
            {
                if (config_hover_index < (sizeof(CONFIG_BUFFER)/sizeof(CONFIG_BUFFER[0]) - 1))
                    config_hover_index++; // navigate downwards on the config menu
            } else {
                switch (active_config_index)
                {
                case 0:
                    // adjust STEP_SIZE_FOR_INCREMENTS - decrease
                    STEP_SIZE_FOR_INCREMENTS -= STEP_INCREMENT;
                    break;
                case 1:
                    // adjust PUMP_THRESHOLD - decrease
                    PUMP_THRESHOLD -= STEP_SIZE_FOR_INCREMENTS;
                    break;
                case 2:
                    // adjust SPRAY_THRESHOLD - decrease
                    SPRAY_THRESHOLD -= STEP_SIZE_FOR_INCREMENTS;
                    break;
                case 3:
                    // adjust ENABLE_TRIGGER_VALUE - toggle
                    ENABLE_TRIGGER_VALUE = 0;
                    break;
                case 4:
                    // adjust ENABLE_ALERT_VALUE - toggle
                    ENABLE_ALERT_VALUE = 0;
                    break;
                default:
                    break;
                }
            }
        }
    }
    else if (key == 3)
    {
        if (active_menu_index == -1)
        {
            if (menu_hover_index > 0)
                menu_hover_index--;
        }
        else if (active_menu_index == 4)
        {
            // user is in the live view menu
            if (live_view_hover_index > 0)
                live_view_hover_index--;
        }
        else if (active_menu_index == 5)
        {
            if (active_trigger_index == -1)
            {
                if (trigger_hover_index > 0)
                    trigger_hover_index--;
            } else {
                switch (active_trigger_index) {
                    case 0:
                        // adjust MINIMUM_CAPACITY_BEFORE_REFILLING_TRIGGER - decrease
                        MINIMUM_CAPACITY_BEFORE_REFILLING_TRIGGER += STEP_SIZE_FOR_INCREMENTS;
                        break;
                    case 1:
                        // adjust MAXIMUM_TEMPERATURE_BEFORE_PUMPING - decrease
                        MAXIMUM_TEMPERATURE_BEFORE_PUMPING += STEP_SIZE_FOR_INCREMENTS;
                        break;
                    default:
                        break;
                }
            }
        } 
        else if (active_menu_index == 6) {
            if (message_hover_index > 0)
                message_hover_index--;
        }
        else if (active_menu_index == 7)
        {
            if (active_config_index == -1)
            {
                if (config_hover_index > 0)
                    config_hover_index--;
            } else {
            switch (active_config_index)
                {
                case 0:
                    // adjust STEP_SIZE_FOR_INCREMENTS - decrease
                    STEP_SIZE_FOR_INCREMENTS += STEP_INCREMENT;
                    break;
                case 1:
                    // adjust PUMP_THRESHOLD - decrease
                    PUMP_THRESHOLD += STEP_SIZE_FOR_INCREMENTS;
                    break;
                case 2:
                    // adjust SPRAY_THRESHOLD - decrease
                    SPRAY_THRESHOLD += STEP_SIZE_FOR_INCREMENTS;
                    break;
                case 3:
                    // adjust ENABLE_TRIGGER_VALUE - toggle
                    ENABLE_TRIGGER_VALUE = 1;
                    break;
                case 4:
                    // adjust ENABLE_ALERT_VALUE - toggle
                    ENABLE_ALERT_VALUE = 1;
                    break;
                default:
                    break;
                }
            }
        }
    }
    else if (key == 4)
    {
        if (active_menu_index == -1)
        {
            active_menu_index = menu_hover_index;
        }
        else if (active_menu_index == 4)
        {
            if (active_live_view_index == -1)
            {
                active_live_view_index = live_view_hover_index;
            }
        }
        else if (active_menu_index == 5)
        {
            if (active_trigger_index == -1)
            {
                active_trigger_index = trigger_hover_index;
            }
        }
        else if (active_menu_index == 7)
        {
            if (active_config_index == -1)
            {
                active_config_index = config_hover_index;
            }
        }
    }
}

ISR(TIMER2_OVF_vect)
{
    /* Keep ISR short: only update tick and set a flag for main loop to do sensor work */
    tick++;

    if (tick >= 248) {
        tick = 0;
        one_second_event = 1;
    }
}

int main(void)
{
    initQueue(&height_per_second);
    initStrQueue(&MESSAGES_Q);
    LCD_1602A_init();
    HCSR04_init();
    KEYPAD_init();
    LED_SYSTEM_ACTIVE_INIT();
    TIMER2_INIT();

    uint8_t pressed_key = KEYPAD_NO_KEY;
	
	int tank_height = HCSR04_get_distance();
    enqueue(&height_per_second, tank_height);

    while (1)
    {
        ui_show_display();


        if (SYSTEM_SIGNAL) {
            char *msgptr = NULL;
            SYSTEM_SIGNAL = 0;
            peekFrontStr(&MESSAGES_Q, &msgptr);
            display_set("ALERT", msgptr ? msgptr : "System alert");
            _delay_ms(1000); // show alert briefly
        }

        if (one_second_event) {
            one_second_event = 0;

            int tank_height = HCSR04_get_distance();
            enqueue(&height_per_second, tank_height);

            float capacity = get_tank_capacity_at_height(tank_height * 1.5);
            float soil_temp = get_soil_temperature();

            if (ENABLE_TRIGGER_VALUE) {
                if (capacity <= MINIMUM_CAPACITY_BEFORE_REFILLING_TRIGGER) {
                    char msg[64];
                    snprintf(msg, sizeof(msg), "Refill triggered at %.1f Ltrs", capacity);
                    enqueueStr(&MESSAGES_Q, msg);
                    SYSTEM_SIGNAL = 1;
                }

                if (soil_temp >= MAXIMUM_TEMPERATURE_BEFORE_PUMPING) {
                    char msg[64];
                    snprintf(msg, sizeof(msg), "Cooling triggered at %.2f degrees", soil_temp);
                    enqueueStr(&MESSAGES_Q, msg);
                    SYSTEM_SIGNAL = 1;
                }
            }
        }

        while (active_menu_index == 4 && active_live_view_index != -1)
        {
            while ((pressed_key = KEYPAD_read()) == KEYPAD_NO_KEY)
            {

                switch (live_view_hover_index)
                {
                    case 0:
                        if (!(SHOW_LOADING_WIDGET)) {
                            display_set("CAPACITY", "Updating...");
                            SHOW_LOADING_WIDGET = 1;
                            current_tank_capacity = get_tank_capacity();
                        }
                        else {
                            format_float(buffer, sizeof(buffer), current_tank_capacity, 1, "Ltrs");
                            display_set("CAPACITY", buffer);
                        }

                        break;
                    case 1:
                        if (!(SHOW_LOADING_WIDGET))
                        {
                            display_set("REFILL RATE", "Updating...");
                            SHOW_LOADING_WIDGET = 1;
                            current_refill_rate = get_refill_rate();
                        } else {
                            format_float(buffer, sizeof(buffer), current_refill_rate, 1, "Ltrs per min");
                            display_set("REFILL RATE", buffer);
                        }
                        break;
                    case 2:
                        if (!(SHOW_LOADING_WIDGET))
                        {
                            display_set("LEAK RATE", "Updating...");
                            SHOW_LOADING_WIDGET = 1;
                            current_leak_rate = get_leak_rate();
                        } else {
                            format_float(buffer, sizeof(buffer), current_leak_rate, 1, "Ltrs per min");
                            display_set("LEAK RATE", buffer);
                        }
                        
                        break;

                    case 3:
                        if (!(SHOW_LOADING_WIDGET))
                        {
                            display_set("SOIL TEMP", "Updating...");
                            SHOW_LOADING_WIDGET = 1;
                            current_soil_temperature = get_soil_temperature();
                        } else {
                            format_float(buffer, sizeof(buffer), current_soil_temperature, 1, "degrees");
                            display_set("SOIL TEMP", buffer);
                        }
                        
                        break;
                }
                
            }

            SHOW_LOADING_WIDGET = 0;

            if (pressed_key == 1)
            {
                // exit live view on key 1 press
                active_menu_index = -1;
                pressed_key = KEYPAD_NO_KEY;
                break;
            } else {
                ui_process_key_command(pressed_key);
            }

            while (KEYPAD_read() != KEYPAD_NO_KEY);
        }


        // _delay_ms(20);

        if (pressed_key == KEYPAD_NO_KEY) {
            LED_system_active_on();
            while ((pressed_key = KEYPAD_read()) == KEYPAD_NO_KEY);
            LED_system_active_off();
        }
        
        // _delay_ms(20);
        if (KEYPAD_read() == pressed_key)
        {
            ui_process_key_command(pressed_key);

            while (KEYPAD_read() != KEYPAD_NO_KEY); // wait until the key is released
        }

        pressed_key = KEYPAD_NO_KEY;
    }
}