// AtMega328P 16MHz
#ifndef F_CPU
  #define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>

#define LED_PIN_LEFT (1<<7)
#define LED_PIN_RIGHT (1<<6)
#define SW_PIN_LEFT (1<<5)
#define SW_PIN_RIGHT (1<<4)
#define ENABLE_SENSOR_RIGHT 1
#define ENABLE_SENSOR_LEFT 0

int data_sensor[11][10];
int data_avg[11];

static void gpio_init(void){
  DDRD |= LED_PIN_LEFT | LED_PIN_RIGHT | SW_PIN_LEFT | SW_PIN_RIGHT;
  PORTD &= ~LED_PIN_LEFT | ~LED_PIN_RIGHT | ~SW_PIN_LEFT | ~SW_PIN_RIGHT;
  DDRC = 0x3F;
}

static void gpio_write(uint8_t p, uint8_t state){
  PORTD &= p;

  if(state)
    PORTD |= p;
}

static void uart_init(void){
  UBRR0 = 8;
  UCSR0B |= (1<<TXEN0);
  UCSR0C |= (1<<UCSZ00) | (1<<UCSZ01);
}

static void uart_transmit(char c){
  while(!(UCSR0A & (1<<UDRE0)));
  UDR0 = c;
}

static void uart_transmits(char* s){
  while(*s){
    uart_transmit(*s);
    s++;
  }
}
static void adc_init(uint8_t p){
  ADCSRA |= (1<<ADEN);
  ADCSRA |= (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2);

  ADMUX = 0x80;
  ADMUX = 0x80 | p;
}

static uint16_t adc_read(uint8_t p){
  uint16_t adc_avg = 0;
  adc_init(p);
  ADCSRA |= (1<<ADSC);
  for(uint8_t x=0;x<10;x++){
    while(ADCSRA & (1<<ADSC));
    adc_avg += ADCW;
  }
  return (adc_avg/10);
}

static uint8_t baca_sensor(uint8_t index){
  int temp = 0;
  PORTD |= SW_PIN_LEFT;
    for(int y=0;y<5;y++){
      for(int x=0;x<10;x++){
        data_sensor[y][x] = adc_read(y);
      }
    }
  PORTD &= ~SW_PIN_LEFT;
  PORTD |= SW_PIN_RIGHT;
    for(int y=5;y<11;y++){
      for(int x=0;x<10;x++){
        data_sensor[y][x] = adc_read(y);
      }
    }
  PORTD &= ~SW_PIN_RIGHT;

  for(int y=0;y<11;y++){
    for(int x=0;x<10;x++){
      temp += data_sensor[y][x];
    }
  
    temp /= 10;
    data_avg[y] = temp;
    temp = 0;
  }
  
  return data_avg[index];
}

int main(void){
  gpio_init();
  uart_init();
  gpio_write((LED_PIN_LEFT | LED_PIN_RIGHT),1);
  // uart_transmit(((PIND & (1<<6)) + 48));

  char data_str[10];
  
  while(1){
    // break;
    for (uint8_t x=0;x<11;x++){
      itoa(baca_sensor(y), data_str, 10);
      uart_transmits(data_str);
      uart_transmit(';');
    }

    uart_transmits((char*)"\r\n");// carriege return // newline
  }
}
