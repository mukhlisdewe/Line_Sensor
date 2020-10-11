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

double PV=0;
char* t[6];


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

uint16_t baca_sensor (){
  uint16_t sensor=0;
  
  PORTD |= SW_PIN_LEFT;
  if((PINC & (1<<0))==0)sensor=sensor |0b10000000000;
  if((PINC & (1<<1))==0)sensor=sensor |0b01000000000;
  if((PINC & (1<<2))==0)sensor=sensor |0b00100000000;
  if((PINC & (1<<3))==0)sensor=sensor |0b00010000000;
  if((PINC & (1<<4))==0)sensor=sensor |0b00001000000;
  if((PINC & (1<<5))==0)sensor=sensor |0b00000100000;
  PORTD &= ~SW_PIN_LEFT;
  PORTD |= SW_PIN_RIGHT;
  if((PINC & (1<<0))==0)sensor=sensor |0b00000000001;
  if((PINC & (1<<1))==0)sensor=sensor |0b00000000010;
  if((PINC & (1<<2))==0)sensor=sensor |0b00000000100;
  if((PINC & (1<<3))==0)sensor=sensor |0b00000001000;
  if((PINC & (1<<4))==0)sensor=sensor |0b00000010000;
  PORTD &= ~SW_PIN_RIGHT;

  return sensor;
}

int map_case(){
  switch(baca_sensor()) {
  
    case 0b00000000001  :   PV= 30; break;
    case 0b00000000011  :   PV= 5; break;
    case 0b00000000111  :   PV= 4; break;
    case 0b00000001110  :   PV= 3; break;
    case 0b00000011100  :   PV= 2; break;
    case 0b00000111000  :   PV= 1; break;

    case 0b00001110000  :   PV= 0; break;  //lurus

    case 0b00011100000  :   PV= -1; break;
    case 0b00111000000  :   PV= -2; break;
    case 0b01110000000  :   PV= -3; break;
    case 0b11100000000  :   PV= -4; break;
    case 0b11000000000  :   PV= -5; break;
    case 0b10000000000  :   PV= -30; break;

    case 0b11111111111  :  
      if (PV>=-3&&PV<=3){ PV=0; }
      else {
        if (PV>0){ PV=10; }
        else {PV=-10; }}
    break;

    case 0b00000000000  : 
      if (PV>0){ PV=12; }
      else {PV=-12; }
    break;
                  
  }
      return PV;
}

void pid (){
    char temp[10];
    uint16_t M1, M2;
    double error=0, setpoint=0, error1=0, outPID=0;
    double kp=30, kd=10, dt=0.020, def_ka = 150, def_ki=160;
    error=setpoint-PV;
    outPID = (kp*error) + ((kd/dt)*(error-error1));
    error1=error;
    M1 = def_ka + outPID;
    M2 = def_ki - outPID;


    //max PWM
    if (M1>255){
        M1=255;
    }
    else if (M1 < (unsigned)-255){
        M1=-255;
    }

    if (M2>255){
        M2=255;
    }
    else if (M2 < (unsigned)-255){
        M2=-255;
    }

    if (M1<0) t[0]=(char*)"-";
    else t[0]=(char*)"+";
    
    M1=(M1<0?(M1*-1):M1);
    itoa (M1,temp,10);
    t[1]=temp;

    if (M2<0) t[2]=(char*)"-";
    else t[2]=(char*)"+";

    M2=(M2<0?(M2*-1):M2);
    itoa (M2,temp,10);
    t[3]=temp;    

    if (error<0) t[4]=(char*)"-";
    else t[4]=(char*)"+";

    error=(error<0?(error*-1):error);
    itoa (error,temp,10);
    t[5]=temp; 
    
}

int main(void){
  gpio_init();
  uart_init();
  uint8_t x;
  uart_transmit(((PIND & (1<<6)) + 48));
  int a;

  while(1){
    // break;
    gpio_write((LED_PIN_LEFT | LED_PIN_RIGHT),1);
    map_case();
    pid();
    for (x=0;x<6;x++){
      uart_transmits(t[x]);
      uart_transmit(';');
    }
    // a = PINC & (1<<0);
    // uart_transmits(sensor);
    uart_transmits((char*)"\r\n");// carriege return // newline
  }
}