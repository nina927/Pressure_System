#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include "pinDefines.h"

#define M0S          0
#define M0D          1
#define M0S_DDR      DDRD
#define M0D_DDR      DDRD
#define M0S_PORT     PORTD
#define M0D_PORT     PORTD

#define Button       2
#define Button_DDR   DDRD  
#define Button_PORT  PORTD
#define Button_PIN   PIND

#define BV(bit)               (1 << bit)
#define set_bit(sfr, bit)     (_SFR_BYTE(sfr) |= BV(bit))
#define clear_bit(sfr, bit)   (_SFR_BYTE(sfr) &= ~BV(bit))
#define toggle_bit(sfr, bit)  (_SFR_BYTE(sfr) ^= BV(bit))  

#define MICROSTEPS    16
#define PRESSURE_INC  71      //set for 0.25 Pa and 3mL syringe
#define NUMBER_INC    8       // set for +/-2 Pa
#define OVERSLOP      200      
#define DELAY         2499     // frequency=100Hz maybe??


// -------- Global Variables --------- //

volatile uint16_t stepCounter = 0;
volatile uint8_t  ButtonState = 0;

// ------ Functions --------- //

void initMotorsButton(void){
  set_bit(M0S_DDR,M0S);
  set_bit(M0D_DDR,M0D);
  clear_bit(Button_DDR,Button);
  set_bit(Button_PORT, Button);
  _delay_ms(1000);
}

void initTimer(void) {
  clock_prescale_set(clock_div_1);   // CPU clock 8 MHz
  set_bit(TCCR1B,WGM12);             // CTC mode
  set_bit(TCCR1B,CS10);              // no prescaler
  OCR1A = DELAY;
  sei();                            // enable global interrupts
  // Notice we haven't set the timer1 interrupt flag yet
}

ISR(TIMER1_COMPA_vect) {
  toggle_bit(M0S_PORT,M0S);
  stepCounter++;
}

void waitForButton(void){ 
  while(bit_is_set(Button_PIN, Button)){ 
  }
  _delay_ms(20);
  while(bit_is_clear(Button_PIN, Button)){    
  }
}

void takeSteps(uint16_t howManySteps, uint8_t direction) {
  if (!direction) {
    clear_bit(M0D_PORT,M0D);
  }
  else {
    set_bit(M0D_PORT,M0D);
  }
  stepCounter = 0;               // initialize steps
  set_bit(TIMSK1, OCIE1A); 	// turn on interrupts, stepping     
  while (!(stepCounter == howManySteps*2*MICROSTEPS)) {;
  }                            // wait
  clear_bit(TIMSK1,OCIE1A);   // turn back off
}


void pressureCycle(void){

  uint8_t forward = 0;
  uint8_t backward = 1;
	
  takeSteps(OVERSLOP,backward);
  takeSteps(OVERSLOP,forward);
  waitForButton();
	
  // Move forward to 10 Pa, 2 Pa
  uint8_t INC = 1;
  while (INC <= NUMBER_INC){
    takeSteps(PRESSURE_INC, forward);
    waitForButton();
    INC++; 
  }	 
	
  INC = 1;
  while (INC <= NUMBER_INC*2){
    takeSteps(PRESSURE_INC,backward);
    INC++;
  }
	
  takeSteps(OVERSLOP,backward);
  _delay_ms(500); // Direction change avoid skip steps
  takeSteps(OVERSLOP,forward);
  waitForButton();
	
  INC = 1;
  while (INC <= NUMBER_INC){
    takeSteps(PRESSURE_INC,forward);
    waitForButton();
    INC++;
  }
	
}


int main(void) {
  initMotorsButton();
  initTimer();
  pressureCycle();
  return 0;
}

