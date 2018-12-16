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

/* It would be a good idea to describe what this ISR does, just to be clear.
 * It appears you are trying to increment a counter and possibly toggle an LED?
 * The toggle may be too fast to be noticable, given it would take a cycle or two 
 * to actually toggle. */
ISR(TIMER1_COMPA_vect) {
  toggle_bit(M0S_PORT,M0S);
  stepCounter++;
}

void waitForButton(void){ 
  while(bit_is_set(Button_PIN, Button)){  /* Depending upon the return value, this may be skipped and not wait for it to be set */
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
  set_bit(TIMSK1, OCIE1A); 	// turn on interrupts, stepping    /* You may want to make a macro for turning this interrupt on for ease of reading */
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
  while (INC <= NUMBER_INC){		/* Be careful about overflow here since you are using unsigned numbers */
    takeSteps(PRESSURE_INC, forward);
    waitForButton();
    INC++; 
  }	 
	
  INC = 1;
  while (INC <= NUMBER_INC*2){ 		/* Be careful about overflow here since you are using unsigned numbers */
    takeSteps(PRESSURE_INC,backward);
    INC++;
  }
	
  takeSteps(OVERSLOP,backward);
  _delay_ms(500); // Direction change avoid skip steps
  takeSteps(OVERSLOP,forward);
  waitForButton();
	
  INC = 1;
  while (INC <= NUMBER_INC){		/* Be careful about overflow here since you are using unsigned numbers */
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
  /* With embedded systems, you usually don't want a reutrn here
  With GCC, you can use the following to make a non returnable function
  and cause it to not remove an infinite loop at the end of your function (Most likley)

  __attribute__((noreturn)) void main (void ){ 
  
  	while( 1 );	Loop to catch errors; get stuck here. maybe watchdog will reset otherwise if you enabeld it
  }

  */
}

