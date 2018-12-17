#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include "pinDefines.h"

#define M0S          0       // Motor 0 Step
#define M0D          1       // Motor 0 Direction
#define M0S_DDR      DDRD
#define M0D_DDR      DDRD
#define M0S_PORT     PORTD
#define M0D_PORT     PORTD

#define Button       2
#define Button_DDR   DDRD  
#define Button_PORT  PORTD
#define Button_PIN   PIND

#define BV(bit)                (1 << bit)
#define set_bit(sfr, bit)     (_SFR_BYTE(sfr) |= BV(bit))
#define clear_bit(sfr, bit)   (_SFR_BYTE(sfr) &= ~BV(bit))
#define toggle_bit(sfr, bit)  (_SFR_BYTE(sfr) ^= BV(bit))
#define turn_on_interrupt       set_bit(TIMSK1, OCIE1A);
#define turn_off_interrupt      clear_bit(TIMSK1, OCIE1A);

#define MICROSTEPS    16
#define PRESSURE_INC  71      /* steps to apply one pressure increment (set for 0.25 Pa with  3mL syringe) */
#define NUMBER_INC    8       /* number of pressure increments to get to max pressure (set for +/-2 Pa) */
#define OVERSLOP      200     /* steps to overcome hysteresis of syringe piston */     
#define DELAY         2499     /* frequency=100Hz maybe?? */


// -------- Global Variables --------- //

volatile uint16_t stepCounter = 0;
volatile uint8_t  ButtonState = 0; // WHAT IS THIS

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
  /*  Notice we haven't set the timer1 interrupt flag yet */
}

/* ISR toggles motor step pin  */
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
  turn_on_interrupt;    	// start stepping
  while (!(stepCounter == howManySteps*2*MICROSTEPS)) {;
  }                            // wait
  turn_off_interrupt;          // stop stepping
}


void pressureCycle(void){

  uint8_t forward = 0;
  uint8_t backward = 1;

  /* Move back and forth to avoid slop */
  takeSteps(OVERSLOP,backward);
  _delay_ms(500); // avoid skipping steps on direction change 
  takeSteps(OVERSLOP,forward);
  waitForButton();
	
  /*  Move forward from zero to max pressure */
  uint8_t INC = 1;
  while (INC <= NUMBER_INC){		/* Be careful about overflow  */
    takeSteps(PRESSURE_INC, forward);
    waitForButton();
    INC++; 
  }	 

  /* Move backwards from max to min pressure */	
  INC = 1;
  while (INC <= NUMBER_INC*2){		/* Be careful about overflow  */
    takeSteps(PRESSURE_INC,backward);
    INC++;
  }
  
  /* Move back and forth to avoid slop */
  takeSteps(OVERSLOP,backward);
  _delay_ms(500);
  takeSteps(OVERSLOP,forward);
  waitForButton();

  /* Move forward from min pressure to zero */
  INC = 1;
  while (INC <= NUMBER_INC){		/* Be careful about overflow  */
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

