#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/power.h>

#define M0S          0       /*  Motor Step Pin */
#define M0D          1       /*  Motor Direction Pin */
#define M0S_DDR      DDRD    
#define M0D_DDR      DDRD
#define M0S_PORT     PORTD
#define M0D_PORT     PORTD

#define Button       2       /*  Button Pin, other side goes to ground */
#define Button_DDR   DDRD  
#define Button_PORT  PORTD
#define Button_PIN   PIND

#define BV(bit)                (1 << bit)
#define set_bit(sfr, bit)     (_SFR_BYTE(sfr) |= BV(bit))
#define clear_bit(sfr, bit)   (_SFR_BYTE(sfr) &= ~BV(bit))
#define toggle_bit(sfr, bit)  (_SFR_BYTE(sfr) ^= BV(bit))
#define turn_on_interrupt       set_bit(TIMSK1, OCIE1A);    /*  Set timer 1 interrupt flag */
#define turn_off_interrupt      clear_bit(TIMSK1, OCIE1A);  /*  Clear timer 1 interrupt flag */

#define MICROSTEPS    16        /* 1/16th microstepping */
#define CLK_FREQ      8000000  /* 8 MHz clock frequency */
#define PRESCALER     1        /* No prescaler */
#define STEP_FREQ     100      /* motor frequency in Hz */

#define PRESSURE_INC  71      /* steps to apply one pressure increment (set for 0.25 Pa with  3mL syringe) */
#define NUMBER_INC    8       /* number of pressure increments to get to max pressure (set for +/-2 Pa) */
#define OVERSLOP      200     /* steps to overcome hysteresis of syringe piston */     

#define FORWARD       0      /* Forward direction bit */
#define BACKWARD      1      /* Backward direction bit */

/*  -------- Global Variables --------- */

volatile uint16_t stepCounter = 0;
static int period = (CLK_FREQ/2/PRESCALER/MICROSTEPS/STEP_FREQ)-1; /* calculate stepping period from frequency */

/* ------ Functions --------- */

void initMotorsButton(void){
  set_bit(M0S_DDR,M0S);     /* Step pin in output mode */
  set_bit(M0D_DDR,M0D);     /* Direction pin in output mode */
  clear_bit(Button_DDR,Button); /* Set button pin in input mode */
  set_bit(Button_PORT, Button); /* Set pullup resistor  */
  _delay_ms(1000);
}

void initTimer(void) {
  clock_prescale_set(clock_div_1);   /*  CPU clock 8 MHz */
  set_bit(TCCR1B,WGM12);             /*  CTC mode */
  set_bit(TCCR1B,CS10);              /*  no prescaler */
  OCR1A = period;
  sei();                            /* enable global interrupts */
  /*  Notice we haven't set the timer1 interrupt flag yet */
}


ISR(TIMER1_COMPA_vect) {      /* ISR toggles motor step pin  */
  toggle_bit(M0S_PORT,M0S);
  stepCounter++;
}


void waitForButton(void){
  while(bit_is_set(Button_PIN, Button)){    /* while button is not pressed */
  }                                         /* wait for button to be pressed */
  _delay_ms(20);                            /* Debounce delay */
  while(bit_is_clear(Button_PIN, Button)){  /* while button is pressed */
  }                                         /*  wait until button is let go  */
}


void takeSteps(uint16_t howManySteps, uint8_t direction) {
  if (!direction) {
    clear_bit(M0D_PORT,M0D);  /* set in forward direction */
  }
  else {
    set_bit(M0D_PORT,M0D);    /* set in backward direction */
  }
  stepCounter = 0;            /*  initialize step counter */
  turn_on_interrupt;          /*  start stepping */
  while (!(stepCounter == howManySteps*2*MICROSTEPS)) {;
  }                            /* wait */
  turn_off_interrupt;          /* stop stepping */
}

void overslopAvoidance(void){   /* Move back and forth to avoid slop */
  takeSteps(OVERSLOP,BACKWARD);
  _delay_ms(500);    /* avoid skipping steps on direction change */
  takeSteps(OVERSLOP,FORWARD);
  waitForButton();
}

void moveForward(void){     /* Move forward one inc at a time, pausing for button press */
  uint8_t inc = 1;
  while (inc <= NUMBER_INC){
    takeSteps(PRESSURE_INC, FORWARD);
    waitForButton();
    inc++;
  }
}

void moveBackward(void){   /* Move backward two inc */
  uint8_t inc = 1;
  while (inc <= 2*NUMBER_INC){
    takeSteps(PRESSURE_INC, BACKWARD);
    inc++;
  }
}

void pressureCycle(void){
  overslopAvoidance();   /* Move back and forth to avoid slop */
  moveForward();        /* Move forward from zero to max pressure */
  moveBackward();       /* Move backward from max to min pressure */
  overslopAvoidance();  /* Move back and forth to avoid slop */
  moveForward();       /* Move forward from min pressure to zero */	
}


int main(void){
  initMotorsButton();
  initTimer();  
  while (1){
  pressureCycle();
  }  
  return 0;
}

