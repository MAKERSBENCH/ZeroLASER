#include "Laser.h"
#include "Analog.h"

static bool      enabled;
static bool      on;
static float     current;
static uint16_t  pwm;
static byte      powerLevel;

void LaserInit() {

  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 (and TCC1)
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;    // Feed GCLK4 to TCC0 (and TCC1)
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(1) |          // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the digital pin D2 
  PORT->Group[g_APinDescription[LASER_PWM_PIN].ulPort].PINCFG[g_APinDescription[LASER_PWM_PIN].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to digital output D2 - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[LASER_PWM_PIN].ulPort].PMUX[g_APinDescription[LASER_PWM_PIN].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F; 

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: 
  REG_TCC0_PER = LASER_PWM_PERIOD;                             // Set the frequency of the PWM on TCC0 to 250kHz
  while (TCC0->SYNCBUSY.bit.PER);                 // Wait for synchronization
  
  // Set the PWM pin duty cycle
  REG_TCC0_CC0 = LASER_PWM_PERIOD+1;              // TCC0 CC0 - on D2
  while (TCC0->SYNCBUSY.bit.CC0);                 // Wait for synchronization
  
  // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization    
}

void LaserPowerLevel( byte pl ) {
  powerLevel = pl;
  pwm = map(powerLevel, 0, 255, LASER_PWM_PERIOD, 0);
  if(on==true) {
    REG_TCC0_CC0 = pwm;
    while (TCC0->SYNCBUSY.bit.CC0);                // Wait for synchronization
  }   
}

byte LaserPowerLevel() {
  return powerLevel;
}

float LaserPwmPercent() {
  return (float)(pwm / LASER_PWM_PERIOD);
}
  
void LaserEnable( bool state ) {
  if( state == true ) {
    on = true;
    REG_TCC0_CC0 = map(powerLevel, 0, 255, LASER_PWM_PERIOD, 0);
    while (TCC0->SYNCBUSY.bit.CC0);
  } else {
    on = false;
    REG_TCC0_CC0 = map(0, 0, 255, LASER_PWM_PERIOD, 0);
    while (TCC0->SYNCBUSY.bit.CC0);
  }
}

bool LaserEnabled() { 
  return enabled; 
}

bool LaserOn() { 
  return on; 
}




