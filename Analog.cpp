
#include "ZeroLASER.h"
#include "Analog.h"


#define ANALOG_TIMER                TC5
#define ANALOG_TIMER_ID             GCM_TC4_TC5

#define ENABLE_ANALOG_INTERRUPT()   NVIC_EnableIRQ(TC5_IRQn)
#define DISABLE_ANALOG_INTERRUPT()  NVIC_DisableIRQ(TC5_IRQn)
#define ANALOG_TIMER_ISR()          void TC5_Handler()
#define ANALOG_TIMER_OCR            TC5->COUNT16.CC[0].reg


#define PIN_LASER_CURRENT            15
#define PIN_TERMISTOR_0              16
#define PIN_TERMISTOR_1              17



#define ANALOG_MAXFREQ     24000000 //(F_CPU / 2);
#define ANALOG_TC_TOP      0xFFFF

static uint32_t prescalerConfigBits;
static uint32_t ccValue;
static volatile uint32_t analogSamples[3];
static uint32_t analogFilter[3];
uint32_t analogZeroOffset[3];


static uint32_t calc_timer_vals(uint32_t freq) {
  ccValue = ANALOG_MAXFREQ / freq - 1;
  prescalerConfigBits = TC_CTRLA_PRESCALER_DIV1;
  if (ccValue > ANALOG_MAXFREQ) {
    ccValue = ANALOG_TC_TOP / freq / 2 - 1;
    prescalerConfigBits = TC_CTRLA_PRESCALER_DIV2;
    if (ccValue > ANALOG_MAXFREQ) {
      ccValue = ANALOG_TC_TOP / freq / 4 - 1;
      prescalerConfigBits = TC_CTRLA_PRESCALER_DIV4;
      if (ccValue > ANALOG_MAXFREQ) {
        ccValue = ANALOG_TC_TOP / freq / 8 - 1;
        prescalerConfigBits = TC_CTRLA_PRESCALER_DIV8;
        if (ccValue > ANALOG_MAXFREQ) {
          ccValue = ANALOG_TC_TOP / freq / 16 - 1;
          prescalerConfigBits = TC_CTRLA_PRESCALER_DIV16;
          if (ccValue > ANALOG_MAXFREQ) {
            ccValue = ANALOG_TC_TOP / freq / 64 - 1;
            prescalerConfigBits = TC_CTRLA_PRESCALER_DIV64;
            if (ccValue > ANALOG_MAXFREQ) {
              ccValue = ANALOG_TC_TOP / freq / 256 - 1;
              prescalerConfigBits = TC_CTRLA_PRESCALER_DIV256;
              if (ccValue > ANALOG_MAXFREQ) {
                ccValue = ANALOG_TC_TOP / freq / 1024 - 1;
                prescalerConfigBits = TC_CTRLA_PRESCALER_DIV1024;
              }
            }
          }
        }
      }
    }
  }
}


void AnalogTimerInit() {

  // Calulate timer vals for 100 Hz
  calc_timer_vals( 100 );
  
  ANALOG_TIMER->COUNT16.CTRLA.reg &= ~(TC_CTRLA_ENABLE);       //disable TC module
  ANALOG_TIMER->COUNT16.CTRLA.reg |=TC_CTRLA_MODE_COUNT16;
  ANALOG_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  ANALOG_TIMER->COUNT16.CTRLA.reg |= prescalerConfigBits;
  ANALOG_TIMER->COUNT16.CC[0].reg = ccValue;
  ANALOG_TIMER->COUNT16.INTENSET.reg = TC_INTFLAG_MC0;
  ANALOG_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  ANALOG_TIMER->COUNT16.INTFLAG.reg = 0xFF;
  ENABLE_ANALOG_INTERRUPT();
 
}


void AnalogInit() {

  analogSamples[ANALOG_CHANNEL_LASER]       = 0;
  analogSamples[ANALOG_CHANNEL_THERM0]      = 0;
  analogSamples[ANALOG_CHANNEL_THERM1]      = 0;

  analogFilter[ANALOG_CHANNEL_LASER]  = 0;
  analogFilter[ANALOG_CHANNEL_THERM0] = 0;
  analogFilter[ANALOG_CHANNEL_THERM1] = 0;
   
  analogZeroOffset[0] = 260;
  analogZeroOffset[1] = 260;
  analogZeroOffset[2] = 260;

  
  //Input control register
  while (ADC->STATUS.bit.SYNCBUSY == 1); 
  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_1X_Val;      // Gain select as 1X
  //Set ADC reference source
//  while (ADC->STATUS.bit.SYNCBUSY == 1); 
  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val; //  2.2297 V Supply VDDANA
  // Set sample length and averaging
  while (ADC->STATUS.bit.SYNCBUSY == 1); 
  ADC->AVGCTRL.reg = 0x00 ;       //Single conversion no averaging
  while (ADC->STATUS.bit.SYNCBUSY == 1); 
  ADC->SAMPCTRL.reg = 0x00;       //Minimal sample length is 1/2 CLK_ADC cycle
  //Control B register
  while (ADC->STATUS.bit.SYNCBUSY == 1); 
  ADC->CTRLB.reg =  0x400     ; // Prescale 64, 12 bit resolution, singel conversion
  // Enable ADC in control B register
  while (ADC->STATUS.bit.SYNCBUSY == 1); 
  ADC->CTRLA.bit.ENABLE = 0x01;  

  AnalogTimerInit();
    
}

// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      

// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   

// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950

// the value of the 'other' resistor (4.3K)
#define SERIESRESISTOR 43000    

#define ANALOG_FILTER_K   4

uint32_t AnalogFilter( int channel ) {
  
  DISABLE_ANALOG_INTERRUPT();
  uint32_t sample = analogSamples[ channel ];
  ENABLE_ANALOG_INTERRUPT();

  analogFilter[channel] = analogFilter[channel] - (analogFilter[channel] >> ANALOG_FILTER_K) + sample;
//  if( channel == 2) {
//    SerialUSB.print("Filter["); 
//    SerialUSB.print(analogFilter[channel]);
//    SerialUSB.print("] counts["); 
//    SerialUSB.print(analogFilter[channel]);
//    SerialUSB.println("]"); 
//  }
  return analogFilter[channel] >> ANALOG_FILTER_K;
}

float AnalogMeasurement( int channel ) {
  
  float steinhart;
  float val;
  float measurement = 0.0;
  
  uint32_t a2d_counts = AnalogFilter( channel );
  
  switch( channel ) {
    case ANALOG_CHANNEL_LASER: 
      val = ((a2d_counts & 0x0FFF) * (2.2297 / 0x0FFF));
      measurement = 1000.0 * (val / CURRENT_SENSE_RESISTOR); // Convert to mA
      break;
    case ANALOG_CHANNEL_THERM0:
    case ANALOG_CHANNEL_THERM1:
      val = (SERIESRESISTOR / (0x0FFE / (a2d_counts - 1)));
      steinhart  = val / THERMISTORNOMINAL;     // (R/Ro)
      steinhart  = log(steinhart);                  // ln(R/Ro)
      steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
      steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
      steinhart  = 1.0 / steinhart;                 // Invert
      steinhart -= 273.15;                         // convert to C      
      measurement = steinhart;
      break;
    default:
      break;  
  }
  return measurement;  
}

int analogState = 0;

ANALOG_TIMER_ISR()
{        
  
  ANALOG_TIMER->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;

  switch( analogState ) {
    
    case 0:
      while (ADC->STATUS.bit.SYNCBUSY == 1); 
      ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ PIN_LASER_CURRENT ].ulADCChannelNumber;
      ADC->INTFLAG.bit.RESRDY = 1;              // Data ready flag cleared
      while (ADC->STATUS.bit.SYNCBUSY == 1); 
      ADC->SWTRIG.bit.START = 1;                // Start ADC conversion
      analogState++;
      break;

    case 1:
      if( ADC->INTFLAG.bit.RESRDY == 1 ) {
        while (ADC->STATUS.bit.SYNCBUSY == 1); 
        analogSamples[ANALOG_CHANNEL_LASER] = ADC->RESULT.reg;     // read the result
        while (ADC->STATUS.bit.SYNCBUSY == 1); 
        ADC->SWTRIG.reg = 0x01;                    //  and flush for good measure
        analogState++;
      }
      break;

    case 2:
      while (ADC->STATUS.bit.SYNCBUSY == 1); 
      ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ PIN_TERMISTOR_0 ].ulADCChannelNumber;
      ADC->INTFLAG.bit.RESRDY = 1;              // Data ready flag cleared
      while (ADC->STATUS.bit.SYNCBUSY == 1); 
      ADC->SWTRIG.bit.START = 1;                // Start ADC conversion
      analogState++;
      break;

    case 3:
      if( ADC->INTFLAG.bit.RESRDY == 1 ) {
        while (ADC->STATUS.bit.SYNCBUSY == 1); 
        analogSamples[ANALOG_CHANNEL_THERM0] = ADC->RESULT.reg;     // read the result
        while (ADC->STATUS.bit.SYNCBUSY == 1); 
        ADC->SWTRIG.reg = 0x01;                    //  and flush for good measure
        analogState++;
      }
      break;

    case 4:
      while (ADC->STATUS.bit.SYNCBUSY == 1); 
      ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ PIN_TERMISTOR_1 ].ulADCChannelNumber;
      ADC->INTFLAG.bit.RESRDY = 1;              // Data ready flag cleared
      while (ADC->STATUS.bit.SYNCBUSY == 1); 
      ADC->SWTRIG.bit.START = 1;                // Start ADC conversion
      analogState++;
      break;

    case 5:
      if( ADC->INTFLAG.bit.RESRDY == 1 ) {
        while (ADC->STATUS.bit.SYNCBUSY == 1); 
        analogSamples[ANALOG_CHANNEL_THERM1] = ADC->RESULT.reg;     // read the result
        while (ADC->STATUS.bit.SYNCBUSY == 1); 
        ADC->SWTRIG.reg = 0x01;                    //  and flush for good measure
        analogState++;
      }
      break;

    default:
        analogState = 0;
      break;
  }
}





