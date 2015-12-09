

/*
 ZeroLASER firmware based on Sprinter
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>. */

/*
  This firmware is a mashup between Sprinter, grbl and parts from marlin.
  (https://github.com/kliment/Sprinter)
  
  Changes by Jeffrey L Smith
  
  Planner is from Simen Svale Skogsrud
  https://github.com/simen/grbl

  Parts of Marlin Firmware from ErikZalm
  https://github.com/ErikZalm/Marlin-non-gen6  

*/

//#include <avr/pgmspace.h>
#include <math.h>

#include "Configuration.h"
#include "pins.h"
#include "ZeroLASER.h"
#include "speed_lookuptable.h"

#include "Display.h"
#include "Laser.h"
#include "Analog.h"

#include <extEEPROM.h>





#ifdef USE_EEPROM_SETTINGS
  #include "store_eeprom.h"
#endif


void __cxa_pure_virtual(){};

//Implemented Codes
//-------------------
// G0  -> G1
// G1  - Coordinated Movement X Y Z E
// G2  - CW ARC
// G3  - CCW ARC
// G4  - Dwell S<seconds> or P<milliseconds>
// G28 - Home all Axis
// G90 - Use Absolute Coordinates
// G91 - Use Relative Coordinates
// G92 - Set current position to cordinates given

//RepRap M Codes
// M104 - Set extruder target temp
// M105 - Read current temp
// M106 - Fan on
// M107 - Fan off
// M109 - Wait for extruder current temp to reach target temp.
// M114 - Display current position

//Custom M Codes
// M42  - Set output on free pins, on a non pwm pin (over pin 13 on an arduino mega) use S255 to turn it on and S0 to turn it off. Use P to decide the pin (M42 P23 S255) would turn pin 23 on
// M82  - Set E codes absolute (default)
// M83  - Set E codes relative while in Absolute Coordinates (G90) mode
// M84  - Disable steppers until next move, 
//        or use S<seconds> to specify an inactivity timeout, after which the steppers will be disabled.  S0 to disable the timeout.
// M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
// M92  - Set axis_steps_per_unit - same syntax as G92
// M93  - Send axis_steps_per_unit
// M115	- Capabilities string
// M119 - Show Endstopper State 
// M201 - Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000)
// M202 - Set maximum feedrate that your machine can sustain (M203 X200 Y200 Z300 E10000) in mm/sec
// M203 - Set temperture monitor to Sx
// M204 - Set default acceleration: S normal moves T filament only moves (M204 S3000 T7000) in mm/sec^2
// M205 - advanced settings:  minimum travel speed S=while printing T=travel only,  X=maximum xy jerk, Z=maximum Z jerk
// M206 - set additional homing offset

// M220 - set speed factor override percentage S=factor in percent 
// M221 - set extruder multiply factor S100 --> original Extrude Speed 


// M400 - Finish all moves

// M500 - stores paramters in EEPROM
// M501 - reads parameters from EEPROM (if you need to reset them after you changed them temporarily).
// M502 - reverts to the default "factory settings". You still need to store them in EEPROM afterwards if you want to.
// M503 - Print settings

// Debug features 
// NO- M603 - Show Free Ram



#define _VERSION_TEXT "1.0.b / 27.11.2015"

static volatile bool endstop_x_hit=false;
static volatile bool endstop_y_hit=false;



//Stepper Movement Variables
char axis_codes[NUM_AXIS] = {'X', 'Y'};
float axis_steps_per_unit[NUM_AXIS] = _AXIS_STEP_PER_UNIT; 


float max_feedrate[NUM_AXIS] = _MAX_FEEDRATE;
float homing_feedrate[NUM_AXIS] = _HOMING_FEEDRATE;
bool axis_relative_modes[NUM_AXIS] = _AXIS_RELATIVE_MODES;

float move_acceleration = _ACCELERATION;         // Normal acceleration mm/s^2
float max_xy_jerk = _MAX_XY_JERK;
uint32_t min_seg_time = _MIN_SEG_TIME;

int32_t  max_acceleration_units_per_sq_second[NUM_AXIS] = _MAX_ACCELERATION_UNITS_PER_SQ_SECOND; // X, Y, Z and E max acceleration in mm/s^2 for printing moves or retracts

//float max_start_speed_units_per_second[] = _MAX_START_SPEED_UNITS_PER_SECOND;
//int32_t  max_travel_acceleration_units_per_sq_second[] = _MAX_TRAVEL_ACCELERATION_UNITS_PER_SQ_SECOND; // X, Y, Z max acceleration in mm/s^2 for travel moves

float mintravelfeedrate = DEFAULT_MINTRAVELFEEDRATE;
float minimumfeedrate = DEFAULT_MINIMUMFEEDRATE;

uint32_t axis_steps_per_sqr_second[NUM_AXIS];
uint32_t plateau_steps;  

//adjustable feed factor for online tuning printer speed
volatile int feedmultiply=100; //100->original / 200 -> Factor 2 / 50 -> Factor 0.5
int saved_feedmultiply;
volatile bool feedmultiplychanged=false;
volatile int extrudemultiply=100; //100->1 200->2

//boolean acceleration_enabled = false, accelerating = false;
//uint32_t interval;
float destination[NUM_AXIS] = {0.0, 0.0};
float current_position[NUM_AXIS] = {0.0, 0.0};
float add_homing[NUM_AXIS]={0,0};


volatile float real_position[NUM_AXIS] = {0.0, 0.0};


static uint16_t virtual_steps_x = 0;
static uint16_t virtual_steps_y = 0;

bool home_all_axis = true;
//unsigned ?? ToDo: Check
int feedrate = 1500, next_feedrate, saved_feedrate;

int32_t gcode_N, gcode_LastN;
bool relative_mode = false;  //Determines Absolute or Relative Coordinates
bool is_homing = false;

//For arc center point coordinates, sent by commands G2/G3
float offset[3] = {0.0, 0.0, 0.0};

#ifdef STEP_DELAY_RATIO
  int32_t long_step_delay_ratio = STEP_DELAY_RATIO * 100;
#endif

///oscillation reduction
#ifdef RAPID_OSCILLATION_REDUCTION
  float cumm_wait_time_in_dir[NUM_AXIS]={0.0,0.0,0.0,0.0};
  bool prev_move_direction[NUM_AXIS]={1,1,1,1};
  float osc_wait_remainder = 0.0;
#endif


// comm variables and Commandbuffer
// BUFSIZE is reduced from 8 to 6 to free more RAM for the PLANNER
#define MAX_CMD_SIZE 96
#define BUFSIZE 6 //8
char cmdbuffer[BUFSIZE][MAX_CMD_SIZE];
bool fromsd[BUFSIZE];

uint8_t bufindr = 0;
uint8_t bufindw = 0;
uint8_t buflen = 0;
char serial_char;
int serial_count = 0;
boolean comment_mode = false;
char *strchr_pointer; // just a pointer to find chars in the cmd string like X, Y, Z, E, etc

//Send Temperature in °C to Host
int hotendtC = 0, bedtempC = 0;
       
//Inactivity shutdown variables
uint32_t previous_millis_cmd = 0;
uint32_t max_inactive_time = 0;
uint32_t stepper_inactive_time = 0;

//Temp Monitor for repetier
uint8_t manage_monitor = 255;



bool ErmergencyStop = false;


void HomeAllAxis() {

  saved_feedrate = feedrate;
  saved_feedmultiply = feedmultiply;
  previous_millis_cmd = millis();
        
  feedmultiply = 100;    
      
  for(int i=0; i < NUM_AXIS; i++)  {
    destination[i] = current_position[i];
  }

  feedrate = 0;
  is_homing = true;

  homing_routine(X_AXIS);
  homing_routine(Y_AXIS);

  is_homing = false;
  feedrate = saved_feedrate;
  feedmultiply = saved_feedmultiply;
  
}

float JogStepSize = 1.0;

void ManualJog(int axis, bool positive)
{
  int32_t help_feedrate = 0;

  if( axis == X_AXIS) {
    if( positive ) {
      destination[X_AXIS] = current_position[X_AXIS] + JogStepSize;
    } else {
      destination[X_AXIS] = current_position[X_AXIS] - JogStepSize;
    }
  }
  else {
    if( positive ) {
      destination[Y_AXIS] = current_position[Y_AXIS] + JogStepSize;
    } else {
      destination[Y_AXIS] = current_position[Y_AXIS] - JogStepSize;
    }
  }

  if (destination[X_AXIS] < 0) destination[X_AXIS] = 0.0;
  if (destination[Y_AXIS] < 0) destination[Y_AXIS] = 0.0;

  help_feedrate = ((int32_t)feedrate*(int32_t)100);
  
  plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], help_feedrate/6000.0);
  
  for(int i=0; i < NUM_AXIS; i++) {
    current_position[i] = destination[i];
  } 
  
}


//uint32_t ButtonTag        = 0;
//uint32_t ButtonDownCount  = 0;



int RuntimeFreeRam(void)
{
//  extern int  __bss_end;
  extern int  __bss_end__;
  
//  extern int* __brkval;
  extern int* __HeapLimit;
  
  int free_memory;
//  if (reinterpret_cast<int>(__brkval) == 0) {
  if (reinterpret_cast<int>(__HeapLimit) == 0) {
    // if no heap use from end of bss section
//    free_memory = reinterpret_cast<int>(&free_memory) - reinterpret_cast<int>(&__bss_end);
    free_memory = reinterpret_cast<int>(&free_memory) - reinterpret_cast<int>(&__bss_end__);
  } else {
    // use from top of stack to heap
//    free_memory = reinterpret_cast<int>(&free_memory) - reinterpret_cast<int>(__brkval);
    free_memory = reinterpret_cast<int>(&free_memory) - reinterpret_cast<int>(__HeapLimit);
    
  }
  return free_memory;
}


#if 0
void InitLaserPWM() 
{ 

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
  PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to digital output D2 - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXO_F | PORT_PMUX_PMUXE_F; 

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
  REG_TCC0_PER = 1024;                             // Set the frequency of the PWM on TCC0 to 250kHz
  while (TCC0->SYNCBUSY.bit.PER);                 // Wait for synchronization
  
  // Set the PWM signal to output 50% duty cycle
//  REG_TCC0_CC3 = 128;         // TCC0 CC3 - on D7
//  while (TCC0->SYNCBUSY.bit.CC3);                // Wait for synchronization
  REG_TCC0_CC0 = 1025;         // TCC0 CC3 - on D7
  while (TCC0->SYNCBUSY.bit.CC0);                // Wait for synchronization
  
  // Divide the 48MHz signal by 1 giving 48MHz (20.83ns) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}
#endif


//float XStepIncrement  = 0.0;
//float YStepIncrement  = 0.0;


extEEPROM eeprom(kbits_256, 1, 64);         //device size, number of devices, page size


template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++) {
//      EEPROM.write(ee++, *p++);
        eeprom.write(ee++, *p++);
    }
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    unsigned int i;
    for (i = 0; i < sizeof(value); i++)
    //*p++ = EEPROM.read(ee++);
      *p++ = eeprom.read(ee++);
    return i;
}


//------------------------------------------------
// Init 
//------------------------------------------------
void setup()
{ 
  while(!SerialUSB);
  
  SerialUSB.begin(BAUDRATE);

//  laserserial.printf("The compile time & date is: %s, %s", __TIME__, __DATE__);
//  laserserial.printf("Float test: %06.3f\r\n", 123.456);
  
  
//  SerialUSB.begin( 9600 );
  SerialUSB.print(F("Sprinter\r\n"));
  SerialUSB.print(F(_VERSION_TEXT));
  SerialUSB.print(F("\r\n"));
  SerialUSB.print(F("start\r\n"));

  SerialUSB.print(F("Laser init\r\n"));
  LaserInit();



  for(int i = 0; i < BUFSIZE; i++) {
      fromsd[i] = false;
  }
  
  if(DisplayInit() == false) {
    SerialUSB.print(F("No UI Display\r\n"));
  } 

  
  //Initialize Stepper Pins
  pinMode(XY_ENABLE_PIN, OUTPUT);
  digitalWrite(XY_ENABLE_PIN, HIGH);
  pinMode(X_DIR_PIN,  OUTPUT);
  digitalWrite(X_DIR_PIN, LOW);
  pinMode(Y_DIR_PIN,  OUTPUT);
  digitalWrite(Y_DIR_PIN, LOW);

  pinMode(X_STEP_PIN, OUTPUT);
  digitalWrite(X_STEP_PIN, LOW);
  pinMode(Y_STEP_PIN, OUTPUT);
  digitalWrite(Y_STEP_PIN, LOW);

//  pinMode(CONTROLLERFAN_PIN, OUTPUT); //Set pin used for driver cooling fan
//  digitalWrite(CONTROLLERFAN_PIN, LOW);
   
  //endstops and pullups
  pinMode(X_MIN_PIN, INPUT); 
  pinMode(Y_MIN_PIN, INPUT); 
  pinMode(INTERLOCK_PIN, INPUT); 
    
  //Initialize Fan Pin
  pinMode(FAN_PIN, OUTPUT);

  //Initialize LED Pin
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED,LOW);

  pinMode(PIN_LED_RXL, OUTPUT);
  digitalWrite(PIN_LED_RXL,LOW);

  pinMode(PIN_LED_TXL, OUTPUT);
  digitalWrite(PIN_LED_TXL,LOW);
  
  SerialUSB.print(F("Planner Init\r\n"));
  plan_init();  // Initialize planner;

  SerialUSB.print(F("Stepper Init\r\n"));
  st_init();    // Initialize stepper

  SerialUSB.print(F("Analog Init\r\n"));
  AnalogInit();

  #ifdef USE_EEPROM_SETTINGS
  //first Value --> Init with default
  //second value --> Print settings to UART
  EEPROM_RetrieveSettings(false,false);
  #endif

  //Free Ram
  SerialUSB.print(F("Free Ram: "));
  SerialUSB.println(RuntimeFreeRam());
  
  //Planner Buffer Size
  SerialUSB.print(F("Plan Buffer Size:"));
  SerialUSB.print((int)sizeof(block_t)*BLOCK_BUFFER_SIZE);
  SerialUSB.print(F(" / "));
  SerialUSB.println(BLOCK_BUFFER_SIZE);

//  XStepIncrement = 1.0/axis_steps_per_unit[0];
//  YStepIncrement = 1.0/axis_steps_per_unit[1];
  
  for(int8_t i=0; i < NUM_AXIS; i++) {
    axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
  }

  DisplayUpdate();
  

}

int sysTickHook(void) {

  static int msTimer = 0;
  static int state = 0;

  msTimer++;

  switch( msTimer ) {

    case 0:
      SerialUSB.println("SysTick Hook!\r\n");
      break;

    default:
      break;
  }

  if( msTimer == 1000 ) msTimer = 0;
  
  return 0;
  
}

//------------------------------------------------
//MAIN LOOP
//------------------------------------------------
void loop()
{
  if(buflen < (BUFSIZE-1))
    get_command();
  
  if(buflen) {
    process_commands();
    buflen = (buflen-1);
    bufindr++;
    if(bufindr == BUFSIZE) bufindr = 0;
  }
  manage_inactivity(1);
}

//------------------------------------------------
//Check Uart buffer while arc function ist calc a circle
//------------------------------------------------
void check_buffer_while_arc()
{
  if(buflen < (BUFSIZE-1)) {
    get_command();
  }
}

//------------------------------------------------
//READ COMMAND FROM UART
//------------------------------------------------
void get_command() 
{ 
  while( SerialUSB.available() > 0 && buflen < BUFSIZE) {
    digitalWrite( PIN_LED_RXL, LOW );
    serial_char = SerialUSB.read();
    if(serial_char == '\n' || serial_char == '\r' || (serial_char == ':' && comment_mode == false) || serial_count >= (MAX_CMD_SIZE - 1) ) {
      if(!serial_count) { //if empty line
        comment_mode = false; // for new command
        digitalWrite( PIN_LED_RXL, HIGH );
        return;
      }
      cmdbuffer[bufindw][serial_count] = 0; //terminate string

        fromsd[bufindw] = false;
        
        if(strstr(cmdbuffer[bufindw], "N") != NULL) {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'N');
          gcode_N = (strtol(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL, 10));
          if(gcode_N != gcode_LastN+1 && (strstr(cmdbuffer[bufindw], "M110") == NULL) ) {
            SerialUSB.print(F("Serial Error: Line Number is not Last Line Number+1, Last Line:"));
            SerialUSB.println(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            digitalWrite( PIN_LED_RXL, HIGH );
            return;
          }
    
          if(strstr(cmdbuffer[bufindw], "*") != NULL)
          {
            byte checksum = 0;
            byte count = 0;
            while(cmdbuffer[bufindw][count] != '*') checksum = checksum^cmdbuffer[bufindw][count++];
            strchr_pointer = strchr(cmdbuffer[bufindw], '*');
            if( (int)(strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)) != checksum) {
              SerialUSB.print(F("Error: checksum mismatch, Last Line:"));
              SerialUSB.println(gcode_LastN);
              FlushSerialRequestResend();
              serial_count = 0;
              digitalWrite( PIN_LED_RXL, HIGH );
              return;
            }
            //if no errors, continue parsing
          } else {
            SerialUSB.print(F("Error: No Checksum with line number, Last Line:"));
            SerialUSB.println(gcode_LastN);
            FlushSerialRequestResend();
            serial_count = 0;
            digitalWrite( PIN_LED_RXL, HIGH );
            return;
          }
    
          gcode_LastN = gcode_N;
          //if no errors, continue parsing
        } else { // if we don't receive 'N' but still see '*'
          if((strstr(cmdbuffer[bufindw], "*") != NULL)) {
            SerialUSB.print(F("Error: No Line Number with checksum, Last Line:"));
            SerialUSB.println(gcode_LastN);
            serial_count = 0;
            digitalWrite( PIN_LED_RXL, HIGH );
            return;
          }
        }
        
	if((strstr(cmdbuffer[bufindw], "G") != NULL)) {
          strchr_pointer = strchr(cmdbuffer[bufindw], 'G');
          switch((int)((strtod(&cmdbuffer[bufindw][strchr_pointer - cmdbuffer[bufindw] + 1], NULL)))) {
            case 0:
            case 1:
            case 2:  //G2
            case 3:  //G3 arc func
              SerialUSB.println("ok"); 
            break;
            
            default:
            break;
          }
        }
        //Removed modulo (%) operator, which uses an expensive divide and multiplication
        //bufindw = (bufindw + 1)%BUFSIZE;
        bufindw++;
        if(bufindw == BUFSIZE) bufindw = 0;
        buflen += 1;

      comment_mode = false; //for new command
      serial_count = 0; //clear buffer
    } else {
      if(serial_char == ';') comment_mode = true;
      if(!comment_mode) cmdbuffer[bufindw][serial_count++] = serial_char;
    }
  }
  digitalWrite( PIN_LED_RXL, HIGH );
}

//static bool check_endstops = true;
//
//void enable_endstops(bool check)
//{
//  check_endstops = check;
//}

inline float code_value() { return (strtod(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL)); }
inline int32_t code_value_long() { return (strtol(&cmdbuffer[bufindr][strchr_pointer - cmdbuffer[bufindr] + 1], NULL, 10)); }
inline bool code_seen(char code_string[]) { return (strstr(cmdbuffer[bufindr], code_string) != NULL); }  //Return True if the string was found

inline bool code_seen(char code)
{
  strchr_pointer = strchr(cmdbuffer[bufindr], code);
  return (strchr_pointer != NULL);  //Return True if a character was found
}

inline void homing_routine(char axis)
{
  int min_pin, home_dir, max_length, home_bounce;

  switch(axis){
    case X_AXIS:
      min_pin = X_MIN_PIN;
      home_dir = X_HOME_DIR;
      max_length = X_MAX_LENGTH;
      home_bounce = 10;
      break;
    case Y_AXIS:
      min_pin = Y_MIN_PIN;
      home_dir = Y_HOME_DIR;
      max_length = Y_MAX_LENGTH;
      home_bounce = 10;
      break;
    default:
      //never reached
      break;
  }

//  if ( (min_pin > -1 && home_dir==-1) )
//  {
    current_position[axis] = -1.5 * max_length * home_dir;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS]);
    destination[axis] = 0;
    feedrate = homing_feedrate[axis];
    prepare_move();
    st_synchronize();

    current_position[axis] = home_bounce/2 * home_dir;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS]);
    destination[axis] = 0;
    prepare_move();
    st_synchronize();

    current_position[axis] = -home_bounce * home_dir;
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS]);
    destination[axis] = 0;
    feedrate = homing_feedrate[axis]/2;
    prepare_move();
    st_synchronize();

    current_position[axis] = (home_dir == -1) ? 0 : max_length;
    current_position[axis] += add_homing[axis];
    plan_set_position(current_position[X_AXIS], current_position[Y_AXIS]);
    destination[axis] = current_position[axis];
    feedrate = 0;
//  }
}

//uint8_t LaserPowerLevel = 0;

//------------------------------------------------
// CHECK COMMAND AND CONVERT VALUES
//------------------------------------------------
inline void process_commands()
{
  uint32_t codenum; //throw away variable
  char *starpos = NULL;

  if(code_seen('G'))
  {
    switch((int)code_value())
    {
      case 0: // G0 -> G1
      case 1: // G1
        get_coordinates(); // For X Y Z E F
        prepare_move();
        previous_millis_cmd = millis();
        //ClearToSend();
        return;
        //break;
      case 2: // G2  - CW ARC
        get_arc_coordinates();
        prepare_arc_move(true);
        previous_millis_cmd = millis();
        //break;
        return;
      case 3: // G3  - CCW ARC
        get_arc_coordinates();
        prepare_arc_move(false);
        previous_millis_cmd = millis();
        //break;
        return;  
      case 4: // G4 dwell
        codenum = 0;
        if(code_seen('P')) codenum = code_value(); // milliseconds to wait
        if(code_seen('S')) codenum = code_value() * 1000; // seconds to wait
        codenum += millis();  // keep track of when we started waiting
        st_synchronize();  // wait for all movements to finish
        break;
      case 28: //G28 Home all Axis one at a time
        saved_feedrate = feedrate;
        saved_feedmultiply = feedmultiply;
        previous_millis_cmd = millis();
        
        feedmultiply = 100;    
      
//        enable_endstops(true);
      
        for(int i=0; i < NUM_AXIS; i++) 
        {
          destination[i] = current_position[i];
        }
        feedrate = 0;
        is_homing = true;

//        home_all_axis = !((code_seen(axis_codes[0])) || (code_seen(axis_codes[1])) || (code_seen(axis_codes[2])));
        home_all_axis = !((code_seen(axis_codes[X_AXIS])) || (code_seen(axis_codes[Y_AXIS])));

        if((home_all_axis) || (code_seen(axis_codes[X_AXIS]))) 
          homing_routine(X_AXIS);

        if((home_all_axis) || (code_seen(axis_codes[Y_AXIS]))) 
          homing_routine(Y_AXIS);

       
//        #ifdef ENDSTOPS_ONLY_FOR_HOMING
//            enable_endstops(false);
//      	#endif
      
        is_homing = false;
        feedrate = saved_feedrate;
        feedmultiply = saved_feedmultiply;
      
        previous_millis_cmd = millis();
        break;
      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;
      case 92: // G92
        st_synchronize();         
        for(int i=0; i < NUM_AXIS; i++) {
          if(code_seen(axis_codes[i])) current_position[i] = code_value();  
        }
        plan_set_position(current_position[X_AXIS], current_position[Y_AXIS]);
        break;
      default:
            #ifdef SEND_WRONG_CMD_INFO
             digitalWrite( PIN_LED_TXL, LOW );
             SerialUSB.print(F("Unknown G-COM:"));
              SerialUSB.println(cmdbuffer[bufindr]);
             digitalWrite( PIN_LED_TXL, HIGH );
            #endif
      break;
    }
  }
 
  else if(code_seen('S'))
  {
    LaserPowerLevel( code_value() );
    SerialUSB.print("LaserPower:");
    SerialUSB.println( LaserPowerLevel() );
  }
  else if(code_seen('M'))
  {
    
    switch( (int)code_value() ) 
    {

      case 1: // Display Cal
        SerialUSB.println( "Calibrating ToushScreen" );
        DisplayCalibrateTouch();
        SerialUSB.println( "Complete" );
        break;

      
      case 3: // M03 LASER ON
        LaserEnable( true );
        SerialUSB.print("LaserON:");
        SerialUSB.println( LaserPowerLevel() );
        break;
      case 5: // M05 LASER OFF
        LaserEnable( false );
        SerialUSB.print("LaserOFF:");
        SerialUSB.println( LaserPowerLevel() );
        break;
      
      case 42: //M42 -Change pin status via gcode
        if (code_seen('S'))
        {
          st_synchronize(); // wait for all movements to finish
          int pin_status = code_value();
          if (code_seen('P') && pin_status >= 0 && pin_status <= 255)
          {
            int pin_number = code_value();
            for(int i = 0; i < sizeof(sensitive_pins) / sizeof(int); i++)
            {
              if (sensitive_pins[i] == pin_number)
              {
                pin_number = -1;
                break;
              }
            }
            
            if (pin_number > -1)
            {              
              pinMode(pin_number, OUTPUT);
              digitalWrite(pin_number, pin_status);
              //analogWrite(pin_number, pin_status);
            }
          }
        }
        break;
#if 0
      case 104: // M104
          st_synchronize(); // wait for all movements to finish
        if (code_seen('S')) target_raw = temp2analogh(target_temp = code_value());
        #ifdef WATCHPERIOD
            if(target_raw > current_raw)
            {
                watchmillis = max(1,millis());
                watch_raw = current_raw;
            }
            else
            {
                watchmillis = 0;
            }
        #endif
        break;
      case 105: // M105
        digitalWrite( PIN_LED_TXL, LOW );
        #if (TEMP_0_PIN > -1) 
          hotendtC = analog2temp(current_raw);
        #endif
        #if (TEMP_1_PIN > -1)
          bedtempC = analog2tempBed(current_bed_raw);
        #endif
        #if (TEMP_0_PIN > -1) 
            SerialUSB.print(F("ok T:"));
            SerialUSB.print(hotendtC); 
          #if TEMP_1_PIN > -1 || defined BED_USES_AD595
            SerialUSB.print(F(" B:"));
            SerialUSB.println(bedtempC); 
          #else
            SerialUSB.println();
          #endif
        #else
          #error No temperature source available
        #endif
        digitalWrite( PIN_LED_TXL, HIGH );
        return;
        //break;
#endif

      case 106: //M106 Fan On
//          st_synchronize(); // wait for all movements to finish
        if (code_seen('S'))
        {
            uint8_t l_fan_code_val = constrain(code_value(),0,255);
              digitalWrite(FAN_PIN, HIGH);
            
        }
        else 
        {
              digitalWrite(FAN_PIN, HIGH);
        }
        break;
      case 107: //M107 Fan Off
        digitalWrite(FAN_PIN, LOW);
        break;
      case 82:
        axis_relative_modes[3] = false;
        break;
      case 83:
        axis_relative_modes[3] = true;
        break;
      case 84:
        st_synchronize(); // wait for all movements to finish
        if(code_seen('S'))
        {
          stepper_inactive_time = code_value() * 1000; 
        }
        else if(code_seen('T'))
        {
          digitalWrite( XY_ENABLE_PIN, LOW );
        }
        else
        { 
          digitalWrite( XY_ENABLE_PIN, HIGH );
        }
        break;
      case 85: // M85
        code_seen('S');
        max_inactive_time = code_value() * 1000; 
        break;
      case 92: // M92
        for(int i=0; i < NUM_AXIS; i++) 
        {
          if(code_seen(axis_codes[i])) 
          {
            axis_steps_per_unit[i] = code_value();
            axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
          }
        }
        
          // Update start speed intervals and axis order. TODO: refactor axis_max_interval[] calculation into a function, as it
          // should also be used in setup() as well
//        int32_t temp_max_intervals[NUM_AXIS];
//        for(int i=0; i < NUM_AXIS; i++) 
//        {
//          axis_max_interval[i] = 100000000.0 / (max_start_speed_units_per_second[i] * axis_steps_per_unit[i]);//TODO: do this for
//          all steps_per_unit related variables
//        }
        break;
      case 93: // M93 show current axis steps.
       digitalWrite( PIN_LED_TXL, LOW );
	SerialUSB.print(F("ok "));
	SerialUSB.print(F("X:"));
        SerialUSB.print(axis_steps_per_unit[0]);
	SerialUSB.print(F("Y:"));
        SerialUSB.println(axis_steps_per_unit[1]);
             digitalWrite( PIN_LED_TXL, HIGH );
        break;
      case 115: // M115
             digitalWrite( PIN_LED_TXL, LOW );
        SerialUSB.print(F("FIRMWARE_NAME: Sprinter Experimental PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1\r\n"));
        //SerialUSB.println(uuid);
        SerialUSB.print(F(_DEF_CHAR_UUID));
        SerialUSB.print(F("\r\n"));
             digitalWrite( PIN_LED_TXL, HIGH );
        break;
      case 114: // M114
             digitalWrite( PIN_LED_TXL, LOW );
	SerialUSB.print(F("X:"));
        SerialUSB.print(current_position[0]);
	SerialUSB.print(F("Y:"));
        SerialUSB.println(current_position[1]);
             digitalWrite( PIN_LED_TXL, HIGH );
        break;
      case 119: // M119
             digitalWrite( PIN_LED_TXL, LOW );
          SerialUSB.print(F("x_min:"));
          SerialUSB.print( X_MIN_HIT() ? "CLOSED ": "OPEN " );
      	  SerialUSB.print(F("y_min:"));
          SerialUSB.print( Y_MIN_HIT() ? "CLOSED ": "OPEN " );
        SerialUSB.print(F("\r\n"));
             digitalWrite( PIN_LED_TXL, HIGH );
      	break;
      case 201: // M201  Set maximum acceleration in units/s^2 for print moves (M201 X1000 Y1000)

        for(int8_t i=0; i < NUM_AXIS; i++) 
        {
          if(code_seen(axis_codes[i]))
          {
            max_acceleration_units_per_sq_second[i] = code_value();
            axis_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
          }
        }
        break;
      #if 0 // Not used for Sprinter/grbl gen6
      case 202: // M202
        for(int i=0; i < NUM_AXIS; i++) 
        {
          if(code_seen(axis_codes[i])) axis_travel_steps_per_sqr_second[i] = code_value() * axis_steps_per_unit[i];
        }
        break;
      #else  
      case 202: // M202 max feedrate mm/sec
        for(int8_t i=0; i < NUM_AXIS; i++) 
        {
          if(code_seen(axis_codes[i])) max_feedrate[i] = code_value();
        }
      break;
      #endif
      case 203: // M203 Temperature monitor
          if(code_seen('S')) manage_monitor = code_value();
          if(manage_monitor==100) manage_monitor=1; // Set 100 to heated bed
      break;
      case 204: // M204 acceleration S normal moves
          if(code_seen('S')) move_acceleration = code_value() ;
      break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk, E= max E jerk
        if(code_seen('S')) minimumfeedrate = code_value();
        if(code_seen('T')) mintravelfeedrate = code_value();
      //if(code_seen('B')) minsegmenttime = code_value() ;
        if(code_seen('X')) max_xy_jerk = code_value() ;
      break;
      case 206: // M206 additional homing offset
        if(code_seen('D'))
        {
             digitalWrite( PIN_LED_TXL, LOW );
          SerialUSB.print(F("Addhome X:")); SerialUSB.print(add_homing[0]);
          SerialUSB.print(F(" Y:")); SerialUSB.print(add_homing[1]);
          SerialUSB.print(F(" Z:")); SerialUSB.println(add_homing[2]);
             digitalWrite( PIN_LED_TXL, HIGH );
        }
        for(int8_t cnt_i=0; cnt_i < 2; cnt_i++) 
        {
          if(code_seen(axis_codes[cnt_i])) add_homing[cnt_i] = code_value();
        }
      break;  
      case 220: // M220 S<factor in percent>- set speed factor override percentage
      {
        if(code_seen('S')) 
        {
          feedmultiply = code_value() ;
          feedmultiply = constrain(feedmultiply, 20, 200);
          feedmultiplychanged=true;
        }
      }
      break;
      case 221: // M221 S<factor in percent>- set extrude factor override percentage
      {
        if(code_seen('S')) 
        {
          extrudemultiply = code_value() ;
          extrudemultiply = constrain(extrudemultiply, 40, 200);
        }
      }
      break;
      case 400: // M400 finish all moves
      {
      	st_synchronize();	
      }
      break;
#ifdef USE_EEPROM_SETTINGS
      case 500: // Store settings in EEPROM
      {
        EEPROM_StoreSettings();
      }
      break;
      case 501: // Read settings from EEPROM
      {
        EEPROM_RetrieveSettings(false,true);
        for(int8_t i=0; i < NUM_AXIS; i++)
        {
          axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
        }
      }
      break;
      case 502: // Revert to default settings
      {
        EEPROM_RetrieveSettings(true,true);
        for(int8_t i=0; i < NUM_AXIS; i++)
        {
          axis_steps_per_sqr_second[i] = max_acceleration_units_per_sq_second[i] * axis_steps_per_unit[i];
        }
      }
      break;
      case 503: // print settings currently in memory
      {
        EEPROM_printSettings();
      }
      break;  
#endif      

      case 603: // M603  Free RAM
            SerialUSB.print(F("Free Ram: "));
            SerialUSB.println(RuntimeFreeRam()); 
      break;
      default:
            #ifdef SEND_WRONG_CMD_INFO
             digitalWrite( PIN_LED_TXL, LOW );
              SerialUSB.print(F("Unknown M-COM:"));
              SerialUSB.println(cmdbuffer[bufindr]);
             digitalWrite( PIN_LED_TXL, HIGH );
            #endif
      break;

    }
    
  }
  else{
             digitalWrite( PIN_LED_TXL, LOW );
      SerialUSB.print(F("Unknown command:\r\n"));
      SerialUSB.println(cmdbuffer[bufindr]);
             digitalWrite( PIN_LED_TXL, HIGH );
  }
  
  ClearToSend();
      
}



void FlushSerialRequestResend()
{
  //char cmdbuffer[bufindr][100]="Resend:";
             digitalWrite( PIN_LED_TXL, LOW );
  SerialUSB.flush();
  SerialUSB.print(F("Resend:"));
  SerialUSB.println(gcode_LastN + 1);
             digitalWrite( PIN_LED_TXL, HIGH );
  ClearToSend();
}

void ClearToSend()
{
             digitalWrite( PIN_LED_TXL, LOW );
  previous_millis_cmd = millis();
  SerialUSB.print(F("ok\r\n"));
  //SerialUSB.println("ok");
             digitalWrite( PIN_LED_TXL, HIGH );
}

inline void get_coordinates()
{
  for(int i=0; i < NUM_AXIS; i++)
  {
    if(code_seen(axis_codes[i])) destination[i] = (float)code_value() + (axis_relative_modes[i] || relative_mode)*current_position[i];
    else destination[i] = current_position[i];                                                       //Are these else lines really needed?
  }
  
  if(code_seen('F'))
  {
    next_feedrate = code_value();
    if(next_feedrate > 0.0) feedrate = next_feedrate;
  }
}

void get_arc_coordinates()
{
   get_coordinates();
   if(code_seen('I')) {
     offset[0] = code_value();
   } 
   else {
     offset[0] = 0.0;
   }
   if(code_seen('J')) {
     offset[1] = code_value();
   }
   else {
     offset[1] = 0.0;
   }
}

void prepare_move()
{
  int32_t help_feedrate = 0;

  if(!is_homing){
//    if (min_software_endstops) 
//    {
      if (destination[X_AXIS] < 0) destination[X_AXIS] = 0.0;
      if (destination[Y_AXIS] < 0) destination[Y_AXIS] = 0.0;
//    }
  }

  help_feedrate = ((int32_t)feedrate*(int32_t)100);
  
  plan_buffer_line(destination[X_AXIS], destination[Y_AXIS], help_feedrate/6000.0);
  
  for(int i=0; i < NUM_AXIS; i++)
  {
    current_position[i] = destination[i];
  } 
}

// The arc is approximated by generating a huge number of tiny, linear segments. The length of each 
// segment is configured in settings.mm_per_arc_segment.  
void mc_arc(float *position, float *target, float *offset, uint8_t axis_0, uint8_t axis_1, float feed_rate, float radius, uint8_t isclockwise)
{      
  //   int acceleration_manager_was_enabled = plan_is_acceleration_manager_enabled();
  //   plan_set_acceleration_manager_enabled(false); // disable acceleration management for the duration of the arc
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;
  
  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (angular_travel < 0) { angular_travel += 2*M_PI; }
  if (isclockwise) { angular_travel -= 2*M_PI; }
  
  float millimeters_of_travel = hypot(angular_travel*radius, 0.0 );
  if (millimeters_of_travel < 0.001) { return; }
  uint16_t segments = floor(millimeters_of_travel/MM_PER_ARC_SEGMENT);
  if(segments == 0) segments = 1;

  /*  
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
    // all segments.
    if (invert_feed_rate) { feed_rate *= segments; }
  */
  float theta_per_segment = angular_travel/segments;
  
  /* Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
     and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
         r_T = [cos(phi) -sin(phi);
                sin(phi)  cos(phi] * r ;
     
     For arc generation, the center of the circle is the axis of rotation and the radius vector is 
     defined from the circle center to the initial position. Each line segment is formed by successive
     vector rotations. This requires only two cos() and sin() computations to form the rotation
     matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
     all double numbers are single precision on the Arduino. (True double precision will not have
     round off issues for CNC applications.) Single precision error can accumulate to be greater than
     tool precision in some cases. Therefore, arc path correction is implemented. 

     Small angle approximation may be used to reduce computation overhead further. This approximation
     holds for everything, but very small circles and large mm_per_arc_segment values. In other words,
     theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
     to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for 
     numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
     issue for CNC machines with the single precision Arduino calculations.
     
     This approximation also allows mc_arc to immediately insert a line segment into the planner 
     without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
     a correction, the planner should have caught up to the lag caused by the initial mc_arc overhead. 
     This is important when there are successive arc motions. 
  */
  // Vector rotation matrix values
  float cos_T = 1-0.5*theta_per_segment*theta_per_segment; // Small angle approximation
  float sin_T = theta_per_segment;
  
  float arc_target[2];
  float sin_Ti;
  float cos_Ti;
  float r_axisi;
  uint16_t i;
  int8_t count = 0;

  for (i = 1; i<segments; i++) 
  { // Increment (segments-1)
    
    if((count == 10) || (count == 15))
    {
    //Read the next two Commands while arc is calculating
      check_buffer_while_arc();
    }

    if (count < N_ARC_CORRECTION)  //25 pieces
    {
      // Apply vector rotation matrix 
      r_axisi = r_axis0*sin_T + r_axis1*cos_T;
      r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
      r_axis1 = r_axisi;
      count++;
    }
    else
    {
      // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
      // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
      cos_Ti  = cos(i*theta_per_segment);
      sin_Ti  = sin(i*theta_per_segment);
      r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
      r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
      count = 0;
    }

    // Update arc_target location
    arc_target[axis_0] = center_axis0 + r_axis0;
    arc_target[axis_1] = center_axis1 + r_axis1;
    
    plan_buffer_line(arc_target[X_AXIS], arc_target[Y_AXIS], feed_rate);
    
  }
  // Ensure last segment arrives at target location.
  plan_buffer_line(target[X_AXIS], target[Y_AXIS], feed_rate);

  //   plan_set_acceleration_manager_enabled(acceleration_manager_was_enabled);
}


void prepare_arc_move(char isclockwise) 
{

  float r = hypot(offset[X_AXIS], offset[Y_AXIS]); // Compute arc radius for mc_arc
  int32_t help_feedrate = 0;

    help_feedrate = ((int32_t)feedrate*(int32_t)100);

  // Trace the arc
  mc_arc(current_position, destination, offset, X_AXIS, Y_AXIS, help_feedrate/6000.0, r, isclockwise);
  
  // As far as the parser is concerned, the position is now == target. In reality the
  // motion control system might still be processing the action and the real tool position
  // in any intermediate location.
  for(int8_t i=0; i < NUM_AXIS; i++) 
  {
    current_position[i] = destination[i];
  }
}

inline void kill()
{
//  #if TEMP_0_PIN > -1
//    target_raw=0;
//  #endif
//  
//  #if TEMP_1_PIN > -1
//    target_bed_raw=0;
//  #endif
    digitalWrite(XY_ENABLE_PIN, HIGH );
}

inline void manage_inactivity(byte debug) 
{ 
  
  if( (millis()-previous_millis_cmd) >  max_inactive_time ) if(max_inactive_time) kill(); 
  
  if( (millis()-previous_millis_cmd) >  stepper_inactive_time ) if(stepper_inactive_time) 
  { 
    digitalWrite(XY_ENABLE_PIN, HIGH );
  }
  check_axes_activity();

  DisplayUpdate();
}


// Planner with Interrupt for Stepper

/*  
 Reasoning behind the mathematics in this module (in the key of 'Mathematica'):
 
 s == speed, a == acceleration, t == time, d == distance
 
 Basic definitions:
 
 Speed[s_, a_, t_] := s + (a*t) 
 Travel[s_, a_, t_] := Integrate[Speed[s, a, t], t]
 
 Distance to reach a specific speed with a constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, d, t]
 d -> (m^2 - s^2)/(2 a) --> estimate_acceleration_distance()
 
 Speed after a given distance of travel with constant acceleration:
 
 Solve[{Speed[s, a, t] == m, Travel[s, a, t] == d}, m, t]
 m -> Sqrt[2 a d + s^2]    
 
 DestinationSpeed[s_, a_, d_] := Sqrt[2 a d + s^2]
 
 When to start braking (di) to reach a specified destionation speed (s2) after accelerating
 from initial speed s1 without ever stopping at a plateau:
 
 Solve[{DestinationSpeed[s1, a, di] == DestinationSpeed[s2, a, d - di]}, di]
 di -> (2 a d - s1^2 + s2^2)/(4 a) --> intersection_distance()
 
 IntersectionDistance[s1_, s2_, a_, d_] := (2 a d - s1^2 + s2^2)/(4 a)
 */


static block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instructions
static volatile uint8_t block_buffer_head;           // Index of the next block to be pushed
static volatile uint8_t block_buffer_tail;           // Index of the block to process now

//===========================================================================
//=============================private variables ============================
//===========================================================================

// Returns the index of the next block in the ring buffer
// NOTE: Removed modulo (%) operator, which uses an expensive divide and multiplication.
static int8_t next_block_index(int8_t block_index) {
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


// Returns the index of the previous block in the ring buffer
static int8_t prev_block_index(int8_t block_index) {
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}

// The current position of the tool in absolute steps
static int32_t position[NUM_AXIS];   
static float previous_speed[NUM_AXIS]; // Speed of previous path line segment
static float previous_nominal_speed; // Nominal speed of previous path line segment
static uint8_t G92_reset_previous_speed = 0;


// Calculates the distance (not time) it takes to accelerate from initial_rate to target_rate using the 
// given acceleration:
inline float estimate_acceleration_distance(float initial_rate, float target_rate, float acceleration)
{
  if (acceleration!=0) {
  return((target_rate*target_rate-initial_rate*initial_rate)/
         (2.0*acceleration));
  }
  else {
    return 0.0;  // acceleration was 0, set acceleration distance to 0
  }
}

// This function gives you the point at which you must start braking (at the rate of -acceleration) if 
// you started at speed initial_rate and accelerated until this point and want to end at the final_rate after
// a total travel of distance. This can be used to compute the intersection point between acceleration and
// deceleration in the cases where the trapezoid has no plateau (i.e. never reaches maximum speed)

inline float intersection_distance(float initial_rate, float final_rate, float acceleration, float distance) 
{
 if (acceleration!=0) {
  return((2.0*acceleration*distance-initial_rate*initial_rate+final_rate*final_rate)/
         (4.0*acceleration) );
  }
  else {
    return 0.0;  // acceleration was 0, set intersection distance to 0
  }
}

// Calculates trapezoid parameters so that the entry- and exit-speed is compensated by the provided factors.

void calculate_trapezoid_for_block(block_t *block, float entry_factor, float exit_factor) {
  uint32_t initial_rate = ceil(block->nominal_rate*entry_factor); // (step/min)
  uint32_t final_rate = ceil(block->nominal_rate*exit_factor); // (step/min)

  // Limit minimal step rate (Otherwise the timer will overflow.)
  if(initial_rate < 120) {initial_rate = 120; }
  if(final_rate   < 120) {final_rate   = 120; }
  
  int32_t acceleration = block->acceleration_st;
  int32_t accelerate_steps = ceil( estimate_acceleration_distance(block->initial_rate, block->nominal_rate, acceleration));
  int32_t decelerate_steps = floor(estimate_acceleration_distance(block->nominal_rate, block->final_rate,  -acceleration));
    
  // Calculate the size of Plateau of Nominal Rate.
  int32_t plateau_steps = block->step_event_count - accelerate_steps-decelerate_steps;
  
  // Is the Plateau of Nominal Rate smaller than nothing? That means no cruising, and we will
  // have to use intersection_distance() to calculate when to abort acceleration and start breaking
  // in order to reach the final_rate exactly at the end of this block.
  if (plateau_steps < 0) {
    accelerate_steps = ceil( intersection_distance(block->initial_rate, block->final_rate, acceleration, block->step_event_count));
    accelerate_steps = max(accelerate_steps, 0 ); // Check limits due to numerical round-off
    accelerate_steps = min(accelerate_steps,block->step_event_count);
    plateau_steps = 0;
  }

  
 // block->accelerate_until = accelerate_steps;
 // block->decelerate_after = accelerate_steps+plateau_steps;
  noInterrupts();  // Fill variables used by the stepper in a critical section
  if(block->busy == false) { // Don't update variables if block is busy.
    block->accelerate_until = accelerate_steps;
    block->decelerate_after = accelerate_steps+plateau_steps;
    block->initial_rate     = initial_rate;
    block->final_rate       = final_rate;
  }
  interrupts();
}                    

// Calculates the maximum allowable speed at this point when you must be able to reach target_velocity using the 
// acceleration within the allotted distance.
inline float max_allowable_speed(float acceleration, float target_velocity, float distance) {
  return sqrt(target_velocity * target_velocity - 2 * acceleration * distance);
}

// "Junction jerk" in this context is the immediate change in speed at the junction of two blocks.
// This method will calculate the junction jerk as the euclidean distance between the nominal 
// velocities of the respective blocks.
//inline float junction_jerk(block_t *before, block_t *after) {
//  return sqrt(
//    pow((before->speed_x-after->speed_x), 2)+pow((before->speed_y-after->speed_y), 2));
//}



// The kernel called by planner_recalculate() when scanning the plan from last to first entry.
void planner_reverse_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  
  if(!current) { return; }
  
    if (next) {
    // If entry speed is already at the maximum entry speed, no need to recheck. Block is cruising.
    // If not, block in state of acceleration or deceleration. Reset entry speed to maximum and
    // check for maximum allowable speed reductions to ensure maximum possible planned speed.
    if (current->entry_speed != current->max_entry_speed) {
    
      // If nominal length true, max junction speed is guaranteed to be reached. Only compute
      // for max allowable speed if block is decelerating and nominal length is false.
      if ((!current->nominal_length_flag) && (current->max_entry_speed > next->entry_speed)) {
        current->entry_speed = min( current->max_entry_speed, max_allowable_speed(-current->acceleration,next->entry_speed,current->millimeters));
      } else {
        current->entry_speed = current->max_entry_speed;
      }
      current->recalculate_flag = true;
    
    }
  } // Skip last block. Already initialized and set for recalculation.
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the reverse pass.
void planner_reverse_pass() {
  uint8_t block_index = block_buffer_head;
  
  //Make a local copy of block_buffer_tail, because the interrupt can alter it
  noInterrupts();
  uint8_t tail = block_buffer_tail;
  interrupts();
  
  if(((block_buffer_head-tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1)) > 3) 
  {
    block_index = (block_buffer_head - 3) & (BLOCK_BUFFER_SIZE - 1);
    block_t *block[3] = { NULL, NULL, NULL };
    while(block_index != tail) { 
      block_index = prev_block_index(block_index); 
      block[2]= block[1];
      block[1]= block[0];
      block[0] = &block_buffer[block_index];
      planner_reverse_pass_kernel(block[0], block[1], block[2]);
    }
  }
}


// The kernel called by planner_recalculate() when scanning the plan from first to last entry.
void planner_forward_pass_kernel(block_t *previous, block_t *current, block_t *next) {
  if(!previous) { return; }
  
  // If the previous block is an acceleration block, but it is not long enough to complete the
  // full speed change within the block, we need to adjust the entry speed accordingly. Entry
  // speeds have already been reset, maximized, and reverse planned by reverse planner.
  // If nominal length is true, max junction speed is guaranteed to be reached. No need to recheck.
  if (!previous->nominal_length_flag) {
    if (previous->entry_speed < current->entry_speed) {
      double entry_speed = min( current->entry_speed, max_allowable_speed(-previous->acceleration,previous->entry_speed,previous->millimeters) );
      // Check for junction speed change
      if (current->entry_speed != entry_speed) {
        current->entry_speed = entry_speed;
        current->recalculate_flag = true;
      }
    }
  }
}

// planner_recalculate() needs to go over the current plan twice. Once in reverse and once forward. This 
// implements the forward pass.
void planner_forward_pass() {
  uint8_t block_index = block_buffer_tail;
  block_t *block[3] = { NULL, NULL, NULL };
  while(block_index != block_buffer_head) {
    block[0] = block[1];
    block[1] = block[2];
    block[2] = &block_buffer[block_index];
    planner_forward_pass_kernel(block[0],block[1],block[2]);
    block_index = next_block_index(block_index);
  }
  planner_forward_pass_kernel(block[1], block[2], NULL);
}

// Recalculates the trapezoid speed profiles for all blocks in the plan according to the 
// entry_factor for each junction. Must be called by planner_recalculate() after 
// updating the blocks.
void planner_recalculate_trapezoids() {
  int8_t block_index = block_buffer_tail;
  block_t *current;
  block_t *next = NULL;
  while(block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    if (current) {
      // Recalculate if current block entry or exit junction speed has changed.
      if (current->recalculate_flag || next->recalculate_flag) {
        // NOTE: Entry and exit factors always > 0 by all previous logic operations.
        calculate_trapezoid_for_block(current, current->entry_speed/current->nominal_speed,next->entry_speed/current->nominal_speed);
        current->recalculate_flag = false; // Reset current only to ensure next trapezoid is computed
      }
    }
    block_index = next_block_index( block_index );
  }
  // Last/newest block in buffer. Exit speed is set with MINIMUM_PLANNER_SPEED. Always recalculated.
  if(next != NULL) {
    calculate_trapezoid_for_block(next, next->entry_speed/next->nominal_speed,MINIMUM_PLANNER_SPEED/next->nominal_speed);
    next->recalculate_flag = false;
  }
}

// Recalculates the motion plan according to the following algorithm:
//
//   1. Go over every block in reverse order and calculate a junction speed reduction (i.e. block_t.entry_factor) 
//      so that:
//     a. The junction jerk is within the set limit
//     b. No speed reduction within one block requires faster deceleration than the one, true constant 
//        acceleration.
//   2. Go over every block in chronological order and dial down junction speed reduction values if 
//     a. The speed increase within one block would require faster accelleration than the one, true 
//        constant acceleration.
//
// When these stages are complete all blocks have an entry_factor that will allow all speed changes to 
// be performed using only the one, true constant acceleration, and where no junction jerk is jerkier than 
// the set limit. Finally it will:
//
//   3. Recalculate trapezoids for all blocks.

void planner_recalculate() {   
  planner_reverse_pass();
  planner_forward_pass();
  planner_recalculate_trapezoids();
}

void plan_init() {
  block_buffer_head = 0;
  block_buffer_tail = 0;
  memset(position, 0, sizeof(position)); // clear position
  previous_speed[X_AXIS] = 0.0;
  previous_speed[Y_AXIS] = 0.0;
  previous_nominal_speed = 0.0;
}



inline void plan_discard_current_block() {
  if (block_buffer_head != block_buffer_tail) {
    block_buffer_tail = (block_buffer_tail + 1) & BLOCK_BUFFER_MASK;  
  }
}

inline block_t *plan_get_current_block() {
  if (block_buffer_head == block_buffer_tail) { 
    return(NULL); 
  }
  block_t *block = &block_buffer[block_buffer_tail];
  block->busy = true;
  return(block);
}

// Gets the current block. Returns NULL if buffer empty
//inline bool blocks_queued() 
bool blocks_queued() 
{
  if (block_buffer_head == block_buffer_tail) { 
    return false; 
  }
  else
    return true;
}

void check_axes_activity() {
  uint8_t x_active = 0;
  uint8_t y_active = 0;  
  block_t *block;
  if(block_buffer_tail != block_buffer_head) {
    uint8_t block_index = block_buffer_tail;
    while(block_index != block_buffer_head) {
      block = &block_buffer[block_index];
      if(block->steps_x != 0) x_active++;
      if(block->steps_y != 0) y_active++;
      block_index = (block_index+1) & (BLOCK_BUFFER_SIZE - 1);
    }
  }
  if( (x_active == 0) && (y_active == 0) ) {
    digitalWrite( XY_ENABLE_PIN, HIGH );    
  }
}


float junction_deviation = 0.1;

// Add a new linear movement to the buffer. steps_x, _y and _z is the absolute position in 
// mm. Microseconds specify how many microseconds the move should take to perform. To aid acceleration
// calculation the caller must also provide the physical length of the line in millimeters.
void plan_buffer_line(float x, float y, float feed_rate)
{
  // Calculate the buffer head after we push this byte
  int next_buffer_head = next_block_index(block_buffer_head);

  // If the buffer is full: good! That means we are well ahead of the robot. 
  // Rest here until there is room in the buffer.
  while(block_buffer_tail == next_buffer_head) { 
    manage_inactivity(1); 
  }

  // The target position of the tool in absolute steps
  // Calculate target position in absolute steps
  //this should be done after the wait, because otherwise a M92 code within the gcode disrupts this calculation somehow
  int32_t target[2];
  target[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
  target[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);
  
  // Prepare to set up new block
  block_t *block = &block_buffer[block_buffer_head];
  
  // Mark block as not busy (Not executed by the stepper interrupt)
  block->busy = false;

  // Number of steps for each axis
  block->steps_x = labs(target[X_AXIS]-position[X_AXIS]);
  block->steps_y = labs(target[Y_AXIS]-position[Y_AXIS]);
  block->step_event_count = max(block->steps_x, block->steps_y);

  // Bail if this is a zero-length block
  if (block->step_event_count <=dropsegments) { return; };

  // Compute direction bits for this block 
  block->direction_bits = 0;
  if (target[X_AXIS] < position[X_AXIS]) { block->direction_bits |= (1<<X_AXIS); }
  if (target[Y_AXIS] < position[Y_AXIS]) { block->direction_bits |= (1<<Y_AXIS); }
 

  //enable active axes
  digitalWrite( XY_ENABLE_PIN, LOW );
//  if(block->steps_x != 0) enable_x();
//  if(block->steps_y != 0) enable_y();
 
 	if(feed_rate<minimumfeedrate) feed_rate=minimumfeedrate;

  // slow down when the buffer starts to empty, rather than wait at the corner for a buffer refill
  int moves_queued=(block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
#ifdef SLOWDOWN  
  if(moves_queued < (BLOCK_BUFFER_SIZE * 0.5) && moves_queued > 1) feed_rate = feed_rate*moves_queued / (BLOCK_BUFFER_SIZE * 0.5); 
#endif

  float delta_mm[4];
  delta_mm[X_AXIS] = (target[X_AXIS]-position[X_AXIS])/axis_steps_per_unit[X_AXIS];
  delta_mm[Y_AXIS] = (target[Y_AXIS]-position[Y_AXIS])/axis_steps_per_unit[Y_AXIS];
  
//  if ( block->steps_x <= dropsegments && block->steps_y <= dropsegments && block->steps_z <= dropsegments ) {
//    block->millimeters = fabs(delta_mm[E_AXIS]);
//  } else {
    block->millimeters = sqrt(sq(delta_mm[X_AXIS]) + sq(delta_mm[Y_AXIS]));
//  }
  
  float inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple divides 
  
  // Calculate speed in mm/second for each axis. No divide by zero due to previous checks.
  float inverse_second = feed_rate * inverse_millimeters;
  
  block->nominal_speed = block->millimeters * inverse_second; // (mm/sec) Always > 0
  block->nominal_rate = ceil(block->step_event_count * inverse_second); // (step/sec) Always > 0

 // Calculate and limit speed in mm/sec for each axis
  float current_speed[NUM_AXIS];
  float speed_factor = 1.0; //factor <=1 do decrease speed
  for(int i=0; i < NUM_AXIS; i++) {
    current_speed[i] = delta_mm[i] * inverse_second;
    if(fabs(current_speed[i]) > max_feedrate[i])
      speed_factor = min(speed_factor, max_feedrate[i] / fabs(current_speed[i]));
  }

  // Correct the speed  
  if( speed_factor < 1.0) {
    for(uint8_t i=0; i < NUM_AXIS; i++) {
      current_speed[i] *= speed_factor;
    }
    block->nominal_speed *= speed_factor;
    block->nominal_rate *= speed_factor;
  }

  // Compute and limit the acceleration rate for the trapezoid generator.  
  float steps_per_mm = block->step_event_count/block->millimeters;
  block->acceleration_st = ceil(move_acceleration * steps_per_mm); // convert to: acceleration steps/sec^2
  // Limit acceleration per axis
  if(((float)block->acceleration_st * (float)block->steps_x / (float)block->step_event_count) > axis_steps_per_sqr_second[X_AXIS])
    block->acceleration_st = axis_steps_per_sqr_second[X_AXIS];
  if(((float)block->acceleration_st * (float)block->steps_y / (float)block->step_event_count) > axis_steps_per_sqr_second[Y_AXIS])
    block->acceleration_st = axis_steps_per_sqr_second[Y_AXIS];
  block->acceleration = block->acceleration_st / steps_per_mm;
  block->acceleration_rate = (int32_t)((float)block->acceleration_st * 8.388608);
  
  // Start with a safe speed
  float vmax_junction = max_xy_jerk/2; 
  float vmax_junction_factor = 1.0; 

  if(G92_reset_previous_speed == 1)
  {
    vmax_junction = 0.1;
    G92_reset_previous_speed = 0;  
  }

  vmax_junction = min(vmax_junction, block->nominal_speed);
  float safe_speed = vmax_junction;

  if ((moves_queued > 1) && (previous_nominal_speed > 0.0001)) {
    float jerk = sqrt( pow( (current_speed[X_AXIS] - previous_speed[X_AXIS]), 2) );
    //    if((fabs(previous_speed[X_AXIS]) > 0.0001) || (fabs(previous_speed[Y_AXIS]) > 0.0001)) {
    vmax_junction = block->nominal_speed;
    //    }
    if (jerk > max_xy_jerk) {
      vmax_junction_factor = (max_xy_jerk/jerk);
    } 
    vmax_junction = min(previous_nominal_speed, vmax_junction * vmax_junction_factor); // Limit speed to max previous speed
  }
  block->max_entry_speed = vmax_junction;

  // Initialize block entry speed. Compute based on deceleration to user-defined MINIMUM_PLANNER_SPEED.
  double v_allowable = max_allowable_speed(-block->acceleration,MINIMUM_PLANNER_SPEED,block->millimeters);
  block->entry_speed = min(vmax_junction, v_allowable);

  // Initialize planner efficiency flags
  // Set flag if block will always reach maximum junction speed regardless of entry/exit speeds.
  // If a block can de/ac-celerate from nominal speed to zero within the length of the block, then
  // the current block and next block junction speeds are guaranteed to always be at their maximum
  // junction speeds in deceleration and acceleration, respectively. This is due to how the current
  // block nominal speed limits both the current and next maximum junction speeds. Hence, in both
  // the reverse and forward planners, the corresponding block junction speed will always be at the
  // the maximum junction speed and may always be ignored for any speed reduction checks.
  if (block->nominal_speed <= v_allowable) { 
    block->nominal_length_flag = true; 
  }
  else { 
    block->nominal_length_flag = false; 
  }
  block->recalculate_flag = true; // Always calculate trapezoid for new block

  // Update previous path unit_vector and nominal speed
  memcpy(previous_speed, current_speed, sizeof(previous_speed)); // previous_speed[] = current_speed[]
  previous_nominal_speed = block->nominal_speed;

  calculate_trapezoid_for_block(block, block->entry_speed/block->nominal_speed,
    safe_speed/block->nominal_speed);
    
  // Move buffer head
  block_buffer_head = next_buffer_head;
  
  // Update position
  memcpy(position, target, sizeof(target)); // position[] = target[]

  planner_recalculate();
//  st_wake_up();
}

int calc_plannerpuffer_fill(void)
{
  int moves_queued=(block_buffer_head-block_buffer_tail + BLOCK_BUFFER_SIZE) & (BLOCK_BUFFER_SIZE - 1);
  return(moves_queued);
}

void plan_set_position(float x, float y)
{
  position[X_AXIS] = lround(x*axis_steps_per_unit[X_AXIS]);
  position[Y_AXIS] = lround(y*axis_steps_per_unit[Y_AXIS]);

  virtual_steps_x = 0;
  virtual_steps_y = 0;

  previous_nominal_speed = 0.0; // Resets planner junction speeds. Assumes start from rest.
  previous_speed[X_AXIS] = 0.0;
  previous_speed[Y_AXIS] = 0.0;
  
  G92_reset_previous_speed = 1;
}




// Stepper


static block_t *current_block;  // A pointer to the block currently being traced

// Variables used by The Stepper Driver Interrupt
static uint8_t out_bits;        // The next stepping-bits to be output
static int32_t counter_x,       // Counter variables for the bresenham line tracer
            counter_y;
static uint32_t step_events_completed; // The number of step events executed in the current block
static uint8_t busy = false; // TRUE when SIG_OUTPUT_COMPARE1A is being serviced. Used to avoid retriggering that handler.
static uint32_t acceleration_time, deceleration_time;
static uint32_t acc_step_rate; // needed for deceleration start point
static char step_loops;
static unsigned short step_loops_nominal;
static uint32_t TimerCount_nominal;

static bool old_x_min_endstop=false;
static bool old_x_max_endstop=false;
static bool old_y_min_endstop=false;
static bool old_y_max_endstop=false;

#define STEPPER_TIMER      TC4
#define STEPPER_TIMER_ID  GCM_TC4_TC5

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  NVIC_EnableIRQ(TC4_IRQn)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() NVIC_DisableIRQ(TC4_IRQn)
#define STEPPER_TIMER_ISR() void TC4_Handler()
#define STEPPER_TIMER_OCR TC4->COUNT16.CC[0].reg

#define MultiU16X8toH16(intRes, charIn1, intIn2) ((charIn1)*(intIn2)>>8)
#define MultiU24X24toH16(intRes, longIn1, longIn2)  ((intRes) = (longIn1) * (longIn2)>>24)



//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
// 
//  The trapezoid is the shape of the speed curve over time. It starts at block->initial_rate, accelerates 
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until 
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.


inline unsigned short calc_timer(unsigned short step_rate) {
  unsigned short timer;
  if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;

  if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
    step_rate = (step_rate >> 2)&0x3fff;
    step_loops = 4;
  }
  else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
    step_rate = (step_rate >> 1)&0x7fff;
    step_loops = 2;
  }
  else {
    step_loops = 1;
  }

  if(step_rate < (2000000/50000)) step_rate = (2000000/50000);
  step_rate -= (2000000/50000); // Correct for minimal speed
  if(step_rate >= (8*256)){ // higher step rate
    unsigned int table_address = (unsigned int)&speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0];
    unsigned char tmp_step_rate = (step_rate & 0x00ff);
    unsigned short gain = (unsigned short)pgm_read_word_near(table_address+2);
    MultiU16X8toH16(timer, tmp_step_rate, gain);
    timer = (unsigned short)pgm_read_word_near(table_address) - timer;
  }
  else { // lower step rates
    unsigned int table_address = (unsigned int)&speed_lookuptable_slow[0][0];
    table_address += ((step_rate)>>1) & 0xfffc;
    timer = (unsigned short)pgm_read_word_near(table_address);
    timer -= (((unsigned short)pgm_read_word_near(table_address+2) * (unsigned char)(step_rate & 0x0007))>>3);
  }
  if(timer < 100) { timer = 100; SerialUSB.print("Stepper Freq Too High!:"); SerialUSB.println(step_rate); }//(20kHz this should never happen)
  return timer;
}



// Initializes the trapezoid generator from the current block. Called whenever a new 
// block begins.
inline void trapezoid_generator_reset()
{
  deceleration_time = 0;
  // step_rate to timer interval
  TimerCount_nominal = calc_timer(current_block->nominal_rate);
  // make a note of the number of step loops required at nominal speed
  step_loops_nominal = step_loops;
  acc_step_rate = current_block->initial_rate;
  acceleration_time = calc_timer(acc_step_rate);
  STEPPER_TIMER_OCR = acceleration_time;
  
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.  
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately. 
//void STEPPER_TC_ISR()
STEPPER_TIMER_ISR()
{        
  digitalWrite(PIN_LED,HIGH);

    STEPPER_TIMER->COUNT16.INTFLAG.reg = TC_INTFLAG_MC0;
  
  digitalWrite(PIN_LED,LOW);

  if( ErmergencyStop == true ) {
    return;
  }
    
  bool x_inc = true;
  bool y_inc = true;
  
  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
      trapezoid_generator_reset();
      counter_x = -(current_block->step_event_count >> 1);
      counter_y = counter_x;
      step_events_completed = 0;
    } 
    else {
      //calc_timer(1000);
      //SetTimer(Timer_prescalerConfigBits, Timer_ccValue);
      //setTimer(2000);
      STEPPER_TIMER_OCR=2000; // 1kHz.
    }    
  } 

  if (current_block != NULL) {
    // Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
    out_bits = current_block->direction_bits;

    // Set direction and check limit switches
    if ((out_bits & (1<<X_AXIS)) != 0) {   // -direction
      Clr_X_DIR();
      x_inc = false;
          bool x_min_endstop = X_MIN_HIT();
          if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
            real_position[0] = 0.0;
            if(!is_homing) {
              endstop_x_hit=true;
            } else {
              step_events_completed = current_block->step_event_count;
            }
          }
          else
          {
            endstop_x_hit=false;
          }
          old_x_min_endstop = x_min_endstop;
    }
    else { // +direction 
      Set_X_DIR();
      x_inc = true;
          endstop_x_hit=false;
    }

    if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
      Clr_Y_DIR();
      y_inc = false;
          bool y_min_endstop = Y_MIN_HIT();
          if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
            real_position[1] = 0.0;
            if(!is_homing) {
              endstop_y_hit=true;
            } else {
              step_events_completed = current_block->step_event_count;
            }
          }
          else
          {
            endstop_y_hit=false;
          }
          old_y_min_endstop = y_min_endstop;
    }
    else { // +direction
      Set_Y_DIR();
      y_inc = true;
      endstop_y_hit=false;  
    }

    
    for(int8_t i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves) 
      counter_x += current_block->steps_x;
      if (counter_x > 0) {
        if(!endstop_x_hit)
        {
          if(virtual_steps_x) {
            virtual_steps_x--;
          }
          else {
            Set_X_STEP();
            if(x_inc) {
              real_position[0] += 0.005f; //X_MM_PER_STEP;
            } else {
              real_position[0] -= 0.005f; //X_MM_PER_STEP;
            }
          }
        }
        else {
          virtual_steps_x++;
        }
        counter_x -= current_block->step_event_count;
        Clr_X_STEP();
      }

      counter_y += current_block->steps_y;
      if (counter_y > 0) {
        if(!endstop_y_hit)
        {
          if(virtual_steps_y) {
            virtual_steps_y--;
          }
          else {
            Set_Y_STEP();
            if(y_inc) {
              real_position[1] += 0.005f; //Y_MM_PER_STEP;
            } else {
              real_position[1] -= 0.005f; //Y_MM_PER_STEP;
            }
          }
        }
        else {
          virtual_steps_y++;
        }
        counter_y -= current_block->step_event_count;
        Clr_Y_STEP();
      }

      step_events_completed += 1;  
      if(step_events_completed >= current_block->step_event_count) break;
      
    }
    // Calculare new timer value
    uint32_t timer;
    uint32_t step_rate;
    if (step_events_completed <= (uint32_t)current_block->accelerate_until) {

      MultiU24X24toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      acc_step_rate += current_block->initial_rate;

      // upper limit
      if(acc_step_rate > current_block->nominal_rate)
        acc_step_rate = current_block->nominal_rate;

      // step_rate to timer interval
      timer = calc_timer(acc_step_rate);
      STEPPER_TIMER_OCR = timer;
      acceleration_time += timer;


    } 
    else if (step_events_completed > (uint32_t)current_block->decelerate_after) {   

      MultiU24X24toH16(step_rate, deceleration_time, current_block->acceleration_rate);

      if(step_rate > acc_step_rate) { // Check step_rate stays positive
        step_rate = current_block->final_rate;
      }
      else {
        step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
      }

      // lower limit
      if(step_rate < current_block->final_rate)
        step_rate = current_block->final_rate;

      // step_rate to timer interval
      timer = calc_timer(step_rate);
      STEPPER_TIMER_OCR = timer;
      deceleration_time += timer;

     
    }
    else {
        STEPPER_TIMER_OCR = TimerCount_nominal;
        // ensure we're running at the correct step rate, even if we just came off an acceleration
        step_loops = step_loops_nominal;
    }

    // If current block is finished, reset pointer 
    if (step_events_completed >= current_block->step_event_count) {
      real_position[0] = destination[0];
      real_position[1] = destination[1];
      current_block = NULL;
      plan_discard_current_block();
    }   
  } 
}

void initStepperTimer(void)
{
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID( STEPPER_TIMER_ID )) ;
  
  STEPPER_TIMER->COUNT16.CTRLA.reg &= ~(TC_CTRLA_ENABLE);       //disable TC module
  STEPPER_TIMER->COUNT16.CTRLA.reg |=TC_CTRLA_MODE_COUNT16;
  STEPPER_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
  STEPPER_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4;
  STEPPER_TIMER->COUNT16.CC[0].reg = 2000;
  STEPPER_TIMER->COUNT16.INTENSET.reg = TC_INTFLAG_MC0;
  STEPPER_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
  STEPPER_TIMER->COUNT16.INTFLAG.reg = 0xFF;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}


void st_init()
{

  initStepperTimer();

//  enable_endstops(true);

  interrupts();

}

// Block until all buffered steps are executed
void st_synchronize()
{
  while(blocks_queued()) {
    manage_inactivity(1);
  }   
}



