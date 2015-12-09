// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// Licence: GPL


//Check Version of Arduino and then include the right libraries
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>  
#endif

#include "Configuration.h"

#include <avr/dtostrf.h>
#include "FT_VM800P43_50.h"
#include "pins.h"
#include "Display.h"
#include "Laser.h"


extern "C" void __cxa_pure_virtual();

#define  FORCE_INLINE __attribute__((always_inline)) inline


#define X_AXIS 0
#define Y_AXIS 1

#define JOGSTEPSIZE_0     0.001
#define JOGSTEPSIZE_1     0.010
#define JOGSTEPSIZE_2     0.100
#define JOGSTEPSIZE_3     1.0
#define JOGSTEPSIZE_4     10.0
#define JOGSTEPSIZE_5     100.0






// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in 
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y;  // Step count along each axis

  uint32_t step_event_count;                    // The number of step events required to complete this block
  long accelerate_until;           // The index of the step event on which to stop acceleration
  long decelerate_after;           // The index of the step event on which to start decelerating
  long acceleration_rate;          // The acceleration rate used for acceleration calculation
  unsigned char direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)

  // Fields used by the motion planner to manage acceleration
//  float speed_x, speed_y, speed_z, speed_e;          // Nominal mm/minute for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/min  
  float entry_speed;                                 // Entry speed at previous-current junction in mm/min
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/min
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned char recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned char nominal_length_flag;                 // Planner flag for nominal speed always reached


  // Settings for the trapezoid generator
  uint32_t nominal_rate;                                 // The nominal step rate for this block in step_events/sec 
  uint32_t initial_rate;                        // The jerk-adjusted step rate at start of block  
  uint32_t final_rate;                          // The minimal rate at exit
  uint32_t acceleration_st;                              // acceleration steps/sec^2
  volatile char busy;
} block_t;


void FlushSerialRequestResend();
void ClearToSend();

void analogWrite_check(uint8_t check_pin, int val);
void showString (PGM_P s);

void manage_inactivity(byte debug);

void get_command();
void get_coordinates();
void prepare_move();
void prepare_arc_move(char isclockwise);
FORCE_INLINE void process_commands();
#ifdef USE_ARC_FUNCTION
  FORCE_INLINE void get_arc_coordinates();
#endif

void kill(byte debug);

void check_axes_activity();
void plan_init();
void st_init();
void tp_init();
void plan_buffer_line(float x, float y, float feed_rate);
void plan_set_position(float x, float y);
void st_wake_up();
void st_synchronize();
void st_set_position(const long &x, const long &y);

void check_buffer_while_arc();

void HomeAllAxis();
void ManualJog(int axis, bool positive);

//==========================================================
// G L O B A L S 
//==========================================================
extern volatile float real_position[NUM_AXIS];
extern bool ErmergencyStop;


