


#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include "stdint.h"
#include "Arduino.h"
#include "Configuration.h"
#include "FT_VM800P43_50.h"


#define DISPLAY_DEBUG
#ifdef DISPLAY_DEBUG
#define DISPLAY_SERIAL_DEVICE   PLATFORM_SERIAL
#endif

//#define DISPLAY_CUSTOM_FONTS
#ifdef DISPLAY_CUSTOM_FONTS
#include "DigitalFont.h"
#endif

#ifdef DISPLAY_CUSTOM_FONTS
#define FNT_DIGITAL         6
#endif

#define FNT_30              30

typedef enum Screen
{  
  SCRN_BOOTING          = 0,
  SCRN_WARNING          = 1,
  SCRN_MAIN_MENU        = 2,
  SCRN_MANUAL_MOVE      = 3,
  SCRN_MANUAL_LASER     = 4,
  SCRN_MONITOR          = 5,
  SCRN_NO_DISPLAY       = -99
} Screen;

#define STEPSIZE_DIAL_DIV     (65535/12)
#define STEPSIZE_DIAL_MIN     ( 2 * STEPSIZE_DIAL_DIV)
#define STEPSIZE_DIAL_MAX     (10 * STEPSIZE_DIAL_DIV)

#define STEPSIZE_DIAL_0       ( 3 * STEPSIZE_DIAL_DIV)
#define STEPSIZE_DIAL_1       ( 5 * STEPSIZE_DIAL_DIV)
#define STEPSIZE_DIAL_2       ( 7 * STEPSIZE_DIAL_DIV)
#define STEPSIZE_DIAL_3       ( 9 * STEPSIZE_DIAL_DIV)
#define STEPSIZE_DIAL_4       ( 3 * STEPSIZE_DIAL_DIV)
#define STEPSIZE_DIAL_5       ( 3 * STEPSIZE_DIAL_DIV)
#define STEPSIZE_DIAL_6       ( 3 * STEPSIZE_DIAL_DIV)

#define STEPSIZE_TICK_0     (12288)
#define STEPSIZE_TICK_1     (20480)
#define STEPSIZE_TICK_2     (28672)
#define STEPSIZE_TICK_3     (36864)
#define STEPSIZE_TICK_4     (45056)
#define STEPSIZE_TICK_5     (53284)

#define BUTTON_BOOT_CONTINUE  'C'
#define BUTTON_WARN_OK        'W'
#define BUTTON_MAIN_MENU      'T'
#define BUTTON_MANUAL_MOVE    'M'
#define BUTTON_LASER_CAL      'F'
#define BUTTON_MONITOR        'G'

#define BUTTON_MOVE_HOME      'H'
#define BUTTON_MOVE_PLUS_X    'R'
#define BUTTON_MOVE_MINUS_X   'L'
#define BUTTON_MOVE_PLUS_Y    'U'
#define BUTTON_MOVE_MINUS_Y   'D'
#define BUTTON_LASER_PULSE    'P'
#define BUTTON_EMG_STOP       'S'

#define TOGGLE_LASER_ENABLE   'l'
#define TOGGLE_MOVE_ENABLE    's'

#define DIAL_STEPSIZE         'I'


#define BUTTON_NONE           65535

extern bool DisplayInit();
extern void DisplayFadeIn();
extern void DisplayFadeOut();
//extern uint32_t DisplayGetTags();
extern uint8_t DisplayButtonPressed();
extern uint8_t DisplayReadButtons();
extern void DisplaySetScreen( Screen scrn );
extern bool DisplayAttached();
extern void DisplayUpdateLimitSwitches( bool xmin, bool ymin, bool interlock );
extern void DisplayUpdateLocation( float xloc, float yloc );
extern void DisplayUpdateLaserPower( float mA, float pwm );
extern void DisplayUpdateWaterCooling( float in, float out );
extern void DisplayCalibrateTouch();
extern void DisplayScreen();
extern uint16_t DisplayReadDial( uint8_t tag );
extern void DisplaySetStepSizeDial( uint16_t val );
extern void DisplayUpdate();

extern bool blocks_queued();


#endif

