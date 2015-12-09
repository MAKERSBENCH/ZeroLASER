#ifndef PINS_H
#define PINS_H

#ifndef __SAMD21G18A__
#error Oops!  Make sure you have 'Arduino Zero' selected from the boards menu.
#endif



#define X_MIN_HIT()         ((REG_PORT_IN0 & PORT_PA11) ? false : true)
#define Y_MIN_HIT()         ((REG_PORT_IN0 & PORT_PA10) ? false : true)
#define INTERLOCK_SET()     ((REG_PORT_IN0 & PORT_PA05) ? false : true)

#define Set_X_STEP()        (REG_PORT_OUTSET0 = PORT_PA07)
#define Clr_X_STEP()        (REG_PORT_OUTCLR0 = PORT_PA07)
//#define Toggle_X_STEP()     (REG_PORT_OUTTGL0 = PORT_PA07)

#define Set_Y_STEP()        (REG_PORT_OUTSET0 = PORT_PA06)
#define Clr_Y_STEP()        (REG_PORT_OUTCLR0 = PORT_PA06)
//#define Toggle_Y_STEP()     (REG_PORT_OUTTGL0 = PORT_PA06)

#define Set_X_DIR()         (REG_PORT_OUTSET0 = PORT_PA21)
#define Clr_X_DIR()         (REG_PORT_OUTCLR0 = PORT_PA21)

#define Set_Y_DIR()         (REG_PORT_OUTSET0 = PORT_PA20)
#define Clr_Y_DIR()         (REG_PORT_OUTCLR0 = PORT_PA20)


#define PIN_LASERn          2     // PA14

#define XY_ENABLE_PIN       5

#define X_STEP_PIN          9
#define X_DIR_PIN           7
#define X_MIN_PIN           0

#define Y_STEP_PIN          8
#define Y_DIR_PIN           6
#define Y_MIN_PIN           1

//#define PIN_LED            13
#define FAN_PIN            11
#define ALARM_PIN          -1
#define INTERLOCK_PIN      18

#define TEMP_0_PIN          1    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!
#define TEMP_1_PIN          2    // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!!

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
const int sensitive_pins[] = {0, 1, X_STEP_PIN, X_DIR_PIN, XY_ENABLE_PIN, X_MIN_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_MIN_PIN, FAN_PIN, TEMP_0_PIN, TEMP_1_PIN};

#endif


