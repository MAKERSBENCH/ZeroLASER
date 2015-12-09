


#ifndef _LASER_H_
#define _LASER_H_

#include "Arduino.h"
#include "stdint.h"
#include "Configuration.h"


#define LASER_DEBUG
#ifdef LASER_DEBUG
#define LASER_SERIAL_DEVICE   PLATFORM_SERIAL
#endif

#define LASER_PWM_PERIOD    1024
#define LASER_TCC           TCC0
#define LASER_TCC_CLOCKS    GCLK_CLKCTRL_ID_TCC0_TCC1
#define LASER_PWM_PIN       2

extern void LaserInit(); 
extern void LaserPowerLevel( byte pl );
extern byte LaserPowerLevel();
extern float LaserPwmPercent();
extern void LaserEnable( bool state );
extern bool LaserEnabled();
extern bool LaserOn();

#endif

