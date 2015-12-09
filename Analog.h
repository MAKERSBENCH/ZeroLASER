
#ifndef __ANALOG_H__
#define __ANALOG_H__

#define ANALOG_CHANNEL_LASER      0
#define ANALOG_CHANNEL_THERM0     1
#define ANALOG_CHANNEL_THERM1     2

#define ANALOG_REF_V              (3.3f)
#define CURRENT_SENSE_RESISTOR    (100.0f)


extern void AnalogInit();
extern float AnalogMeasurement( int channel );



#endif
