#ifndef Includes_h
#define Includes_h
#ifdef __cplusplus

#define MAP_SPD 13        // Size of speed dimension of map
#define MAP_TPS 11        // Size of throttle position dimension of map

#define TOOTH_OFFSET 240  // How many degrees after missing tooth is TDC
#define TOOTH_PIN 2       // The missing tooth pulse input
#define FUEL_PIN 12       // The fuel injector control pin
#define IGN_PIN 13        // the pin that the ignition coild is attached to

#define THROTTLE_PIN 0    // Throttle position sensor analog input
#define LAMBDA_PIN 1      // Lambda sensor analog input pin
#define FUEL_TRIM 2       // Fuel trim potentiometer analog input
#define IGN_TRIM 3        // Ignition trim potentiometer analog input

// Maps
// The engine maps store values for open loop operation
// The first dimension in the array are the engine load/throttle position in multiples of 10 between 0 and 100
// The second dimension is for the engine speed in multiples of 100 up to 6000 RPM
//       0      500     1000    1500    2000    2500    3000    3500    4000    4500    5000    5500  6000
extern unsigned fuelMap[2][MAP_TPS][MAP_SPD];

void mapReset( char );
void mapOutput( char );
void mapDisplay( char );

#endif
#endif
