#include "Arduino.h"
#include "includes.h"

// Maps
// The engine maps store values for open loop operation
// The first dimension in the array are the engine load/throttle position in multiples of 10 between 0 and 100
// The second dimension is for the engine speed in multiples of 100 up to 6000 RPM
//       0      500     1000    1500    2000    2500    3000    3500    4000    4500    5000    5500  6000
unsigned fuelMap[2][MAP_TPS][MAP_SPD] = {{
	{0,	112,	224,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	248,	336,	600,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	348,	561,	864,	1200,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	520,	786,	1232,	1536,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	656,	1011,	1600,	2352,	2832,	0,	0,	0,	0,	0,	0,	0},
	{0,	792,	1236,	1968,	2604,	3312,	0,	0,	0,	0,	0,	0,	0},
	{0,	928,	1461,	2336,	3138,	4288,	4984,	0,	0,	0,	0,	0,	0},
	{0,	1064,	1686,	2704,	3672,	4679,	5680,	0,	0,	0,	0,	0,	0},
	{0,	1200,	1991,	3072,	4206,	5365,	6752,	7552,	0,	0,	0,	0,	0},
	{0,	1336,	2136,	3440,	4740,	6051,	6992,	8352,	8600,	0,	0,	0,	0},
	{0,	1472,	2368,	3808,	5280,	6720,	7648,	8960,	9000,	0,	0,	0,	0}
},
{
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0}
}};

 void mapReset(char dim ){
  for(char tps = 0; tps < MAP_TPS; tps++){
    for(char spd = 0; spd < MAP_SPD; spd++){
      fuelMap[dim][tps][spd] = 0;
    }
  }
 }
 
 void mapOutput(char dim){
   Serial.print("\nunsigned fuelMap[");Serial.print((byte)dim);Serial.print("][11][13] = {");
    for(char tps = 0; tps < MAP_TPS; tps++){
      Serial.print("\n\t{");
      for(char spd = 0; spd < MAP_SPD; spd++){
         Serial.print(fuelMap[dim][tps][spd]);
         
         if(spd != MAP_SPD - 1){
            Serial.print(",\t");
         }
      }
      Serial.print("}");
      if(tps != MAP_TPS - 1){
         Serial.print(",");
      }
    }
    Serial.print("\n};\n");
 }
 
 void mapDisplay(char dim){
   Serial.print("\n");
    for(char spd = -1; spd < MAP_SPD; spd++){
      for(char tps = -1; tps < MAP_TPS; tps++){
          if(spd == -1){
            Serial.print(tps*10);Serial.print("\t"); 
          }else{
           if(tps == -1){
             Serial.print(spd*500);
           }else{
             Serial.print(fuelMap[dim][tps][spd]);
           }
           Serial.print("\t");
          }
      }
      Serial.print("\n");
    }
    Serial.print("\n"); 
 }
 
 // Do a short pulse on the given pin
// Can be used to show events as a pin output
void outputMarker(unsigned pin){
   digitalWrite(pin,LOW);
   delayMicroseconds(10);
   digitalWrite(pin,HIGH);
   delayMicroseconds(200);
   digitalWrite(pin,LOW); 
   delayMicroseconds(10);
}

//  Calculate the number of timer counts required for the given time with the given prescaler
//  Time is in nanoseconds.
//  Returns the integer value to load into the timer counter register
unsigned calcTime(unsigned long time, unsigned prescaler){
  return (int)(time/(prescaler >> 4));
}
