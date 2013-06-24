#include "Arduino.h"
#include "includes.h"

// Maps
// The engine maps store values for open loop operation
// The first dimension in the array are the engine load/throttle position in multiples of 10 between 0 and 100
// The second dimension is for the engine speed in multiples of 100 up to 6000 RPM
//       0      500     1000    1500    2000    2500    3000    3500    4000    4500    5000    5500  6000
// actual recorded mapped map:
/*
-10	0	10	20	30	40	50	60	70	80	90	100	
0	0	0	0	0	0	0	0	0	0	0	0	
500	0	0	0	0	0	0	0	0	0	0	0	
1000	0	0	0	0	0	0	0	0	0	0	0	
1500	0	0	0	0	0	1797	0	0	0	0	0	
2000	1729	0	0	0	0	0	0	0	0	0	0	
2500	0	0	0	0	0	0	0	0	0	0	0	
3000	0	1719	1696	0	0	0	0	0	0	0	0	
3500	0	0	1676	1725	0	0	0	0	0	0	0	
4000	0	0	0	0	0	0	0	0	0	0	0	
4500	0	0	0	0	1752	0	0	0	0	0	0	
5000	0	1448	0	0	0	1809	0	0	0	0	0	
5500	0	1696	1602	0	1828	1766	1816	0	0	0	0	
6000	0	0	0	1608	0	0	1827	0	0	0	0

	{0,	0,	0,	0,	1729,	0,	0,	0,	0,	0,	0,	0,	0},    // 0
	{0,	0,	0,	0,	0,	0,	1719,	0,	0,	0,	1448,	1696,	0},    // 1
	{0,	0,	0,	0,	0,	0,	1696,	1676,	0,	0,	0,	1602,	0},    // 2
	{0,	0,	0,	0,	0,	0,	0,	1725,	0,	0,	0,	0,	1608}, // 3
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	1752,	0,	1828,	0},    // 4
	{0,	0,	0,	1797,	0,	0,	0,	0,	0,	0,	1809,	1766,	0},    // 5
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1816,	1827}, // 6
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 7
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 8
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 9
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 10
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 11
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 12
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 13
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 14
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 15
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 16
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 17
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 18
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},    // 19
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0}     // 20
*/
unsigned fuelMap[2][MAP_TPS][MAP_SPD] = {{
  //     0      500     1000    1500    2000    2500    3000    3500    4000    4500    5000    5500    6000
	{1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729},  // 0
	{1719,	1719,	1719,	1719,	1719,	1719,	1719,	1651,	1584,	1516,	1448,	1696,	1696},  // 1
	{1696,	1696,	1696,	1696,	1696,	1696,	1696,	1676,	1656,	1639,	1621,	1602,	1584},  // 2
	{1725,	1725,	1725,	1725,	1725,	1725,	1864,	1844,	1820,	1678,	1654,	1631,	1608},  // 3
	{1752,	1752,	1752,	1752,	1872,	1752,	1872,	1810,	1752,	1818,	1790,	1828,	1850},  // 4
	{1809,	1809,	1809,	1809,	1809,	1809,	1809,	1809,	1869,	1809,	1809,	1766,	1776},  // 5
	{1816,	1816,	1816,	1816,	1816,	1816,	1816,	1852,	1816,	1816,	1816,	1816,	1827},  // 6
	{1816,	1816,	1816,	1816,	1816,	1816,	1816,	1816,	1883,	1816,	1816,	1816,	1816},  // 7
	{0,	0,	0,	0,	0,	0,	0,	0,	1834,	0,	0,	0,	0},     // 8
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},     // 9
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},     // 10
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},     // 11
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},     // 12
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},     // 13
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},     // 14
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},     // 15
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},     // 16
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},     // 17
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},     // 18
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},     // 19
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0}      // 20
},
{
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 0
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 1
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 2
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 3
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 4
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 5
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 6
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 7
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 8
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 9
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 10
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 11
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 12
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 13
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 14
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 15
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 16
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 17
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 18
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35},    // 19
	{25,	25,	25,	25,	35,	35,	37,	35,	35,	35,	18,	35,	35}     // 20
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
            Serial.print(tps*MAP_TPS_DIV);Serial.print("\t"); 
          }else{
           if(tps == -1){
             Serial.print(spd*MAP_SPD_DIV);
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

// Calculate the adjusted throttle position, used to loop up throttle position in map
int readThrottle(){
  //return  round((float)map(analogRead(THROTTLE_PIN),12,83,0,100)/MAP_TPS_DIV);
  return map(analogRead(THROTTLE_PIN),129,827,0,100); 
  //return analogRead(THROTTLE_PIN);
}
