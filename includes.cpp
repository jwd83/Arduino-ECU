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

	{0,	0,	0,	0,	1729,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	1719,	0,	0,	0,	1448,	1696,	0},
	{0,	0,	0,	0,	0,	0,	1696,	1676,	0,	0,	0,	1602,	0},
	{0,	0,	0,	0,	0,	0,	0,	1725,	0,	0,	0,	0,	1608},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	1752,	0,	1828,	0},
	{0,	0,	0,	1797,	0,	0,	0,	0,	0,	0,	1809,	1766,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	1816,	1827},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0}
*/
unsigned fuelMap[2][MAP_TPS][MAP_SPD] = {{
	{1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729,	1729},
	{1719,	1719,	1719,	1719,	1719,	1719,	1719,	1669,	1619,	1559,	1448,	1696,	1696},
	{1696,	1696,	1696,	1696,	1696,	1696,	1696,	1676,	1660,	1645,	1630,	1602,	1602},
	{1725,	1725,	1725,	1725,	1725,	1725,	1725,	1725,	1705,	1685,	1665,	1645,	1608},
	{1752,	1752,	1752,	1752,	1752,	1752,	1752,	1752,	1752,	1752,	1790,	1828,	1850},
	{1809,	1809,	1809,	1809,	1809,	1809,	1809,	1809,	1809,	1809,	1809,	1766,	1776},
	{1816,	1816,	1816,	1816,	1816,	1816,	1816,	1816,	1816,	1816,	1816,	1816,	1827},
	{1816,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0},
	{0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0,	0}
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
