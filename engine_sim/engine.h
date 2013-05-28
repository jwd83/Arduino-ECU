#ifndef Engine_h
#define Engine_h
#ifdef __cplusplus
#include "Energia.h"

class Engine{
 public:
   float TL;			// Load Torque (Nm)
   float s;			// Engine speed in RPM
   float T;			// Torque (Nm)
   float F;			// Fuel pulse width
   float AFR;                   // Calculated AFR
   float equiv;                 // Equivalence ratio
   float throttle;              // Throttle position 0-100%
   Engine();
   int test(int in);
   void simulate(float);
 private:
   float w;			// Angular speed in rads/s
   
   float TN;			// Net Torque (Nm)
   float TF;			// Friction Torque (Nm)
   float wdot;			// Angular acceleration (rads/s-2)
   float J;            // Moment of Inertia
   float k1;		// Combustion process/efficiency
   float kf0;	        // static coefficient of friction
   float kf1;		// coefficient of friction
   float kf2;		// Squared coefficient of friction
};

#endif
#endif
