/*
 * timer_blink - C API
 */
#include "Energia.h"
#include "inc/hw_ints.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "timer_blink.h"
 
void Timer0IntHandler()
{
  // Clear the timer interrupt
  TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  scott();
}
 
void initTimer(unsigned Hz)
{
  //GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  TimerConfigure(TIMER0_BASE, TIMER_CFG_32_BIT_PER);
  unsigned long ulPeriod = (SysCtlClockGet() / Hz) / 2;
  TimerLoadSet(TIMER0_BASE, TIMER_A, ulPeriod -1);
  
  TimerIntRegister(TIMER0_BASE, TIMER_A,Timer0IntHandler);
  TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  TimerEnable(TIMER0_BASE, TIMER_A);
  IntEnable(INT_TIMER0A);
}
 
void setTimer(unsigned Hz){
  unsigned long ulPeriod = (SysCtlClockGet() / Hz) / 2;
  TimerLoadSet(TIMER0_BASE, TIMER_A, ulPeriod -1);
}
