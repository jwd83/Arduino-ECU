#ifdef __cplusplus
extern "C" {
#endif
 
void initTimer(unsigned);
void Timer0IntHandler(void);
void setTimer(unsigned);

extern int crank_angle;
#ifdef __cplusplus
}
#endif
