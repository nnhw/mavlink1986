

#ifndef _DRVH
#define _DRVH


#define YES 1u
#define NO 0u

#define CLOCKWISE 1u
#define COUNTERCLOCKWISE 0u

#define START 1u
#define STOP 0u

#define FILTER_COEFF 0.3

#define AILERON 1
#define ELEVATOR 2
#define RUDDER 4

//#define	PROPORTIONAL_COEFFICIENT 160

//#define	INTEGRAL_COEFFICIENT 0

//#define	DIFFERENTIAL_COEFFICIENT 0

#define	PID_TIME_INTERVAL 1


#define	CONVERSION_COEFFICIENT 1

#define	 INTERGRAL_ARRAY_LENGHT 19

#define PI 3.1416 //как грубо

#define DEG_IN_RAD 57.29578

#define MANUAL 0

#define PROGRAM 1

#define STABILIZE 2

#define TOLERABLE_ERROR 0.05


void manual_mode_control(void);
void proportional_control(void);
void stabilization(void);

void PID_control(void);

void system_default_state(void);

#endif
