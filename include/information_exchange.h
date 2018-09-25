#ifndef _TIMERH
#define _TIMERH



void inf_exhange(void);
void send_heartbeat(void);
void send_servo_interval(void);
extern uint16_t mavlink_angle_1;
extern uint16_t mavlink_angle_2;
extern uint16_t mavlink_angle_3;
extern uint16_t mavlink_angle_4;

extern uint16_t mavlink_mode;

extern int16_t mavlink_raw_xacc; 
extern int16_t mavlink_raw_yacc;
extern int16_t mavlink_raw_zacc;

extern float mavlink_roll; 
extern float mavlink_roll_setpoint; 

#endif
