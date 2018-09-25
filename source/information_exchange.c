#include <MDR32Fx.h> 
#include <MDR32F9Qx_adc.h>
#include "information_exchange.h"
#include <MDR32F9Qx_uart.h>
#include "mavlink.h"


#include "hardware.h"
#include "drive.h"




static int packet_drops = 0;

uint8_t global_data;


uint16_t mavlink_angle_1 = 0; //roughly from 700 to 2000
uint16_t mavlink_angle_2 = 0;
uint16_t mavlink_angle_3 = 0;
uint16_t mavlink_angle_4 = 0;

int16_t mavlink_xacc = 0; // N/A
int16_t mavlink_yacc = 0;	// N/A
int16_t mavlink_zacc = 0;	// N/A

int16_t mavlink_xacc2 = 0; //roughly from -1000 to 1000 at 1g
int16_t mavlink_yacc2 = 0;
int16_t mavlink_zacc2 = 0;

int16_t mavlink_xacc3 = 0; // N/A
int16_t mavlink_yacc3 = 0;	// N/A
int16_t mavlink_zacc3 = 0;	// N/A

int16_t mavlink_raw_xacc = 0; //roughly from -1000 to 1000 at 1g
int16_t mavlink_raw_yacc = 0;
int16_t mavlink_raw_zacc = 0;

int16_t mavlink_raw_xgyro = 0;
int16_t mavlink_raw_ygyro = 0;
int16_t mavlink_raw_zgyro = 0;


float mavlink_abs_press = 0;
float mavlink_diff_press = 0;
int16_t mavlink_temp = 0; //0.01 degrees centigrade

float mavlink_hires_xacc = 0; // N/A
float mavlink_hires_yacc = 0;	// N/A
float mavlink_hires_zacc = 0;	// N/A

float mavlink_roll = 0; // Roll angle (rad, -pi..+pi)
float mavlink_pitch = 0;
float mavlink_yaw = 0;

float mavlink_roll_setpoint = 0; // Roll angle in degrees
float mavlink_pitch_setpoint = 0;
float mavlink_yaw_setpoint = 0;

uint16_t mavlink_mode = MANUAL;

void UART1_IRQHandler(void){
 
}


void send_servo_interval(void)
{

uint16_t iterator = 0;
	

// Initialize the required buffers
mavlink_message_t msg_t;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
// Pack the message

mavlink_msg_message_interval_pack(20,  MAV_COMP_ID_IMU, &msg_t, MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 1000);

// Copy the message to the send buffer
uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_t);
 

while (iterator < len)
{
	while (UART_GetFlagStatus(MDR_UART1,UART_FLAG_TXFF));
	UART_SendData (MDR_UART1,buf[iterator]);
	iterator++;
}


	
}


void send_heartbeat(void)
{
	
uint16_t iterator = 0;
	
mavlink_system_t mavlink_system;
 
mavlink_system.sysid = 20;                   ///< ID 20 for this airplane
mavlink_system.compid = MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process
 
// Define the system type, in this case an airplane
uint8_t system_type = MAV_TYPE_FIXED_WING;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
 
uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight
 
// Initialize the required buffers
mavlink_message_t msg_t;
uint8_t buf[MAVLINK_MAX_PACKET_LEN];
 
// Pack the message
mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg_t, system_type, autopilot_type, system_mode, custom_mode, system_state);
 
// Copy the message to the send buffer
uint16_t len = mavlink_msg_to_send_buffer(buf, &msg_t);
 
// Send the message with the standard UART send function
// uart0_send might be named differently depending on
// the individual microcontroller / library in use.

//UART_SendData(MDR_UART1,len);


while (iterator < len)
{
	while (UART_GetFlagStatus(MDR_UART1,UART_FLAG_TXFF));
	UART_SendData (MDR_UART1,buf[iterator]);
	iterator++;
}


}



void inf_exhange(void)
{
	mavlink_message_t msg;
	mavlink_status_t status;
 
	// COMMUNICATION THROUGH EXTERNAL UART PORT (XBee serial)
 
	while(!(UART_GetFlagStatus(MDR_UART1,UART_FLAG_RXFE)))
	{
		global_data = UART_ReceiveData(MDR_UART1);
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, global_data, &msg, &status)) {
			// Handle message
 
			switch(msg.msgid)
			{
			        case MAVLINK_MSG_ID_HEARTBEAT:
			        {
								xled1;
								send_heartbeat();

			        }
			        break;
					case MAVLINK_MSG_ID_SERVO_OUTPUT_RAW:
					{
						mavlink_angle_1 = mavlink_msg_servo_output_raw_get_servo1_raw(&msg);
						mavlink_angle_2 = mavlink_msg_servo_output_raw_get_servo2_raw(&msg);
						mavlink_angle_3 = mavlink_msg_servo_output_raw_get_servo3_raw(&msg);
						mavlink_angle_4 = mavlink_msg_servo_output_raw_get_servo4_raw(&msg);
					}
					break;
					// case MAVLINK_MSG_ID_SCALED_IMU:
					// {
					// 	mavlink_xacc = mavlink_msg_scaled_imu_get_xacc(&msg);
					// 	mavlink_yacc = mavlink_msg_scaled_imu_get_yacc(&msg);
					// 	mavlink_zacc = mavlink_msg_scaled_imu_get_zacc(&msg);
					// }
					// break;
					// case MAVLINK_MSG_ID_SCALED_IMU2:
					// {
					// 	mavlink_xacc2 = mavlink_msg_scaled_imu2_get_xacc(&msg);
					// 	mavlink_yacc2 = mavlink_msg_scaled_imu2_get_yacc(&msg);
					// 	mavlink_zacc2 = mavlink_msg_scaled_imu2_get_zacc(&msg);
					// }
					// break;
					// case MAVLINK_MSG_ID_SCALED_IMU3:
					// {
					// 	mavlink_xacc3 = mavlink_msg_scaled_imu3_get_xacc(&msg);
					// 	mavlink_yacc3 = mavlink_msg_scaled_imu3_get_yacc(&msg);
					// 	mavlink_zacc3 = mavlink_msg_scaled_imu3_get_zacc(&msg);
					// }
					// break;
					case MAVLINK_MSG_ID_RAW_IMU: 
					{
						mavlink_raw_xacc = mavlink_msg_raw_imu_get_xacc(&msg);
						mavlink_raw_yacc = mavlink_msg_raw_imu_get_yacc(&msg);
						mavlink_raw_zacc = mavlink_msg_raw_imu_get_zacc(&msg);

						mavlink_raw_xgyro = mavlink_msg_raw_imu_get_xgyro(&msg);
						mavlink_raw_ygyro = mavlink_msg_raw_imu_get_ygyro(&msg);
						mavlink_raw_zgyro = mavlink_msg_raw_imu_get_zgyro(&msg);
						
					}
					break;
					case MAVLINK_MSG_ID_SCALED_PRESSURE: 
					{
						mavlink_abs_press = mavlink_msg_scaled_pressure_get_press_abs(&msg);
						mavlink_diff_press = mavlink_msg_scaled_pressure_get_press_diff(&msg);
						mavlink_temp = mavlink_msg_scaled_pressure_get_temperature(&msg);
					}
					break;
					// case MAVLINK_MSG_ID_HIGHRES_IMU: 
					// {
					// 	mavlink_hires_xacc = mavlink_msg_highres_imu_get_xacc(&msg);
					// 	mavlink_hires_yacc = mavlink_msg_highres_imu_get_yacc(&msg);
					// 	mavlink_hires_zacc = mavlink_msg_highres_imu_get_zacc(&msg);
					// }
					// break;
					case MAVLINK_MSG_ID_ATTITUDE: 
					{
						mavlink_roll = mavlink_msg_attitude_get_roll(&msg);
						mavlink_pitch = mavlink_msg_attitude_get_pitch(&msg);
						mavlink_yaw = mavlink_msg_attitude_get_yaw(&msg);
					}
					break;
					case MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT: 
					{
						mavlink_roll_setpoint = mavlink_msg_nav_controller_output_get_nav_roll(&msg);
						mavlink_pitch_setpoint = mavlink_msg_nav_controller_output_get_nav_pitch(&msg);
						mavlink_yaw_setpoint = mavlink_msg_nav_controller_output_get_nav_bearing(&msg);
					}
					break;
				default:
				//Do nothing
				break;
			}
		}
	}
	
	// Update global packet drops counter
	packet_drops += status.packet_rx_drop_count;

}

