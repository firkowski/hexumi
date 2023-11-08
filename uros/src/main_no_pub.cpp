#include <stdio.h>
#include "pico/stdlib.h"

#include "rclc/rclc.h"

#include <time.h>
#include <vector>
#include <memory>
#include <map>
#include <cmath>

#include "servo2040_uros/msg/feet_contacts.h"
#include "servo2040_uros/msg/current.h"
#include "servo2040_uros/msg/joint_angles.h"
#include "sensor_msgs/msg/joint_state.h"

extern "C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "pico_uart_transports.h"
}

#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"
#include "button.hpp"

#define NUM_LEGS 6
#define NUM_SERVOS_PER_LEG 3
#define NUM_SERVOS 18

#define SERVO_L1C 0 //1
#define SERVO_L1F 1 //2
#define SERVO_L1T 2 //3
#define SERVO_L2C 3 //4
#define SERVO_L2F 4 //5
#define SERVO_L2T 5 //6
#define SERVO_L3C 6 //7
#define SERVO_L3F 7 //8
#define SERVO_L3T 8 //9
#define SERVO_R1C 9 //16
#define SERVO_R1F 10 //17
#define SERVO_R1T 11 //18
#define SERVO_R2C 12 //13 
#define SERVO_R2F 13 //14
#define SERVO_R2T 14 //15
#define SERVO_R3C 15 //10
#define SERVO_R3F 16 //11
#define SERVO_R3T 17                                                                                                                                                                                                                     //12

#define WIDE_ANGLE_RANGE 180

#define SENSOR_L1 0
#define SENSOR_L2 1
#define SENSOR_L3 2
#define SENSOR_R1 3
#define SENSOR_R2 4
#define SENSOR_R3 5

#define SENSOR_VOLTAGE_THRESHOLD 2.0

using namespace servo;

std::vector<std::vector<int>> pulses = {
        {550, 1550, 2550}, // L1C, 1
        {460, 1440, 2460}, // L1F, 2
        {580, 1580, 2580}, // L1T, 3
        {580, 1580, 2580}, // L2C, 4
        {585, 1585, 2585}, // L2F, 5
        {455, 1455, 2455}, // L2T, 6
        {500, 1500, 2500}, // L3C, 7
        {475, 1475, 2475}, // L3F, 8
        {500, 1500, 2500}, // L3T, 9
        {580, 1580, 2580}, // R1C, 10
        {500, 1500, 2500}, // R1F, 11
        {580, 1580, 2580}, // R1T, 12
        {550, 1550, 2550}, // R2C, 13
        {470, 1470, 2470}, // R2F, 14
        {540, 1540, 2540}, // R2T, 15
        {520, 1520, 2520}, // R3C, 16
        {650, 1650, 2650}, // R3F, 17
        {610, 1650, 2610}  // R3T, 18
    };


std::map<int, int> servo_IDs {
	{0, SERVO_L1C},
	{1, SERVO_L1F},
	{2, SERVO_L1T},
	{3, SERVO_L2C},
	{4, SERVO_L2F},
	{5, SERVO_L2T},
	{6, SERVO_L3C},
	{7, SERVO_L3F},
	{8, SERVO_L3T},
	{9, SERVO_R3C},
	{10, SERVO_R3F},
	{11, SERVO_R3T},
	{12, SERVO_R2C},
	{13, SERVO_R2F},
	{14, SERVO_R2T},
	{15, SERVO_R1C},
	{16, SERVO_R1F},
	{17, SERVO_R1T},
};

std::map<int, int> sensor_IDs {
	{0,  SENSOR_L1},
	{1,  SENSOR_L2},
	{2,  SENSOR_L3},
	{3,  SENSOR_R1},
	{4,  SENSOR_R2},
	{5,  SENSOR_R3}
};

rcl_publisher_t    feet_contacts_pub_;
rcl_publisher_t    current_pub_;
rcl_subscription_t joint_angles_sub_;
servo2040_uros__msg__FeetContacts   feet_contacts_msg;
servo2040_uros__msg__Current current_msg;
servo2040_uros__msg__JointAngles    joint_angles_msg;

rcl_publisher_t	joint_state_pub_;
sensor_msgs__msg__JointState		 joint_state_msg;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

const uint START_PIN = 0;
const uint END_PIN = NUM_SERVOS;
ServoCluster servos = ServoCluster(pio0, 0, START_PIN, END_PIN);
std::array<std::unique_ptr<Calibration>, NUM_SERVOS> cals;

Analog sen_adc = Analog(servo2040::SHARED_ADC);
Analog vol_adc = Analog(servo2040::SHARED_ADC, servo2040::VOLTAGE_GAIN);
Analog cur_adc = Analog(servo2040::SHARED_ADC, servo2040::CURRENT_GAIN,
                        servo2040::SHUNT_RESISTOR, servo2040::CURRENT_OFFSET);

AnalogMux mux = AnalogMux(servo2040::ADC_ADDR_0, servo2040::ADC_ADDR_1, servo2040::ADC_ADDR_2,
                          PIN_UNUSED, servo2040::SHARED_ADC);

float angle_map(int i, float deg) {
	float angle = deg;
	int side = (i < (NUM_SERVOS / 2)) ? 1 : -1;
	// 0 for C, 1 for F, 2 for T
	int joint = i % 3;

	switch (joint) {
		case 0:
			break;
		case 1:
			angle *= side;
			break;
		case 2:
			angle = -side*(angle + 90.0);
			break;
	}
	return angle;
}

void init() {

	servos.init();

	for (int i = 0; i < NUM_SERVOS; i++) {
		Calibration &cal_ref = servos.calibration(i);
		cal_ref.apply_three_pairs(pulses[i][0], pulses[i][1], pulses[i][2],
                                        -90.0,            0,         90.0);
	}

	servos.enable_all();

	for(auto i = 0u; i < servo2040::NUM_SENSORS; i++) {
   		mux.configure_pulls(servo2040::SENSOR_1_ADDR + i, false, true);
  	}

	joint_state_msg.position.capacity = NUM_SERVOS;
	joint_state_msg.position.data = (double*) malloc(joint_state_msg.position.capacity * sizeof(double));
	joint_state_msg.position.size = 0;

	for (int i = 0; i < NUM_SERVOS; i++) {
		joint_state_msg.position.data[i] = 0.0;
		joint_state_msg.position.size++;
	}

	rcl_ret_t feet_contacts_ret = rcl_publish(&joint_state_pub_, &joint_state_msg, NULL);

}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{	
	
}

void subscription_callback(const void * msgin)
{
	const servo2040_uros__msg__JointAngles * msg = (const servo2040_uros__msg__JointAngles *)msgin;

	if (msg == NULL) 
		return;

    for (size_t i = 0; i < NUM_SERVOS; i++) {
		int j = servo_IDs[i];
		float deg = static_cast<float>(msg->joint_angles[i]) * 180.0 / M_PI;
		//joint_state_msg.position.data[i] = deg;

		float angle = angle_map(j, deg);
		servos.value(j, angle);
    }
	//rcl_ret_t feet_contacts_ret = rcl_publish(&joint_state_pub_, &joint_state_msg, NULL);
}

bool pingAgent(){
    const int timeout_ms = 1000;
    const uint8_t attempts = 10;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK){
    	return false;
    } 
    return true;
}

void createEntities() {

	allocator = rcl_get_default_allocator();
	rclc_support_init(&support, 0, NULL, &allocator);

	rclc_node_init_default(&node, "servo2040_node", "", &support);

	rclc_subscription_init_default(
		&joint_angles_sub_,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(servo2040_uros, msg, JointAngles),
		"/joint_angles");

	rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(100),
		timer_callback);

	// create publishers
	/*
	rclc_publisher_init_default(
		&feet_contacts_pub_,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(servo2040_uros, msg, FeetContacts),
		"/servo2040/feet_contacts");

	rclc_publisher_init_default(
		&joint_state_pub_,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		"/servo2040/joint_states");
    
    rclc_publisher_init_default(
		&current_pub_,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(servo2040_uros, msg, Current),
		"/servo2040/current");
	*/
	rclc_executor_init(&executor, &support.context, 2, &allocator);
	rclc_executor_add_timer(&executor, &timer);

	rclc_executor_add_subscription(&executor,
			&joint_angles_sub_,
			&joint_angles_msg,
			&subscription_callback,
			ON_NEW_DATA);
}

void destroyEntities(){
	rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
	(void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

	//rcl_publisher_fini(&feet_contacts_pub_, &node);
	//rcl_publisher_fini(&current_pub_, &node);
	//rcl_publisher_fini(&joint_state_pub_, &node);
	rcl_subscription_fini(&joint_angles_sub_, &node);
	rcl_timer_fini(&timer);
	rclc_executor_fini(&executor);
	rcl_node_fini(&node);
	rclc_support_fini(&support);
}


int main()
{	
	stdio_init_all();

	init();

    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);
	
  	state = WAITING_AGENT;

	while (true){
    	switch (state) {
    	    case WAITING_AGENT:
    	      state = pingAgent() ? AGENT_AVAILABLE : WAITING_AGENT;
    	      break;
    	    case AGENT_AVAILABLE:
    	      createEntities();
    	      state = AGENT_CONNECTED ;
    	      break;
    	    case AGENT_CONNECTED:
    	      state = pingAgent() ? AGENT_CONNECTED : AGENT_DISCONNECTED;
    	      if (state == AGENT_CONNECTED) {
    	        rclc_executor_spin(&executor);
    	      }
    	      break;
    	    case AGENT_DISCONNECTED:
    	      destroyEntities();
    	      state = WAITING_AGENT;
    	      break;
    	    default:
    	      break;
    	  }
    }
}
