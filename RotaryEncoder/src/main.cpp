#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

// I base on this
// https://haizairenmei.com/2019/12/05/rotary-encoder/
//
ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
std_msgs::Int32MultiArray position;
ros::Publisher pub_position("position", &position);

#define PIN_A PB6
#define PIN_B PB7
#define PIN_SW PB8

const int8_t ENCODER_TABLE[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
volatile bool StatePinA = 1;
volatile bool StatePinB = 1;
volatile int StatePinSW = 0;
volatile uint8_t State = 0;
volatile long Count = 0;
volatile long InterruptCount = 0;
volatile long InterruptSwitch = 0;
 
void ChangePinAB(){
    StatePinA = digitalRead(PIN_A);
    StatePinB = digitalRead(PIN_B);
    State = (State<<1) + StatePinA;
    State = (State<<1) + StatePinB;
    State = State & 0b00001111;
    Count += ENCODER_TABLE[State];
    InterruptCount++;
}

void ChangePinSW(){
    InterruptSwitch++;
    StatePinSW = !digitalRead(PIN_SW);
}

void setup() { 
	nh.getHardware()->setBaud(115200);
	nh.initNode(); //Default=57600bps
	nh.advertise(chatter);
	nh.advertise(pub_position);

    pinMode(PIN_A,INPUT_PULLUP);
    pinMode(PIN_B,INPUT_PULLUP);
    pinMode(PIN_SW,INPUT_PULLUP);
    attachInterrupt(PIN_A, ChangePinAB, CHANGE);
    attachInterrupt(PIN_B, ChangePinAB, CHANGE);
    attachInterrupt(PIN_SW, ChangePinSW, CHANGE);
    StatePinSW = !digitalRead(PIN_SW);

	// Size of array
	position.data_length = 2;
	// Create data
	position.data = (long int*)malloc(sizeof(long int) * position.data_length);
} 
 
void loop() { 
	char buffer[64];
	sprintf(buffer, "InterruptCount=%d InterruptSwitch=%d Count=%d StatePinSW=%d", InterruptCount, InterruptSwitch, Count, StatePinSW);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	
	position.data[0] = Count;
	position.data[1] = StatePinSW;
	pub_position.publish( &position );

	nh.spinOnce();
	delay(100);
}
