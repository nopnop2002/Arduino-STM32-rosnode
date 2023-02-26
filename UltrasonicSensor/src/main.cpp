#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
std_msgs::Float32 duration;
ros::Publisher pub_duration("duration", &duration);
std_msgs::Float32 distance;
ros::Publisher pub_distance("distance", &distance);


// STM32 has a terminal that can accept a 5V voltage signal. 
// The specific terminal is described as "FT" (Five voltage Tolerant) in the datasheet.
// Even if the power supply voltage of the microcomputer is 3.6V or less, 5V voltage can be applied to this pin.
int TRIG = PB6;
int ECHO = PB7; // Must be FT 

int temperature = 25;
double speed_of_sound = 331.5 + 0.6 * temperature; // Assuming a temperature of 25 degrees

void setup() {
	nh.getHardware()->setBaud(115200);
	nh.initNode(); //Default=57600bps
	nh.advertise(chatter);
	nh.advertise(pub_duration);
	nh.advertise(pub_distance);

	pinMode(ECHO, INPUT );
	pinMode(TRIG, OUTPUT );
}

void loop() {
	char buffer[64];
	char wk[10];

	digitalWrite(TRIG, LOW); 
	delayMicroseconds(2); 
	digitalWrite( TRIG, HIGH );
	delayMicroseconds( 10 ); 
	digitalWrite( TRIG, LOW );
	//double duration = pulseIn( ECHO, HIGH ); // The round trip microseconds time is returned
	duration.data = pulseIn( ECHO, HIGH ); // The round trip microseconds time is returned

	if (duration.data > 0) {
		duration.data = duration.data / 2; // Outbound trip time
		dtostrf(duration.data, 7, 2, wk);
		strcpy(buffer, "Duration:");
		strcat(buffer, wk);
		strcat(buffer, " microsec");
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		pub_duration.publish( &duration );

		distance.data = duration.data * speed_of_sound * 100 / 1000000;
		//Serial.print("Distance:");
		//Serial.print(distance.data);
		//Serial.println(" cm");
		dtostrf(distance.data, 7, 2, wk);
		strcpy(buffer, "Distance:");
		strcat(buffer, wk);
		strcat(buffer, " cm");
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		pub_distance.publish( &distance );
	}

	nh.spinOnce();
	delay(500);
}
