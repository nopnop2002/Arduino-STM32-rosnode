#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <Adafruit_BMP085.h>


ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
std_msgs::Float32 temperature;
ros::Publisher pub_temperature("temperature", &temperature);
std_msgs::Int32 pressure;
ros::Publisher pub_pressure("pressure", &pressure);
std_msgs::Float32 altitude;
ros::Publisher pub_altitude("altitude", &altitude);
std_msgs::Int32 sealeve;
ros::Publisher pub_sealeve("sealeve", &sealeve);

char buffer[64];

/*************************************************** 
  This is an example for the BMP085 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/391

  These pressure and temperature sensors use I2C to communicate, 2 pins
  are required to interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

Adafruit_BMP085 bmp;
  
void setup() {
	nh.getHardware()->setBaud(115200);
	nh.initNode(); //Default=57600bps
	nh.advertise(chatter);
	nh.advertise(pub_temperature);
	nh.advertise(pub_pressure);
	nh.advertise(pub_altitude);
	nh.advertise(pub_sealeve);
	if (!bmp.begin()) {
		sprintf(buffer, "Could not find a valid BMP085 sensor, check wiring!");
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		nh.spinOnce();
		while (1) {}
	}
}
  
void loop() {
	char wk[10];
	//Serial.print("Temperature = ");
	//Serial.print(bmp.readTemperature());
	//Serial.println(" *C");
	temperature.data = bmp.readTemperature();
	//dtostrf(bmp.readTemperature(), 5, 2, wk);
	dtostrf(temperature.data, 5, 2, wk);
	sprintf(buffer, "Temperature = %s *C", wk);
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	pub_temperature.publish( &temperature );
	
	//Serial.print("Pressure = ");
	//Serial.print(bmp.readPressure());
	//Serial.println(" Pa");
	pressure.data = bmp.readPressure();
	sprintf(buffer, "Pressure = %d Pa", pressure.data);
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	pub_pressure.publish( &pressure );
	
	// Calculate altitude assuming 'standard' barometric
	// pressure of 1013.25 millibar = 101325 Pascal
	//Serial.print("Altitude = ");
	//Serial.print(bmp.readAltitude());
	//Serial.println(" meters");
	altitude.data = bmp.readAltitude();
	//dtostrf(bmp.readAltitude(), 6, 2, wk);
	dtostrf(altitude.data, 6, 2, wk);
	sprintf(buffer, "Altitude = %s meters", wk);
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	pub_altitude.publish( &altitude );

	//Serial.print("Pressure at sealevel (calculated) = ");
	//Serial.print(bmp.readSealevelPressure());
	//Serial.println(" Pa");
	sealeve.data = bmp.readSealevelPressure();
	//sprintf(buffer, "Pressure at sealevel (calculated) = %d Pa", bmp.readSealevelPressure());
	sprintf(buffer, "Pressure at sealevel (calculated) = %d Pa", sealeve.data);
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	pub_sealeve.publish( &sealeve );

	// you can get a more precise measurement of altitude
	// if you know the current sea level pressure which will
	// vary with weather and such. If it is 1015 millibars
	// that is equal to 101500 Pascals.
	//Serial.print("Real altitude = ");
	//Serial.print(bmp.readAltitude(101500));
	//Serial.println(" meters");
	
	//Serial.println();
	nh.spinOnce();
	
	delay(500);
}

