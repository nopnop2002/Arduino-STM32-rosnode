#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
std_msgs::Float32 temperature;
ros::Publisher pub_temperature("temperature", &temperature);
std_msgs::Float32 pressure;
ros::Publisher pub_pressure("pressure", &pressure);
std_msgs::Float32 altitude;
ros::Publisher pub_altitude("altitude", &altitude);

char buffer[64];

#define BMP_SCK  (PIN_SPI_SCK)
#define BMP_MISO (PIN_SPI_MISO)
#define BMP_MOSI (PIN_SPI_MOSI)
#define BMP_CS	 (PIN_SPI_SS)

//Adafruit_BMP280 bmp; // I2C
//Adafruit_BMP280 bmp(BMP_CS); // hardware SPI
Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

void setup() {
	nh.getHardware()->setBaud(115200);
	nh.initNode(); //Default=57600bps
	nh.advertise(chatter);
	nh.advertise(pub_temperature);
	nh.advertise(pub_pressure);
	nh.advertise(pub_altitude);
	unsigned status;
	//status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
	status = bmp.begin();
	if (!status) {
		//Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
		//		"try a different address!"));
		//Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
		//Serial.print("		  ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
		//Serial.print("	 ID of 0x56-0x58 represents a BMP 280,\n");
		//Serial.print("		  ID of 0x60 represents a BME 280.\n");
		//Serial.print("		  ID of 0x61 represents a BME 680.\n");
		strcpy(buffer, "Could not find a valid BMP280 sensor, check wiring.");
		Serial.println(buffer);
		while (1) delay(10);
	}

	/* Default settings from datasheet. */
	bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,		/* Operating Mode. */
		Adafruit_BMP280::SAMPLING_X2,		/* Temp. oversampling */
		Adafruit_BMP280::SAMPLING_X16,	/* Pressure oversampling */
		Adafruit_BMP280::FILTER_X16,		/* Filtering. */
		Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
	char wk[10];
	temperature.data = bmp.readTemperature();
	//Serial.print(F("Temperature = "));
	//Serial.print(temperature.data);
	//Serial.println(" *C");
	dtostrf(temperature.data, 5, 2, wk);
	sprintf(buffer, "Temperature = %s *C", wk);
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	pub_temperature.publish( &temperature );

	pressure.data = bmp.readPressure();
	//Serial.print(F("Pressure = "));
	//Serial.print(pressure.data);
	//Serial.println(" Pa");
	dtostrf(pressure.data, 5, 2, wk);
	sprintf(buffer, "Pressure = %s Pa", wk);
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	pub_pressure.publish( &pressure );

	altitude.data = bmp.readAltitude(1013.25); /* Adjusted to local forecast! */
	//Serial.print(F("Approx altitude = "));
	//Serial.print(altitude.data); /* Adjusted to local forecast! */
	//Serial.println(" m");
	dtostrf(altitude.data, 5, 2, wk);
	sprintf(buffer, "Approx altitude = %s m", wk);
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	pub_altitude.publish( &altitude );

	//Serial.println();
	nh.spinOnce();
	delay(500);
}

