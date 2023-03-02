#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include "Wire.h"
#include "MPU9250.h"

ros::NodeHandle  nh;
std_msgs::String str_msg;
sensor_msgs::Imu imu;
sensor_msgs::MagneticField mag;

ros::Publisher chatter("chatter", &str_msg);
ros::Publisher pubimu("imu/data_raw", &imu);
ros::Publisher pubmag("imu/mag", &mag);

// MPU9250 Configuration
// Specify sensor full scale
/* Choices are:
 *	Gscale: GFS_250 == 250 dps, GFS_500 DPS == 500 dps, GFS_1000 == 1000 dps, and GFS_2000DPS == 2000 degrees per second gyro full scale
 *	Ascale: AFS_2G == 2 g, AFS_4G == 4 g, AFS_8G == 8 g, and AFS_16G == 16 g accelerometer full scale
 *	Mscale: MFS_14BITS == 0.6 mG per LSB and MFS_16BITS == 0.15 mG per LSB
 *	Mmode: Mmode == M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
 *	(1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so
 *	sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
 */
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
uint8_t Mscale = MFS_16BITS;
uint8_t Mmode = M_100Hz;
uint8_t sampleRate = 0x04;

float aRes, gRes, mRes; // scale resolutions per LSB for the sensors
int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
int16_t magCount[3]; // Stores the 16-bit signed magnetometer sensor output
float magCalibration[3] = {0, 0, 0}; // Factory mag calibration and mag bias
float SelfTest[6]; // holds results of gyro and accelerometer self test

// Bias corrections for gyro and accelerometer
float magBias[3] = {71.04, 122.43, -36.90};
float magScale[3]  = {1.01, 1.03, 0.96};

// These can be measured once and entered here or can be calculated each time the device is powered on
float gyroBias[3];
float accelBias[3];

MPU9250 MPU9250(0); // instantiate MPU9250 class

unsigned long nextMillis;
char buffer[64];
char wk[10];

void setupMPU() {
	Wire.begin(); // set master mode, default on SDA/SCL for Ladybug
	Wire.setClock(400000); // I2C frequency at 400 kHz

	//MPU9250.I2Cscan(); // should detect MPU9250 at 0x68

	/* Configure the MPU9250 */
	// Read the WHO_AM_I register, this is a good test of communication
	strcpy(buffer, "MPU9250 9-axis motion sensor...");
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	//nh.spinOnce();

	uint8_t c = MPU9250.getMPU9250ID();
	sprintf(buffer, "MPU9250 I AM 0x%x I should be 0x71", c);
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );

	if (c == 0x71 ) // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255 
	{  
		strcpy(buffer, "MPU9250 is online...");
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
	
		MPU9250.resetMPU9250(); // start by resetting MPU9250
	
		MPU9250.SelfTest(SelfTest); // Start by performing self test and reporting values
		dtostrf(SelfTest[0], 4, 1, wk);
		sprintf(buffer, "x-axis self test: acceleration trim within : %s %% of factory value", wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		dtostrf(SelfTest[1], 4, 1, wk);
		sprintf(buffer, "y-axis self test: acceleration trim within : %s %% of factory value", wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		dtostrf(SelfTest[2], 4, 1, wk);
		sprintf(buffer, "z-axis self test: acceleration trim within : %s %% of factory value", wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		dtostrf(SelfTest[3], 4, 1, wk);
		sprintf(buffer, "x-axis self test: gyration trim within : %s %% of factory value", wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		str_msg.data = buffer;
		dtostrf(SelfTest[4], 4, 1, wk);
		sprintf(buffer, "y-axis self test: gyration trim within : %s %% of factory value", wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		str_msg.data = buffer;
		dtostrf(SelfTest[5], 4, 1, wk);
		sprintf(buffer, "z-axis self test: gyration trim within : %s %% of factory value", wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		// get sensor resolutions, only need to do this once
		aRes = MPU9250.getAres(Ascale);
		gRes = MPU9250.getGres(Gscale);
		mRes = MPU9250.getMres(Mscale);

		// Comment out if using pre-measured, pre-stored offset biases
		MPU9250.calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

		strcpy(buffer, "accel biases (mg): ");
		dtostrf(1000.*accelBias[0], 5, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(1000.*accelBias[1], 5, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(1000.*accelBias[2], 5, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		strcpy(buffer, "gyro biases (dps): ");
		dtostrf(gyroBias[0], 5, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(gyroBias[1], 5, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(gyroBias[2], 5, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
  
		MPU9250.initMPU9250(Ascale, Gscale, sampleRate); 
		strcpy(buffer, "MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
  

  
	} else {
		sprintf(buffer, "Could not connect to MPU9250: 0x%x", c);
		//Serial.println(buffer);
		str_msg.data = buffer;
		while(1) {
			chatter.publish( &str_msg );
			nh.spinOnce();
			delay(1000);
		}
	}

	strcpy(buffer, "MPU9250 Initialization done.");
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	nh.spinOnce();
}


void setupMAG() {
	// Read the WHO_AM_I register of the magnetometer, this is a good test of communication
	byte d = MPU9250.getAK8963CID();
	sprintf(buffer, "AK8963 I AM 0x%x I should be 0x48", d);
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );

	if (d == 0x48) {
		strcpy(buffer, "AK8963 is online...");
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		// Get magnetometer calibration from AK8963 ROM
		MPU9250.initAK8963Slave(Mscale, Mmode, magCalibration);
		strcpy(buffer, "AK8963 initialized for active data mode....");
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		// Comment out if using pre-measured, pre-stored offset biases
		//MPU9250.magcalMPU9250(magBias, magScale);

		strcpy(buffer, "AK8963 mag biases (mG): ");
		dtostrf(magBias[0], 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(magBias[1], 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(magBias[2], 6, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		strcpy(buffer, "AK8963 mag scale (mG):	");
		dtostrf(magScale[0], 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(magScale[1], 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(magScale[2], 6, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		delay(2000); // add delay to see results before serial spew of data

		strcpy(buffer, "AK8963 Calibration values: ");
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		strcpy(buffer, "X-Axis sensitivity adjustment value: ");
		dtostrf(magCalibration[0], 5, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		str_msg.data = buffer;
		strcpy(buffer, "Y-Axis sensitivity adjustment value: ");
		dtostrf(magCalibration[1], 5, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		str_msg.data = buffer;
		strcpy(buffer, "Z-Axis sensitivity adjustment value: ");
		dtostrf(magCalibration[2], 5, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
	} else {
		sprintf(buffer, "Could not connect to AK8963: 0x%x", d);
		//Serial.println(buffer);
		str_msg.data = buffer;
		while(1) {
			chatter.publish( &str_msg );
			nh.spinOnce();
			delay(1000);
		};
	}

	strcpy(buffer, "AK8963 Initialization done.");
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	nh.spinOnce();
}

void setup() {
	nh.getHardware()->setBaud(115200);
	nh.initNode(); //Default=57600bps
	nh.advertise(chatter);
	nh.advertise(pubimu);
	nh.advertise(pubmag);

	for (int i=0;i<100;i++) {
		sprintf(buffer, "Start until %d millSec", 100-i);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		nh.spinOnce();
		delay(100);
	}

	setupMPU();
	setupMAG();

	strcpy(buffer, "Initialization all done.");
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	nh.spinOnce();
	nextMillis = millis() + 500;
}

void loop() {
	// A variables to holds latest sensor data values 
	float ax, ay, az, gx, gy, gz, mx, my, mz;
	// A variable that holds cumulative sensor data values
	static float ax_sum = 0.0;
	static float ay_sum = 0.0;
	static float az_sum = 0.0;
	static int accelSteps = 0;
	static float gx_sum = 0.0;
	static float gy_sum = 0.0;
	static float gz_sum = 0.0;
	static int gyroSteps = 0;
	static float mx_sum = 0.0;
	static float my_sum = 0.0;
	static float mz_sum = 0.0;
	static int magSteps = 0;

	  
	MPU9250.readMPU9250Data(MPU9250Data); // INT cleared on any read
	 
	// Now we'll calculate the accleration value into actual g's
	ax = (float)MPU9250Data[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
	ay = (float)MPU9250Data[1]*aRes - accelBias[1];   
	az = (float)MPU9250Data[2]*aRes - accelBias[2];  
	ax_sum += ax;
	ay_sum += ay;
	az_sum += az;
	accelSteps++;

	// Calculate the gyro value into actual degrees per second
	gx = (float)MPU9250Data[4]*gRes;  // get actual gyro value, this depends on scale being set
	gy = (float)MPU9250Data[5]*gRes;  
	gz = (float)MPU9250Data[6]*gRes; 
	gx_sum += gx;
	gy_sum += gy;
	gz_sum += gz;
	gyroSteps++;
  
	MPU9250.readMagData(magCount);	// Read the x/y/z adc values
  
	// Calculate the magnetometer values in milliGauss
	// Include factory calibration per data sheet and user environmental corrections
	// get actual magnetometer value, this depends on scale being set
	mx = (float)magCount[0]*mRes*magCalibration[0] - magBias[0];
	my = (float)magCount[1]*mRes*magCalibration[1] - magBias[1];  
	mz = (float)magCount[2]*mRes*magCalibration[2] - magBias[2];  
	mx *= magScale[0];
	my *= magScale[1];
	mz *= magScale[2]; 
	mx_sum += mx;
	my_sum += my;
	mz_sum += mz;
	magSteps++;

	if (millis() > nextMillis) {
		sprintf(buffer, "accelSteps=%d gyroSteps=%d magSteps=%d", accelSteps, gyroSteps, magSteps);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		// The outputs of the accelerometer are in g
		strcpy(buffer, "ax=");
		dtostrf(ax_sum/accelSteps, 5, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ay=");
		dtostrf(ay_sum/accelSteps, 5, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " az=");
		dtostrf(az_sum/accelSteps, 5, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		// The outputs of the gyroscope are in degrees per second
		// Now convert to radian per second
		float gx_radian =(gx_sum/gyroSteps) * 0.0174533;
		float gy_radian =(gy_sum/gyroSteps) * 0.0174533;
		float gz_radian =(gz_sum/gyroSteps) * 0.0174533;
		strcpy(buffer, "gx=");
		dtostrf(gx_radian, 5, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " gy=");
		dtostrf(gy_radian, 5, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " gz=");
		dtostrf(gz_radian, 5, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		// The outputs of the compass are in micro tesla
		strcpy(buffer, "mx=");
		dtostrf(mx_sum/magSteps, 7, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " my=");
		dtostrf(my_sum/magSteps, 7, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " mz=");
		dtostrf(mz_sum/magSteps, 7, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );

		imu.header.frame_id = "imu";
		imu.header.stamp = nh.now();
		imu.angular_velocity.x = gx_radian;
		imu.angular_velocity.y = gy_radian;
		imu.angular_velocity.z = gz_radian;
		imu.linear_acceleration.x = ax_sum/accelSteps;
		imu.linear_acceleration.y = ay_sum/accelSteps;
		imu.linear_acceleration.z = az_sum/accelSteps;
		pubimu.publish(&imu);

		/// The output of AK8963 is mucro Tesla
		mag.header.frame_id = "imu";
		mag.header.stamp = nh.now();
		mag.magnetic_field.x = mx_sum/magSteps; // [uTesla]
		mag.magnetic_field.y = my_sum/magSteps; // [uTesla]
		mag.magnetic_field.z = mz_sum/magSteps; // [uTesla]
		pubmag.publish(&mag);

		nextMillis = millis() + 500;
		ax_sum = 0.0;
		ay_sum = 0.0;
		az_sum = 0.0;
		accelSteps = 0;
		gx_sum = 0.0;
		gy_sum = 0.0;
		gz_sum = 0.0;
		gyroSteps = 0;
		mx_sum = 0.0;
		my_sum = 0.0;
		mz_sum = 0.0;
		magSteps = 0;

		nh.spinOnce();
	}
}
