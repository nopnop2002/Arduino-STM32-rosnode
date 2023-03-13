#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <MPU6050.h>
//#include <MPU6050_6Axis_MotionApps20.h>

ros::NodeHandle  nh;
std_msgs::String str_msg;
sensor_msgs::Imu imu;

ros::Publisher chatter("chatter", &str_msg);
ros::Publisher pubimu("imu/data_raw", &imu);

MPU6050 mpu;

// Accel & Gyro scale factor
float accel_sensitivity;
float gyro_sensitivity;

// sampling interval
#define INTERVAL 500
unsigned long nextMillis;

char buffer[64];
char wk[10];

void setupMPU() {
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	mpu.initialize();

	// Set Digital Low Pass Filter
	mpu.setDLPFMode(6);

	// Get LSB Sensitivity
	//Serial.print("getFullScaleAccelRange()=");
	uint8_t _AccelRange = mpu.getFullScaleAccelRange();
	//Serial.println(_AccelRange);
	if (_AccelRange == 0) {
		accel_sensitivity = 16384.0;
	} else if (_AccelRange == 1) {
		accel_sensitivity = 8192.0;
	} else if (_AccelRange == 2) {
		accel_sensitivity = 4096.0;
	} else if (_AccelRange == 3) {
		accel_sensitivity = 2048.0;
	}

	//Serial.print("getFullScaleGyroRange()=");
	uint8_t _GyroRange = mpu.getFullScaleGyroRange();
	//Serial.println(_GyroRange);
	if (_GyroRange == 0) {
		gyro_sensitivity = 131.0;
	} else if (_GyroRange == 1) {
		gyro_sensitivity = 65.5;
	} else if (_GyroRange == 2) {
		gyro_sensitivity = 32.8;
	} else if (_GyroRange == 3) {
		gyro_sensitivity = 16.4;
	}
}

void _getMotion6(float *_ax, float *_ay, float *_az, float *_gx, float *_gy, float *_gz) {
	int16_t ax,ay,az;
	int16_t gx,gy,gz;
	// read raw accel/gyro measurements from device
	// The accelerometer output is a 16-bit signed integer relative value.
	// The gyroscope output is a relative value in degrees per second (dps).
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	// Convert relative to absolute
	*_ax = (float)ax / accel_sensitivity;
	*_ay = (float)ay / accel_sensitivity;
	*_az = (float)az / accel_sensitivity;
	// Convert relative degree per second to absolute radian per second
	*_gx = ((float)gx / gyro_sensitivity) * 0.0174533;
	*_gy = ((float)gy / gyro_sensitivity) * 0.0174533;
	*_gz = ((float)gz / gyro_sensitivity) * 0.0174533;
}

void setup() {
	nh.getHardware()->setBaud(115200);
	nh.initNode(); //Default=57600bps
	nh.advertise(chatter);
	nh.advertise(pubimu);

	for (int i=0;i<100;i++) {
		sprintf(buffer, "Start until %d millSec", 100-i);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		nh.spinOnce();
		delay(100);
	}
	setupMPU();
	nextMillis = millis() + INTERVAL;
}

void loop() {
	static float ax_sum = 0.0;
	static float ay_sum = 0.0;
	static float az_sum = 0.0;
	static float gx_sum = 0.0;
	static float gy_sum = 0.0;
	static float gz_sum = 0.0;
	static int dataSteps = 0;

	float ax, ay, az;
	float gx, gy, gz;
	_getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	ax_sum += ax;
	ay_sum += ay;
	az_sum += az;
	gx_sum += gx;
	gy_sum += gy;
	gz_sum += gz;
	dataSteps++;
  
	if (millis() > nextMillis) {
		strcpy(buffer, "accel_sensitivity=");
		dtostrf(accel_sensitivity, 8, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " gyro_sensitivity=");
		dtostrf(gyro_sensitivity, 8, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();

		sprintf(buffer, "dataSteps=%d", dataSteps);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();

		float ax_avr = ax_sum/dataSteps;
		float ay_avr = ay_sum/dataSteps;
		float az_avr = az_sum/dataSteps;
		float gx_avr = gx_sum/dataSteps;
		float gy_avr = gy_sum/dataSteps;
		float gz_avr = gz_sum/dataSteps;

		strcpy(buffer, "ax_avr: ");
		dtostrf(ax_avr, 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ay_avr: ");
		dtostrf(ay_avr, 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " az_avr: ");
		dtostrf(az_avr, 6, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();

		strcpy(buffer, "gx_avr: ");
		dtostrf(gx_avr, 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " gy_avr: ");
		dtostrf(gy_avr, 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " gz_avr: ");
		dtostrf(gz_avr, 6, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();

		imu.header.frame_id = "imu";
		imu.header.stamp = nh.now();
		imu.angular_velocity.x = gx_avr;
		imu.angular_velocity.y = gy_avr;
		imu.angular_velocity.z = gz_avr;
		imu.linear_acceleration.x = ax_avr;
		imu.linear_acceleration.y = ay_avr;
		imu.linear_acceleration.z = az_avr;
		pubimu.publish(&imu);

		ax_sum = 0.0;
		ay_sum = 0.0;
		az_sum = 0.0;
		gx_sum = 0.0;
		gy_sum = 0.0;
		gz_sum = 0.0;
		dataSteps = 0;

		nh.spinOnce();
		nextMillis = millis() + INTERVAL;
	}
}
