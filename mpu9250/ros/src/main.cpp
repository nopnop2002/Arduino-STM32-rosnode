#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <AK8963.h>

ros::NodeHandle  nh;
std_msgs::String str_msg;
sensor_msgs::Imu imu;
sensor_msgs::MagneticField mag;

ros::Publisher chatter("chatter", &str_msg);
ros::Publisher pubimu("imu/data_raw", &imu);
ros::Publisher pubmag("imu/mag", &mag);

#define mpu_address 0x68
#define mag_address 0x0C

MPU6050 mpu;
AK8963 ak8963(0x0C);

// MPU control/status vars
uint8_t mpuIntStatus;	// holds actual interrupt status byte from MPU
uint8_t devStatus;		// return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;	// expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;			// [w, x, y, z]			quaternion container
VectorFloat gravity;	// [x, y, z]			gravity vector
float ypr[3];			// [roll, pitch, yaw]	roll/pitch/yaw container and gravity vector

int16_t accel_raw[3];
int16_t gyro_raw[3];
float accel_real[3];
float gyro_real[3];
float accel_sensitivity;
float gyro_sensitivity;

// Value from Fuse ROM
uint8_t MagAdjustmentValue[3];
float magCalibration[3];

// sampling interval
#define INTERVAL 500
unsigned long nextMillis;

char buffer[64];
char wk[10];

void write_mpu(byte add, byte data) {
	Wire.beginTransmission(mpu_address);
	Wire.write(add);
	Wire.write(data);
	Wire.endTransmission();
}

byte read_mpu(byte add) {
	byte k;
	Wire.beginTransmission(mpu_address);
	Wire.write(add);
	Wire.endTransmission();
	Wire.requestFrom(mpu_address, 1);
	while (Wire.available()) {
		k = Wire.read();
	}
	return k;
}

byte read_mag(byte add) {
	byte k;
	Wire.beginTransmission(mag_address);
	Wire.write(add);
	Wire.endTransmission();
	Wire.requestFrom(mag_address, 1);
	while (Wire.available()) {
		k = Wire.read();
	}
	return k;
}

bool getMagInt(int16_t *intData) {
	strcpy(buffer, "magCalibration X:");
	dtostrf(magCalibration[0], 5, 2, wk);
	strcat(buffer, wk);
	strcat(buffer, " Y:");
	dtostrf(magCalibration[1], 5, 2, wk);
	strcat(buffer, wk);
	strcat(buffer, " Z:");
	dtostrf(magCalibration[2], 5, 2, wk);
	strcat(buffer, wk);
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	//nh.spinOnce();

	uint8_t rawData[7];
	for (int i=0;i<7;i++) {
		uint8_t reg = i + 0x03;
		rawData[i] = read_mag(reg);
		sprintf(buffer, "read_mag(0x%d)=%x", reg, rawData[i]);
		//Serial.println(buffer);
		str_msg.data = buffer;
		//chatter.publish( &str_msg );
		//nh.spinOnce();
	}

	if(rawData[6] & 0x08) {
		strcpy(buffer, "*****magnetic sensor overflow*****");
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();
		return false;
	}
	intData[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	intData[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	intData[2] = ((int16_t)rawData[4] << 8) | rawData[5];
	sprintf(buffer, "intData=0x%x 0x%x 0x%x", intData[0], intData[1], intData[2]);
	//Serial.println(buffer);
	str_msg.data = buffer;
	//chatter.publish( &str_msg );
	//nh.spinOnce();
	return true;
}

void getMagTesla(int16_t *intData, float *microTesla) {
	float toGauss = 10.*4912./32760.0; // Proper scale to return milliGauss
	float toTesla = 0.15;
	// Convert to micro tesla
	for (int i=0;i<3;i++) {
		microTesla[i] = (float)intData[i] * magCalibration[i] * toTesla; // micro Tesla
		//microGauss[i] = (float)intData[i] * magCalibration[i] * toGauss; // mill Gauss
	}
}

void getMagGauss(int16_t *intData, float *millGauss) {
	float toGauss = 10.*4912./32760.0; // Proper scale to return milliGauss
	float toTesla = 0.15;
	// Convert to mill Gauss
	for (int i=0;i<3;i++) {
		//microTesla[i] = (float)intData[i] * magCalibration[i] * toTesla; // micro Tesla
		millGauss[i] = (float)intData[i] * magCalibration[i] * toGauss; // mill Gauss
	}
}

bool getMagData(int16_t *intMagData, float *microTesla, float *millGauss) {
	//sprintf(buffer, "ak8963.getDeviceID()=0x%x", ak8963.getDeviceID());
	//Serial.println(buffer);
	if (ak8963.getDeviceID() != 0x48) {
		sprintf(buffer, "ak8963.getDeviceID()=0x%x", ak8963.getDeviceID());
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();
		strcpy(buffer, "*****AK8963 connection lost*****");
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();
		write_mpu(0x37,0x02); // connect AK8963
		delay(500);
		return false;
	}

	//sprintf(buffer, "ak8963.getMode()=0x%x", ak8963.getMode());
	//Serial.println(buffer);
	if (ak8963.getMode() != 0x06) {
		sprintf(buffer, "ak8963.getMode()=0x%x", ak8963.getMode());
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();
		strcpy(buffer, "*****AK8963 illegal data mode*****");
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();
		write_mpu(0x37,0x02); // connect AK8963
		delay(500);
		return false;
	}

	//sprintf(buffer, "ak8963.getDataReady()=0x%x", ak8963.getDataReady());
	//Serial.println(buffer);
	if (!ak8963.getDataReady()) {
		sprintf(buffer, "ak8963.getDataReady()=0x%x", ak8963.getDataReady());
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();
		strcpy(buffer, "*****AK8963 data not ready*****");
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();
		return false;
	}


	//int16_t intMagData[3];
	if (getMagInt(intMagData)) {
		sprintf(buffer, "intMagData=%d %d %d", intMagData[0], intMagData[1], intMagData[2]);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();

		getMagTesla(intMagData, microTesla);
		strcpy(buffer, "microTesla:");
		dtostrf(microTesla[0], 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(microTesla[1], 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(microTesla[2], 6, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();

		getMagGauss(intMagData, millGauss);
		strcpy(buffer, "millGauss:");
		dtostrf(millGauss[0], 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(millGauss[1], 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " ");
		dtostrf(millGauss[2], 6, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();
		return true;
	} else {
		strcpy(buffer, "*****AK8963 magnetic sensor overflow*****");
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();
		return false;
	}
	return false;
}

void setupMPU() {
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
	mpu.initialize();
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXAccelOffset(-3232);
	mpu.setYAccelOffset(-464);
	mpu.setZAccelOffset(688);
	mpu.setXGyroOffset(151);
	mpu.setYGyroOffset(23);
	mpu.setZGyroOffset(18);

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// Calibration Time: generate offsets and calibrate our MPU6050
		mpu.CalibrateAccel(6);
		mpu.CalibrateGyro(6);
		mpu.setDMPEnabled(true);
		packetSize = mpu.dmpGetFIFOPacketSize();
	} else {
		//Serial.print("DMP Initialization failed.");
		strcpy(buffer, "DMP Initialization failed.");
		while(1) {
			//Serial.println(buffer);
			str_msg.data = buffer;
			chatter.publish( &str_msg );
			nh.spinOnce();
			delay(1000);
		}
	}

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

void setupMAG() {
	// Bypass Enable Configuration
	// Connect AK8963
	write_mpu(0x37,0x02);

	// When user wants to change operation mode, transit to power-down mode first and then transit to other modes. 
	// Goto Powerdown Mode
	ak8963.setMode(0x00);
	// After power-down mode is set, at least 100ms(Twat) is needed before setting another mode.
	delay(200);

	// Goto Fuse ROM access mode
	ak8963.setMode(0x0F);
	delay(200);

	// Get Sensitivity adjustment value from fuse ROM
	// Sensitivity adjustment values for each axis are written in the fuse ROM at the time of shipment.
	MagAdjustmentValue[0] = ak8963.getAdjustmentX();
	MagAdjustmentValue[1] = ak8963.getAdjustmentY();
	MagAdjustmentValue[2] = ak8963.getAdjustmentZ();
	sprintf(buffer, "MagAdjustmentValue: %x %x %x", MagAdjustmentValue[0], MagAdjustmentValue[1], MagAdjustmentValue[2]);
	//Serial.println(buffer);
	str_msg.data = buffer;
	chatter.publish( &str_msg );
	nh.spinOnce();

	for (int i=0;i<3;i++) {
		magCalibration[i] = (float)(MagAdjustmentValue[i] - 128)/256.0f + 1.0f;
		dtostrf(magCalibration[i], 5, 2, wk);
		sprintf(buffer, "magCalibration[%d]=%s",i, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		nh.spinOnce();
	}

	// Goto Powerdown Mode
	ak8963.setMode(0x00);
	delay(200);

	// Goto oparation mode
	// mode1:Automatically repeat sensor measurements at 8Hz
	// mode2:Automatically repeat sensor measurements at 100Hz
	// 0x?0:Power doen mode.
	// 0x?1:Single measurement mode.
	// 0x?2:Continuous measurement mode 1.
	// 0x?6:Continuous measurement mode 2.
	// 0x?8:External trigger measurement mode.
	// 0x0?:14 bit output 0.60 uT/LSB --> 1 = 0.60uT
	// 0x1?:16 bit output 0.15 uT/LSB --> 1 = 0.15uT
	Serial.println("mag mode set to 0x16");
	Serial.println("Continuous measurement mode 2");
	Serial.println("6 bit output 0.15 uT/LSB");
	ak8963.setMode(0x16);
}

void getRawData() {
	int16_t ax,ay,az;
	int16_t gx,gy,gz;
	// read raw accel/gyro measurements from device
	// The accelerometer output is a 16-bit signed integer relative value.
	// The gyroscope output is a relative value in degrees per second (dps).
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	accel_raw[0] = ax;
	accel_raw[1] = ay;
	accel_raw[2] = az;
	gyro_raw[0] = gx;
	gyro_raw[1] = gy;
	gyro_raw[2] = gz;

	// Convert relative to absolute
	accel_real[0] = (float)ax / accel_sensitivity;
	accel_real[1] = (float)ay / accel_sensitivity;
	accel_real[2] = (float)az / accel_sensitivity;
	// Convert relative degree per second to absolute radian per second
	//gyro_real[0] = gx * 0.0174533;
	//gyro_real[1] = gy * 0.0174533;
	//gyro_real[2] = gz * 0.0174533;
	gyro_real[0] = ((float)gx / gyro_sensitivity) * 0.0174533;
	gyro_real[1] = ((float)gy / gyro_sensitivity) * 0.0174533;
	gyro_real[2] = ((float)gz / gyro_sensitivity) * 0.0174533;
#if 0
	Serial.print("ax: ");
	Serial.print(accel_real[0]);
	Serial.print(" ay: ");
	Serial.print(accel_real[1]);
	Serial.print(" az: ");
	Serial.print(accel_real[2]);
	Serial.print(" gx: ");
	Serial.print(gyro_real[0]);
	Serial.print(" gy: ");
	Serial.print(gyro_real[1]);
	Serial.print(" gz: ");
	Serial.print(gyro_real[2]);
	Serial.println();
#endif
}


void getYawPitchRoll() {
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		//Serial.print(ypr[2] * 180 / M_PI);
		//Serial.print(",");
		//Serial.print(ypr[1] * 180 / M_PI);
		//Serial.print(",");
		//Serial.println(ypr[0] * 180 / M_PI);
		float roll = ypr[2] * 180 / M_PI;
		float pitch = ypr[1] * 180 / M_PI;
		float yaw = ypr[0] * 180 / M_PI;
		strcpy(buffer, "roll:");
		dtostrf(roll, 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " pitch:");
		dtostrf(pitch, 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " yaw:");
		dtostrf(yaw, 6, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();
	}
}

void getQuaternion() {
	if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		//Serial.print(q.x);
		//Serial.print(",");
		//Serial.print(q.y);
		//Serial.print(",");
		//Serial.print(q.z);
		//Serial.print(",");
		//Serial.println(q.w);
		strcpy(buffer, "x:");
		dtostrf(q.x, 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " y:");
		dtostrf(q.y, 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " z:");
		dtostrf(q.z, 6, 2, wk);
		strcat(buffer, wk);
		strcat(buffer, " w:");
		dtostrf(q.w, 6, 2, wk);
		strcat(buffer, wk);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		//nh.spinOnce();
	}
}

void setup() {
	nh.getHardware()->setBaud(115200);
	nh.initNode(); //Default=57600bps
	nh.advertise(chatter);
	nh.advertise(pubimu);
	nh.advertise(pubmag);

	for (int i=0;i<100;i++) {
		sprintf(buffer, "Start until %d millSec", 100-i);
		//Serial.println(buffer);
		str_msg.data = buffer;
		chatter.publish( &str_msg );
		nh.spinOnce();
		delay(100);
	}
	setupMPU();
	setupMAG();
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

	getRawData();
	ax_sum += accel_real[0];
	ay_sum += accel_real[1];
	az_sum += accel_real[2];
	gx_sum += gyro_real[0];
	gy_sum += gyro_real[1];
	gz_sum += gyro_real[2];
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

		getYawPitchRoll();
		//getQuaternion();

		int16_t intMagData[3];
		float microTeslaData[3];
		float millGaussData[3];
		if (getMagData(intMagData, microTeslaData, millGaussData)) {
			mag.header.frame_id = "imu";
			mag.header.stamp = nh.now();
			mag.magnetic_field.x = microTeslaData[0];
			mag.magnetic_field.y = microTeslaData[1];
			mag.magnetic_field.z = microTeslaData[2];
			pubmag.publish(&mag);

#if 0
			strcpy(buffer, "microTesla:");
			dtostrf(microTeslaData[0], 6, 2, wk);
			strcat(buffer, wk);
			strcat(buffer, " ");
			dtostrf(microTeslaData[1], 6, 2, wk);
			strcat(buffer, wk);
			strcat(buffer, " ");
			dtostrf(microTeslaData[2], 6, 2, wk);
			strcat(buffer, wk);
			//Serial.println(buffer);
			//str_msg.data = buffer;
			//chatter.publish( &str_msg );
			//nh.spinOnce();

			strcpy(buffer, "millGauss:");
			dtostrf(millGaussData[0], 6, 2, wk);
			strcat(buffer, wk);
			strcat(buffer, " ");
			dtostrf(millGaussData[1], 6, 2, wk);
			strcat(buffer, wk);
			strcat(buffer, " ");
			dtostrf(millGaussData[2], 6, 2, wk);
			strcat(buffer, wk);
			//Serial.println(buffer);
			//str_msg.data = buffer;
			//chatter.publish( &str_msg );
			//nh.spinOnce();
#endif
		}

		nh.spinOnce();
		nextMillis = millis() + INTERVAL;
	}
}
