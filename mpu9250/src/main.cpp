#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mySensor;

ros::NodeHandle  nh;
std_msgs::String str_msg;
sensor_msgs::Imu imu;
sensor_msgs::MagneticField mag;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher pubimu("imu/data_raw", &imu);
ros::Publisher pubmag("imu/mag", &mag);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode(); //Default=57600bps
  nh.advertise(chatter);
  nh.advertise(pubimu);
  nh.advertise(pubmag);

  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();

  // You can set your own offset for mag values
  // mySensor.magXOffset = -50;
  // mySensor.magYOffset = -55;
  // mySensor.magZOffset = -10;
}

void loop() {
  uint8_t sensorId;
  int result;
  float aX, aY, aZ, aSqrt, gX, gY, gZ, mDirection, mX, mY, mZ;
  char buffer[64];
  char wk[10];

  result = mySensor.readId(&sensorId);
  if (result == 0) {
    sprintf(buffer, "sensorId: %d", sensorId);
  } else {
    sprintf(buffer, "Cannot read sensorId %d", result);
  }
  str_msg.data = buffer;
  chatter.publish( &str_msg );

  result = mySensor.accelUpdate();
  if (result == 0) {
    aX = mySensor.accelX();
    aY = mySensor.accelY();
    aZ = mySensor.accelZ();
    aSqrt = mySensor.accelSqrt();
    //Serial.println("accelX: " + String(aX));
    //Serial.println("accelY: " + String(aY));
    //Serial.println("accelZ: " + String(aZ));
    //Serial.println("accelSqrt: " + String(aSqrt));
    dtostrf(aX, 9, 6, wk);
    strcpy(buffer, "accelX: ");
    strcat(buffer, wk);
    dtostrf(aY, 9, 6, wk);
    strcat(buffer, " accelY: ");
    strcat(buffer, wk);
    dtostrf(aZ, 9, 6, wk);
    strcat(buffer, " accelZ: ");
    strcat(buffer, wk);
    str_msg.data = buffer;
    chatter.publish( &str_msg );
  } else {
    sprintf(buffer, "Cannod read accel values %d", result);
  	str_msg.data = buffer;
 	chatter.publish( &str_msg );
  }

  result = mySensor.gyroUpdate();
  if (result == 0) {
    gX = mySensor.gyroX();
    gY = mySensor.gyroY();
    gZ = mySensor.gyroZ();
    //Serial.println("gyroX: " + String(gX));
    //Serial.println("gyroY: " + String(gY));
    //Serial.println("gyroZ: " + String(gZ));
    dtostrf(gX, 9, 6, wk);
    strcpy(buffer, "gyroX: ");
    strcat(buffer, wk);
    dtostrf(gY, 9, 6, wk);
    strcat(buffer, " gyroY: ");
    strcat(buffer, wk);
    dtostrf(gZ, 9, 6, wk);
    strcat(buffer, " gyroZ: ");
    strcat(buffer, wk);
    str_msg.data = buffer;
    chatter.publish( &str_msg );
  } else {
    sprintf(buffer, "Cannot read gyro values %d", result);
  	str_msg.data = buffer;
 	chatter.publish( &str_msg );
  }

  result = mySensor.magUpdate();
  if (result == 0) {
    mX = mySensor.magX();
    mY = mySensor.magY();
    mZ = mySensor.magZ();
    mDirection = mySensor.magHorizDirection();
    //Serial.println("magX: " + String(mX));
    //Serial.println("maxY: " + String(mY));
    //Serial.println("magZ: " + String(mZ));
    //Serial.println("horizontal direction: " + String(mDirection));
    dtostrf(mX, 9, 6, wk);
    strcpy(buffer, "magX: ");
    strcat(buffer, wk);
    dtostrf(mY, 9, 6, wk);
    strcat(buffer, " magY: ");
    strcat(buffer, wk);
    dtostrf(mZ, 9, 6, wk);
    strcat(buffer, " magZ: ");
    strcat(buffer, wk);
    dtostrf(mDirection, 9, 6, wk);
    strcat(buffer, " horizontal direction: ");
    strcat(buffer, wk);
    str_msg.data = buffer;
    chatter.publish( &str_msg );
  } else {
    sprintf(buffer, "Cannot read mag values %d", result);
  	str_msg.data = buffer;
 	chatter.publish( &str_msg );
  }

  imu.header.frame_id = "imu";
  imu.header.stamp = nh.now();
  imu.angular_velocity.x = gX;
  imu.angular_velocity.y = gY;
  imu.angular_velocity.z = gZ; // [rad/sec]
  imu.linear_acceleration.x = aX;      
  imu.linear_acceleration.y = aY;  
  imu.linear_acceleration.z = aZ; 
  pubimu.publish(&imu);

  mag.header.frame_id = "imu";
  mag.header.stamp = nh.now();
  mag.magnetic_field.x = mX;
  mag.magnetic_field.y = mY;
  mag.magnetic_field.z = mZ; // [μT]
  pubmag.publish(&mag);
  nh.spinOnce();

  delay(500);
}
