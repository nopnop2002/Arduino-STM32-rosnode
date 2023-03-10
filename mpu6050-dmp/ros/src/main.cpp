#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <MPU6050_6Axis_MotionApps20.h>

#define MPU_ADDRESS 0x68

MPU6050 mpu;

ros::NodeHandle  nh;
std_msgs::String str_msg;
geometry_msgs::PoseStamped pose;
ros::Publisher chatter("chatter", &str_msg);
ros::Publisher pubpose("pose", &pose);

// MPU control/status vars
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [roll, pitch, yaw]   roll/pitch/yaw container and gravity vector

// chatter stuff
char buffer[64];
char wk[10];

void setupMPU() {
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  mpu.initialize();

  // make sure it worked (returns 0 if so)
  devStatus = mpu.dmpInitialize();
  if (devStatus != 0) {
    //Serial.print("DMP Initialization failed.");
    strcpy(buffer, "DMP Initialization failed.");
    str_msg.data = buffer;
    chatter.publish( &str_msg );
    nh.spinOnce();
    while(1) {}
  }

  // supply your own offsets here, scaled for min sensitivity
  mpu.setXAccelOffset(-2889);
  mpu.setYAccelOffset(-444);
  mpu.setZAccelOffset(698);
  mpu.setXGyroOffset(149);
  mpu.setYGyroOffset(27);
  mpu.setZGyroOffset(17);

  // Calibration Time: generate offsets and calibrate our MPU6050
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);
}

void getYawPitchRoll() {
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
}

//char frame_id[] = "mpu6050";
char frame_id[] = "map";

void getQuaternion() {
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  //Serial.print(q.x);
  //Serial.print(",");
  //Serial.print(q.y);
  //Serial.print(",");
  //Serial.print(q.z);
  //Serial.print(",");
  //Serial.println(q.w);
  strcpy(buffer, "q.x:");
  dtostrf(q.x, 6, 2, wk);
  strcat(buffer, wk);
  strcat(buffer, " q.y:");
  dtostrf(q.y, 6, 2, wk);
  strcat(buffer, wk);
  strcat(buffer, " q.z:");
  dtostrf(q.z, 6, 2, wk);
  strcat(buffer, wk);
  strcat(buffer, " q.w:");
  dtostrf(q.w, 6, 2, wk);
  strcat(buffer, wk);
  //Serial.println(buffer);
  str_msg.data = buffer;
  chatter.publish( &str_msg );

  pose.header.stamp = nh.now();
  pose.header.frame_id = frame_id;
  pose.pose.orientation.x = q.x;
  pose.pose.orientation.y = q.y;
  pose.pose.orientation.z = q.z;
  pose.pose.orientation.w = q.w;
  pubpose.publish( &pose );
}

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode(); //Default=57600bps
  nh.advertise(chatter);
  nh.advertise(pubpose);
  //Serial.begin(115200);
  setupMPU();
}

void loop() {
  uint8_t device;
  I2Cdev::readByte(MPU_ADDRESS, MPU6050_RA_WHO_AM_I, &device);
  sprintf(buffer, "device id is 0x%x", device);
  str_msg.data = buffer;
  chatter.publish( &str_msg );

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    getYawPitchRoll();
    getQuaternion();
    nh.spinOnce();
  }
  delay(100);
}
