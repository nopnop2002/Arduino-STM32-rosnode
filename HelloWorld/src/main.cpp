#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

//Serial1 is not used even if there is
//#define USE_STM32_HW_SERIAL

ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
//char hello[13] = "hello world!";

void led_cb(const std_msgs::Bool& msg){
  char wk[64];
  sprintf(wk, "msg.data=%d", msg.data);
  nh.loginfo(wk);
  if(msg.data)digitalWrite(LED_BUILTIN, HIGH);
  else digitalWrite(LED_BUILTIN, LOW);
}
ros::Subscriber<std_msgs::Bool> sub0("led", &led_cb);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode(); //Default=57600bps
  nh.advertise(chatter);
  nh.subscribe(sub0);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  char hello[64];
  sprintf(hello, "hello world! %ld", millis());
  //nh.loginfo(hello);
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(500);
}
