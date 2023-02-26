#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

//Serial1 is not used even if there is
//#define USE_STM32_HW_SERIAL

ros::NodeHandle  nh;
std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char wk[64];

void led_cb(const std_msgs::Bool& msg){
  sprintf(wk, "msg.data=%d", msg.data);
  nh.loginfo(wk);
#if defined (LED_BUILTIN)
  sprintf(wk, "PB11=%d PB12=%d PC13=%d LED_BUILTIN=%d", PB11, PB12, PC13, LED_BUILTIN);
  nh.loginfo(wk);
  if(msg.data) {
    strcpy(wk, "LED_BUILTIN gose HIGH");
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    strcpy(wk, "LED_BUILTIN gose LOW");
    digitalWrite(LED_BUILTIN, LOW);
  }
#else
  strcpy(wk, "LED_BUILTIN is undefined");
#endif
  nh.loginfo(wk);
}
ros::Subscriber<std_msgs::Bool> sub0("led", &led_cb);

void setup() {
  nh.getHardware()->setBaud(115200);
  nh.initNode(); //Default=57600bps
  nh.advertise(chatter);
  nh.subscribe(sub0);

#if defined (LED_BUILTIN)
  pinMode(LED_BUILTIN, OUTPUT);
#endif
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
