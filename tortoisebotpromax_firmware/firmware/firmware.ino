#include <ros.h>
#include <tortoisebotpromax_firmware/Diff.h>

#define rpwmPin 3
#define rdirPin 4
#define lpwmPin 5
#define ldirPin 6

int rigpwm = 0;
int lefpwm = 0;

ros::NodeHandle nh;

void diffCb(const tortoisebotpromax_firmware::Diff& toggle_msg){

  analogWrite(rpwmPin, toggle_msg.rpwm.data);
  analogWrite(lpwmPin, toggle_msg.lpwm.data);

  digitalWrite(rdirPin, toggle_msg.rdir.data);
  digitalWrite(ldirPin, toggle_msg.ldir.data);
  
}

ros::Subscriber<tortoisebotpromax_firmware::Diff> sub_diff("diff", diffCb );

void setup() {
  
  pinMode (rpwmPin, OUTPUT);
  pinMode (rdirPin, OUTPUT);
  pinMode (lpwmPin, OUTPUT);
  pinMode (ldirPin, OUTPUT);

  analogWrite(rpwmPin, 0);
  analogWrite(lpwmPin, 0);
  
  nh.initNode();
  nh.subscribe(sub_diff);
  
}

void loop() {

  nh.spinOnce();
  delay(10);

}
