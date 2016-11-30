#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <Servo.h>

typedef enum pin_num {not_0 = 1, not_1, IN1, not_2, IN2, IN3, mA_EN, mB_EN, IN4, not_3, not_4, not_5, not_6};
//                            1      2  ~3     4    ~5   ~6    7      8     ~9   ~10    ~11     12     13

void setup() {
  // put your setup code here, to run once:
  pinMode ( 
}

void loop() {
  // put your main code here, to run repeatedly:

}
