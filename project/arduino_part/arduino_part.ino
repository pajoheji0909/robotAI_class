#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Point.h>
#include <NewPing.h>
#include <SoftwareSerial.h>

#define cds_sensor A0

typedef enum pin_num {to_mode0 = 1, interrupt_pin, IN1, human_sensor, IN2, IN3, bt_tx, bt_rx, IN4, sonic1_tr, sonic1_ec,  sonic2_tr, sonic2_ec};
//                            1             2       ~3       4        ~5   ~6      7    8      ~9     ~10        ~11         12           13

void motor_control();
void interrupt_func();  
void sensor_read();     // cds + sonic sensor
int  detected_human(); 
int  detect_human();
int  mode_robot = 0;
/* mode 0 hide                   *  
 * mode 1 stop                   *
 * mode 2 detected by player     *
 * mode 3 seek                   * 
 * mode 4 detect player          */
float sonic_max = 250; // maximum range that i can trust sensor value 
float cm_one = 0 ; 
float cm_two = 0 ;
float cm_long = 0;

SoftwareSerial mySerial (8,7);

ros::NodeHandle  nh;

std_msgs::Int32 mode_sel;
std_msgs::Int32 hide_way;

void ros_subscribing (const geometry_msgs::Point & toggle_msgs)
{ 
  if (toggle_msgs.x != -1) { // if raspberry pi is not on mode 3 -> ros publishes -1 
    int wide_pixel = 600 ; // change after check the pixels
    cm_one = sonic_max*(toggle_msgs.x - 0.5*wide_pixel)/wide_pixel;
    cm_two = sonic_max - sonic_max*(toggle_msgs.x - 0.5*wide_pixel)/wide_pixel;
    cm_long = toggle_msgs.z;
  }
  delay(5); // change if it have a problem
}

ros::Subscriber<geometry_msgs::Point> mode_sub ("point data from raspberry pi : ", &ros_subscribing);
ros::Publisher mode_pub ("mode value to raspberry pi : ", &mode_sel);

void ros_publishing () 
{
  mode_sel.data = mode_robot;
  mode_pub.publish(&mode_sel);
  delay(5); // change if it have a problem
}

void setup() 
{
  // put your setup code here, to run once:
  nh.initNode();
  nh.subscribe(mode_sub);
  nh.advertise(mode_pub);
  pinMode (to_mode0, INPUT);
  pinMode (to_mode3, INPUT);
  pinMode (cds_sensor, INPUT);
  pinMode (human_sensor, INPUT);
  pinMode (sonic1_tr, OUTPUT);
  pinMode (sonic1_ec, INPUT);
  pinMode (sonic2_tr, OUTPUT);
  pinMode (sonic2_ec, INPUT);
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);
  pinMode (to_mode0, INPUT);
  pinMode (to_mode3, INPUT);
  pinMode (human_sensor, INPUT);  
  //pinMode (mA_EN, LOW);
  //pinMode (mB_EN, LOW);
  Serial.begin(9600);
  mySerial.begin(9600);
  attachInterrupt (0, interrupt_func, FALLING);
  while (!Serial) {
      
  }
}

void motor_control() 
{ 
  if (mode_robot == 1 || mode_robot == 2 || mode_robot == 4) {
    //pinMode (mA_EN, LOW);
    //pinMode (mB_EN, LOW);
    analogWrite(IN1, 0);
    analogWrite(IN2, LOW);
    analogWrite(IN3, 0);
    analogWrite(IN4, LOW);
  }
  int v1 = cm_two/3 + 30;
  int v2 = cm_one/3 + 30;
  if (mode_robot == 0 || mode_robot == 3){
    //pinMode (mA_EN, HIGH);
    //pinMode (mB_EN, HIGH);
    analogWrite(IN2, v1);
    analogWrite(IN1, LOW);
    analogWrite(IN4, LOW);
    analogWrite(IN3, v2);
    delay(10);
  }
}

void interrupt_func() 
{ //fill in later ^m^
   /*
   if (mode_robot != 0)
      mode_robot = 0;
   if (mode_robot != 3)
      mode_robot = 3;
   */
}

void sensor_read() 
{
  float sonic_bund = 12.0;
  NewPing sonar[2] = {
    NewPing(sonic1_tr, sonic1_ec, sonic_max),
    NewPing(sonic2_tr, sonic2_ec, sonic_max)
  };
  cm_one = sonar[0].ping_cm();
  cm_two = sonar[1].ping_cm();
    
  int cds_bund = 200; // trigger value for cds sensor
  int cds_value = 0;
  cds_value = analogRead(A0);
  if (cm_one < sonic_bund && cm_two < sonic_bund) {
      mode_robot = 1;        
  }
  else { 
      mode_robot = 0;
  }
}

int detected_human() 
{
  int result = -1;
  digitalRead(human_sensor); // based on pull-down switch
  return result;
}

int detect_human() 
{
  if (cm_long > 200) 
    mode_robot = 4;
}

void loop() 
{  
  sensor_read();
  
  int sw_mode = -1;
  sw_mode = digitalRead(0);
  
  if (sw_mode == 1) {
    if (mode_robot == 0)
      mode_robot = 3;
    else if (mode_robot == 3) {
      mode_robot = 0;
    }
  }
  
  switch(mode_robot) {
    case(0):
    motor_control();
    
    case(1):
      char tmp;
      if (mySerial.available()) {
        tmp = (mySerial.read()); 
        Serial.write(mySerial.read());
      } 
      if (tmp == 's') { 
        mySerial.println("success"); 
        analogWrite(IN2, LOW);
        analogWrite(IN1, 50);
        analogWrite(IN4, 50);
        analogWrite(IN3, LOW);
        delay(1000);
      }
      if (tmp == 'w') { 
        mySerial.println("success"); 
        analogWrite(IN2, 60);
        analogWrite(IN1, LOW);
        analogWrite(IN4, LOW);
        analogWrite(IN3, 60);
        delay(1000);
      }
      if (tmp == 'a') { 
        mySerial.println("success"); 
        analogWrite(IN2, 70);
        analogWrite(IN1, LOW);
        analogWrite(IN4, 30);
        analogWrite(IN3, LOW);
        delay(300);
      }
      if (tmp == 'd') { 
        mySerial.println("success"); 
        analogWrite(IN2, LOW);
        analogWrite(IN1, 30);
        analogWrite(IN4, LOW);
        analogWrite(IN3, 70);
        delay(300);
      }            
      if (tmp == 'x') { 
        mySerial.println("success"); 
        analogWrite(IN2, LOW);
        analogWrite(IN1, 0);
        analogWrite(IN4, LOW);
        analogWrite(IN3, 0);
        delay(10000);
      } 
      nh.spinOnce();
    case(2):
      for (int i = 0; i < 10; i++) {
        nh.spinOnce();
      }
    case(3):
      nh.spinOnce();
      detect_human();
      motor_control();
    case(4):
      motor_control();
      for (int i = 0; i < 10; i ++) {
        nh.spinOnce(); 
      }
   default:  
      break;
   }
   Serial.println(mode_robot);
}

