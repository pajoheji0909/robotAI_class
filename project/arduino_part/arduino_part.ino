#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Int32.h>
#include <NewPing.h>

#define cds_sensor A0

typedef enum pin_num {to_mode0 = 1, interrupt_pin, IN1, human_sensor, IN2, IN3, detect_button, to_mode3, IN4, sonic1_tr, sonic1_ec,  sonic2_tr, sonic2_ec};
//                            1             2       ~3       4        ~5   ~6       7            8      ~9     ~10        ~11         12           13

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
int sonic1_val = 0; int sonic2_val = 0; // sonic 1 is left, sonic 2 is right.if value is 1 = closer
float sonic_max = 200; // maximum range that i can trust sensor value 
float cm_one = 0; float cm_two = 0;

ros::NodeHandle  nh;
std_msgs::Int32 mode_sel;
std_msgs::Int32 hide_way;
void ros_subscribing (const std_msgs::Int32 &toggle_msgs) 
{ 
  if (toggle_msgs.data != -1) { // if raspberry pi is not on mode 3 -> ros publishes -1 
    int wide_pixel = 1000; // change after check the pixels
    cm_one = sonic_max*(toggle_msgs.data - 0.5*wide_pixel)/wide_pixel;
    cm_two = sonic_max - sonic_max*(toggle_msgs.data - 0.5*wide_pixel)/wide_pixel;
  }
  delay(10); // change if it have a problem
}

ros::Subscriber<std_msgs::Int32> mode_sub ("mode value from raspberry pi : ", &ros_subscribing);
ros::Publisher mode_pub ("mode value to raspberry pi : ", &mode_sel);

void ros_publishing () 
{
  mode_sel.data = mode_robot;
  mode_pub.publish(&mode_sel);
  delay(10); // change if it have a problem
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
  attachInterrupt (0, interrupt_func, CHANGE);
}

void motor_control() 
{ 
  // a for sonic1_val, b for sonic2_val
  if (mode_robot == 1 || mode_robot == 2 || mode_robot == 4) {
    //pinMode (mA_EN, LOW);
    //pinMode (mB_EN, LOW);
    analogWrite(IN1, 0);
    analogWrite(IN2, LOW);
    analogWrite(IN3, 0);
    analogWrite(IN4, LOW);
  }
  else if (mode_robot == 0){
    //pinMode (mA_EN, HIGH);
    //pinMode (mB_EN, HIGH);
    analogWrite(IN1, cm_one/2);
    analogWrite(IN2, LOW);
    analogWrite(IN3, LOW);
    analogWrite(IN4, cm_two/2);
    delay(10);
  }
}

void interrupt_func() 
{ //fill in later ^m^
   if (mode_robot != 0)
      mode_robot = 0;
   if (mode_robot != 3)
      mode_robot = 3;
}

void sensor_read() 
{
  float sonic_bund = 5;
  NewPing sonar[2] = {
    NewPing(sonic1_tr, sonic1_ec, sonic_max),
    NewPing(sonic2_tr, sonic2_ec, sonic_max)
  };
  cm_one = sonar[0].ping_cm();
  cm_two = sonar[1].ping_cm();
  
  if (cm_one < sonic_bund)
    sonic1_val = 1;
  else
    sonic1_val = 0; 
  if (cm_two < sonic_bund)
    sonic2_val = 1;
  else
    sonic2_val = 0;
    
  int cds_bund = 200; // trigger value for cds sensor
  int cds_value = 0;
  cds_value = analogRead(A0);
 
  if (sonic1_val ==1 && sonic2_val == 1) {
    // if (cds_value < cds_bund) for test! later you should change here!
      mode_robot = 1;
      delay(2000);
      mode_robot = 0;        
  }
}

int detected_human() 
{
  int result = 0;
  digitalRead(human_sensor); // based on pull-down switch
  return result;
}

int detect_human() 
{
  
}

void loop() 
{
  // put your main code here, to run repeatedly:
  switch(mode_robot) {
  case(0):
    sensor_read();
    delay(10);
    motor_control();
    delay(50);
    nh.spinOnce();
  case(1):
    //if (detected_human() == 1)
    //mode_robot = 2;
    nh.spinOnce();
  case(2):
    nh.spinOnce();
  case(3):
    motor_control();
    nh.spinOnce();
  default:  
    break;
  }
}

