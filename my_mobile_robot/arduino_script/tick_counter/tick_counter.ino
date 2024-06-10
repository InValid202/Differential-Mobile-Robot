#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle nh;

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 4
#define ENC_IN_RIGHT_B 8

#define in1 5
#define in2 6
  
#define in3 9
#define in4 10

#define MAX_SPEED 255

#define WHEEL_DISTANCE 0.201
#define mps_ticks 224.299082191

float left_speed_setpoint = 0;
float right_speed_setpoint = 0;

float left_speed_actual = 0;
float right_speed_actual = 0;
 
// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const long encoder_minimum = -32768;
const long encoder_maximum = 32768;


// One-second interval for measurements
int interval = 100;
long previousMillis = 0;
long currentMillis = 0;


// PID Parameter

double Kp = 10.0; // Proportional gain
double Ki = 0.0001; // Integral gain
double Kd = 0.001; // Derivative gain

double integral_left, integral_right, prev_error_left, prev_error_right;


std_msgs::Int16 right_wheel_tick_count;
std_msgs::Int16 left_wheel_tick_count;

// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}

 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}


// MOTOR CONTROL FUNCTION

void cmdVelCallback(const geometry_msgs::Twist &msg) 
{

  float linearVel = msg.linear.x;
  float angularVel = msg.angular.z;
  
  // Calculate motor speeds based on the robot's wheel distance
  left_speed_setpoint = (linearVel - (WHEEL_DISTANCE * angularVel) / 2)*mps_ticks;
  right_speed_setpoint = (linearVel + (WHEEL_DISTANCE * angularVel) / 2)*mps_ticks;

}


void setLeftMotorSpeed(int speed) 
{
  if (speed > 0) 
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    analogWrite(in1, speed);
  } 
  else if (speed < 0) 
  {
    speed = -speed;
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    analogWrite(in2, speed);
  } 
  else 
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(in1, 0);
  }
}


// Function to set right motor speed
void setRightMotorSpeed(int speed) 
{
  if (speed > 0) 
  {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    if (speed > 255)
    {
       speed = 255;
    }
    analogWrite(in3, speed);
  } 
  else if (speed < 0) 
  {
    speed = -speed;
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    if (speed > 255)
    {
      speed = 255;
    }
    analogWrite(in4, speed);
  } 
  else 
  {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    analogWrite(in3, 0);
  }
}


void pidControlLoop() 
{
  // Read encoder values (AB encoders)
  // Calculate current left and right wheel speeds (in m/s)
  
  // Calculate errors
  double error_left = left_speed_setpoint - left_speed_actual;
  double error_right = right_speed_setpoint - right_speed_actual;
  
  // Update integral terms
  integral_left += error_left;
  integral_right += error_right;
  
  // Calculate PID control outputs
  double control_output_left = Kp * error_left + Ki * integral_left + Kd * (error_left - prev_error_left);
  double control_output_right = Kp * error_right + Ki * integral_right + Kd * (error_right - prev_error_right);
  
  
  setLeftMotorSpeed(control_output_left);
  setRightMotorSpeed(control_output_right);
  
  
  // Update previous errors
  prev_error_left = error_left;
  prev_error_right = error_right;
}


// declare publisher and subscriber

// Keep track of the number of wheel ticks
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

//Subscribe to cmdvel topic to control robot with rqt or other via cmd_vel
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", cmdVelCallback); 


// start void setup
void setup() 
{

 
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(cmdVelSub);
}

//Doesn't finish yet
void loop() 
{
  // Record the time
  currentMillis = millis();
 
  // If one second has passed, print the number of ticks
  if (currentMillis - previousMillis > interval) 
  {
    previousMillis = currentMillis;

    left_speed_actual = 0; // Convert encoder ticks to speed (assuming 10 ms interval)
    right_speed_actual = 0; // Convert encoder ticks to speed (assuming 10 ms interval)
    pidControlLoop();
     
    rightPub.publish( &right_wheel_tick_count );
    leftPub.publish( &left_wheel_tick_count );
  }
  nh.spinOnce();
  delay(1);
}
