#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>


// Initialize ROS node
ros::NodeHandle nh;

// Buffer for string message
char str_msg_buffer[50];

// Initialize string message
std_msgs::String str_msg;
ine constants

#define WHEEL_DISTANCE 0.195
#define ENCODER_A_LEFT 2
#define ENCODER_B_LEFT 4
#define ENCODER_A_RIGHT 3
#define ENCODER_B_RIGHT 8

#define MOTOR_LEFT_IN1 5
#define MOTOR_LEFT_IN2 6
#define MOTOR_RIGHT_IN1 9
#define MOTOR_RIGHT_IN2 10

#define MAX_SPEED 255

//960 ticks per round 213 mm
// Wheel Diameter 68mm
// PI*68 = 213.714 mm
// Define variables

#define mps_ticks 224.299082191 // 1/0.004458333
#define LSPEED 0.0
#define RSPEED 0.0
#define DT 50 // PID control interval

volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

double left_speed_setpoint = LSPEED * mps_ticks;
double right_speed_setpoint = RSPEED * mps_ticks;

//pid out
double left_motor_output = 0;
double right_motor_output = 0;

double left_speed_actual = 0;
double right_speed_actual = 0;
unsigned long last_time = 0;

double Kp = 10.0; // Proportional gain
double Ki = 0.001; // Integral gain
double Kd = 0.001; // Derivative gain

double integral_left, integral_right, prev_error_left, prev_error_right;


void cmdVelCallback(const geometry_msgs::Twist& msg) 
{

  float linearVel = msg.linear.x;
  float angularVel = msg.angular.z;
  
  // Calculate motor speeds based on the robot's wheel distance
  left_speed_setpoint = (linearVel - (WHEEL_DISTANCE * angularVel) / 2)*mps_ticks;
  right_speed_setpoint = (linearVel + (WHEEL_DISTANCE * angularVel) / 2)*mps_ticks;

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


// Interrupt service routine for left encoder
void leftEncoderISR() 
{
  int encoder_b_left_state = digitalRead(ENCODER_B_LEFT);
  // Increment or decrement left_encoder_ticks based on the state
  
  if (encoder_b_left_state == HIGH) 
  {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
}


// Interrupt service routine for right encoder
void rightEncoderISR() 
{
  int encoder_b_right_state = digitalRead(ENCODER_B_RIGHT);
  
  if (encoder_b_right_state == LOW) 
  {
    right_encoder_ticks++;
  } else 
  {
    right_encoder_ticks--;
  }
}


// Function to set left motor speed
void setLeftMotorSpeed(int speed) 
{
  if (speed > 0) 
  {
    digitalWrite(MOTOR_LEFT_IN1, HIGH);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    analogWrite(MOTOR_LEFT_IN1, speed);
  } 
  else if (speed < 0) 
  {
    speed = -speed;
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, HIGH);
    if (speed > MAX_SPEED) speed = MAX_SPEED;
    analogWrite(MOTOR_LEFT_IN2, speed);
  } 
  else 
  {
    digitalWrite(MOTOR_LEFT_IN1, LOW);
    digitalWrite(MOTOR_LEFT_IN2, LOW);
    analogWrite(MOTOR_LEFT_IN1, 0);
  }
}


// Function to set right motor speed
void setRightMotorSpeed(int speed) 
{
  if (speed > 0) 
  {
    digitalWrite(MOTOR_RIGHT_IN1, HIGH);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    if (speed > 255)
    {
       speed = 255;
    }
    analogWrite(MOTOR_RIGHT_IN1, speed);
  } 
  else if (speed < 0) 
  {
    speed = -speed;
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, HIGH);
    if (speed > 255)
    {
      speed = 255;
    }
    analogWrite(MOTOR_RIGHT_IN2, speed);
  } 
  else 
  {
    digitalWrite(MOTOR_RIGHT_IN1, LOW);
    digitalWrite(MOTOR_RIGHT_IN2, LOW);
    analogWrite(MOTOR_RIGHT_IN1, 0);
  }
}


// Initialize cmd_vel subscriber
ros::Subscriber<geometry_msgs::Twist> cmdVelSub("cmd_vel", cmdVelCallback);

// Setup function
void setup() 
{

  // nh.getHardware()->setBaud(115200); // Set baud rate to 1 Mbps
  nh.initNode();
  nh.subscribe(cmdVelSub);
  
  
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), leftEncoderISR, RISING);
  pinMode(ENCODER_B_LEFT, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), rightEncoderISR, RISING);
  pinMode(ENCODER_B_RIGHT, INPUT);
}



// Loop function
void loop() 
{
  nh.spinOnce();
  
  unsigned long current_time = millis();
  if (current_time - last_time >= DT) 
  {
    left_speed_actual = left_encoder_ticks; // Convert encoder ticks to speed (assuming 10 ms interval)
    right_speed_actual = right_encoder_ticks; // Convert encoder ticks to speed (assuming 10 ms interval)
    pidControlLoop();
    
    last_time = current_time;
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
  }
  delay(5);
}
