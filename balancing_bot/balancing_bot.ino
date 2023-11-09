#include <deque>
#include <algorithm>
#include <JY901.h>
#include <Wire.h>
#include <VL53L0X.h>

// PID controller constants
float error_integral = 0; // keep track of accumulated error for PID control
float Kp = 1.0;
float Ki = 0.0001;
float Kd = 0.2;
float balance_anlge = 0; // default balance angle

float error_integral_move = 0; // keep track of accumulated error for PID control
float Kp_move = 1.0;
float Ki_move = 0.0001;
float Kd_move = 0.2;
float forward_balance_angle = -1.5; // balance while driving forward at this angle
float follow_distance = 3000; // move toward object when it is further away than follow_distance
float backward_balance_angle = 2; // balance while driving backward at this angle
float backward_distance = 500; // move away from object when it is closer than backward_distance

// stepper constants
const float stepsPerRevolution = 200.0;  
const int leftDIR = 12;
const int leftSTEP = 14;
const int rightDIR = 27;
const int rightSTEP = 26;
const float micros_delay_btw_pulses = 800;


// sensor reading 
bool sensor_read_loop_1ms = false;
bool sensor_read_loop_5ms = false;
float control_loop_interval = 2.0; // about every 2ms we should get control output
bool control_loop = false; 
int counter_1ms = 0; // count how many times "1ms" has passed a.k.a how many times we have read the 1000 Hertz sensor
unsigned long micros_timer_start = 0.0;
unsigned long micros_curr = 0.0;
float old_anlge = 0.0;
float old_angular_vel = 0.0;
float alpha = 0.9; // alpha for low-pass filter

VL53L0X lidar;

struct GyroData {
  float gyro_x;
  float gyro_y;
  float gyro_z;
};

struct AngleData {
  float angle_x;
  float angle_y;
  float angle_z;
};


void setup() {
  Serial.begin(115200);
  // attach stepper pins
  pinMode(leftDIR, OUTPUT);
  pinMode(leftSTEP, OUTPUT);
  pinMode(rightDIR, OUTPUT);
  pinMode(rightSTEP, OUTPUT);
  // WT901 pins
  Serial1.begin(9600, SERIAL_8N1, 16, 17);
  Wire.begin();
  lidar.init();
  lidar.setAddress(0x29);  // Use the default address if not specified.
  lidar.setTimeout(500);
  lidar.startContinuous(); // Start continuous back-to-back mode (the sensor will range continuously with frequent updates).
  // start timer for reading sensors at 1000 Hertz
  micros_timer_start = micros();
}

// function to read the sensor
void read_wt901(){

  while (Serial1.available()) 
  {
    JY901.CopeSerialData(Serial1.read()); //Call JY901 data cope function
  }

}

// move both steppers one step
void moveOneStep(bool clockwise=true){
  if (clockwise){
    digitalWrite(rightDIR, HIGH);
    digitalWrite(leftDIR, HIGH);
  } else {
    digitalWrite(rightDIR, LOW);
    digitalWrite(leftDIR, LOW);
  }
  
  digitalWrite(rightSTEP, HIGH);
  digitalWrite(leftSTEP, HIGH);
  delayMicroseconds(micros_delay_btw_pulses);
  digitalWrite(rightSTEP, LOW);
  digitalWrite(leftSTEP, LOW);
  delayMicroseconds(micros_delay_btw_pulses);
}

// utility function converting angle to steps. Clockwise is positive
float angleToSteps(long angle){
  return angle / (360.0 / stepsPerRevolution);
}

// utility function calculating the delay that we should have between each steps
// so that the stepper moves the given number of steps in the given time step size
float stepperDelay(float steps, float time_step_size){
  float time_for_each_step = abs(time_step_size / steps);
  return time_for_each_step - 2 * micros_delay_btw_pulses;
}

// move both steppers angle degrees in time_step_size (microseconds)
void moveAngle(float angle, float time_step_size){
  long steps = angleToSteps(angle);
  int int_steps = (int)round(steps);
  if (steps == 0){
    return;
  }
  long stepper_delay = stepperDelay(int_steps, time_step_size);
  
  for (int i = 0; i < abs(int_steps); i++){
    if (angle > 0){
      moveOneStep();
    } else {
      moveOneStep(false);
    }
    delayMicroseconds(stepper_delay);
  }

}

// SENSOR READING

GyroData readAngularRates1k() {
  GyroData gyroValues;
  gyroValues.gyro_x = ((float)JY901.stcGyro.w[0] / 32768 * 2000);
  gyroValues.gyro_y = ((float)JY901.stcGyro.w[1] / 32768 * 2000);
  gyroValues.gyro_z = ((float)JY901.stcGyro.w[2] / 32768 * 2000);

  //Serial.print("Gyro:");Serial.print(gyroValues.gyro_x, 4); Serial.print(","); Serial.print(gyroValues.gyro_y, 4); Serial.print(","); Serial.println(gyroValues.gyro_z, 4); 

  return gyroValues;
}

// read angles from WT901
AngleData readAngles1k(){
  AngleData angleValues;

  // output of angle readings (look at the example serial file in the JY901 library for some other outputs if desired (like accel or gyro) but probably you just need the angle)
  angleValues.angle_x = (((float)JY901.stcAngle.Angle[0] / 32768 * 180));
  angleValues.angle_y = (((float)JY901.stcAngle.Angle[1] / 32768 * 180));
  angleValues.angle_z = (((float)JY901.stcAngle.Angle[2] / 32768 * 180));


  // output the desired angles 
  Serial.print("Angles:");Serial.print(angleValues.angle_x, 4); Serial.print(","); Serial.print(angleValues.angle_y, 4); Serial.print(","); Serial.println(angleValues.angle_z, 4); 
  return angleValues;
}


float lowPassFilter(float new_val, float old_val){
  return alpha * new_val + (1 - alpha) * old_val;
}


// CONTROL
// given current angle and angular_vel, calculate the control output

float PID(float angle, float angular_vel, float target_angle){
  float error =  - (target_angle - angle);
  error_integral  = error_integral + error; // update the accumulated error
  return Kp * error + Ki * error_integral + Kd * angular_vel;
}

float PID_forward(float angle, float angular_vel){
  float error = - (forward_balance_angle - angle);
  error_integral_move  = error_integral_move + error; // update the accumulated error
  return Kp_move * error + Ki_move * error_integral + Kd_move * angular_vel;
}


void loop() {
 
  
  read_wt901();
  GyroData gyroData = readAngularRates1k(); // get current angular rate
  float new_angular_rates_reading = lowPassFilter(gyroData.gyro_y, old_angular_vel);
  old_angular_vel = new_angular_rates_reading;

  AngleData angleData = readAngles1k(); // get current angle
  float new_angle_reading = lowPassFilter(angleData.angle_y, old_anlge);
  old_anlge = new_angle_reading;

  int new_distance = lidar.readRangeContinuousMillimeters();
  Serial.print("distance: "); Serial.println(new_distance, 4);
  micros_curr = micros();

  
  if (micros_curr - micros_timer_start >= control_loop_interval * 1000){
     
    // if object is more than follow_distance away, we move forward to follow it
    if (new_distance >= follow_distance){
      float control_out = PID(new_angle_reading, new_angular_rates_reading, forward_balance_angle);
      Serial.print("forward control out: ");Serial.println(control_out);
      // calculate the max angle the motor can cover in the given control loop interval
      float motor_max_angle = control_loop_interval * (float)1000.0 / (2 * micros_delay_btw_pulses) * ((float)360.0 / stepsPerRevolution);
      float sign = 1.0;
      if (control_out < 0){sign = -1.0;}
      // cap input to the moveAngle function at motor max
      if (abs(control_out) < motor_max_angle) {
        moveAngle(control_out, control_loop_interval * 1000);
      } else {
        moveAngle(motor_max_angle * sign, control_loop_interval * 1000);
      }
    } else if (new_distance <= backward_distance){
      // object is too close, back away
      float control_out = PID(new_angle_reading, new_angular_rates_reading, backward_balance_angle);
      Serial.print("backward control out: ");Serial.println(control_out);
      // calculate the max angle the motor can cover in the given control loop interval
      float motor_max_angle = control_loop_interval * (float)1000.0 / (2 * micros_delay_btw_pulses) * ((float)360.0 / stepsPerRevolution);
      float sign = 1.0;
      if (control_out < 0){sign = -1.0;}
      if (abs(control_out) < motor_max_angle) {
        moveAngle(control_out, control_loop_interval * 1000);
      } else {
        moveAngle(motor_max_angle * sign, control_loop_interval * 1000);
      }
    } else {
      // object is within normal range, maintain normal balance mode
      float control_out = PID(new_angle_reading, new_angular_rates_reading, 0);
      Serial.print("control out: ");Serial.println(control_out);
      // calculate the max angle the motor can cover in the given control loop interval
      float motor_max_angle = control_loop_interval * (float)1000.0 / (2 * micros_delay_btw_pulses) * ((float)360.0 / stepsPerRevolution);
      float sign = 1.0;
      if (control_out < 0){sign = -1.0;}
      if (abs(control_out) < motor_max_angle) {
        moveAngle(control_out, control_loop_interval * 1000);
      } else {
        moveAngle(motor_max_angle * sign, control_loop_interval * 1000);
      }
    }
    
  }

  micros_timer_start = micros_curr;
  
}