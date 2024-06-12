#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Instantiate the VL53L0X ToF Sensor
Adafruit_VL53L0X tof = Adafruit_VL53L0X();

// Motor A (left motor) connections
const int motorA_PWM = 9;  // ENA
const int motorA_dir1 = 2; // IN1
const int motorA_dir2 = 3; // IN2

// Motor B (right motor) connections
const int motorB_PWM = 10; // ENB
const int motorB_dir1 = 4; // IN3
const int motorB_dir2 = 5; // IN4

// PD controller constants
float Kp = 22.0;  // Proportional gain
float Kd = 21.0;  // Derivative gain

// Desired distance from the right wall (in cm)
const float desired_distance = 30.0;

// Previous distance error for derivative term
float previous_error = 0;

void setup() {
  Serial.begin(9600);

  // Initialize motor control pins as outputs
  pinMode(motorA_PWM, OUTPUT);
  pinMode(motorA_dir1, OUTPUT);
  pinMode(motorA_dir2, OUTPUT);
  pinMode(motorB_PWM, OUTPUT);
  pinMode(motorB_dir1, OUTPUT);
  pinMode(motorB_dir2, OUTPUT);

  // Start the ToF sensor
  if (!tof.begin()) {
    Serial.println("Failed to boot VL53L0X");
    while (1);
  }
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  tof.rangingTest(&measure, false);  // get measurement from ToF sensor

  if (measure.RangeStatus != 4) {  // range status is good
    float current_distance = measure.RangeMilliMeter / 10.0; // Convert mm to cm
    float error = current_distance - desired_distance; // Error calculation for right side
    float P = Kp * error;
    float D = Kd * (error - previous_error);
    previous_error = error;  // Update previous error for next iteration
    float control_signal = P + D;

    // Calculate motor speeds for clockwise motion
    int baseSpeed = 150;  // Base speed of motors
    int leftSpeed = baseSpeed - control_signal; // Decrease left wheel speed to turn left
    int rightSpeed = baseSpeed + control_signal; // Increase right wheel speed to turn left

    // Set motor speeds using PWM
    analogWrite(motorA_PWM, constrain(leftSpeed, 0, 255));
    analogWrite(motorB_PWM, constrain(rightSpeed, 0, 255));

    // Set motor directions
    digitalWrite(motorA_dir1, HIGH);
    digitalWrite(motorA_dir2, LOW);
    digitalWrite(motorB_dir1, HIGH);
    digitalWrite(motorB_dir2, LOW);
  }
  delay(50);
}

