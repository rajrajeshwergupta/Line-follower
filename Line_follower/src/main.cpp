#include <Arduino.h>
#include <TaskScheduler.h>
#include <QuickPID.h>

// Constants
#define HISTORY_SIZE 10  // Number of readings to keep in history for line detection
#define NO_LINE_COUNT 20  // Number of consecutive readings without line detection before stopping

// Motor Pin definitions for L298N motor driver
#define IN1 27  // Motor A direction pin 1
#define IN2 26  // Motor A direction pin 2
#define IN3 25  // Motor B direction pin 1
#define IN4 33  // Motor B direction pin 2
#define IR1 32
#define IR2 35  
#define IR3 34
#define ENA 23  // Motor A speed control pin (PWM)
#define ENB 15  // Motor B speed control pin (PWM)

const int offsetA = 1;  // Offset for motor A (not used in this example)
const int offsetB = 1;  // Offset for motor B (not used in this example)

// Motor speed limits
int leftMinSpeed = 0;   // Minimum speed for left motor
int rightMinSpeed = 0;  // Minimum speed for right motor
int leftMaxSpeed = 255;  // Maximum speed for left motor
int rightMaxSpeed = 255; // Maximum speed for right motor

// Sensor definitions
const uint8_t SensorCount = 3; // Number of IR sensors
uint16_t sensorValues[SensorCount]; // Array to hold sensor readings
float sensorWeights[SensorCount] = {0.5, 0.75, 0.5}; // Weights for the sensors to calculate line error

uint16_t readingHistory[SensorCount][HISTORY_SIZE]; // History of sensor readings

int32_t lineError = 0; // Variable to hold the calculated line error
uint16_t lineThreshold = 800; // Threshold for line detection
uint8_t noLineCount = 0; // Counter for consecutive readings without line detection

// PID variables
float Kp = 1; // Proportional gain
float Ki = 0.05; // Integral gain
float Kd = 0.25; // Derivative gain

// PID input and output variables
float setPointLeft = 0; // Desired setpoint for left motor
float inputLeft = 0; // Input for left PID controller
float outputLeft = 0; // Output for left PID controller

float setPointRight = 0; // Desired setpoint for right motor
float inputRight = 0; // Input for right PID controller
float outputRight = 0; // Output for right PID controller

// Create PID controllers for left and right motors
QuickPID leftPid(&inputLeft, &outputLeft, &setPointLeft, Kp, Ki, Kd, QuickPID::Action::direct);
QuickPID rightPid(&inputRight, &outputRight, &setPointRight, Kp, Ki, Kd, QuickPID::Action::direct);

// Function prototypes
void readAndCalculateError();
void calculateMotorSpeed();
void setMotorSpeed();
void recordHistory();
void print();

// Task definitions for scheduling
Task readAndCalculateErrorTask(10, TASK_FOREVER, &readAndCalculateError);
Task calculateMotorSpeedTask(10, TASK_FOREVER, &calculateMotorSpeed);
Task setMotorSpeedTask(200, TASK_FOREVER, &setMotorSpeed);
Task recordHistoryTask(30, TASK_FOREVER, &recordHistory);
Task printTask(20, TASK_FOREVER, &print);

Scheduler runner; // Create a Scheduler instance to manage tasks

void setup() {
    Serial.begin(115200); // Start serial communication for debugging
    Serial.println("Starting...");

    // Set motor control pins as outputs
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Initialize motor speed to 0 (stop)
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    // Set PID controllers to automatic mode
    leftPid.SetMode(QuickPID::Control::automatic);
    rightPid.SetMode(QuickPID::Control::automatic);

    // Add tasks to the scheduler
    runner.addTask(readAndCalculateErrorTask);
    runner.addTask(calculateMotorSpeedTask);
    runner.addTask(setMotorSpeedTask);
    runner.addTask(recordHistoryTask);
    runner.addTask(printTask);

    // Enable tasks
    readAndCalculateErrorTask.enable();
    calculateMotorSpeedTask.enable();
    setMotorSpeedTask.enable();
    recordHistoryTask.enable();
    printTask.enable();
}

void loop() {
    // Execute scheduled tasks
    runner.execute();
}

void readAndCalculateError() {
    // Read sensor values (replace with actual sensor reading logic)
    // For example, using analogRead() for IR sensors connected to analog pins
    sensorValues[0] = analogRead(IR1); // Read left IR sensor
    sensorValues[1] = analogRead(IR2); // Read center IR sensor
    sensorValues[2] = analogRead(IR3); // Read right IR sensor

    // Check if line exists
    bool lineExists = false; // Flag to indicate if a line is detected
    lineError = 0; // Reset line error for this iteration

    // Calculate line error and direction based on sensor readings
    for (int i = 0; i < SensorCount; i++) {
        lineError += (i - 1) * sensorValues[i] * sensorWeights[i]; // Calculate weighted error
        if (sensorValues[i] > lineThreshold) { // Check if the sensor reading exceeds the threshold
            lineExists = true; // Line detected
        }
    }

    if (!lineExists) { // If no line is detected
        noLineCount++; // Increment no line counter
        if (noLineCount > NO_LINE_COUNT) { // If no line detected for too long
            lineError = 0; // Reset line error to stop or go straight
        }
    } else {
        noLineCount = 0; // Reset no line counter if line is detected
    }
}

void recordHistory() {
    // Record sensor values in history for potential future use
    for (int i = 0; i < SensorCount; i++) {
        for (int j = HISTORY_SIZE - 1; j > 0; j--) {
            readingHistory[i][j] = readingHistory[i][j - 1]; // Shift history to make room for new reading
        }
        readingHistory[i][0] = sensorValues[i]; // Store current reading in history
    }
}

void calculateMotorSpeed() {
    // Calculate motor speeds based on line error using PID control
    inputLeft = lineError; // Set input for left motor PID
    inputRight = lineError; // Set input for right motor PID

    leftPid.Compute(); // Compute output for left motor
    rightPid.Compute(); // Compute output for right motor
}

// void setMotorSpeed() {
//     // Set motor speeds based on PID output
//     int leftSpeed = constrain(outputLeft, leftMinSpeed, leftMaxSpeed); // Constrain left speed
//     int rightSpeed = constrain(outputRight, rightMinSpeed, rightMaxSpeed); // Constrain right speed

//     // Control motors using L298N motor driver
//     if (lineError < 0) { // If line error is negative, turn right
//         analogWrite(ENA, leftSpeed); // Set left motor speed
//         analogWrite(ENB, 0); // Stop right motor
//     } else if (lineError > 0) { // If line error is positive, turn left
//         analogWrite(ENA, 0); // Stop left motor
//         analogWrite(ENB, rightSpeed); // Set right motor speed
//     } else { // If line error is zero, move forward
//         analogWrite(ENA, leftSpeed); // Set left motor speed
//         analogWrite(ENB, rightSpeed); // Set right motor speed
//     }
// }

void setMotorSpeed() {
    int leftSpeed = constrain(outputLeft, leftMinSpeed, leftMaxSpeed);
    int rightSpeed = constrain(outputRight, rightMinSpeed, rightMaxSpeed);

    if (leftSpeed >= 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
    } else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
    }

    if (rightSpeed >= 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
    } else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
    }

    analogWrite(ENA, abs(leftSpeed));
    analogWrite(ENB, abs(rightSpeed));
}

void print() {
    // Print sensor values and motor speeds for debugging purposes
    Serial.print("Sensor Values: "); // Print sensor readings
    for (int i = 0; i < SensorCount; i++) {
        Serial.print(sensorValues[i]); // Print each sensor value
        Serial.print(" "); // Space between values
    }
    Serial.print(" | Line Error: "); // Print line error
    Serial.print(lineError); // Print current line error
    Serial.print(" | Left Speed: "); // Print left motor speed
    Serial.print(outputLeft); // Print left motor output
    Serial.print(" | Right Speed: "); // Print right motor speed
    Serial.println(outputRight); // Print right motor output

    Serial.print("Set Goal");
    Serial.println(setPointLeft);
    Serial.print("Set Goal");
    Serial.println(setPointRight);
}