// Pin definitions for the shift register
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8

// Motor 1 control pins
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR1_PWM 11

// Motor 2 control pins
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR2_PWM 3

// Motor 3 control pins
#define MOTOR3_A 5
#define MOTOR3_B 7 
#define MOTOR3_PWM 6

// Motor 4 control pins 
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR4_PWM 5

// Define movements 
#define FORWARD 1
#define REVERSE 2
#define RELEASE 3
#define BRAKE 4
#define LEFT_FORWARD 5
#define RIGHT_FORWARD 6
#define LEFT_REVERSE 7
#define RIGHT_REVERSE 8


// Define forward US sensor stepper motor pins and step sequence
const int forwardSensorStepperPins[4] = {26, 28, 30, 32};

// Full-step sequence
const int stepCount = 4;
int stepSequence[stepCount][4] = {
  {1, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 1},
  {1, 0, 0, 1}
};
int currentStep = 0;
const int stepDelay = 2;   // 2ms delay between steps
const int steps45deg = 512; // 4096 steps for full rotation when accounting for reduction ratio

void setup() {

  Serial.begin(9600);
  Serial.println("Commencing setup...");

  // Initialize the shift register
  Serial.println("Initializing the shift register");
  pinMode(MOTORLATCH, OUTPUT);
  pinMode(MOTORENABLE, OUTPUT);
  pinMode(MOTORDATA, OUTPUT);
  pinMode(MOTORCLK, OUTPUT);

  // Set default values for the shift register
  Serial.println("Setting default values for the shift register");
  digitalWrite(MOTORDATA, LOW);
  digitalWrite(MOTORLATCH, LOW);
  digitalWrite(MOTORCLK, LOW);
  digitalWrite(MOTORENABLE, LOW);

  // Reset all drive motors to LOW 
  Serial.println("Resetting all drive motors to LOW...");
  for (int i = 1; i <= 4; i++) { 
    drive_motor(i, RELEASE, 0); // Set each motor to "release" with 0 speed
  }
  Serial.println("All motors set to LOW.");

  // Initiate pins for stepper motor for front sensor as output and low 
  Serial.println("Initializing front sensor stepper motor");
  for (int i = 0; i < 4; i++) {
    pinMode(forwardSensorStepperPins[i], OUTPUT);
    digitalWrite(forwardSensorStepperPins[i], LOW);
  }

  Serial.println("Setup Complete.");
}

void loop() {
  // drive_motor(1, FORWARD, 150);
  // drive_motor(2, FORWARD, 150);
  // drive_motor(3, FORWARD, 150);
  // drive_motor(4, FORWARD, 150);
  // delay(2000); // Run for 2 seconds


  // Serial.println("Rotating front sensor 45 degrees clockwise");
  // rotateFrontSensorMotor(steps45deg, 1);
  // delay(1000);
  // Serial.println("Rotatint front sensor 45 degrees anticlockwise");
  // rotateFrontSensorMotor(steps45deg, -1);
  // delay(1000);

}

void drive_motor(int motor_num, int command, int speed) {
  int motorA, motorB, motorPWM;
  switch (motor_num) {
    case 1:
      motorA = MOTOR1_A;
      motorB = MOTOR1_B;
      motorPWM = MOTOR1_PWM;
      break;
    case 2:
      motorA = MOTOR2_A;
      motorB = MOTOR2_B;
      motorPWM = MOTOR2_PWM;
      break;
    case 3:
      motorA = MOTOR3_A;
      motorB = MOTOR3_B;
      motorPWM = MOTOR3_PWM;
      break;
    case 4:
      motorA = MOTOR4_A;
      motorB = MOTOR4_B;
      motorPWM = MOTOR4_PWM;
      break;
  }

  switch (command) {
    case FORWARD: 
      shiftWrite(motorA, HIGH);
      shiftWrite(motorB, LOW);
      analogWrite(motorPWM, speed);
      break;
    case REVERSE:
      shiftWrite(motorA, LOW);
      shiftWrite(motorB, HIGH);
      analogWrite(motorPWM, speed);
      break;
    case RELEASE:
      shiftWrite(motorA, LOW);
      shiftWrite(motorB, LOW);
      analogWrite(motorPWM, 0);
      break;
    case BRAKE:
      shiftWrite(motorA, HIGH);
      shiftWrite(motorB, HIGH);
      analogWrite(motorPWM, 255);
      break;
  }

}

void shiftWrite(int output, int high_low) {
  static int latch_copy;
  static int shift_register_initialized = false;

  // Initialize the shift register on first use
  if (!shift_register_initialized) {
    latch_copy = 0;
    shift_register_initialized = true;
  }

  // Update the latch copy with the new output state
  bitWrite(latch_copy, output, high_low);

  // Shift out the data to the shift register
  shiftOut(MOTORDATA, MOTORCLK, MSBFIRST, latch_copy);

  // Latch the data to the output pins
  digitalWrite(MOTORLATCH, HIGH);
  delayMicroseconds(5); // Small delay for safety
  digitalWrite(MOTORLATCH, LOW);
}

// Function to rotate the stepper motor for the front US sensor in a given direction
void rotateFrontSensorMotor(int steps, int direction) {
  for (int i = 0; i < steps; i++) {
    currentStep = (currentStep + direction + stepCount) % stepCount;
    
    for (int pin = 0; pin < 4; pin++) {
      digitalWrite(forwardSensorStepperPins[pin], stepSequence[currentStep][pin]);
    }
    delay(stepDelay);
  }
}
