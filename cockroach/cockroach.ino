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

// Initiate echo sensor pins 
#define TRIG 23
#define ECHO 25
#define USMAX 3000

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

  // Initiate front US sensor
  Serial.println("Setting up front US sensor");
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT); 
  digitalWrite(TRIG, LOW);

  Serial.println("Setup Complete.");
}

void loop() {
  
  //TO DO
    // - record time for which driving forward 
    // - use time forward to calculate reverse time if obstical encountered 
    // - for closer distances move at slower speed (n.b. may require kick to overcome friction and intertia)


  // Drive sequence
  int distance = getFrontDistance(15000);

  // Drive forward at full speed while more than 15cm or pulse times out
  while (distance > 15 || distance == -1) {
    Serial.println("All drive motors FORWARD at full speed");
    for(int i = 1; i <=4; i++) {
      drive_motor(i, FORWARD, 255);
    }
    delay(500);
    distance = getFrontDistance(15000);
  }

  //stop if obstical detected; double check distance and if true reverse and change direction;
  if (distance < 5 || distance == -2) {
    distance = getFrontDistance(15000); 
    if (distance < 5 || distance == -2) {
      Serial.println("Obstical detected. STOPPING all drive motors");
    }
    for(int i = 1; i <= 4; i++) {
      drive_motor(i, RELEASE, 0);
    }
    Serial.println("Moving away from obstical. REVERSING all drive motors");
    for(int i = 1; i <=4; i++) {
      drive_motor(i, REVERSE, 255);
    }
    delay(1500);

    Serial.println("Moved away from obstical. STOPPING all drive motors");
    for(int i = 1; i <= 4; i++) {
      drive_motor(i, RELEASE, 0);
    }

    Serial.println("Determining direction to turn");
    Serial.println("Rotating front sensor 45 degrees clockwise");
    rotateFrontSensorMotor(steps45deg, 1);
    int distance_right = getFrontDistance(15000);
    Serial.print("Distance to right: ");
    Serial.println(distance_right);
    Serial.println("Rotating front sensor 90 degrees anticlockwise");
    rotateFrontSensorMotor(steps45deg * 2, -1);
    int distance_left = getFrontDistance(15000);
    Serial.print("Distance to left: ");
    Serial.println(distance_left);
    Serial.println("Returning front sensor to neutral position");
    rotateFrontSensorMotor(steps45deg, 1);

    if (distance_left > distance_right) {
      Serial.println("Turning left");
    } else if (distance_right > distance_left) {
      Serial.println("Turning right");
    } else if (distance_right == distance_left && distance_right > 0) {
      // pick random direction to turn
    }

  }
}

// Functions to set drive motors to specific directions and speed
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
      break;
    case BRAKE:
      shiftWrite(motorA, HIGH);
      shiftWrite(motorB, HIGH);
      analogWrite(motorPWM, 255);
      break;
    default:
      Serial.print("Invalid motor command: ");
      Serial.println(command);
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

// Function to determine distance in cm from front to obstical 
int getFrontDistance(long utimeout) {
  long startTime, pulseDuration;

  // Ensure sensor is ready for new reading
  if (digitalRead(ECHO) == HIGH) {
    Serial.println("Echo pin stuck HIGH, returning 0.");
    return -3;
  }

  // Send the trigger pulse
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  // Wait for ECHO to go HIGH (start of pulse)
  startTime = micros();
  while (digitalRead(ECHO) == LOW) {
    if ((micros() - startTime) > 2000) {  // 2ms timeout to detect pulse start
      Serial.println("Echo start timeout.");
      return -3;
    }
  }

  // Capture start time
  startTime = micros();

  // Wait for ECHO to go LOW (end of pulse)
  while (digitalRead(ECHO) == HIGH) {
    if ((micros() - startTime) > utimeout) {  // Sensor timeout
      Serial.println("Echo pulse timeout.");
      return -1;
    }
  }

  // Calculate pulse duration
  pulseDuration = micros() - startTime;

  // Reject very short pulses (noise)
  if (pulseDuration < 200) {
    Serial.println("Invalid pulse detected.");
    return -2;
  }

  // Convert pulse time to distance (cm)
  int distance = pulseDuration / 58;

  // Debug output
  Serial.print("Front distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}


